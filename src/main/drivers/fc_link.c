/*
 * This file is part of Rotorflight.
 *
 * Rotorflight is free software. You can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Rotorflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software. If not, see <https://www.gnu.org/licenses/>.
 */

// Secondary UART link between two flight controllers sharing one SBUS/FBUS
// redundancy bus. Three frame types share the wire:
//  - a fast heartbeat carrying role/arm/failsafe/RX status, a compact
//    compatibility/config fingerprint, plus the sender's current servo
//    channel values (PWM and bus servos alike), so a SLAVE can relay the
//    MASTER's actual output everywhere the MASTER outputs it. The redundancy
//    bus device only checks frame validity, not channel content, so two
//    independently-computed streams could otherwise disagree every frame;
//    relaying keeps them identical while the link is up.
//  - a slower "tuning" frame carrying the MASTER's currently active PID and
//    rate profile, so a SLAVE's fallback tuning matches whatever MASTER is
//    actually flying with (including in-flight adjustment-function changes
//    that were never saved to EEPROM), not just its own last-saved values.
//  - a "config" frame family used only for a one-shot base-config sync
//    (mixer, servos, ports, etc.): SLAVE requests it (at boot, once, if its
//    config fingerprint disagrees with MASTER's, or any time via CLI);
//    MASTER streams every parameter group except an exclusion list of
//    per-board identity settings (serial port function assignment, chiefly);
//    SLAVE applies them to its own live registry, persists via the existing
//    writeEEPROM() and reboots. Unlike the other two frame types this is
//    never continuous -- it only ever runs once per request.
// Channel/tuning mirroring falls back to the SLAVE's own local computation
// the moment the peer heartbeat is lost. Role comes solely from which port
// function was assigned (FUNCTION_FC_LINK_MASTER/_SLAVE) and is fixed for
// the life of the boot -- there is no negotiation to get wrong.

#include <math.h>
#include <string.h>

#include "platform.h"

#ifdef USE_FC_LINK

#include "build/build_config.h"
#include "build/version.h"
#include "common/crc.h"
#include "common/maths.h"
#include "common/utils.h"
#include "common/time.h"

#include "drivers/time.h"
#include "drivers/fc_link.h"
#include "drivers/system.h"

#include "io/serial.h"

#include "config/config.h"
#include "config/config_eeprom.h"

#include "pg/fc_link.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "fc/rc_rates.h"
#include "fc/runtime_config.h"
#include "scheduler/scheduler.h"
#include "flight/failsafe.h"
#include "flight/pid.h"
#include "flight/setpoint.h"
#include "rx/rx.h"

#define FC_LINK_BAUDRATE 460800
#define FC_LINK_SYNC_BYTE 0xF7
#define FC_LINK_TUNING_SYNC_BYTE 0xF8
#define FC_LINK_CONFIG_SYNC_BYTE 0xF9
#define FC_LINK_TUNING_RATE_HZ 4
#define FC_LINK_CONFIG_SEND_RATE_HZ 20

// Comfortably above the largest single registered parameter group (the
// biggest is the per-profile PID array, a few hundred bytes worst case).
#define FC_LINK_CONFIG_CHUNK_MAX 768

// Sentinel pgn values framing a config-sync session; real PG numbers are
// small (see pg/pg_ids.h), so these sit safely outside that range.
#define FC_LINK_CONFIG_PGN_REQUEST    0xFFFD
#define FC_LINK_CONFIG_PGN_SYNC_START 0xFFFE
#define FC_LINK_CONFIG_PGN_SYNC_END   0xFFFF

#define MS2US(ms) ((ms) * 1000)

typedef struct __attribute__((packed)) {
    uint8_t sync;
    uint8_t role;        // fcLinkRole_e of the sender
    uint8_t flags;        // bit0 armed, bit1 failsafeActive, bit2 rxReceivingSignal
    uint16_t seq;
    uint8_t eepromConfVersion;   // EEPROM_CONF_VERSION -- must match to accept a config sync
    uint8_t fcVersionMajor;
    uint8_t fcVersionMinor;
    uint16_t configFingerprint;  // folded CRC over all syncable PGs; mismatch triggers a SLAVE auto-sync
    uint16_t channels[FC_LINK_MAX_CHANNELS]; // sender's current servo output, in microseconds
    uint8_t checksum;
} fcLinkFrame_t;

// Mirrors the sender's currently *active* tuning (post adjustment-function,
// pre/post save -- whatever it's actually flying with right now), not the
// full EEPROM config. Applied directly into the receiver's live profile;
// never written to EEPROM by this path.
typedef struct __attribute__((packed)) {
    uint8_t sync;
    uint16_t seq;
    pidProfile_t pidProfile;
    controlRateConfig_t rateProfile;
    uint8_t checksum;
} fcLinkTuningFrame_t;

// One frame per parameter group (or a sentinel pgn framing the session).
// A single fixed-size frame keeps the receive state machine simple; nothing
// registered comes close to FC_LINK_CONFIG_CHUNK_MAX so no PG needs to be
// split across more than one of these.
typedef struct __attribute__((packed)) {
    uint8_t sync;
    uint16_t pgn;    // real PG number, or one of the FC_LINK_CONFIG_PGN_* sentinels
    uint16_t size;   // SYNC_START: total PG count to follow. PG frame: payload bytes used. SYNC_END/REQUEST: unused.
    uint8_t payload[FC_LINK_CONFIG_CHUNK_MAX];
    uint8_t checksum;
} fcLinkConfigFrame_t;

enum {
    FC_LINK_FLAG_ARMED           = (1 << 0),
    FC_LINK_FLAG_FAILSAFE_ACTIVE = (1 << 1),
    FC_LINK_FLAG_RX_RECEIVING    = (1 << 2),
};

static serialPort_t *fcLinkPort = NULL;
static fcLinkRole_e localRole = FC_LINK_ROLE_MASTER;

static uint16_t txSeq = 0;
static timeUs_t nextSendTimeUs = 0;

static uint16_t tuningTxSeq = 0;
static timeUs_t nextTuningSendTimeUs = 0;

static timeUs_t lastPeerFrameUs = 0;
static bool everReceivedPeerFrame = false;
static fcLinkPeerState_t peerState;
static uint16_t peerChannels[FC_LINK_MAX_CHANNELS];
static uint8_t peerEepromConfVersion = 0;
static uint8_t peerFcVersionMajor = 0;
static uint8_t peerFcVersionMinor = 0;
static uint16_t peerConfigFingerprint = 0;

static uint16_t localChannels[FC_LINK_MAX_CHANNELS];

// Producer (ISR) / consumer (fcLinkUpdate, scheduler task context) handoff.
// The apply steps touch live state a control-loop task also reads, so they
// must not run from interrupt context -- deferring them here means they're
// serialized against other tasks by the (non-preemptive) scheduler instead
// of racing an arbitrary main-loop read.
static fcLinkTuningFrame_t pendingTuningFrame;
static volatile bool tuningFramePending = false;

static fcLinkConfigFrame_t pendingConfigFrame;
static volatile bool configFramePending = false;

static uint8_t rxBuf[sizeof(fcLinkConfigFrame_t)];
static uint8_t rxIdx = 0;
static uint16_t rxExpectedLen = 0;

// SLAVE receive-side config-sync progress.
static bool configSyncAutoAttempted = false;
static bool configSyncInProgress = false;
static uint16_t configSyncExpectedCount = 0;
static uint16_t configSyncReceivedCount = 0;

// MASTER send-side config-sync progress.
typedef enum {
    CONFIG_SYNC_SEND_IDLE = 0,
    CONFIG_SYNC_SEND_START,
    CONFIG_SYNC_SEND_PGS,
    CONFIG_SYNC_SEND_END,
} configSyncSendState_e;

static volatile bool configSyncRequestPending = false;
static configSyncSendState_e configSyncSendState = CONFIG_SYNC_SEND_IDLE;
static uint16_t configSyncSendIndex = 0;
static timeUs_t nextConfigSyncSendTimeUs = 0;

static uint8_t fcLinkChecksumBytes(const uint8_t *bytes, size_t len)
{
    uint8_t checksum = 0;
    for (size_t i = 0; i < len; i++) {
        checksum ^= bytes[i];
    }
    return checksum;
}

// Per-board identity settings that must never be copied from a peer --
// chiefly which UART carries which FUNCTION_FC_LINK_* role. Syncing this
// would make a SLAVE overwrite its own port assignment with MASTER's.
static bool fcLinkConfigPgnExcluded(uint16_t pgn)
{
    switch (pgn) {
        case PG_SERIAL_CONFIG:
            return true;
        default:
            return false;
    }
}

// Folded CRC over every syncable PG's live bytes. Two boards running
// identical firmware produce the same value iff their syncable config is
// byte-for-byte identical, since PG_FOREACH order is fixed by the firmware
// image itself.
static uint16_t fcLinkComputeConfigFingerprint(void)
{
    uint16_t crc = 0xFFFF;
    PG_FOREACH(reg) {
        if (fcLinkConfigPgnExcluded(pgN(reg))) {
            continue;
        }
        crc = crc16_ccitt_update(crc, reg->address, pgSize(reg));
    }
    return crc;
}

static bool fcLinkPeerConfigCompatible(void)
{
    return everReceivedPeerFrame &&
        peerEepromConfVersion == EEPROM_CONF_VERSION &&
        peerFcVersionMajor == FC_VERSION_MAJOR &&
        peerFcVersionMinor == FC_VERSION_MINOR;
}

static void fcLinkHandleFrame(const fcLinkFrame_t *frame)
{
    lastPeerFrameUs = micros();
    everReceivedPeerFrame = true;

    peerState.armed = frame->flags & FC_LINK_FLAG_ARMED;
    peerState.failsafeActive = frame->flags & FC_LINK_FLAG_FAILSAFE_ACTIVE;
    peerState.rxReceivingSignal = frame->flags & FC_LINK_FLAG_RX_RECEIVING;
    peerState.seq = frame->seq;

    peerEepromConfVersion = frame->eepromConfVersion;
    peerFcVersionMajor = frame->fcVersionMajor;
    peerFcVersionMinor = frame->fcVersionMinor;
    peerConfigFingerprint = frame->configFingerprint;

    memcpy(peerChannels, frame->channels, sizeof(peerChannels));
}

static void fcLinkDataReceive(uint16_t c, void *data)
{
    UNUSED(data);

    if (rxIdx == 0) {
        if ((uint8_t)c == FC_LINK_SYNC_BYTE) {
            rxExpectedLen = sizeof(fcLinkFrame_t);
        } else if ((uint8_t)c == FC_LINK_TUNING_SYNC_BYTE) {
            rxExpectedLen = sizeof(fcLinkTuningFrame_t);
        } else if ((uint8_t)c == FC_LINK_CONFIG_SYNC_BYTE) {
            rxExpectedLen = sizeof(fcLinkConfigFrame_t);
        } else {
            return;
        }
    }

    rxBuf[rxIdx++] = (uint8_t)c;

    if (rxIdx >= rxExpectedLen) {
        switch (rxBuf[0]) {
            case FC_LINK_SYNC_BYTE: {
                fcLinkFrame_t frame;
                memcpy(&frame, rxBuf, sizeof(frame));
                if (fcLinkChecksumBytes(rxBuf, offsetof(fcLinkFrame_t, checksum)) == frame.checksum) {
                    fcLinkHandleFrame(&frame);
                }
                break;
            }
            case FC_LINK_TUNING_SYNC_BYTE: {
                if (fcLinkChecksumBytes(rxBuf, offsetof(fcLinkTuningFrame_t, checksum)) == rxBuf[offsetof(fcLinkTuningFrame_t, checksum)]) {
                    memcpy(&pendingTuningFrame, rxBuf, sizeof(pendingTuningFrame));
                    tuningFramePending = true;
                }
                break;
            }
            case FC_LINK_CONFIG_SYNC_BYTE: {
                if (fcLinkChecksumBytes(rxBuf, offsetof(fcLinkConfigFrame_t, checksum)) == rxBuf[offsetof(fcLinkConfigFrame_t, checksum)]) {
                    fcLinkConfigFrame_t frame;
                    memcpy(&frame, rxBuf, sizeof(frame));
                    if (frame.pgn == FC_LINK_CONFIG_PGN_REQUEST) {
                        configSyncRequestPending = true;
                    } else {
                        memcpy(&pendingConfigFrame, &frame, sizeof(pendingConfigFrame));
                        configFramePending = true;
                    }
                }
                break;
            }
            default:
                break;
        }
        rxIdx = 0;
    }
}

static void fcLinkSendHeartbeat(timeUs_t currentTimeUs)
{
    if (serialTxBytesFree(fcLinkPort) < sizeof(fcLinkFrame_t)) {
        return;
    }

    fcLinkFrame_t frame = {
        .sync = FC_LINK_SYNC_BYTE,
        .role = localRole,
        .flags = (uint8_t)((ARMING_FLAG(ARMED) ? FC_LINK_FLAG_ARMED : 0) |
                            (failsafeIsActive() ? FC_LINK_FLAG_FAILSAFE_ACTIVE : 0) |
                            (rxIsReceivingSignal() ? FC_LINK_FLAG_RX_RECEIVING : 0)),
        .seq = txSeq++,
        .eepromConfVersion = EEPROM_CONF_VERSION,
        .fcVersionMajor = FC_VERSION_MAJOR,
        .fcVersionMinor = FC_VERSION_MINOR,
        .configFingerprint = fcLinkComputeConfigFingerprint(),
    };
    memcpy(frame.channels, localChannels, sizeof(frame.channels));
    frame.checksum = fcLinkChecksumBytes((const uint8_t *)&frame, offsetof(fcLinkFrame_t, checksum));

    serialWriteBuf(fcLinkPort, (const uint8_t *)&frame, sizeof(frame));

    nextSendTimeUs = currentTimeUs + (1000000 / constrain(fcLinkConfig()->rateHz, FC_LINK_RATE_MIN_HZ, FC_LINK_RATE_MAX_HZ));
}

static void fcLinkSendTuning(timeUs_t currentTimeUs)
{
    if (serialTxBytesFree(fcLinkPort) < sizeof(fcLinkTuningFrame_t)) {
        return;
    }

    fcLinkTuningFrame_t frame;
    frame.sync = FC_LINK_TUNING_SYNC_BYTE;
    frame.seq = tuningTxSeq++;
    memcpy(&frame.pidProfile, currentPidProfile, sizeof(frame.pidProfile));
    memcpy(&frame.rateProfile, currentControlRateProfile, sizeof(frame.rateProfile));
    frame.checksum = fcLinkChecksumBytes((const uint8_t *)&frame, offsetof(fcLinkTuningFrame_t, checksum));

    serialWriteBuf(fcLinkPort, (const uint8_t *)&frame, sizeof(frame));

    nextTuningSendTimeUs = currentTimeUs + (1000000 / FC_LINK_TUNING_RATE_HZ);
}

static void fcLinkApplyTuningFrame(const fcLinkTuningFrame_t *frame)
{
    // Only a SLAVE ever adopts a peer's tuning; MASTER just discards it.
    if (localRole != FC_LINK_ROLE_SLAVE) {
        return;
    }

    memcpy(currentPidProfile, &frame->pidProfile, sizeof(*currentPidProfile));
    memcpy(currentControlRateProfile, &frame->rateProfile, sizeof(*currentControlRateProfile));

    // Bake the raw gains/cutoffs into the live controller state, same as any
    // other profile change -- without this the copied values would sit
    // unused until the next unrelated profile reload.
    pidLoadProfile(currentPidProfile);
    setpointInitProfile();
}

static bool fcLinkSendConfigSyncRequest(void)
{
    if (serialTxBytesFree(fcLinkPort) < sizeof(fcLinkConfigFrame_t)) {
        return false;
    }

    fcLinkConfigFrame_t frame = { .sync = FC_LINK_CONFIG_SYNC_BYTE, .pgn = FC_LINK_CONFIG_PGN_REQUEST, .size = 0 };
    memset(frame.payload, 0, sizeof(frame.payload));
    frame.checksum = fcLinkChecksumBytes((const uint8_t *)&frame, offsetof(fcLinkConfigFrame_t, checksum));

    serialWriteBuf(fcLinkPort, (const uint8_t *)&frame, sizeof(frame));
    return true;
}

bool fcLinkTriggerConfigSync(void)
{
    if (!fcLinkIsEnabled() || localRole != FC_LINK_ROLE_SLAVE || !fcLinkPeerConfigCompatible()) {
        return false;
    }
    return fcLinkSendConfigSyncRequest();
}

// SLAVE side: apply one received config-sync frame (called from fcLinkUpdate,
// scheduler task context -- never from the RX ISR, since a successful
// session ends by writing EEPROM and rebooting).
static void fcLinkApplyConfigFrame(const fcLinkConfigFrame_t *frame)
{
    if (localRole != FC_LINK_ROLE_SLAVE) {
        return;
    }

    if (frame->pgn == FC_LINK_CONFIG_PGN_SYNC_START) {
        configSyncInProgress = true;
        configSyncExpectedCount = frame->size;
        configSyncReceivedCount = 0;
        return;
    }

    if (frame->pgn == FC_LINK_CONFIG_PGN_SYNC_END) {
        const bool complete = configSyncInProgress
            && configSyncExpectedCount > 0
            && configSyncReceivedCount == configSyncExpectedCount;
        configSyncInProgress = false;

        // Never commit while either board might be flying -- this is a
        // config write + reboot, the one genuinely irreversible-in-the-air
        // action anywhere in fc_link.
        if (complete && !ARMING_FLAG(ARMED) && !peerState.armed) {
            // The flash write is slow enough that the scheduler would
            // otherwise flag this task as having overrun its budget --
            // same courtesy call saveConfigAndNotify() makes before writeEEPROM().
            schedulerIgnoreTaskExecTime();
            writeEEPROM();
            systemReset(RESET_NONE);
            // does not return
        }
        return;
    }

    if (!configSyncInProgress) {
        return; // stray PG frame outside a session; ignore
    }

    const pgRegistry_t *reg = pgFind(frame->pgn);
    if (reg && !fcLinkConfigPgnExcluded(frame->pgn)
        && pgSize(reg) == frame->size
        && frame->size <= sizeof(frame->payload)) {
        memcpy(reg->address, frame->payload, frame->size);
        configSyncReceivedCount++;
    }
    // If not found/excluded/size-mismatched, just don't count it -- the
    // SYNC_END count check catches the shortfall and the whole session is
    // discarded rather than applying a partial config.
}

// MASTER side: advance the PG-streaming state machine by (at most) one frame
// per call, paced well inside the SLAVE's drain rate so the single-slot
// pending-frame handoff on the receive side never has to overwrite an
// unconsumed frame.
static void fcLinkServiceConfigSyncSend(timeUs_t currentTimeUs)
{
    if (configSyncRequestPending) {
        configSyncRequestPending = false;
        if (localRole == FC_LINK_ROLE_MASTER
            && configSyncSendState == CONFIG_SYNC_SEND_IDLE
            && !ARMING_FLAG(ARMED)) {
            configSyncSendState = CONFIG_SYNC_SEND_START;
            configSyncSendIndex = 0;
        }
    }

    if (configSyncSendState == CONFIG_SYNC_SEND_IDLE) {
        return;
    }

    if (cmpTimeUs(currentTimeUs, nextConfigSyncSendTimeUs) < 0) {
        return;
    }
    if (serialTxBytesFree(fcLinkPort) < sizeof(fcLinkConfigFrame_t)) {
        return;
    }

    fcLinkConfigFrame_t frame;
    memset(&frame, 0, sizeof(frame));
    frame.sync = FC_LINK_CONFIG_SYNC_BYTE;

    switch (configSyncSendState) {
        case CONFIG_SYNC_SEND_START: {
            uint16_t count = 0;
            PG_FOREACH(reg) {
                if (!fcLinkConfigPgnExcluded(pgN(reg))) {
                    count++;
                }
            }
            frame.pgn = FC_LINK_CONFIG_PGN_SYNC_START;
            frame.size = count;
            configSyncSendState = CONFIG_SYNC_SEND_PGS;
            break;
        }

        case CONFIG_SYNC_SEND_PGS: {
            uint16_t index = 0;
            const pgRegistry_t *found = NULL;
            PG_FOREACH(reg) {
                if (fcLinkConfigPgnExcluded(pgN(reg))) {
                    continue;
                }
                if (index == configSyncSendIndex) {
                    found = reg;
                    break;
                }
                index++;
            }

            if (found) {
                frame.pgn = pgN(found);
                frame.size = pgSize(found);
                memcpy(frame.payload, found->address, MIN(frame.size, sizeof(frame.payload)));
                configSyncSendIndex++;
            } else {
                configSyncSendState = CONFIG_SYNC_SEND_END;
                return; // re-enter next tick to send the END frame
            }
            break;
        }

        case CONFIG_SYNC_SEND_END:
            frame.pgn = FC_LINK_CONFIG_PGN_SYNC_END;
            configSyncSendState = CONFIG_SYNC_SEND_IDLE;
            break;

        case CONFIG_SYNC_SEND_IDLE:
        default:
            return;
    }

    frame.checksum = fcLinkChecksumBytes((const uint8_t *)&frame, offsetof(fcLinkConfigFrame_t, checksum));
    serialWriteBuf(fcLinkPort, (const uint8_t *)&frame, sizeof(frame));
    nextConfigSyncSendTimeUs = currentTimeUs + (1000000 / FC_LINK_CONFIG_SEND_RATE_HZ);
}

bool fcLinkIsEnabled(void)
{
    return fcLinkPort != NULL;
}

fcLinkRole_e fcLinkGetRole(void)
{
    return localRole;
}

bool fcLinkIsMaster(void)
{
    return !fcLinkIsEnabled() || localRole == FC_LINK_ROLE_MASTER;
}

bool fcLinkPeerLost(void)
{
    if (!everReceivedPeerFrame) {
        return true;
    }
    return cmpTimeUs(micros(), lastPeerFrameUs) > (timeDelta_t)MS2US(fcLinkConfig()->peerTimeoutMs);
}

const fcLinkPeerState_t *fcLinkGetPeerState(void)
{
    return &peerState;
}

void fcLinkPublishChannels(uint8_t startIndex, const float *channels, uint8_t count)
{
    if (startIndex >= FC_LINK_MAX_CHANNELS) {
        return;
    }
    if (startIndex + count > FC_LINK_MAX_CHANNELS) {
        count = FC_LINK_MAX_CHANNELS - startIndex;
    }
    for (uint8_t i = 0; i < count; i++) {
        localChannels[startIndex + i] = (uint16_t)lrintf(channels[i]);
    }
}

bool fcLinkShouldRelay(void)
{
    return fcLinkIsEnabled() && localRole == FC_LINK_ROLE_SLAVE && !fcLinkPeerLost();
}

void fcLinkGetRelayChannels(uint8_t startIndex, float *out, uint8_t count)
{
    if (startIndex >= FC_LINK_MAX_CHANNELS) {
        return;
    }
    if (startIndex + count > FC_LINK_MAX_CHANNELS) {
        count = FC_LINK_MAX_CHANNELS - startIndex;
    }
    for (uint8_t i = 0; i < count; i++) {
        out[i] = (float)peerChannels[startIndex + i];
    }
}

void fcLinkUpdate(timeUs_t currentTimeUs)
{
    if (!fcLinkPort) {
        return;
    }

    if (tuningFramePending) {
        tuningFramePending = false;
        fcLinkApplyTuningFrame(&pendingTuningFrame);
    }

    if (configFramePending) {
        configFramePending = false;
        fcLinkApplyConfigFrame(&pendingConfigFrame);
    }

    // A stalled session (peer vanished mid-transfer) should not wait forever
    // for a SYNC_END that's never coming.
    if (configSyncInProgress && fcLinkPeerLost()) {
        configSyncInProgress = false;
    }

    // Auto-trigger: once per boot, only if the peer is a compatible build
    // and its config actually differs from ours.
    if (localRole == FC_LINK_ROLE_SLAVE && !configSyncAutoAttempted && !fcLinkPeerLost()) {
        configSyncAutoAttempted = true;
        if (fcLinkPeerConfigCompatible() && peerConfigFingerprint != fcLinkComputeConfigFingerprint()) {
            fcLinkSendConfigSyncRequest();
        }
    }

    fcLinkServiceConfigSyncSend(currentTimeUs);

    if (cmpTimeUs(currentTimeUs, nextSendTimeUs) >= 0) {
        fcLinkSendHeartbeat(currentTimeUs);
    }

    if (cmpTimeUs(currentTimeUs, nextTuningSendTimeUs) >= 0) {
        fcLinkSendTuning(currentTimeUs);
    }
}

void fcLinkInit(void)
{
    serialPortFunction_e function = FUNCTION_FC_LINK_MASTER;
    const serialPortConfig_t *portConfig = findSerialPortConfig(function);

    if (!portConfig) {
        function = FUNCTION_FC_LINK_SLAVE;
        portConfig = findSerialPortConfig(function);
    }

    if (!portConfig) {
        fcLinkPort = NULL;
        return;
    }

    localRole = (function == FUNCTION_FC_LINK_MASTER) ? FC_LINK_ROLE_MASTER : FC_LINK_ROLE_SLAVE;

    memset(&peerState, 0, sizeof(peerState));

    // Neutral midpoint until the first local channel publish arrives.
    for (uint8_t i = 0; i < FC_LINK_MAX_CHANNELS; i++) {
        localChannels[i] = 1500;
        peerChannels[i] = 1500;
    }

    fcLinkPort = openSerialPort(
        portConfig->identifier, function, fcLinkDataReceive, NULL, FC_LINK_BAUDRATE, MODE_RXTX,
        SERIAL_STOPBITS_1 | SERIAL_PARITY_NO |
            (fcLinkConfig()->inverted ? SERIAL_INVERTED : SERIAL_NOT_INVERTED) |
            (fcLinkConfig()->pinSwap ? SERIAL_PINSWAP : SERIAL_NOSWAP));
}

#endif

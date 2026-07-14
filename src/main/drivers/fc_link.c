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
// redundancy bus. Two frame types share the wire:
//  - a fast heartbeat carrying role/arm/failsafe/RX status plus the sender's
//    current servo channel values (PWM and bus servos alike), so a SLAVE can
//    relay the MASTER's actual output everywhere the MASTER outputs it. The
//    redundancy bus device only checks frame validity, not channel content,
//    so two independently-computed streams could otherwise disagree every
//    frame; relaying keeps them identical while the link is up.
//  - a slower "tuning" frame carrying the MASTER's currently active PID and
//    rate profile, so a SLAVE's fallback tuning matches whatever MASTER is
//    actually flying with (including in-flight adjustment-function changes
//    that were never saved to EEPROM), not just its own last-saved values.
// Both fall back to the SLAVE's own local computation/tuning the moment the
// peer heartbeat is lost. Role comes solely from which port function was
// assigned (FUNCTION_FC_LINK_MASTER/_SLAVE) and is fixed for the life of the
// boot -- there is no negotiation to get wrong.

#include <math.h>
#include <string.h>

#include "platform.h"

#ifdef USE_FC_LINK

#include "build/build_config.h"
#include "common/maths.h"
#include "common/utils.h"
#include "common/time.h"

#include "drivers/time.h"
#include "drivers/fc_link.h"

#include "io/serial.h"

#include "pg/fc_link.h"

#include "fc/rc_rates.h"
#include "fc/runtime_config.h"
#include "flight/failsafe.h"
#include "flight/pid.h"
#include "flight/setpoint.h"
#include "rx/rx.h"

#define FC_LINK_BAUDRATE 460800
#define FC_LINK_SYNC_BYTE 0xF7
#define FC_LINK_TUNING_SYNC_BYTE 0xF8
#define FC_LINK_TUNING_RATE_HZ 4

#define MS2US(ms) ((ms) * 1000)

typedef struct __attribute__((packed)) {
    uint8_t sync;
    uint8_t role;        // fcLinkRole_e of the sender
    uint8_t flags;        // bit0 armed, bit1 failsafeActive, bit2 rxReceivingSignal
    uint16_t seq;
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

static uint16_t localChannels[FC_LINK_MAX_CHANNELS];

// Producer (ISR) / consumer (fcLinkUpdate, scheduler task context) handoff.
// The apply step touches live PID coefficients that the control loop task
// also reads, so it must not run from interrupt context -- deferring it here
// means it's serialized against other tasks by the (non-preemptive) scheduler
// instead of racing an arbitrary control-loop read.
static fcLinkTuningFrame_t pendingTuningFrame;
static volatile bool tuningFramePending = false;

static uint8_t rxBuf[sizeof(fcLinkTuningFrame_t)];
static uint8_t rxIdx = 0;
static uint8_t rxExpectedLen = 0;

static uint8_t fcLinkChecksumBytes(const uint8_t *bytes, size_t len)
{
    uint8_t checksum = 0;
    for (size_t i = 0; i < len; i++) {
        checksum ^= bytes[i];
    }
    return checksum;
}

static void fcLinkHandleFrame(const fcLinkFrame_t *frame)
{
    lastPeerFrameUs = micros();
    everReceivedPeerFrame = true;

    peerState.armed = frame->flags & FC_LINK_FLAG_ARMED;
    peerState.failsafeActive = frame->flags & FC_LINK_FLAG_FAILSAFE_ACTIVE;
    peerState.rxReceivingSignal = frame->flags & FC_LINK_FLAG_RX_RECEIVING;
    peerState.seq = frame->seq;

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
        } else {
            return;
        }
    }

    rxBuf[rxIdx++] = (uint8_t)c;

    if (rxIdx >= rxExpectedLen) {
        if (rxBuf[0] == FC_LINK_SYNC_BYTE) {
            fcLinkFrame_t frame;
            memcpy(&frame, rxBuf, sizeof(frame));
            if (fcLinkChecksumBytes(rxBuf, offsetof(fcLinkFrame_t, checksum)) == frame.checksum) {
                fcLinkHandleFrame(&frame);
            }
        } else {
            if (fcLinkChecksumBytes(rxBuf, offsetof(fcLinkTuningFrame_t, checksum)) == rxBuf[offsetof(fcLinkTuningFrame_t, checksum)]) {
                memcpy(&pendingTuningFrame, rxBuf, sizeof(pendingTuningFrame));
                tuningFramePending = true;
            }
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

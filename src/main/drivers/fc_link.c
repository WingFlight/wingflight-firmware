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

#include "pg/fbus_master.h"
#include "pg/fc_link.h"
#include "pg/motor.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/sbus_output.h"
#include "pg/telemetry.h"

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

// A dropped/corrupted byte mid-frame must not be allowed to silently
// misalign every frame that follows. Bytes within one frame stream back to
// back off the TX FIFO (~2.2us/bit at FC_LINK_BAUDRATE); a gap this much
// wider than one byte time can only mean the rest of that frame is gone.
#define FC_LINK_RX_BYTE_GAP_MAX_US 100

// How often a SLAVE re-checks the peer's (saved) config fingerprint for
// drift once already running -- not just at boot. A REQUEST frame is cheap,
// so this can be fairly frequent without being wasteful; a full re-stream
// only actually happens when a real, saved mismatch is found.
#define FC_LINK_AUTO_SYNC_CHECK_INTERVAL_MS 5000

// Comfortably above the largest single registered parameter group (the
// biggest is the per-profile PID array, a few hundred bytes worst case).
#define FC_LINK_CONFIG_CHUNK_MAX 768

// Sentinel pgn values framing a config-sync session; real PG numbers are
// small (see pg/pg_ids.h), so these sit safely outside that range.
#define FC_LINK_CONFIG_PGN_REQUEST    0xFFFD
#define FC_LINK_CONFIG_PGN_SYNC_START 0xFFFE
#define FC_LINK_CONFIG_PGN_SYNC_END   0xFFFF

#define MS2US(ms) ((ms) * 1000)

// See fcLinkConfigPgnPartialRanges().
#define FC_LINK_PARTIAL_RANGE_MAX 2

typedef struct {
    uint16_t offset;
    uint16_t length;
} fcLinkByteRange_t;

// User-facing base-config-sync categories. Coarse on purpose: every
// non-excluded PG (see fcLinkConfigPgnExcluded()/fcLinkConfigPgnCategory())
// falls into exactly one of these.
typedef enum {
    FC_LINK_SYNC_CATEGORY_MIXER_SERVOS = 0,
    FC_LINK_SYNC_CATEGORY_PID_RATES,
    FC_LINK_SYNC_CATEGORY_RX,
    FC_LINK_SYNC_CATEGORY_MOTOR,
    FC_LINK_SYNC_CATEGORY_TELEMETRY,
    FC_LINK_SYNC_CATEGORY_MODES_ADJUSTMENTS,
    FC_LINK_SYNC_CATEGORY_GPS,
    FC_LINK_SYNC_CATEGORY_OSD,
    FC_LINK_SYNC_CATEGORY_VTX,
    FC_LINK_SYNC_CATEGORY_OTHER,
    FC_LINK_SYNC_CATEGORY_COUNT,
} fcLinkSyncCategory_e;

typedef struct __attribute__((packed)) {
    uint8_t sync;
    uint8_t role;        // fcLinkRole_e of the sender
    uint8_t flags;        // bit0 armed, bit1 failsafeActive, bit2 rxReceivingSignal
    uint16_t seq;
    uint8_t eepromConfVersion;   // EEPROM_CONF_VERSION -- must match to accept a config sync
    uint8_t fcVersionMajor;
    uint8_t fcVersionMinor;
    uint16_t configFingerprint[FC_LINK_SYNC_CATEGORY_COUNT]; // per-category folded CRC; a mismatch in a category the SLAVE has enabled triggers an auto-sync
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
static uint16_t peerConfigFingerprint[FC_LINK_SYNC_CATEGORY_COUNT];

// Snapshot of our own *saved* config, refreshed only by fcLinkNotifyConfigSaved()
// (after a real EEPROM write) -- never recomputed from live memory on a timer,
// so a value mid-edit in some tool that hasn't saved yet can never look like
// drift. Broadcast as-is in every heartbeat; fcLinkInit() seeds it from
// whatever's already loaded (i.e. the persisted config) at boot.
static uint16_t localConfigFingerprint[FC_LINK_SYNC_CATEGORY_COUNT];

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
static uint16_t rxIdx = 0;
static uint16_t rxExpectedLen = 0;
static timeUs_t lastRxByteUs = 0;

// Bench-debug counters (cheap, always-on) -- exposed via `fc_link debug` so a
// dead link can be diagnosed from the CLI without a logic analyzer: whether
// bytes are arriving at all, whether they're framing as any known sync byte,
// whether frames are stalling mid-receive, and which frame types pass/fail
// their checksum.
static fcLinkDebugStats_t debugStats;

// SLAVE receive-side config-sync progress.
static timeUs_t nextAutoSyncCheckTimeUs = 0;
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
        // Per-board electrical settings (pinswap/inverted especially exist
        // specifically to handle two boards being wired asymmetrically) --
        // copying these from the peer would silently undo whatever this
        // board's own wiring required, exactly like the serial port function
        // assignment above.
        case PG_DRIVER_FC_LINK_CONFIG:
            return true;

        // Pure pin-map PGs -- every field is an ioTag_t (or equivalent),
        // nothing lost by excluding. PG_SERVO_CONFIG in particular is only
        // ioTags; the actual servo tuning lives in the separate, syncable
        // PG_SERVO_PARAMS.
        case PG_SERVO_CONFIG:
        case PG_PWM_CONFIG:
        case PG_PPM_CONFIG:
        case PG_RX_SPI_CONFIG:
        case PG_RX_CC2500_SPI_CONFIG:
        case PG_RX_EXPRESSLRS_SPI_CONFIG:
        case PG_BEEPER_DEV_CONFIG:
        case PG_SONAR_CONFIG:
        case PG_VTX_IO_CONFIG:
        case PG_GYRO_DEVICE_CONFIG:
        case PG_ADC_CONFIG:
        case PG_I2C_CONFIG:
        case PG_SPI_PIN_CONFIG:
        case PG_QUADSPI_CONFIG:
        case PG_MCO_CONFIG:
        case PG_SERIAL_UART_CONFIG:
        case PG_SERIAL_PIN_CONFIG:
        case PG_ESCSERIAL_CONFIG:
        case PG_SDIO_CONFIG:
        case PG_SDIO_PIN_CONFIG:
        case PG_PULLUP_CONFIG:
        case PG_PULLDOWN_CONFIG:
        case PG_PINIO_CONFIG:
        case PG_PINIOBOX_CONFIG:
        case PG_TIMER_IO_CONFIG:
        case PG_TIMER_UP_CONFIG:
        case PG_FLASH_CONFIG:
        case PG_SDCARD_CONFIG:
        case PG_CAMERA_CONTROL_CONFIG:
        case PG_MAX7456_CONFIG:
        case PG_USB_CONFIG:
        case PG_DASHBOARD_CONFIG:
        case PG_STATUS_LED_CONFIG:
            return true;

        // Per-unit sensor calibration -- manufacturing-tolerance trims, not
        // shared tuning. Wrong even between two identical-model boards.
        case PG_ACCELEROMETER_CONFIG:
        case PG_COMPASS_CONFIG:
        case PG_BAROMETER_CONFIG:
        case PG_CURRENT_SENSOR_ADC_CONFIG:
        case PG_VOLTAGE_SENSOR_ADC_CONFIG:
        case PG_FREQ_SENSOR_CONFIG:
            return true;

        // Board identity.
        case PG_BOARD_ALIGNMENT:
        case PG_BOARD_CONFIG:
            return true;

        // Per-board wiring flags, same rationale as PG_DRIVER_FC_LINK_CONFIG
        // above (pinSwap/halfDuplex). PG_DRIVER_SPORT_MASTER_CONFIG is
        // *entirely* pinSwap+inverted (2 fields, no tuning content at all),
        // so it belongs here rather than needing partial-range treatment.
        case PG_ESC_SENSOR_CONFIG:
        case PG_DRIVER_SPORT_MASTER_CONFIG:
            return true;

        // Mixed pin-map + tuning PG, excluded wholesale: fc_link syncs whole
        // PGs by default, and a wrong pin assignment here could break the
        // LED strip's data pin. Unlike PG_MOTOR_CONFIG below, its tuning
        // content (brightness/beacon/blink timing) isn't worth the extra
        // partial-sync machinery.
        case PG_LED_STRIP_CONFIG:
            return true;

        // PG_MOTOR_CONFIG, PG_DRIVER_SBUS_OUT_CONFIG, PG_DRIVER_FBUS_MASTER_CONFIG,
        // and PG_TELEMETRY_CONFIG are intentionally NOT excluded here -- see
        // fcLinkConfigPgnPartialRanges(). Each mixes hardware pin/protocol/
        // wiring assignment with genuine tuning; only the tuning portion is
        // ever synced, never the hardware fields (ioTags, pinSwap, inverted,
        // halfDuplex, etc).

        // Accumulated runtime stats (flight time/distance), not config.
        case PG_STATS_CONFIG:
            return true;

        default:
            return false;
    }
}

// Which user-facing sync category a (non-excluded) PG belongs to. Only
// called for PGs that already pass fcLinkConfigPgnExcluded() == false.
static fcLinkSyncCategory_e fcLinkConfigPgnCategory(uint16_t pgn)
{
    switch (pgn) {
        case PG_GENERIC_MIXER_CONFIG:
        case PG_GENERIC_MIXER_RULES:
        case PG_GENERIC_MIXER_CURVES:
        case PG_GENERIC_MIXER_INPUTS:
        case PG_SERVO_PARAMS:
        case PG_GOVERNOR_CONFIG:          // propulsion/motor-speed control, not attitude
            return FC_LINK_SYNC_CATEGORY_MIXER_SERVOS;

        case PG_PID_PROFILE:
        case PG_CONTROL_RATE_PROFILES:
        case PG_PID_CONFIG:
        case PG_GAIN_CURVES:      // stick-gain scaling feeding setpoint
        case PG_RPM_FILTER_CONFIG:
        case PG_DYN_NOTCH_CONFIG:
        case PG_GYRO_CONFIG:      // filtering feeds the PID loop directly
        case PG_IMU_CONFIG:       // DCM kp/ki attitude-estimation gains, flight-tuning-adjacent
            return FC_LINK_SYNC_CATEGORY_PID_RATES;

        case PG_RX_CONFIG:
        case PG_RC_CONTROLS_CONFIG:
        case PG_RX_FAILSAFE_CHANNEL_CONFIG:
        case PG_FAILSAFE_CONFIG:
        case PG_FLYSKY_CONFIG:
        case PG_RX_SPEKTRUM_SPI_CONFIG:
            return FC_LINK_SYNC_CATEGORY_RX;

        // Only the partial (tuning) range of each ever participates -- see
        // fcLinkConfigPgnPartialRanges().
        case PG_MOTOR_CONFIG:
            return FC_LINK_SYNC_CATEGORY_MOTOR;

        case PG_TELEMETRY_CONFIG:
        case PG_DRIVER_SBUS_OUT_CONFIG:
        case PG_DRIVER_FBUS_MASTER_CONFIG:
            return FC_LINK_SYNC_CATEGORY_TELEMETRY;

        // General-purpose gating primitives (a logic condition can reference
        // RC channels, modes, sensors, or other conditions -- not mixer-
        // specific despite feeding mixerRule_t.condition too) grouped with
        // the mode/adjustment-range config that uses them the same way.
        case PG_MODE_ACTIVATION_PROFILE:
        case PG_MODE_ACTIVATION_CONFIG:
        case PG_ADJUSTMENT_RANGE_CONFIG:
        case PG_GENERIC_LOGIC_CONDITIONS:
            return FC_LINK_SYNC_CATEGORY_MODES_ADJUSTMENTS;

        case PG_GPS_CONFIG:
        case PG_GPS_RESCUE:
        case PG_POSITION:
            return FC_LINK_SYNC_CATEGORY_GPS;

        case PG_OSD_CONFIG:
        case PG_OSD_ELEMENT_CONFIG:
        case PG_DISPLAY_PORT_MSP_CONFIG:
        case PG_DISPLAY_PORT_MAX7456_CONFIG:
        case PG_VCD_CONFIG:
            return FC_LINK_SYNC_CATEGORY_OSD;

        case PG_VTX_CONFIG:
        case PG_VTX_SETTINGS_CONFIG:
        case PG_VTX_TABLE_CONFIG:
            return FC_LINK_SYNC_CATEGORY_VTX;

        default:
            return FC_LINK_SYNC_CATEGORY_OTHER;
    }
}

// Some PGs mix hardware pin/protocol/wiring assignment with genuine tuning
// in one struct that fc_link can't cleanly split into separate PGs. For
// those, only the listed byte ranges participate in fingerprinting,
// streaming, or applying -- never the rest of the PG -- so the hardware
// portion is never touched regardless of category settings. Ranges are
// listed in ascending offset order and never overlap. Returns 0 (ranges[]
// untouched) for every other PG, meaning "use the whole PG" as before.
static uint8_t fcLinkConfigPgnPartialRanges(uint16_t pgn, fcLinkByteRange_t ranges[FC_LINK_PARTIAL_RANGE_MAX])
{
    switch (pgn) {
        case PG_MOTOR_CONFIG:
            // Everything from minthrottle onward; motorConfig_t.dev (pins,
            // PWM protocol/rate, transport, dshot options) is never touched.
            ranges[0].offset = offsetof(motorConfig_t, minthrottle);
            ranges[0].length = sizeof(motorConfig_t) - ranges[0].offset;
            return 1;

        case PG_DRIVER_SBUS_OUT_CONFIG:
            // frameRate only; pinSwap/inverted (the rest of the struct) are
            // per-board wiring, never synced.
            ranges[0].offset = 0;
            ranges[0].length = offsetof(sbusOutConfig_t, pinSwap);
            return 1;

        case PG_DRIVER_FBUS_MASTER_CONFIG:
            // frameRate, then telemetryRate onward; pinSwap/inverted sit in
            // between and are skipped.
            ranges[0].offset = 0;
            ranges[0].length = offsetof(fbusMasterConfig_t, pinSwap);
            ranges[1].offset = offsetof(fbusMasterConfig_t, telemetryRate);
            ranges[1].length = sizeof(fbusMasterConfig_t) - ranges[1].offset;
            return 2;

        case PG_TELEMETRY_CONFIG:
            // gpsNoFixLatitude/Longitude, then frsky_coordinate_format
            // onward; telemetry_inverted/halfDuplex/pinSwap sit in between
            // and are skipped.
            ranges[0].offset = 0;
            ranges[0].length = offsetof(telemetryConfig_t, telemetry_inverted);
            ranges[1].offset = offsetof(telemetryConfig_t, frsky_coordinate_format);
            ranges[1].length = sizeof(telemetryConfig_t) - ranges[1].offset;
            return 2;

        default:
            return 0;
    }
}

// This board's own willingness to accept a given category (SLAVE-side only
// concept, but harmless to evaluate on a MASTER too since it's never
// consulted there).
static bool fcLinkSyncCategoryEnabled(fcLinkSyncCategory_e category)
{
    switch (category) {
        case FC_LINK_SYNC_CATEGORY_MIXER_SERVOS:
            return fcLinkConfig()->syncMixerServos;
        case FC_LINK_SYNC_CATEGORY_PID_RATES:
            return fcLinkConfig()->syncPidRates;
        case FC_LINK_SYNC_CATEGORY_RX:
            return fcLinkConfig()->syncRx;
        case FC_LINK_SYNC_CATEGORY_MOTOR:
            return fcLinkConfig()->syncMotor;
        case FC_LINK_SYNC_CATEGORY_TELEMETRY:
            return fcLinkConfig()->syncTelemetry;
        case FC_LINK_SYNC_CATEGORY_MODES_ADJUSTMENTS:
            return fcLinkConfig()->syncModesAdjustments;
        case FC_LINK_SYNC_CATEGORY_GPS:
            return fcLinkConfig()->syncGps;
        case FC_LINK_SYNC_CATEGORY_OSD:
            return fcLinkConfig()->syncOsd;
        case FC_LINK_SYNC_CATEGORY_VTX:
            return fcLinkConfig()->syncVtx;
        case FC_LINK_SYNC_CATEGORY_OTHER:
        default:
            return fcLinkConfig()->syncOther;
    }
}

// Per-category folded CRC over every syncable PG's live bytes, computed in a
// single PG_FOREACH pass. Two boards running identical firmware produce the
// same per-category value iff that category's PGs are byte-for-byte
// identical, since PG_FOREACH order is fixed by the firmware image itself.
// Deliberately NOT called on a timer/every heartbeat -- only at boot and
// from fcLinkNotifyConfigSaved(), so it only ever reflects actually-saved
// config (see localConfigFingerprint's doc comment). Every board computes
// all FC_LINK_SYNC_CATEGORY_COUNT values unconditionally -- categorization
// is classification, not filtering; only the SLAVE's local sync-category
// toggles decide which mismatches actually matter (see fcLinkUpdate()).
static void fcLinkComputeConfigFingerprint(uint16_t out[FC_LINK_SYNC_CATEGORY_COUNT])
{
    for (int i = 0; i < FC_LINK_SYNC_CATEGORY_COUNT; i++) {
        out[i] = 0xFFFF;
    }
    PG_FOREACH(reg) {
        const uint16_t pgn = pgN(reg);
        if (fcLinkConfigPgnExcluded(pgn)) {
            continue;
        }
        const fcLinkSyncCategory_e category = fcLinkConfigPgnCategory(pgn);
        fcLinkByteRange_t ranges[FC_LINK_PARTIAL_RANGE_MAX];
        const uint8_t rangeCount = fcLinkConfigPgnPartialRanges(pgn, ranges);
        if (rangeCount == 0) {
            out[category] = crc16_ccitt_update(out[category], reg->address, pgSize(reg));
        } else {
            for (uint8_t i = 0; i < rangeCount; i++) {
                out[category] = crc16_ccitt_update(out[category], reg->address + ranges[i].offset, ranges[i].length);
            }
        }
    }
}

void fcLinkNotifyConfigSaved(void)
{
    if (!fcLinkIsEnabled()) {
        return;
    }
    fcLinkComputeConfigFingerprint(localConfigFingerprint);
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
    memcpy(peerConfigFingerprint, frame->configFingerprint, sizeof(peerConfigFingerprint));

    memcpy(peerChannels, frame->channels, sizeof(peerChannels));
}

static void fcLinkDataReceive(uint16_t c, void *data)
{
    UNUSED(data);

    debugStats.rxByteTotal++;

    const timeUs_t nowUs = microsISR();
    if (rxIdx > 0 && cmpTimeUs(nowUs, lastRxByteUs) > (timeDelta_t)FC_LINK_RX_BYTE_GAP_MAX_US) {
        // Mid-frame stall (dropped/corrupted byte) -- abandon the partial
        // frame now rather than keep accumulating into the wrong offset
        // until FC_LINK_CONFIG_CHUNK_MAX-ish bytes of noise happen to fail
        // a checksum on their own.
        rxIdx = 0;
        debugStats.rxFrameAbandoned++;
    }
    lastRxByteUs = nowUs;

    if (rxIdx == 0) {
        if ((uint8_t)c == FC_LINK_SYNC_BYTE) {
            rxExpectedLen = sizeof(fcLinkFrame_t);
        } else if ((uint8_t)c == FC_LINK_TUNING_SYNC_BYTE) {
            rxExpectedLen = sizeof(fcLinkTuningFrame_t);
        } else if ((uint8_t)c == FC_LINK_CONFIG_SYNC_BYTE) {
            rxExpectedLen = sizeof(fcLinkConfigFrame_t);
        } else {
            debugStats.rxUnsyncedByte++;
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
                    debugStats.heartbeatOk++;
                    fcLinkHandleFrame(&frame);
                } else {
                    debugStats.heartbeatChecksumFail++;
                }
                break;
            }
            case FC_LINK_TUNING_SYNC_BYTE: {
                if (fcLinkChecksumBytes(rxBuf, offsetof(fcLinkTuningFrame_t, checksum)) == rxBuf[offsetof(fcLinkTuningFrame_t, checksum)]) {
                    debugStats.tuningOk++;
                    memcpy(&pendingTuningFrame, rxBuf, sizeof(pendingTuningFrame));
                    tuningFramePending = true;
                } else {
                    debugStats.tuningChecksumFail++;
                }
                break;
            }
            case FC_LINK_CONFIG_SYNC_BYTE: {
                if (fcLinkChecksumBytes(rxBuf, offsetof(fcLinkConfigFrame_t, checksum)) == rxBuf[offsetof(fcLinkConfigFrame_t, checksum)]) {
                    debugStats.configOk++;
                    fcLinkConfigFrame_t frame;
                    memcpy(&frame, rxBuf, sizeof(frame));
                    if (frame.pgn == FC_LINK_CONFIG_PGN_REQUEST) {
                        debugStats.configRequestSeen++;
                        configSyncRequestPending = true;
                    } else {
                        memcpy(&pendingConfigFrame, &frame, sizeof(pendingConfigFrame));
                        configFramePending = true;
                    }
                } else {
                    debugStats.configChecksumFail++;
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
        debugStats.txHeartbeatSkipped++;
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
    };
    memcpy(frame.configFingerprint, localConfigFingerprint, sizeof(frame.configFingerprint));
    memcpy(frame.channels, localChannels, sizeof(frame.channels));
    frame.checksum = fcLinkChecksumBytes((const uint8_t *)&frame, offsetof(fcLinkFrame_t, checksum));

    serialWriteBuf(fcLinkPort, (const uint8_t *)&frame, sizeof(frame));
    debugStats.txHeartbeatSent++;

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
    if (reg && !fcLinkConfigPgnExcluded(frame->pgn)) {
        fcLinkByteRange_t ranges[FC_LINK_PARTIAL_RANGE_MAX];
        const uint8_t rangeCount = fcLinkConfigPgnPartialRanges(frame->pgn, ranges);
        uint16_t expectedSize = pgSize(reg);
        if (rangeCount > 0) {
            expectedSize = 0;
            for (uint8_t i = 0; i < rangeCount; i++) {
                expectedSize += ranges[i].length;
            }
        }

        if (expectedSize == frame->size && frame->size <= sizeof(frame->payload)) {
            // Counts toward completion regardless of this board's category
            // choice -- MASTER's SYNC_START count includes every non-excluded
            // PG unconditionally, so the completion check must too, or a
            // session with any category disabled would always come up short
            // and get discarded as if it had failed. Only the actual write
            // into the live registry is gated by category. For a partial PG,
            // each range's own offset keeps the write inside the tuning
            // portion only -- the hardware portion (pins/protocol/wiring) is
            // never touched.
            configSyncReceivedCount++;
            if (fcLinkSyncCategoryEnabled(fcLinkConfigPgnCategory(frame->pgn))) {
                if (rangeCount == 0) {
                    memcpy(reg->address, frame->payload, frame->size);
                } else {
                    uint16_t read = 0;
                    for (uint8_t i = 0; i < rangeCount; i++) {
                        memcpy(reg->address + ranges[i].offset, frame->payload + read, ranges[i].length);
                        read += ranges[i].length;
                    }
                }
            }
        }
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
                fcLinkByteRange_t ranges[FC_LINK_PARTIAL_RANGE_MAX];
                const uint8_t rangeCount = fcLinkConfigPgnPartialRanges(pgN(found), ranges);
                frame.pgn = pgN(found);
                if (rangeCount == 0) {
                    frame.size = pgSize(found);
                    memcpy(frame.payload, found->address, MIN(frame.size, sizeof(frame.payload)));
                } else {
                    // Concatenate each range's bytes back-to-back; the SLAVE
                    // re-derives the same ranges (same order) from pgn to
                    // unpack, so the wire format itself doesn't need to know
                    // how many ranges there were.
                    uint16_t written = 0;
                    for (uint8_t i = 0; i < rangeCount; i++) {
                        const uint16_t remaining = sizeof(frame.payload) - written;
                        const uint16_t chunk = MIN(ranges[i].length, remaining);
                        memcpy(frame.payload + written, found->address + ranges[i].offset, chunk);
                        written += chunk;
                    }
                    frame.size = written;
                }
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

fcLinkPeerVersionInfo_t fcLinkGetPeerVersionInfo(void)
{
    return (fcLinkPeerVersionInfo_t){
        .eepromConfVersion = peerEepromConfVersion,
        .fcVersionMajor = peerFcVersionMajor,
        .fcVersionMinor = peerFcVersionMinor,
    };
}

const fcLinkDebugStats_t *fcLinkGetDebugStats(void)
{
    return &debugStats;
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

    // Auto-trigger: periodically (not just once at boot) check whether the
    // peer's *saved* config differs from ours in a category we currently
    // accept -- a category this board has deliberately excluded is allowed
    // to differ from the peer forever without ever triggering a sync.
    // Skipped entirely while a sync is already in flight (nothing to gain by
    // requesting another) or while either board might be flying, since a
    // mismatch found then could never actually commit anyway (see the
    // SYNC_END handling in fcLinkApplyConfigFrame()) -- no point spending
    // bandwidth re-streaming the whole config every interval for no effect.
    if (localRole == FC_LINK_ROLE_SLAVE && !fcLinkPeerLost() && !configSyncInProgress
        && !ARMING_FLAG(ARMED) && !peerState.armed
        && cmpTimeUs(currentTimeUs, nextAutoSyncCheckTimeUs) >= 0) {
        nextAutoSyncCheckTimeUs = currentTimeUs + MS2US(FC_LINK_AUTO_SYNC_CHECK_INTERVAL_MS);
        if (fcLinkPeerConfigCompatible()) {
            for (int i = 0; i < FC_LINK_SYNC_CATEGORY_COUNT; i++) {
                if (fcLinkSyncCategoryEnabled((fcLinkSyncCategory_e)i)
                    && localConfigFingerprint[i] != peerConfigFingerprint[i]) {
                    fcLinkSendConfigSyncRequest();
                    break;
                }
            }
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

    // Baseline fingerprint: whatever's already loaded into the live registry
    // at this point in boot *is* the persisted config (nothing has run yet
    // that could have live-edited it), so this is a valid "saved" snapshot.
    fcLinkComputeConfigFingerprint(localConfigFingerprint);
}

#endif

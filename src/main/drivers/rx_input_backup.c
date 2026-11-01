/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_RX_INPUT_BACKUP

#include "drivers/rx_input_backup.h"

#include "common/maths.h"
#include "common/utils.h"

#include "drivers/time.h"

#include "fc/runtime_config.h"

#include "io/serial.h"

#include "pg/rx.h"
#include "pg/rx_input_backup.h"

#ifdef USE_RX_INPUT_BACKUP_SBUS
#include "drivers/rx_input_backup_sbus.h"
#endif

#ifdef USE_RX_INPUT_BACKUP_FBUS
#include "drivers/rx_input_backup_fbus.h"
#endif

#ifdef USE_RX_INPUT_BACKUP_FPORT
#include "drivers/rx_input_backup_fport.h"
#endif

#ifdef USE_RX_INPUT_BACKUP_EXBUS
#include "drivers/rx_input_backup_exbus.h"
#endif

#ifdef USE_RX_INPUT_BACKUP_CRSF
#include "drivers/rx_input_backup_crsf.h"
#endif

// How long without a decoded frame before a backup link is considered down.
// ~3 missed frames at a typical ~6-14ms/frame rate - same margin the original
// SBUS-only driver used, kept here since it's a property of "how stale is too
// stale to trust", not of any one protocol's framing.
#define RX_INPUT_BACKUP_STALE_MS 50

// Speed search for providers with a second speed (ops.altBaudRate), same
// rules as rx/fbus.c: switch until valid frames arrive, then keep the speed.
#define RX_INPUT_BACKUP_BAUD_HUNT_WINDOW_MS 150
#define RX_INPUT_BACKUP_BAUD_LOCK_FRAMES 3
#define RX_INPUT_BACKUP_BAUD_RELOCK_MS 1000

// Everything one backup port owns. One per slot, in priority order.
typedef struct rxInputBackupPort_s {
    serialPort_t *port;                 // NULL while not opened (= disabled)
    rxInputBackupOps_t ops;

    float channel[RX_INPUT_BACKUP_MAX_CHANNEL];
    uint8_t frameChannelCount;          // channels in the latest valid frame, 0 before the first
    timeMs_t lastValidFrameMs;
    // False until the first genuinely valid frame has been decoded. Without this,
    // IsActive would read as "active" for up to RX_INPUT_BACKUP_STALE_MS right
    // after boot/config-change purely because lastValidFrameMs's zero-init
    // happens to be within that window of millis()'s own startup value -
    // reporting the backup available (and, if the main link were already down at
    // that moment, feeding zeroed channels) before any real frame has been seen.
    bool hasValidFrame;

    uint32_t baudRate;
    bool baudLocked;
    uint8_t baudGoodFrames;
    timeMs_t baudLastEventMs;           // last speed switch or last valid frame
} rxInputBackupPort_t;

static rxInputBackupPort_t rxInputBackupPorts[RX_INPUT_BACKUP_COUNT];

static const serialPortFunction_e rxInputBackupFunctions[RX_INPUT_BACKUP_COUNT] = {
    FUNCTION_RX_INPUT_BACKUP,
    FUNCTION_RX_INPUT_BACKUP2,
};

serialPortFunction_e rxInputBackupPortFunction(uint8_t index)
{
    return index < RX_INPUT_BACKUP_COUNT ? rxInputBackupFunctions[index] : FUNCTION_NONE;
}

static void rxInputBackupBaudService(rxInputBackupPort_t *bp, bool validFrame)
{
    const timeMs_t now = millis();

    if (!bp->ops.altBaudRate || !bp->port->vTable->serialSetBaudRate) {
        return;
    }

    if (validFrame) {
        bp->baudLastEventMs = now;
        if (!bp->baudLocked && ++bp->baudGoodFrames >= RX_INPUT_BACKUP_BAUD_LOCK_FRAMES) {
            bp->baudLocked = true;
        }
        return;
    }

    const int32_t idleMs = cmp32(now, bp->baudLastEventMs);
    if (bp->baudLocked) {
        // A lost link while armed is the radio, not a changed receiver setting
        if (ARMING_FLAG(ARMED) || idleMs < RX_INPUT_BACKUP_BAUD_RELOCK_MS) {
            return;
        }
        bp->baudLocked = false;
    } else if (idleMs < RX_INPUT_BACKUP_BAUD_HUNT_WINDOW_MS) {
        return;
    }

    bp->baudRate = (bp->baudRate == bp->ops.baudRate) ? bp->ops.altBaudRate : bp->ops.baudRate;
    serialSetBaudRate(bp->port, bp->baudRate);
    bp->baudGoodFrames = 0;
    bp->baudLastEventMs = now;
}

static bool portIsActive(const rxInputBackupPort_t *bp)
{
    if (!bp->port || !bp->hasValidFrame) {
        return false;
    }

    return (timeMs_t)(millis() - bp->lastValidFrameMs) < RX_INPUT_BACKUP_STALE_MS;
}

static uint8_t portChannelCount(const rxInputBackupPort_t *bp)
{
    return bp->frameChannelCount ? bp->frameChannelCount : bp->ops.channelCount;
}

static float portChannel(const rxInputBackupPort_t *bp, uint8_t channel)
{
    // Channels the latest frame doesn't carry (e.g. CH19-24 from a 16-channel
    // F.Bus frame) read as stick center, so a takeover centers them.
    if (channel >= portChannelCount(bp)) {
        return rcControlsConfig()->rc_center;
    }
    return bp->channel[channel];
}

// Decodes any newly-completed frame (via the selected provider's update()) and
// refreshes freshness state. Must be called every cycle regardless of whether the
// main RX link is up or a backup is currently "needed" - it used to be called
// only as a side effect of rxInputBackupIsActive(), which rx.c only evaluates once
// the main link is already down (short-circuiting `!rxSignalReceived && ...`).
// That starved this of any real-time decoding whenever the main link was healthy,
// leaving diagnostics/MSP polling as the only thing driving it (once every poll
// interval instead of every cycle) and meaning the very first backup frame used at
// the instant of a real failover could already be stale.
void rxInputBackupPoll(void)
{
    for (int i = 0; i < RX_INPUT_BACKUP_COUNT; i++) {
        rxInputBackupPort_t *bp = &rxInputBackupPorts[i];
        if (!bp->port) {
            continue;
        }

        const uint8_t frameChannels = bp->ops.update(bp->ops.state, bp->channel, bp->ops.channelCount);
        if (frameChannels) {
            bp->frameChannelCount = frameChannels;
            bp->hasValidFrame = true;
            bp->lastValidFrameMs = millis();
        }

        rxInputBackupBaudService(bp, frameChannels != 0);

        if (bp->hasValidFrame && (timeMs_t)(millis() - bp->lastValidFrameMs) >= RX_INPUT_BACKUP_STALE_MS) {
            // No explicit parser reset here - each provider's own frame-timing
            // logic already self-heals from a stale gap the moment bytes resume
            // (see e.g. rx_input_backup_sbus.c's frameTime check).
            bp->hasValidFrame = false;
        }
    }
}

bool rxInputBackupPortIsEnabled(uint8_t index)
{
    return index < RX_INPUT_BACKUP_COUNT && rxInputBackupPorts[index].port != NULL;
}

bool rxInputBackupPortIsActive(uint8_t index)
{
    return index < RX_INPUT_BACKUP_COUNT && portIsActive(&rxInputBackupPorts[index]);
}

uint8_t rxInputBackupPortGetChannelCount(uint8_t index)
{
    return index < RX_INPUT_BACKUP_COUNT ? portChannelCount(&rxInputBackupPorts[index]) : 0;
}

rxInputBackupProvider_e rxInputBackupPortGetProvider(uint8_t index)
{
    if (!rxInputBackupPortIsEnabled(index)) {
        return RX_INPUT_BACKUP_NONE;
    }
    return rxInputBackupConfig()->port[index].provider;
}

float rxInputBackupPortGetChannel(uint8_t index, uint8_t channel)
{
    if (index >= RX_INPUT_BACKUP_COUNT) {
        return rcControlsConfig()->rc_center;
    }
    return portChannel(&rxInputBackupPorts[index], channel);
}

bool rxInputBackupIsEnabled(void)
{
    for (int i = 0; i < RX_INPUT_BACKUP_COUNT; i++) {
        if (rxInputBackupPorts[i].port) {
            return true;
        }
    }
    return false;
}

int8_t rxInputBackupGetActiveIndex(void)
{
    for (int i = 0; i < RX_INPUT_BACKUP_COUNT; i++) {
        if (portIsActive(&rxInputBackupPorts[i])) {
            return i;
        }
    }
    return -1;
}

bool rxInputBackupIsActive(void)
{
    return rxInputBackupGetActiveIndex() >= 0;
}

bool rxInputBackupAllLinked(void)
{
    bool anyEnabled = false;
    for (int i = 0; i < RX_INPUT_BACKUP_COUNT; i++) {
        const rxInputBackupPort_t *bp = &rxInputBackupPorts[i];
        if (!bp->port) {
            continue;
        }
        anyEnabled = true;
        if (!portIsActive(bp)) {
            return false;
        }
    }
    return anyEnabled;
}

// The port a takeover reads: the active one, else the first enabled one (so
// diagnostics still show something sensible while no backup link is up).
static const rxInputBackupPort_t *sourcePort(void)
{
    const int8_t active = rxInputBackupGetActiveIndex();
    if (active >= 0) {
        return &rxInputBackupPorts[active];
    }
    for (int i = 0; i < RX_INPUT_BACKUP_COUNT; i++) {
        if (rxInputBackupPorts[i].port) {
            return &rxInputBackupPorts[i];
        }
    }
    return &rxInputBackupPorts[0];
}

uint8_t rxInputBackupGetChannelCount(void)
{
    return portChannelCount(sourcePort());
}

float rxInputBackupGetChannel(uint8_t channel)
{
    return portChannel(sourcePort(), channel);
}

static void rxInputBackupPortInit(uint8_t index)
{
    rxInputBackupPort_t *bp = &rxInputBackupPorts[index];
    const rxInputBackupPortConfig_t *config = &rxInputBackupConfig()->port[index];

    bp->port = NULL;

    const serialPortConfig_t *portConfig = findSerialPortConfig(rxInputBackupFunctions[index]);
    if (!portConfig) {
        return;
    }

    bp->ops = (rxInputBackupOps_t){ 0 };
    bool providerReady = false;

    switch (config->provider) {
#ifdef USE_RX_INPUT_BACKUP_SBUS
    case RX_INPUT_BACKUP_SBUS:
        providerReady = rxInputBackupSbusInit(&bp->ops, config, index);
        break;
#endif
#ifdef USE_RX_INPUT_BACKUP_FBUS
    case RX_INPUT_BACKUP_FBUS:
        providerReady = rxInputBackupFbusInit(&bp->ops, config, index);
        break;
    case RX_INPUT_BACKUP_FPORT2:
        providerReady = rxInputBackupFport2Init(&bp->ops, config, index);
        break;
#endif
#ifdef USE_RX_INPUT_BACKUP_FPORT
    case RX_INPUT_BACKUP_FPORT:
        providerReady = rxInputBackupFportInit(&bp->ops, config, index);
        break;
#endif
#ifdef USE_RX_INPUT_BACKUP_EXBUS
    case RX_INPUT_BACKUP_EXBUS:
        providerReady = rxInputBackupExbusInit(&bp->ops, config, index);
        break;
#endif
#ifdef USE_RX_INPUT_BACKUP_CRSF
    case RX_INPUT_BACKUP_CRSF:
        providerReady = rxInputBackupCrsfInit(&bp->ops, config, index);
        break;
#endif
    case RX_INPUT_BACKUP_NONE:
    default:
        break;
    }

    if (!providerReady) {
        return;
    }

    bp->lastValidFrameMs = 0;
    bp->hasValidFrame = false;
    bp->frameChannelCount = 0;

    bp->baudRate = bp->ops.baudRate;
    bp->baudLocked = false;
    bp->baudGoodFrames = 0;
    bp->baudLastEventMs = millis();

    // Only pinSwap is applied generically here - inverted/halfDuplex are
    // protocol-specific (see rxInputBackupOps_t's own comment) and already
    // baked into ops.portOptions by the provider's Init function. The
    // provider's state pointer rides along as the ISR's data argument so one
    // ISR can serve every port running that provider.
    bp->port = openSerialPort(portConfig->identifier,
        rxInputBackupFunctions[index],
        bp->ops.isrFn,
        bp->ops.state,
        bp->ops.baudRate,
        MODE_RX,
        bp->ops.portOptions |
            (config->pinSwap ? SERIAL_PINSWAP : SERIAL_NOSWAP));
}

void rxInputBackupInit(void)
{
    for (int i = 0; i < RX_INPUT_BACKUP_COUNT; i++) {
        rxInputBackupPortInit(i);
    }
}

// Backup-port wiring auto-detect - mirrors rx/rx.c's rxSerialTrial* mechanism
// (see https://doc.wingflight.org/contributing/tech/rx-wiring-autodetect/) applied to one backup port instead:
// cycle inverted/halfDuplex/pinSwap live and report which combo (if any)
// produces a valid frame. No persistence here either - the caller (MSP)
// only ever reads the winning combo back out of a SUCCESS status and applies
// it through the normal config-save path.
#define RX_INPUT_BACKUP_TRIAL_COMBO_COUNT 8
#define RX_INPUT_BACKUP_TRIAL_DEBOUNCE_MS 200
#define RX_INPUT_BACKUP_TRIAL_WATCHDOG_MS 3000
#define RX_INPUT_BACKUP_TRIAL_DEFAULT_SETTLE_MS 1000
#define RX_INPUT_BACKUP_TRIAL_HANDSHAKE_SETTLE_MS 2200

typedef struct rxInputBackupTrialRuntime_s {
    rxInputBackupTrialState_e state;
    uint8_t index;              // backup port under trial
    uint8_t comboIndex;
    uint8_t comboOrder[RX_INPUT_BACKUP_TRIAL_COMBO_COUNT];
    timeMs_t comboStartedAt;
    timeMs_t signalSince;
    timeMs_t lastPollAt;
    uint8_t savedInverted;
    uint8_t savedHalfDuplex;
    uint8_t savedPinSwap;
} rxInputBackupTrialRuntime_t;

static rxInputBackupTrialRuntime_t rxInputBackupTrial = { .state = RX_INPUT_BACKUP_TRIAL_IDLE };

static rxInputBackupPortConfig_t *trialConfig(void)
{
    return &rxInputBackupConfigMutable()->port[rxInputBackupTrial.index];
}

static timeMs_t rxInputBackupTrialSettleMs(void)
{
    switch (trialConfig()->provider) {
    case RX_INPUT_BACKUP_EXBUS:
        return RX_INPUT_BACKUP_TRIAL_HANDSHAKE_SETTLE_MS;
    default:
        return RX_INPUT_BACKUP_TRIAL_DEFAULT_SETTLE_MS;
    }
}

// Closing the old port explicitly (rather than just calling
// rxInputBackupPortInit() again) matters here: openSerialPort() refuses to
// reopen an identifier that's still marked in-use by a previous open, so
// calling Init a second time without this first would leave the port NULL
// (backup silently disabled) instead of reconfigured.
static void rxInputBackupTrialReinit(void)
{
    const serialPortConfig_t *portConfig = findSerialPortConfig(rxInputBackupFunctions[rxInputBackupTrial.index]);
    if (portConfig) {
        serialPortUsage_t *usage = findSerialPortUsageByIdentifier(portConfig->identifier);
        if (usage && usage->serialPort) {
            closeSerialPort(usage->serialPort);
        }
    }

    rxInputBackupPortInit(rxInputBackupTrial.index);
}

static void rxInputBackupTrialApplyCombo(uint8_t combo)
{
    trialConfig()->inverted   = (combo & (1 << 0)) ? 1 : 0;
    trialConfig()->halfDuplex = (combo & (1 << 1)) ? 1 : 0;
    trialConfig()->pinSwap    = (combo & (1 << 2)) ? 1 : 0;

    rxInputBackupTrialReinit();

    rxInputBackupTrial.comboStartedAt = millis();
    rxInputBackupTrial.signalSince = 0;
}

static void rxInputBackupTrialRestore(void)
{
    trialConfig()->inverted   = rxInputBackupTrial.savedInverted;
    trialConfig()->halfDuplex = rxInputBackupTrial.savedHalfDuplex;
    trialConfig()->pinSwap    = rxInputBackupTrial.savedPinSwap;

    rxInputBackupTrialReinit();
}

bool rxInputBackupTrialStart(uint8_t index)
{
    if (rxInputBackupTrial.state == RX_INPUT_BACKUP_TRIAL_RUNNING) {
        return false;
    }

    if (ARMING_FLAG(ARMED) || index >= RX_INPUT_BACKUP_COUNT) {
        rxInputBackupTrial.state = RX_INPUT_BACKUP_TRIAL_REJECTED;
        return false;
    }

    rxInputBackupTrial.index = index;

    if (trialConfig()->provider == RX_INPUT_BACKUP_NONE || !findSerialPortConfig(rxInputBackupFunctions[index])) {
        rxInputBackupTrial.state = RX_INPUT_BACKUP_TRIAL_REJECTED;
        return false;
    }

    rxInputBackupTrial.savedInverted   = trialConfig()->inverted;
    rxInputBackupTrial.savedHalfDuplex = trialConfig()->halfDuplex;
    rxInputBackupTrial.savedPinSwap    = trialConfig()->pinSwap;

    const uint8_t current = (rxInputBackupTrial.savedInverted ? (1 << 0) : 0)
        | (rxInputBackupTrial.savedHalfDuplex ? (1 << 1) : 0)
        | (rxInputBackupTrial.savedPinSwap ? (1 << 2) : 0);
    int n = 0;
    for (int distance = 0; distance <= 3; distance++) {
        for (int combo = 0; combo < RX_INPUT_BACKUP_TRIAL_COMBO_COUNT; combo++) {
            if ((int)BITCOUNT((uint8_t)(combo ^ current)) == distance) {
                rxInputBackupTrial.comboOrder[n++] = (uint8_t)combo;
            }
        }
    }

    rxInputBackupTrial.comboIndex = 0;
    rxInputBackupTrial.lastPollAt = millis();
    rxInputBackupTrial.state = RX_INPUT_BACKUP_TRIAL_RUNNING;
    rxInputBackupTrialApplyCombo(rxInputBackupTrial.comboOrder[0]);

    return true;
}

void rxInputBackupTrialStop(void)
{
    if (rxInputBackupTrial.state == RX_INPUT_BACKUP_TRIAL_IDLE) {
        return;
    }

    // REJECTED never touched the config or the port - nothing to restore.
    if (rxInputBackupTrial.state != RX_INPUT_BACKUP_TRIAL_REJECTED) {
        rxInputBackupTrialRestore();
    }
    rxInputBackupTrial.state = RX_INPUT_BACKUP_TRIAL_IDLE;
}

void rxInputBackupTrialTick(void)
{
    if (rxInputBackupTrial.state != RX_INPUT_BACKUP_TRIAL_RUNNING) {
        return;
    }

    const timeMs_t now = millis();

    if (cmp32(now, rxInputBackupTrial.lastPollAt) > RX_INPUT_BACKUP_TRIAL_WATCHDOG_MS) {
        rxInputBackupTrialRestore();
        rxInputBackupTrial.state = RX_INPUT_BACKUP_TRIAL_IDLE;
        return;
    }

    if (rxInputBackupPortIsActive(rxInputBackupTrial.index)) {
        if (rxInputBackupTrial.signalSince == 0) {
            rxInputBackupTrial.signalSince = now;
        } else if (cmp32(now, rxInputBackupTrial.signalSince) >= RX_INPUT_BACKUP_TRIAL_DEBOUNCE_MS) {
            rxInputBackupTrial.state = RX_INPUT_BACKUP_TRIAL_SUCCESS;
        }
        return;
    }

    rxInputBackupTrial.signalSince = 0;

    if (cmp32(now, rxInputBackupTrial.comboStartedAt) < (int32_t)rxInputBackupTrialSettleMs()) {
        return;
    }

    if (rxInputBackupTrial.comboIndex + 1 >= RX_INPUT_BACKUP_TRIAL_COMBO_COUNT) {
        rxInputBackupTrialRestore();
        rxInputBackupTrial.state = RX_INPUT_BACKUP_TRIAL_FAILED;
        return;
    }

    rxInputBackupTrial.comboIndex++;
    rxInputBackupTrialApplyCombo(rxInputBackupTrial.comboOrder[rxInputBackupTrial.comboIndex]);
}

rxInputBackupTrialStatus_t rxInputBackupTrialGetStatus(void)
{
    const int32_t elapsedMs = cmp32(millis(), rxInputBackupTrial.comboStartedAt);
    const rxInputBackupPortConfig_t *config = &rxInputBackupConfig()->port[rxInputBackupTrial.index];

    const rxInputBackupTrialStatus_t status = {
        .state = rxInputBackupTrial.state,
        .comboIndex = rxInputBackupTrial.comboIndex,
        .inverted = config->inverted,
        .halfDuplex = config->halfDuplex,
        .pinSwap = config->pinSwap,
        .elapsedMs = (uint16_t)constrain(elapsedMs, 0, 0xFFFF),
        .index = rxInputBackupTrial.index,
    };

    rxInputBackupTrial.lastPollAt = millis();

    return status;
}

#endif // USE_RX_INPUT_BACKUP

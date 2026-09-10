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

// How long without a decoded frame before the backup link is considered down.
// ~3 missed frames at a typical ~6-14ms/frame rate - same margin the original
// SBUS-only driver used, kept here since it's a property of "how stale is too
// stale to trust", not of any one protocol's framing.
#define RX_INPUT_BACKUP_STALE_MS 50

static serialPort_t *rxInputBackupPort = NULL;
static rxInputBackupOps_t rxInputBackupOps;

static float rxInputBackupChannel[RX_INPUT_BACKUP_MAX_CHANNEL];
static timeMs_t rxInputBackupLastValidFrameMs = 0;

// False until the first genuinely valid frame has been decoded. Without this,
// rxInputBackupIsActive() would read as "active" for up to RX_INPUT_BACKUP_STALE_MS
// right after boot/config-change purely because rxInputBackupLastValidFrameMs's
// zero-init happens to be within that window of millis()'s own startup value -
// reporting the backup available (and, if the main link were already down at that
// moment, feeding zeroed channels) before any real frame has ever been seen.
static bool rxInputBackupHasValidFrame = false;

bool rxInputBackupIsEnabled(void)
{
    return rxInputBackupPort != NULL;
}

// Decodes any newly-completed frame (via the selected provider's update()) and
// refreshes freshness state. Must be called every cycle regardless of whether the
// main RX link is up or the backup is currently "needed" - it used to be called
// only as a side effect of rxInputBackupIsActive(), which rx.c only evaluates once
// the main link is already down (short-circuiting `!rxSignalReceived && ...`).
// That starved this of any real-time decoding whenever the main link was healthy,
// leaving diagnostics/MSP polling as the only thing driving it (once every poll
// interval instead of every cycle) and meaning the very first backup frame used at
// the instant of a real failover could already be stale.
void rxInputBackupPoll(void)
{
    if (!rxInputBackupIsEnabled()) {
        return;
    }

    if (rxInputBackupOps.update(rxInputBackupChannel, rxInputBackupOps.channelCount)) {
        rxInputBackupHasValidFrame = true;
        rxInputBackupLastValidFrameMs = millis();
    }

    if (rxInputBackupHasValidFrame && (timeMs_t)(millis() - rxInputBackupLastValidFrameMs) >= RX_INPUT_BACKUP_STALE_MS) {
        // No explicit parser reset here (unlike the pre-refactor SBUS-only driver) -
        // each provider's own frame-timing logic already self-heals from a stale gap
        // the moment bytes resume (see e.g. rx_input_backup_sbus.c's frameTime check),
        // so an external reset call was never load-bearing for correctness, only an
        // (unnecessary) hygiene step this generic layer would otherwise have to poke
        // back into provider-private state to perform.
        rxInputBackupHasValidFrame = false;
    }
}

bool rxInputBackupIsActive(void)
{
    if (!rxInputBackupIsEnabled() || !rxInputBackupHasValidFrame) {
        return false;
    }

    return (timeMs_t)(millis() - rxInputBackupLastValidFrameMs) < RX_INPUT_BACKUP_STALE_MS;
}

uint8_t rxInputBackupGetChannelCount(void)
{
    return rxInputBackupOps.channelCount;
}

rxInputBackupProvider_e rxInputBackupGetProvider(void)
{
    return rxInputBackupConfig()->provider;
}

float rxInputBackupGetChannel(uint8_t channel)
{
    if (channel >= rxInputBackupOps.channelCount) {
        return 0;
    }
    return rxInputBackupChannel[channel];
}

void rxInputBackupInit(void)
{
    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_RX_INPUT_BACKUP);
    if (!portConfig) {
        rxInputBackupPort = NULL;
        return;
    }

    rxInputBackupOps = (rxInputBackupOps_t){ 0 };
    bool providerReady = false;

    switch (rxInputBackupConfig()->provider) {
#ifdef USE_RX_INPUT_BACKUP_SBUS
    case RX_INPUT_BACKUP_SBUS:
        providerReady = rxInputBackupSbusInit(&rxInputBackupOps);
        break;
#endif
#ifdef USE_RX_INPUT_BACKUP_FBUS
    case RX_INPUT_BACKUP_FBUS:
        providerReady = rxInputBackupFbusInit(&rxInputBackupOps);
        break;
    case RX_INPUT_BACKUP_FPORT2:
        providerReady = rxInputBackupFport2Init(&rxInputBackupOps);
        break;
#endif
#ifdef USE_RX_INPUT_BACKUP_FPORT
    case RX_INPUT_BACKUP_FPORT:
        providerReady = rxInputBackupFportInit(&rxInputBackupOps);
        break;
#endif
#ifdef USE_RX_INPUT_BACKUP_EXBUS
    case RX_INPUT_BACKUP_EXBUS:
        providerReady = rxInputBackupExbusInit(&rxInputBackupOps);
        break;
#endif
#ifdef USE_RX_INPUT_BACKUP_CRSF
    case RX_INPUT_BACKUP_CRSF:
        providerReady = rxInputBackupCrsfInit(&rxInputBackupOps);
        break;
#endif
    case RX_INPUT_BACKUP_NONE:
    default:
        break;
    }

    if (!providerReady) {
        rxInputBackupPort = NULL;
        return;
    }

    rxInputBackupLastValidFrameMs = 0;
    rxInputBackupHasValidFrame = false;

    // Only pinSwap is applied generically here - inverted/halfDuplex are
    // protocol-specific (see rxInputBackupOps_t's own comment) and already
    // baked into rxInputBackupOps.portOptions by the provider's Init function.
    rxInputBackupPort = openSerialPort(portConfig->identifier,
        FUNCTION_RX_INPUT_BACKUP,
        rxInputBackupOps.isrFn,
        NULL,
        rxInputBackupOps.baudRate,
        MODE_RX,
        rxInputBackupOps.portOptions |
            (rxInputBackupConfig()->pinSwap ? SERIAL_PINSWAP : SERIAL_NOSWAP));
}

// Backup-port wiring auto-detect - mirrors rx/rx.c's rxSerialTrial* mechanism
// (see docs/rx-wiring-autodetect-design.md) applied to this port instead:
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

static timeMs_t rxInputBackupTrialSettleMs(void)
{
    switch (rxInputBackupConfig()->provider) {
    case RX_INPUT_BACKUP_EXBUS:
        return RX_INPUT_BACKUP_TRIAL_HANDSHAKE_SETTLE_MS;
    default:
        return RX_INPUT_BACKUP_TRIAL_DEFAULT_SETTLE_MS;
    }
}

// Closing the old port explicitly (rather than just calling
// rxInputBackupInit() again) matters here: openSerialPort() refuses to
// reopen an identifier that's still marked in-use by a previous open, so
// calling Init() a second time without this first would leave
// rxInputBackupPort NULL (backup silently disabled) instead of reconfigured.
static void rxInputBackupTrialReinit(void)
{
    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_RX_INPUT_BACKUP);
    if (portConfig) {
        serialPortUsage_t *usage = findSerialPortUsageByIdentifier(portConfig->identifier);
        if (usage && usage->serialPort) {
            closeSerialPort(usage->serialPort);
        }
    }

    rxInputBackupInit();
}

static void rxInputBackupTrialApplyCombo(uint8_t combo)
{
    rxInputBackupConfigMutable()->inverted   = (combo & (1 << 0)) ? 1 : 0;
    rxInputBackupConfigMutable()->halfDuplex = (combo & (1 << 1)) ? 1 : 0;
    rxInputBackupConfigMutable()->pinSwap    = (combo & (1 << 2)) ? 1 : 0;

    rxInputBackupTrialReinit();

    rxInputBackupTrial.comboStartedAt = millis();
    rxInputBackupTrial.signalSince = 0;
}

static void rxInputBackupTrialRestore(void)
{
    rxInputBackupConfigMutable()->inverted   = rxInputBackupTrial.savedInverted;
    rxInputBackupConfigMutable()->halfDuplex = rxInputBackupTrial.savedHalfDuplex;
    rxInputBackupConfigMutable()->pinSwap    = rxInputBackupTrial.savedPinSwap;

    rxInputBackupTrialReinit();
}

bool rxInputBackupTrialStart(void)
{
    if (rxInputBackupTrial.state == RX_INPUT_BACKUP_TRIAL_RUNNING) {
        return false;
    }

    if (ARMING_FLAG(ARMED)) {
        rxInputBackupTrial.state = RX_INPUT_BACKUP_TRIAL_REJECTED;
        return false;
    }

    if (rxInputBackupConfig()->provider == RX_INPUT_BACKUP_NONE || !findSerialPortConfig(FUNCTION_RX_INPUT_BACKUP)) {
        rxInputBackupTrial.state = RX_INPUT_BACKUP_TRIAL_REJECTED;
        return false;
    }

    rxInputBackupTrial.savedInverted   = rxInputBackupConfig()->inverted;
    rxInputBackupTrial.savedHalfDuplex = rxInputBackupConfig()->halfDuplex;
    rxInputBackupTrial.savedPinSwap    = rxInputBackupConfig()->pinSwap;

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

    rxInputBackupTrialRestore();
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

    if (rxInputBackupIsActive()) {
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

    const rxInputBackupTrialStatus_t status = {
        .state = rxInputBackupTrial.state,
        .comboIndex = rxInputBackupTrial.comboIndex,
        .inverted = rxInputBackupConfig()->inverted,
        .halfDuplex = rxInputBackupConfig()->halfDuplex,
        .pinSwap = rxInputBackupConfig()->pinSwap,
        .elapsedMs = (uint16_t)constrain(elapsedMs, 0, 0xFFFF),
    };

    rxInputBackupTrial.lastPollAt = millis();

    return status;
}

#endif // USE_RX_INPUT_BACKUP

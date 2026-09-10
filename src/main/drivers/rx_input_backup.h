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

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "drivers/serial.h"

// A secondary, independent RX input used as a fallback when the main RX link's
// signal is lost. See rx_input_backup.c for the parser/failover rationale and
// rx.c's detectAndApplySignalLossBehaviour() for the takeover logic itself.
//
// Provider-selectable (pg/rx_input_backup.h's `provider` field): only SBUS is
// implemented today, but the split between this generic file and one file per
// provider (e.g. rx_input_backup_sbus.c) exists so adding another protocol is
// "add one file + one dispatch case", not a rework of this file. There is
// deliberately no telemetry support on this link, ever - it exists purely to
// hand over channel data, same as a physical backup satellite receiver would.

// Keep in sync with cli/settings.c's lookupTableRxInputBackupProvider[] (same
// order) and pg/rx_input_backup.h's `provider` field width (uint8_t). Kept
// fully populated regardless of which USE_RX_INPUT_BACKUP_xxx flags a given
// target builds - same convention rx/rx.h's SerialRXType uses - only the
// dispatch switch in rx_input_backup.c's Init function is #ifdef-guarded.
typedef enum {
    // 0, matching this codebase's usual "zero-init means off/disabled"
    // convention. A port assigned FUNCTION_RX_INPUT_BACKUP with this provider
    // is reserved but never opened (rxInputBackupInit() treats it the same as
    // no provider ready). Also the default for a freshly reset config
    // (pgResetFn_rxInputBackupConfig), so assigning the port function alone
    // no longer silently starts decoding SBUS on it before the user has
    // actually chosen a protocol.
    RX_INPUT_BACKUP_NONE = 0,
    RX_INPUT_BACKUP_SBUS = 1,
    RX_INPUT_BACKUP_FBUS = 2,
    RX_INPUT_BACKUP_FPORT = 3,
    RX_INPUT_BACKUP_FPORT2 = 4,
    RX_INPUT_BACKUP_EXBUS = 5,
    RX_INPUT_BACKUP_CRSF = 6,
} rxInputBackupProvider_e;

#define RX_INPUT_BACKUP_MAX_CHANNEL 18

// Populated by a provider's own Init function (see rx_input_backup_sbus.h for
// the SBUS example) and consumed only by rx_input_backup.c - not part of the
// public API the rest of the firmware uses (that's the plain functions below).
typedef struct rxInputBackupOps_s {
    uint32_t baudRate;
    // Protocol-fixed framing (stopbits/parity) PLUS this provider's own
    // translation of the user's shared inverted/halfDuplex config into the
    // correct SERIAL_INVERTED/SERIAL_NOT_INVERTED and SERIAL_BIDIR[_PP]
    // direction/variant for its own protocol - these differ by protocol (SBUS
    // is natively inverted, FBUS/FPort/FPort2 are not; SBUS uses plain
    // SERIAL_BIDIR, FBUS/FPort use SERIAL_BIDIR|SERIAL_BIDIR_PP), exactly
    // mirroring how rx/sbus.c and rx/fbus.c/fport.c each apply the main RX's
    // own serialrx_inverted/halfDuplex differently for the same reason. Only
    // pinSwap (protocol-agnostic) is still ORed in generically by
    // rx_input_backup.c, since every protocol treats it identically.
    portOptions_e portOptions;
    serialReceiveCallbackPtr isrFn;
    uint8_t channelCount;       // <= RX_INPUT_BACKUP_MAX_CHANNEL

    // Called once per rxInputBackupPoll() cycle. Returns true and fills
    // channels[0..channelCount) if a new, valid frame was decoded this call;
    // returns false (leaving channels untouched) otherwise - no frame pending
    // yet, or the pending frame was rejected (e.g. dropped/failsafe).
    bool (*update)(float *channels, uint8_t channelCount);
} rxInputBackupOps_t;

void rxInputBackupInit(void);

// Decodes any newly-completed frame and refreshes channel/freshness state.
// Must be called every cycle from the RX task regardless of main-link state -
// see rx.c's detectAndApplySignalLossBehaviour() for why this can't just be a
// side effect of rxInputBackupIsActive() any more.
void rxInputBackupPoll(void);

// True once a serial port has been assigned FUNCTION_RX_INPUT_BACKUP.
bool rxInputBackupIsEnabled(void);

// True when enabled AND a valid frame has been decoded within the freshness
// window - i.e. the backup link is currently healthy and its channel data
// should be trusted as a fallback for the main RX.
bool rxInputBackupIsActive(void);

// Number of channels the selected provider decodes.
uint8_t rxInputBackupGetChannelCount(void);

// Currently selected provider (valid once rxInputBackupIsEnabled() is true).
rxInputBackupProvider_e rxInputBackupGetProvider(void);

// Channel value in the same convention as rx/rx.c's rcInput[]/rcChannel[] (~880-2012us).
float rxInputBackupGetChannel(uint8_t channel);

// Backup-port wiring auto-detect ("trial mode") - same mechanism and
// numeric state values as rx/rx.h's rxSerialTrialState_e for the main RX
// (see docs/rx-wiring-autodetect-design.md), applied to this port's own
// inverted/halfDuplex/pinSwap instead. Kept as a separate type rather than
// sharing rx.h's, since this driver has no other dependency on rx/rx.h.
typedef enum {
    RX_INPUT_BACKUP_TRIAL_IDLE = 0,
    RX_INPUT_BACKUP_TRIAL_RUNNING,
    RX_INPUT_BACKUP_TRIAL_SUCCESS,
    RX_INPUT_BACKUP_TRIAL_FAILED,
    RX_INPUT_BACKUP_TRIAL_REJECTED,   // no provider selected / no port assigned / already running
} rxInputBackupTrialState_e;

typedef struct rxInputBackupTrialStatus_s {
    uint8_t state;          // rxInputBackupTrialState_e
    uint8_t comboIndex;     // 0..7: combo currently (or, on FAILED, last) being tried
    uint8_t inverted;
    uint8_t halfDuplex;
    uint8_t pinSwap;
    uint16_t elapsedMs;     // time spent on the current/last combo
} rxInputBackupTrialStatus_t;

bool rxInputBackupTrialStart(void);
void rxInputBackupTrialStop(void);
rxInputBackupTrialStatus_t rxInputBackupTrialGetStatus(void);

// Ticked every rxInputBackupPoll() cycle (rx.c's detectAndApplySignalLossBehaviour()) -
// cheap early-out when idle, same as rx.c's own rxSerialTrialTick().
void rxInputBackupTrialTick(void);

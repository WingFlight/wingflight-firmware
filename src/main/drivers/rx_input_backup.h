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

#include "io/serial.h"

#include "pg/rx_input_backup.h"

// Up to RX_INPUT_BACKUP_COUNT independent RX inputs used as fallbacks when
// the main RX link's signal is lost. Failover is a fixed priority chain: the
// main RX, then backup port 0 (FUNCTION_RX_INPUT_BACKUP), then backup port 1
// (FUNCTION_RX_INPUT_BACKUP2). See rx_input_backup.c for the parser/failover
// rationale and rx.c's detectAndApplySignalLossBehaviour() for the takeover
// logic itself.
//
// Provider-selectable per port (pg/rx_input_backup.h's `provider` field). The
// split between this generic file and one file per provider (e.g.
// rx_input_backup_sbus.c) exists so adding another protocol is "add one file
// + one dispatch case", not a rework of this file. Every provider keeps one
// parser state per backup port, so two ports may run the same protocol. There
// is deliberately no telemetry support on these links, ever - telemetry goes
// to the primary RX only; a backup exists purely to hand over channel data,
// same as a physical backup satellite receiver would.

// Keep in sync with cli/settings.c's lookupTableRxInputBackupProvider[] (same
// order) and pg/rx_input_backup.h's `provider` field width (uint8_t). Kept
// fully populated regardless of which USE_RX_INPUT_BACKUP_xxx flags a given
// target builds - same convention rx/rx.h's SerialRXType uses - only the
// dispatch switch in rx_input_backup.c's Init function is #ifdef-guarded.
typedef enum {
    // 0, matching this codebase's usual "zero-init means off/disabled"
    // convention. A port assigned a backup function with this provider is
    // reserved but never opened (rxInputBackupInit() treats it the same as
    // no provider ready). Also the default for a freshly reset config
    // (pgResetFn_rxInputBackupConfig), so assigning the port function alone
    // doesn't silently start decoding SBUS on it before the user has
    // actually chosen a protocol.
    RX_INPUT_BACKUP_NONE = 0,
    RX_INPUT_BACKUP_SBUS = 1,
    RX_INPUT_BACKUP_FBUS = 2,
    RX_INPUT_BACKUP_FPORT = 3,
    RX_INPUT_BACKUP_FPORT2 = 4,
    RX_INPUT_BACKUP_EXBUS = 5,
    RX_INPUT_BACKUP_CRSF = 6,
} rxInputBackupProvider_e;

// 24-channel F.Bus/F.Port2 frames decode to 24 analog + 2 digital channels
#define RX_INPUT_BACKUP_MAX_CHANNEL 26

// Populated by a provider's own Init function (see rx_input_backup_sbus.h for
// the SBUS example) and consumed only by rx_input_backup.c - not part of the
// public API the rest of the firmware uses (that's the plain functions below).
typedef struct rxInputBackupOps_s {
    uint32_t baudRate;
    // Second speed the same protocol may run at (FBUS/F.Port2), 0 if none.
    // rx_input_backup.c switches between the two until valid frames arrive.
    uint32_t altBaudRate;
    // Protocol-fixed framing (stopbits/parity) PLUS this provider's own
    // translation of the port's inverted/halfDuplex config into the
    // correct SERIAL_INVERTED/SERIAL_NOT_INVERTED and SERIAL_BIDIR[_PP]
    // direction/variant for its own protocol - these differ by protocol (SBUS
    // is natively inverted, FBUS/FPort/FPort2 are not; SBUS uses plain
    // SERIAL_BIDIR, FBUS/FPort use SERIAL_BIDIR|SERIAL_BIDIR_PP), exactly
    // mirroring how rx/sbus.c and rx/fbus.c/fport.c each apply the main RX's
    // own serialrx_inverted/halfDuplex differently for the same reason. Only
    // pinSwap (protocol-agnostic) is still ORed in generically by
    // rx_input_backup.c, since every protocol treats it identically.
    portOptions_e portOptions;
    // Receive ISR. Called with `state` as its data argument, so one ISR
    // serves every backup port running this provider.
    serialReceiveCallbackPtr isrFn;
    // This port's own parser state, owned by the provider (one per backup
    // port). Passed back to isrFn and update().
    void *state;
    uint8_t channelCount;       // most channels a frame can carry, <= RX_INPUT_BACKUP_MAX_CHANNEL

    // Called once per rxInputBackupPoll() cycle. If a new, valid frame was
    // decoded this call, fills channels[0..n) and returns n, the number of
    // channels that frame carried (<= channelCount; F.Bus frames can carry
    // 8, 16 or 24). Returns 0 (leaving channels untouched) otherwise - no
    // frame pending yet, or the pending frame was rejected (e.g.
    // dropped/failsafe).
    uint8_t (*update)(void *state, float *channels, uint8_t channelCount);
} rxInputBackupOps_t;

// Serial port function carried by backup port `index`.
serialPortFunction_e rxInputBackupPortFunction(uint8_t index);

void rxInputBackupInit(void);

// Decodes any newly-completed frame on every backup port and refreshes
// channel/freshness state. Must be called every cycle from the RX task
// regardless of main-link state - see rx.c's detectAndApplySignalLossBehaviour()
// for why this can't just be a side effect of rxInputBackupIsActive() any more.
void rxInputBackupPoll(void);

// Per-port state. `index` is 0..RX_INPUT_BACKUP_COUNT-1; out-of-range reads
// as disabled/inactive.

// True once a serial port has been assigned this backup's function and a
// provider other than NONE is selected for it.
bool rxInputBackupPortIsEnabled(uint8_t index);

// True when enabled AND a valid frame has been decoded within the freshness
// window - i.e. this backup link is currently healthy.
bool rxInputBackupPortIsActive(uint8_t index);

// Number of channels in this port's latest valid frame (the provider's
// maximum before the first frame).
uint8_t rxInputBackupPortGetChannelCount(uint8_t index);

// Provider configured for this port (RX_INPUT_BACKUP_NONE when disabled).
rxInputBackupProvider_e rxInputBackupPortGetProvider(uint8_t index);

// Channel value in the same convention as rx/rx.c's rcInput[]/rcChannel[] (~880-2012us).
// Channels the latest frame doesn't carry read as stick center (rc_center).
float rxInputBackupPortGetChannel(uint8_t index, uint8_t channel);

// Aggregate state across all backup ports - what rx.c, the arming check and
// telemetry use.

// True when at least one backup port is enabled.
bool rxInputBackupIsEnabled(void);

// True when at least one backup port is active, i.e. there is a backup link
// whose channel data can be trusted as a fallback for the main RX.
bool rxInputBackupIsActive(void);

// True when every enabled backup port is active (and at least one is
// enabled). The arming check uses this: each configured backup has to prove
// itself before the first arm.
bool rxInputBackupAllLinked(void);

// Index of the backup port that would take over right now - the first active
// one in priority order - or -1 if none is active.
int8_t rxInputBackupGetActiveIndex(void);

// Channel data of the port rxInputBackupGetActiveIndex() names. Only
// meaningful while rxInputBackupIsActive() is true; otherwise they read the
// first enabled port.
uint8_t rxInputBackupGetChannelCount(void);
float rxInputBackupGetChannel(uint8_t channel);

// Backup-port wiring auto-detect ("trial mode") - same mechanism and
// numeric state values as rx/rx.h's rxSerialTrialState_e for the main RX
// (see https://doc.wingflight.org/contributing/tech/rx-wiring-autodetect/), applied to one backup
// port's own inverted/halfDuplex/pinSwap instead. Kept as a separate type rather than
// sharing rx.h's, since this driver has no other dependency on rx/rx.h.
// Only one trial runs at a time, on the port named at start.
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
    uint8_t index;          // backup port the trial runs (or last ran) on
} rxInputBackupTrialStatus_t;

bool rxInputBackupTrialStart(uint8_t index);
void rxInputBackupTrialStop(void);
rxInputBackupTrialStatus_t rxInputBackupTrialGetStatus(void);

// Ticked every rxInputBackupPoll() cycle (rx.c's detectAndApplySignalLossBehaviour()) -
// cheap early-out when idle, same as rx.c's own rxSerialTrialTick().
void rxInputBackupTrialTick(void);

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

#include "common/utils.h"
#include "pg/pg.h"

// Electrical/protocol settings for the backup RX port (drivers/rx_input_backup.c).
// Inversion/pin-swap are deliberately independent from the main RX's
// serialrx_inverted/serialrx_pinswap (pg/rx.h) - the backup port is a different
// physical UART and may need different wiring/inversion than the main receiver's.
typedef struct rxInputBackupConfig_s {
    // Which protocol to decode on this port. Plain uint8_t (not the
    // rxInputBackupProvider_e enum from drivers/rx_input_backup.h) matching how
    // pg/rx.h's serialrx_provider is stored - see cli/settings.c's
    // lookupTableRxInputBackupProvider[] for the CLI-visible names, which must
    // stay in the same order as the drivers/rx_input_backup.h enum.
    uint8_t provider;

    // When OFF (0, default), the UART is set up however the selected provider's
    // own protocol normally expects to be wired (e.g. SBUS's signal is
    // natively inverted, so provider SBUS applies hardware inversion by
    // default; FBUS/FPort/FPort2 are natively non-inverted, so those
    // providers leave the UART non-inverted by default) - each provider's own
    // Init function (e.g. rx_input_backup_sbus.c) decides which
    // SERIAL_INVERTED/SERIAL_NOT_INVERTED direction that maps to, exactly
    // mirroring how rx/sbus.c and rx/fbus.c/fport.c apply the main RX's own
    // serialrx_inverted in opposite directions for the same reason. When ON,
    // the signal is inverted relative to that protocol's own normal
    // convention (e.g. an external inverter, or a receiver that just wires it
    // the other way).
    uint8_t inverted;

    // Half-duplex (single-wire) mode, for boards where only one physical wire
    // is connected for this port. Same per-provider direction/variant
    // consideration as `inverted` - see e.g. rx/sbus.c's plain SERIAL_BIDIR
    // vs rx/fbus.c's SERIAL_BIDIR|SERIAL_BIDIR_PP. Safe to use even though
    // this link is receive-only (MODE_RX, never transmits): the UART driver's
    // half-duplex pin setup is gated purely on this option, not on whether
    // MODE_TX is also requested (drivers/serial_uart_stm32f7xx.c and
    // siblings), so a single-wire receive-only satellite is fully supported.
    uint8_t halfDuplex;

    // Swaps the UART's RX/TX pins, for boards where the backup port's natural
    // RX pin isn't the one that's actually wired. Protocol-agnostic (matches
    // rx/sbus.c's/rx/fbus.c's/rx/fport.c's own identical pinSwap handling),
    // so this one is still applied generically by rx_input_backup.c rather
    // than by each provider.
    uint8_t pinSwap;
} rxInputBackupConfig_t;

PG_DECLARE(rxInputBackupConfig_t, rxInputBackupConfig);

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

#include "drivers/rx_input_backup.h"

// Jeti EX Bus provider for the generic backup-RX framework
// (rx_input_backup.c). Decodes only the fixed 16-channel "Channel Data"
// frame (0x3E ...) rx/jetiexbus.c itself decodes - the other frame type on
// this bus, the FC-addressed telemetry/Jetibox request (0x3D ...), is simply
// never recognized as a frame start and so is ignored outright, matching
// this framework's usual "receive-only, no telemetry, ever" scope (this
// backup link never answers a request, so it should essentially never see
// one addressed to it in the first place).
//
// Unlike every other provider here, real EX Bus receivers are documented as
// requiring an active bus master (the FC) present on the wire, and
// rx/jetiexbus.c itself always opens MODE_RXTX + unconditional SERIAL_BIDIR
// for exactly that reason. This backup link never transmits at all
// (MODE_RX only, imposed generically by rx_input_backup.c). A Jeti EX Bus
// receiver broadcasts its channel-data frames on its own fixed schedule
// regardless of whether anything ever replies (the FC's reply only matters
// for telemetry); confirmed on hardware, the receive-only backup links and
// fails over as expected.

bool rxInputBackupExbusInit(rxInputBackupOps_t *ops, const rxInputBackupPortConfig_t *config, uint8_t index);

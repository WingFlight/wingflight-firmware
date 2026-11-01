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

// FBUS and FPort2 providers for the generic backup-RX framework
// (rx_input_backup.c). Both use the exact same length-prefixed frame format -
// rx/fbus.c's own fbusRxInit(rxConfig, rxRuntimeState, isFPORT2) confirms
// isFPORT2 only ever changes the baud rate passed to openSerialPort(), not
// the framing/checksum/dispatch logic - so one shared parser covers both,
// parameterized only by baud. Decodes the 8-, 16- and 24-channel frame
// lengths, like rx/fbus.c.

bool rxInputBackupFbusInit(rxInputBackupOps_t *ops, const rxInputBackupPortConfig_t *config, uint8_t index);
bool rxInputBackupFport2Init(rxInputBackupOps_t *ops, const rxInputBackupPortConfig_t *config, uint8_t index);

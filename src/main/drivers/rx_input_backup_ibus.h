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

// IBUS (Flysky) provider for the generic backup-RX framework
// (rx_input_backup.c). Supports both wire variants rx/ibus.c does: the
// original fixed-31-byte IA6 legacy frame (sync 0x55, no length byte) and the
// newer length-prefixed IA6B frame (32 bytes, up to 18 channels via the
// upper-nibble extension trick) - auto-detected the same way rx/ibus.c does,
// by the first byte's value. There is no third variant to add later here:
// unlike FBUS, IBUS doesn't have additional frame-length options beyond these
// two.

bool rxInputBackupIbusInit(rxInputBackupOps_t *ops);

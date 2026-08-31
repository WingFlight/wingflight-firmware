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

// FPort provider for the generic backup-RX framework (rx_input_backup.c).
// Genuinely different framing from FBUS/FPort2 (see rx_input_backup_fbus.h) -
// FPort uses HDLC-style byte-stuffing (0x7E/0x7D), not simple length-prefixing.

bool rxInputBackupFportInit(rxInputBackupOps_t *ops);

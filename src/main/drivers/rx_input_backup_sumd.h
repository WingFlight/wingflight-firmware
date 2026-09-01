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

// SUMD (Graupner) provider for the generic backup-RX framework
// (rx_input_backup.c). Simple length-prefixed frame with a CRC16-CCITT
// trailer, verified against rx/sumd.c. Supports both SUMD V1 and V3 frame
// status bytes (rx/sumd.c treats them identically); channel count is
// declared per-frame in the wire format itself, same as rx/sumd.c, but (also
// like rx/sumd.c) this provider still reports one fixed channelCount at Init
// time - RX_INPUT_BACKUP_MAX_CHANNEL - since a per-frame-varying count isn't
// something this framework's Ops interface supports (see
// rx_input_backup_fbus.h's own note on the same limitation for FBUS's
// 8/16/24ch frame-length variants).

bool rxInputBackupSumdInit(rxInputBackupOps_t *ops);

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

// Spektrum DSM provider for the generic backup-RX framework
// (rx_input_backup.c). Two entry points sharing one parser - only the
// per-channel bit resolution differs, mirroring how rx/rx.h's own
// SERIALRX_SPEKTRUM1024/SERIALRX_SPEKTRUM2048 are two distinct provider
// values rather than one provider plus a resolution flag; this backup
// framework follows that same precedent instead of adding a new config
// field. There is no on-wire way to auto-detect which resolution a given
// satellite is transmitting (rx/spektrum.c doesn't either - it's a static
// config choice there too), so the wrong one will decode nonsense until
// corrected. 2048/DSMX is the resolution virtually all modern satellites use.
//
// Unlike every other provider here, Spektrum DSM has no per-frame checksum or
// CRC at all - rx/spektrum.c relies purely on fixed-frame-size timing, and so
// does this provider.

bool rxInputBackupSpektrum1024Init(rxInputBackupOps_t *ops);
bool rxInputBackupSpektrum2048Init(rxInputBackupOps_t *ops);

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

#include "pg/pg_ids.h"
#include "platform.h"

#include "pg/rx_input_backup.h"

#ifdef USE_RX_INPUT_BACKUP

// Version 1: this struct's shape has changed twice since this parameter group
// was first registered (SBUS-only inverted+pinSwap -> provider added ->
// halfDuplex added) without ever bumping this version number, which would
// have let an old saved record's bytes get silently reinterpreted as the
// wrong fields on load (e.g. an old `pinSwap` byte loading into the new
// `inverted` field). Bumped now, before this has ever shipped in a tagged
// release, so nobody's saved config is actually affected - just closing the
// gap for good going forward. Bump this again any time a field is added,
// removed, or reordered.
PG_REGISTER_WITH_RESET_FN(rxInputBackupConfig_t, rxInputBackupConfig,
                          PG_DRIVER_RX_INPUT_BACKUP_CONFIG, 1);

void pgResetFn_rxInputBackupConfig(rxInputBackupConfig_t *config)
{
    config->provider = 0; // RX_INPUT_BACKUP_NONE
    config->inverted = 0;
    config->halfDuplex = 0;
    config->pinSwap = 0;
}

#endif

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

// Version 2: the single provider/inverted/halfDuplex/pinSwap record became
// an array of RX_INPUT_BACKUP_COUNT records, one per backup port. Bump this
// again any time a field is added, removed, or reordered, or the array
// length changes.
PG_REGISTER_WITH_RESET_FN(rxInputBackupConfig_t, rxInputBackupConfig,
                          PG_DRIVER_RX_INPUT_BACKUP_CONFIG, 2);

void pgResetFn_rxInputBackupConfig(rxInputBackupConfig_t *config)
{
    for (int i = 0; i < RX_INPUT_BACKUP_COUNT; i++) {
        config->port[i].provider = 0; // RX_INPUT_BACKUP_NONE
        config->port[i].inverted = 0;
        config->port[i].halfDuplex = 0;
        config->port[i].pinSwap = 0;
    }
}

#endif

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

PG_REGISTER_WITH_RESET_FN(rxInputBackupConfig_t, rxInputBackupConfig,
                          PG_DRIVER_RX_INPUT_BACKUP_CONFIG, 0);

void pgResetFn_rxInputBackupConfig(rxInputBackupConfig_t *config)
{
    config->provider = 4; // RX_INPUT_BACKUP_NONE - see its own comment: a
                           // freshly-assigned port shouldn't silently start
                           // decoding SBUS before the user has actually
                           // chosen a protocol.
    config->inverted = 0;
    config->halfDuplex = 0;
    config->pinSwap = 0;
}

#endif

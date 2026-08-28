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

#include "pg/rx_sbus_input.h"

#ifdef USE_RX_SBUS_INPUT

PG_REGISTER_WITH_RESET_FN(sbusInputConfig_t, sbusInputConfig,
                          PG_DRIVER_RX_SBUS_INPUT_CONFIG, 0);

void pgResetFn_sbusInputConfig(sbusInputConfig_t *config)
{
    config->inverted = 0;
    config->pinSwap = 0;
}

#endif

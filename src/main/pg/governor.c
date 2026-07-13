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
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "types.h"
#include "platform.h"

#include "pg/pg_ids.h"
#include "pg/governor.h"


PG_REGISTER_WITH_RESET_TEMPLATE(governorConfig_t, governorConfig, PG_GOVERNOR_CONFIG, 0);

PG_RESET_TEMPLATE(governorConfig_t, governorConfig,
    .governor_mode = GOVERNOR_MODE_OFF,
    .governor_rpm = 0,
    .governor_gain = 20,
    .governor_i_gain = 30,
    .governor_throttle = 15,
    .governor_handover = 10,
    .governor_ceiling = 30,
    .governor_rpm_min = 0,
    .governor_rpm_max = 0,
);

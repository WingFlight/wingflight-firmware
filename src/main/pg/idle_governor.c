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
#include "pg/idle_governor.h"


PG_REGISTER_WITH_RESET_TEMPLATE(idleGovernorConfig_t, idleGovernorConfig, PG_IDLE_GOVERNOR_CONFIG, 0);

PG_RESET_TEMPLATE(idleGovernorConfig_t, idleGovernorConfig,
    .idle_governor_mode = IDLE_GOVERNOR_MODE_OFF,
    .idle_governor_rpm = 0,
    .idle_governor_gain = 10,
    .idle_governor_throttle = 15,
    .idle_governor_handover = 10,
    .idle_governor_ceiling = 30,
);

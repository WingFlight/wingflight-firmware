/*
 * This file is part of Rotorflight.
 *
 * Rotorflight is free software. You can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Rotorflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software. If not, see <https://www.gnu.org/licenses/>.
 */

#include "types.h"
#include "platform.h"

#include "pg/pg_ids.h"
#include "pg/arming.h"


// v1->v2: removed the wiggle indicator fields -- the feature was heli-derived
// (collective stick bumps to signal ready/armed/error) and never applicable
// to a fixed-wing airframe; existing saved configs reset to these defaults
// rather than reinterpreting trailing bytes that no longer exist.
PG_REGISTER_WITH_RESET_TEMPLATE(armingConfig_t, armingConfig, PG_ARMING_CONFIG, 2);

PG_RESET_TEMPLATE(armingConfig_t, armingConfig,
    .gyro_cal_on_first_arm = 0,
    .auto_disarm_delay = 5,
    .power_on_arming_grace_time = 3,
    .enable_stick_arming = false,
    .enable_stick_commands = false,
);

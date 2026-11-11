/*
 * This file is part of Wingflight.
 *
 * Wingflight is free software. You can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Wingflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software. If not, see <https://www.gnu.org/licenses/>.
 */

#include "platform.h"

#include "config/config_reset.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "pg/reverse_flight.h"

// Defaults are first guesses for a light indoor 4D shockflyer, to be refined
// from blackbox logs. NORMAL keeps every existing setup unchanged.
PG_REGISTER_WITH_RESET_TEMPLATE(reverseFlightConfig_t, reverseFlightConfig, PG_REVERSE_FLIGHT_CONFIG, 0);

PG_RESET_TEMPLATE(reverseFlightConfig_t, reverseFlightConfig,
    .throttle_mode = THROTTLE_MODE_NORMAL,
    .throttle_deadband = 40,
    .throttle_reverse_hysteresis = 50,
    .throttle_reverse_limit = 100,
    .reverse_motor_time = 80,
    .reverse_time_constant = 400,
    .reverse_surface_gain = 100,
    .reverse_fade = 15,
    .reverse_motor_dwell = 0,
    .reverse_coupled_axes = 0,
);

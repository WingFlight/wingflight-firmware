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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "config/config_reset.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "tv_pid.h"


// v0->v1: promoted from a single instance to PID_PROFILE_COUNT independently
// switchable profiles (see currentTvPidProfile/changeTvProfile() in
// config/config.c) -- old single-profile saves reset to defaults rather than
// reinterpreting their stored bytes as profile 0 of the new array.
// v5->v6: default F 100->75 and B 0->35, matching the main loop.
// v6->v7: default P and I raised (roll and pitch P 120, I 60; yaw P 250,
// I 60), matching the main loop. v7->v8: P and I raised by a further 25%
// (roll and pitch P 150, I 75; yaw P 310, I 75), matching the main loop.
// v8->v9: P and I lowered (roll and pitch P 105, I 45; yaw P 190, I 45) and
// F 75 -> 65, matching the main loop. v9->v10: tuned for a nozzle instead of
// copied from the main loop -- see resetTvPidProfile().
PG_REGISTER_ARRAY_WITH_RESET_FN(tvPidProfile_t, PID_PROFILE_COUNT, tvPidProfiles, PG_THRUST_VECTOR_PROFILE, 10);

// Defaults for a vectored nozzle, from prop-hang logs of a single pusher with a
// pitch/yaw nozzle, rather than the main loop's surface defaults:
// - Same gains on every axis: a nozzle is symmetric. The main loop's higher
//   yaw P makes up for a weak rudder, which a nozzle is not.
// - Low F: with no airflow to damp it, a steady nozzle deflection gives a
//   steady rotational acceleration, not a steady rate, so rate feedforward
//   over-drives the hang. P carries the response instead.
// - Small, fast-draining I: the default I parked the nozzle at up to half its
//   travel and took over a second to bleed off after a stick move. TV hold
//   already supplies the long-term correction through its attitude error.
// - A gentler hold gain, so the hold stays well inside the rate loop's response.
void resetTvPidProfile(tvPidProfile_t *tvPidProfile)
{
    RESET_CONFIG(tvPidProfile_t, tvPidProfile,
        .pid = {
            [PID_ROLL]  = { .P = 150, .I = 15, .D = 0, .F = 30, .B = 35, },
            [PID_PITCH] = { .P = 150, .I = 15, .D = 0, .F = 30, .B = 35, },
            [PID_YAW]   = { .P = 150, .I = 15, .D = 0, .F = 30, .B = 35, },
        },
        .master_gain = { [PID_ROLL] = 100, [PID_PITCH] = 100, [PID_YAW] = 100 },
        .gain_curve = { [PID_ROLL] = 0, [PID_PITCH] = 0, [PID_YAW] = 0 },
        .iterm_decay_time = { 30, 30, 30 },
        .iterm_decay_limit = 0,     // no linear cap, so a large I drains exponentially
        .iterm_relax_level = { 22, 22, 22 },
        .iterm_relax = { ITERM_RELAX_DEFAULT, ITERM_RELAX_DEFAULT, ITERM_RELAX_DEFAULT },
        .error_limit = { 25, 25, 25 },
        .dterm_cutoff = { 15, 15, 20 },
        .bterm_cutoff = { 15, 15, 20 },
        .gyro_cutoff = { 50, 50, 100 },
        .hold = { .gain = 25, .deadband = 5, .max_rate = 300 },
    );
}

void pgResetFn_tvPidProfiles(tvPidProfile_t *tvPidProfiles)
{
    for (int i = 0; i < PID_PROFILE_COUNT; i++) {
        resetTvPidProfile(&tvPidProfiles[i]);
    }
}

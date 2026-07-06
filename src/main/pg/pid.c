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

#include "flight/pid.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "pid.h"


PG_REGISTER_WITH_RESET_TEMPLATE(pidConfig_t, pidConfig, PG_PID_CONFIG, 3);

PG_RESET_TEMPLATE(pidConfig_t, pidConfig,
    .pid_process_denom = PID_PROCESS_DENOM_DEFAULT,
    .filter_process_denom = FILTER_PROCESS_DENOM_DEFAULT,
);

PG_REGISTER_ARRAY_WITH_RESET_FN(gainCurve_t, GAIN_CURVE_COUNT, gainCurves, PG_GAIN_CURVES, 0);

void pgResetFn_gainCurves(gainCurve_t *curve)
{
    // Default every curve to a neutral 2-point flat line, so an unconfigured
    // curve slot has no effect if an axis is assigned to it.
    for (int i = 0; i < GAIN_CURVE_COUNT; i++) {
        curve[i].count = 2;
        curve[i].points[0].x = 0;
        curve[i].points[0].y = 100;
        curve[i].points[1].x = 1000;
        curve[i].points[1].y = 100;
    }
}

// v0->v1: added master_gain. v1->v2: master_gain widened from one shared
// value to one per axis. v2->v3: added autohover sub-struct. v3->v4: added
// fixed-wing cross-axis relax settings. v4->v5: added pitch strength for
// cross-axis relax. v5->v6: added gain_curve (index into gainCurves, scales
// master_gain by |stick deflection|) - old saved profiles reset to defaults
// rather than reinterpreting their stored bytes at the new, wider struct layout.
PG_REGISTER_ARRAY_WITH_RESET_FN(pidProfile_t, PID_PROFILE_COUNT, pidProfiles, PG_PID_PROFILE, 6);

void resetPidProfile(pidProfile_t *pidProfile)
{
    RESET_CONFIG(pidProfile_t, pidProfile,
        .profileName = "",
        .pid = {
            [PID_ROLL]  = { .P = 50, .I = 16, .D = 0, .F = 100, .B = 0, },
            [PID_PITCH] = { .P = 50, .I = 16, .D = 0, .F = 100, .B = 0, },
            [PID_YAW]   = { .P = 80, .I = 20, .D = 0, .F = 100, .B = 0, },
        },
        .pid_mode = 1,
        .master_gain = { [PID_ROLL] = 100, [PID_PITCH] = 100, [PID_YAW] = 100 },
        .gain_curve = { [PID_ROLL] = 0, [PID_PITCH] = 0, [PID_YAW] = 0 },
        .fw_tpa_breakpoint = 100,
        .fw_tpa_rate = 0,
        .iterm_decay_time = 6,
        .iterm_decay_limit = 35,
        .iterm_relax_type = ITERM_RELAX_RPY,
        .iterm_relax_level = { 22, 22, 22 },
        .iterm_relax_cutoff = { 10, 10, 10 },
        .error_limit = { 45, 45, 60 },
        .dterm_cutoff = { 15, 15, 20 },
        .bterm_cutoff = { 15, 15, 20 },
        .gyro_cutoff = { 50, 50, 100 },
        .angle.level_strength = 40,
        .angle.level_limit = 55,
        .horizon.level_strength = 40,
        .horizon.transition = 75,
        .horizon.tilt_effect = 75,
        .horizon.tilt_expert_mode = false,
        .trainer.gain = 75,
        .trainer.angle_limit = 20,
        .trainer.lookahead_ms = 50,
        .autohover.gain = 50,
        .autohover.max_angle = 30,
        .autohover.max_rate = 300,
        .cross_axis_relax_strength = 0,
        .cross_axis_relax_level = 100,
        .cross_axis_relax_cutoff = 10,
        .cross_axis_relax_pitch_strength = 0,
    );
}

void pgResetFn_pidProfiles(pidProfile_t *pidProfiles)
{
    for (int i = 0; i < PID_PROFILE_COUNT; i++) {
        resetPidProfile(&pidProfiles[i]);
    }
}

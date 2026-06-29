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

PG_REGISTER_ARRAY_WITH_RESET_FN(pidProfile_t, PID_PROFILE_COUNT, pidProfiles, PG_PID_PROFILE, 0);

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
    );
}

void pgResetFn_pidProfiles(pidProfile_t *pidProfiles)
{
    for (int i = 0; i < PID_PROFILE_COUNT; i++) {
        resetPidProfile(&pidProfiles[i]);
    }
}


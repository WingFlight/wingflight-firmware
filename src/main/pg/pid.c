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
// master_gain by |stick deflection|). v6->v7: added atthold sub-struct
// (reuses the reserved gain/deadband wire slot left by the old atthold
// removal; max_rate is a new field appended at the tail) - old saved
// profiles reset to defaults rather than reinterpreting their stored bytes
// at the new, wider struct layout. v7->v8: replaced fw_tpa_breakpoint/
// fw_tpa_rate with fw_tpa_gain (baseline scale) and fw_tpa_curve (index into
// the shared gainCurves pool), mirroring master_gain/gain_curve. v8->v9:
// master_gain widened from uint8_t to uint16_t per axis to allow values up
// to 1000 (was capped at 255). v9->v10: added autohover.roll_deadband (new
// field appended at the tail of the autohover sub-struct, widening it) - old
// saved profiles reset to defaults rather than reinterpreting their stored
// bytes at the new layout, matching the v6->v7 precedent. v10->v11: added
// autohover.throttle_assist_gain/_max/_trigger_ms (3 new fields appended at
// the tail of the autohover sub-struct, widening it again) - old saved
// profiles reset to defaults, matching the v9->v10 precedent. AUTO HOVER was
// later removed without a version bump: its bytes stay as autohover_reserved.
// v15->v0: default F lowered from 100 to 75 and B raised from 0 to 35 to cut
// bounce back. The version field is 4 bits, so it wraps to 0; equality is
// what pgLoad() checks, and no v0 save survives the EEPROM_CONF_VERSION bumps
// since (174 -> 177), so a stale v0 record cannot be misread. v0->v1:
// default P and I raised (roll and pitch P 120, I 60; yaw P 250, I 60), so
// profiles reset to the new defaults. v1->v2: P and I raised by a further
// 25% (roll and pitch P 150, I 75; yaw P 310, I 75), so the bottom of a 0-200%
// master gain pot reaches a noticeable gain sooner. v2->v3: P and I lowered
// (roll and pitch P 105, I 45; yaw P 190, I 45) and error_limit cut from
// 45/45/60 to 20/20/30: pilots ran the v2 gains at about 80% master gain, and
// above that I wound up on held sticks until the surfaces hit their ends.
PG_REGISTER_ARRAY_WITH_RESET_FN(pidProfile_t, PID_PROFILE_COUNT, pidProfiles, PG_PID_PROFILE, 3);

void resetPidProfile(pidProfile_t *pidProfile)
{
    RESET_CONFIG(pidProfile_t, pidProfile,
        .profileName = "",
        .pid = {
            // Set from flown logs. At 100% master gain, P 105 gives 0.07% of
            // full surface travel per deg/s of rate error, and I 45 builds
            // about 0.9% of full travel per second per deg/s. P 50 / I 16
            // barely moved the surfaces; P 150 / I 75 was flown at about 80%
            // master gain and went wrong above it. Yaw runs hotter.
            [PID_ROLL]  = { .P = 105, .I = 45, .D = 0, .F = 75, .B = 35, },
            [PID_PITCH] = { .P = 105, .I = 45, .D = 0, .F = 75, .B = 35, },
            [PID_YAW]   = { .P = 190, .I = 45, .D = 0, .F = 75, .B = 35, },
        },
        .pid_mode = 1,
        .master_gain = { [PID_ROLL] = 100, [PID_PITCH] = 100, [PID_YAW] = 100 },
        .gain_curve = { [PID_ROLL] = 0, [PID_PITCH] = 0, [PID_YAW] = 0 },
        .fw_tpa_gain = 100,
        .fw_tpa_curve = 0,
        .iterm_decay_time = { 60, 60, 60 },
        .iterm_decay_limit = 35,
        .iterm_relax_level = { 22, 22, 22 },
        .iterm_relax = { ITERM_RELAX_DEFAULT, ITERM_RELAX_DEFAULT, ITERM_RELAX_DEFAULT },
        // Caps I at about 18% of travel on roll and pitch (27% on yaw) at
        // 100% master gain. I-term relax only acts while the stick moves, so
        // on a held stick the airframe cannot follow, I fills to this limit;
        // at 45 it could drive a surface to its end on top of F.
        .error_limit = { 20, 20, 30 },
        .dterm_cutoff = { 15, 15, 20 },
        .bterm_cutoff = { 15, 15, 20 },
        .gyro_cutoff = { 50, 50, 100 },
        .angle.level_strength = 40,
        .angle.level_limit = 55,
        .trainer.gain = 75,
        .trainer.angle_limit = 20,
        .trainer.lookahead_ms = 50,
        .atthold.gain = 40,
        .atthold.deadband = 5,
        .atthold.max_rate = 300,
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

PG_REGISTER_ARRAY(attitudeLimits_t, PID_PROFILE_COUNT, attitudeLimits, PG_ATTITUDE_LIMITS, 0);

PG_REGISTER_ARRAY_WITH_RESET_FN(fwSpaConfig_t, PID_PROFILE_COUNT, fwSpaConfigs, PG_FW_SPA_CONFIG, 0);

void resetFwSpaConfig(fwSpaConfig_t *config)
{
    RESET_CONFIG(fwSpaConfig_t, config,
        .gain = 100,
        .curve = 0,
        .speed_max = FW_SPA_SPEED_MAX_DEFAULT,
    );
}

void pgResetFn_fwSpaConfigs(fwSpaConfig_t *configs)
{
    for (int i = 0; i < PID_PROFILE_COUNT; i++) {
        resetFwSpaConfig(&configs[i]);
    }
}

PG_REGISTER_ARRAY_WITH_RESET_FN(levelConfig_t, PID_PROFILE_COUNT, levelConfigs, PG_LEVEL_CONFIG, 0);

void resetLevelConfig(levelConfig_t *config)
{
    RESET_CONFIG(levelConfig_t, config,
        .damping = 25,
    );
}

void pgResetFn_levelConfigs(levelConfig_t *configs)
{
    for (int i = 0; i < PID_PROFILE_COUNT; i++) {
        resetLevelConfig(&configs[i]);
    }
}

// v0->v1: default hold raised from 150 to 350 ms. Pilots flying pop tops and pinwheels found
// stability came back too abruptly at the end of the snap. This group is separate from
// pidProfile_t, so the bump resets only the snap relax settings.
PG_REGISTER_ARRAY_WITH_RESET_FN(snapRelaxConfig_t, PID_PROFILE_COUNT, snapRelaxConfigs, PG_SNAP_RELAX_CONFIG, 1);

void resetSnapRelaxConfig(snapRelaxConfig_t *config)
{
    RESET_CONFIG(snapRelaxConfig_t, config,
        .strength = 100,
        .threshold = 60,
        .window = 400,
        .hold = 350,
    );
}

void pgResetFn_snapRelaxConfigs(snapRelaxConfig_t *configs)
{
    for (int i = 0; i < PID_PROFILE_COUNT; i++) {
        resetSnapRelaxConfig(&configs[i]);
    }
}

PG_REGISTER_ARRAY_WITH_RESET_FN(propHangConfig_t, PID_PROFILE_COUNT, propHangConfigs, PG_PROP_HANG_CONFIG, 0);

void resetPropHangConfig(propHangConfig_t *config)
{
    RESET_CONFIG(propHangConfig_t, config,
        .strength = 100,
        .angle = 20,
        .fade = 500,
    );
}

void pgResetFn_propHangConfigs(propHangConfig_t *configs)
{
    for (int i = 0; i < PID_PROFILE_COUNT; i++) {
        resetPropHangConfig(&configs[i]);
    }
}

uint8_t attitudeLimitDegrees(uint8_t override, uint8_t legacy, int axis)
{
    if (!override) {
        return legacy;
    }
    const uint8_t maximum = axis == PID_ROLL ? 90 : 75;
    return override < 10 ? 10 : (override > maximum ? maximum : override);
}

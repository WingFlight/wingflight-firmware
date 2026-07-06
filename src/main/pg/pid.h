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

#pragma once

#include <stdbool.h>

#include "common/time.h"
#include "common/axis.h"

#include "pg/pg.h"


typedef struct {
    uint8_t pid_process_denom;
    uint8_t filter_process_denom;
} pidConfig_t;

PG_DECLARE(pidConfig_t, pidConfig);


typedef enum {
    PID_ROLL,
    PID_PITCH,
    PID_YAW,
    PID_ITEM_COUNT
} pidIndex_e;

#define PID_AXIS_COUNT      3
#define CYCLIC_AXIS_COUNT   2

enum {
    ITERM_RELAX_OFF,
    ITERM_RELAX_RP,
    ITERM_RELAX_RPY,
};

typedef struct {
    uint16_t P;
    uint16_t I;
    uint16_t D;
    uint16_t F;
    uint16_t B;
} pidf_t;

typedef struct {
    uint8_t level_strength;
    uint8_t level_limit;           // Max angle in degrees in level mode
} pidAngleMode_t;

typedef struct {
    uint8_t level_strength;
    uint8_t transition;
    uint8_t tilt_effect;           // inclination factor for Horizon mode
    uint8_t tilt_expert_mode;      // OFF or ON
} pidHorizonMode_t;

typedef struct {
    uint8_t gain;                  // The strength of the limiting. Raising may reduce overshoot but also lead to oscillation around the angle limit
    uint8_t angle_limit;           // Acro trainer roll/pitch angle limit in degrees
    uint16_t lookahead_ms;         // The lookahead window in milliseconds used to reduce overshoot
} pidTrainerMode_t;

typedef struct {
    uint8_t  gain;                 // Correction strength back to the held vertical attitude/heading
    uint8_t  max_angle;            // Max degrees the stick may deflect the target off vertical/held heading
    uint16_t max_rate;             // deg/s clamp on the commanded attitude-capture rate (safety limit)
} pidAutoHoverMode_t;

#define MAX_PROFILE_NAME_LENGTH 8u

#define GAIN_CURVE_COUNT   8
#define GAIN_CURVE_POINTS  6

typedef struct {
    uint16_t x;   // 0..1000, |stick deflection| * 1000
    uint16_t y;   // 0..500, percent multiplier (100 = unscaled, matches master_gain's own scale)
} gainCurvePoint_t;

typedef struct {
    uint8_t          count;                      // 0 (disabled) or 2..GAIN_CURVE_POINTS
    gainCurvePoint_t points[GAIN_CURVE_POINTS];   // ascending by x
} gainCurve_t;

PG_DECLARE_ARRAY(gainCurve_t, GAIN_CURVE_COUNT, gainCurves);

typedef struct pidProfile_s {

    char                profileName[MAX_PROFILE_NAME_LENGTH + 1];

    pidf_t              pid[PID_ITEM_COUNT];

    uint8_t             pid_mode;

    uint8_t             master_gain[PID_AXIS_COUNT];  // Live per-axis P/I/D/F scale, percent (100 = unscaled) - in-flight tuning aid, doesn't alter the underlying gains
    uint8_t             gain_curve[PID_AXIS_COUNT];   // 0=none, 1..GAIN_CURVE_COUNT = gainCurves(idx-1), scales master_gain by |stick deflection|

    uint8_t             fw_tpa_breakpoint;
    uint8_t             fw_tpa_rate;

    uint8_t             iterm_decay_time;
    uint8_t             iterm_decay_limit;

    uint8_t             iterm_relax_type;
    uint8_t             iterm_relax_level[PID_AXIS_COUNT];
    uint8_t             iterm_relax_cutoff[PID_AXIS_COUNT];

    uint8_t             error_limit[PID_AXIS_COUNT];

    uint8_t             dterm_cutoff[PID_AXIS_COUNT];
    uint8_t             bterm_cutoff[PID_AXIS_COUNT];
    uint8_t             gyro_cutoff[PID_AXIS_COUNT];

    pidAngleMode_t      angle;
    pidHorizonMode_t    horizon;
    pidTrainerMode_t    trainer;
    pidAutoHoverMode_t  autohover;

    uint8_t             cross_axis_relax_strength; // Percent max roll feedback attenuation from yaw setpoint activity
    uint8_t             cross_axis_relax_level;    // Yaw setpoint level where max attenuation is reached
    uint8_t             cross_axis_relax_cutoff;   // Hz smoothing cutoff for yaw setpoint detector
    uint8_t             cross_axis_relax_pitch_strength; // Percent max pitch feedback attenuation from yaw setpoint activity

} pidProfile_t;

PG_DECLARE_ARRAY(pidProfile_t, PID_PROFILE_COUNT, pidProfiles);

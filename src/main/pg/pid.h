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

// HORIZON removed: kept as 4 reserved bytes so pidProfile_t keeps its layout (PG_PID_PROFILE v15)
typedef struct {
    uint8_t reserved[4];
} pidHorizonReserved_t;

typedef struct {
    uint8_t gain;                  // The strength of the limiting. Raising may reduce overshoot but also lead to oscillation around the angle limit
    uint8_t angle_limit;           // Acro trainer roll/pitch angle limit in degrees
    uint16_t lookahead_ms;         // The lookahead window in milliseconds used to reduce overshoot
} pidTrainerMode_t;

// Reserved: the removed AUTO HOVER settings occupied 10 bytes (2-byte aligned) here. Kept at the
// same size and alignment so saved profiles still load under the same PG version. Never read.
typedef struct {
    uint16_t reserved[5];
} pidAutoHoverReserved_t;

typedef struct {
    uint8_t  gain;                 // Correction strength back to the held (frozen) attitude
    uint8_t  deadband;             // Percent stick deflection below which the hold freezes and corrects;
                                    // above which the target free-tracks current attitude (no correction)
    uint16_t max_rate;             // deg/s clamp on the commanded correction rate (safety limit)
} pidAttHoldMode_t;

#define MAX_PROFILE_NAME_LENGTH 8u

// iterm_decay_time range, in 0.01 s. Capped at 1 s so the rate loop keeps rate-mode feel (longer
// memory is what ATTHOLD is for), and no 0 = "decay off", which would silently be the most locked
// setting at the bottom of the scale.
#define ITERM_DECAY_TIME_MIN    1
#define ITERM_DECAY_TIME_MAX    100

// I-term relax, a 1-10 score per axis (higher = more relax, less bounce-back after a fast stick
// move). pidItermRelaxCutoff() turns it into the I-term relax setpoint filter cutoff. The score,
// not the Hz, is what pilots, the CLI, MSP and the adjustment functions see. Relax is always on.
#define ITERM_RELAX_MIN      1
#define ITERM_RELAX_MAX      10
#define ITERM_RELAX_DEFAULT  5

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

    uint16_t            master_gain[PID_AXIS_COUNT]; // Live per-axis P/I/D scale, percent (100 = unscaled) - in-flight tuning aid, doesn't alter the underlying gains
    uint8_t             gain_curve[PID_AXIS_COUNT];   // 0=none, 1..GAIN_CURVE_COUNT = gainCurves(idx-1), scales master_gain by |stick deflection|

    uint8_t             fw_tpa_gain;                  // Baseline throttle attenuation scale, percent (100 = unscaled) - mirrors master_gain
    uint8_t             fw_tpa_curve;                 // 0=none, 1..GAIN_CURVE_COUNT = gainCurves(idx-1), further scales fw_tpa_gain by throttle - mirrors gain_curve

    uint8_t             iterm_decay_time[PID_AXIS_COUNT]; // Per-axis I-term decay time constant, 0.01 s (ITERM_DECAY_TIME_MIN..MAX)
    uint8_t             iterm_decay_limit;

    uint8_t             iterm_relax_level[PID_AXIS_COUNT];
    uint8_t             iterm_relax[PID_AXIS_COUNT];   // I-term relax score, ITERM_RELAX_MIN..MAX

    uint8_t             error_limit[PID_AXIS_COUNT];

    uint8_t             dterm_cutoff[PID_AXIS_COUNT];
    uint8_t             bterm_cutoff[PID_AXIS_COUNT];
    uint8_t             gyro_cutoff[PID_AXIS_COUNT];

    pidAngleMode_t      angle;
    pidHorizonReserved_t horizon_reserved;
    pidTrainerMode_t    trainer;
    pidAutoHoverReserved_t autohover_reserved;
    pidAttHoldMode_t    atthold;

    uint8_t             cross_axis_relax_strength; // Percent max roll feedback attenuation from yaw setpoint activity
    uint8_t             cross_axis_relax_level;    // Yaw setpoint level where max attenuation is reached
    uint8_t             cross_axis_relax_cutoff;   // Hz smoothing cutoff for yaw setpoint detector
    uint8_t             cross_axis_relax_pitch_strength; // Percent max pitch feedback attenuation from yaw setpoint activity

} pidProfile_t;

PG_DECLARE_ARRAY(pidProfile_t, PID_PROFILE_COUNT, pidProfiles);

// Separate storage preserves existing PID profiles. Zero inherits the legacy shared limit.
typedef struct {
    uint8_t angle_roll;
    uint8_t angle_pitch;
    uint8_t trainer_roll;
    uint8_t trainer_pitch;
} attitudeLimits_t;

PG_DECLARE_ARRAY(attitudeLimits_t, PID_PROFILE_COUNT, attitudeLimits);

// GPS speed attenuation (SPA), one per PID profile. Separate storage, like
// attitudeLimits_t, so adding it did not reset existing PID profiles.
// Mirrors fw_tpa_gain/fw_tpa_curve with GPS speed in place of throttle.
#define FW_SPA_SPEED_MAX_MIN      10
#define FW_SPA_SPEED_MAX_MAX      600
#define FW_SPA_SPEED_MAX_DEFAULT  150

typedef struct {
    uint8_t  gain;       // Baseline scale, percent (100 = unscaled) - mirrors fw_tpa_gain
    uint8_t  curve;      // 0=none (SPA off), 1..GAIN_CURVE_COUNT = gainCurves(idx-1), scales gain by GPS speed
    uint16_t speed_max;  // GPS speed, km/h, at the curve's right edge (x = 1000)
} fwSpaConfig_t;

PG_DECLARE_ARRAY(fwSpaConfig_t, PID_PROFILE_COUNT, fwSpaConfigs);

void resetFwSpaConfig(fwSpaConfig_t *config);

// ANGLE mode settings added after the pidProfile_t layout was fixed, one per PID
// profile. Separate storage, like fwSpaConfig_t, so adding it did not reset
// existing PID profiles.
#define LEVEL_DAMPING_MAX         100

typedef struct {
    uint8_t damping;     // Percent of measured roll/pitch rate subtracted from the level rate command
} levelConfig_t;

PG_DECLARE_ARRAY(levelConfig_t, PID_PROFILE_COUNT, levelConfigs);

void resetLevelConfig(levelConfig_t *config);

// Positive axis overrides use the SAFE-style range; zero preserves the legacy shared value.
uint8_t attitudeLimitDegrees(uint8_t override, uint8_t legacy, int axis);

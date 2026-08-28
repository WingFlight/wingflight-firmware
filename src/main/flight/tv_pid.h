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

#include "common/time.h"

#include "pg/tv_pid.h"
#include "pg/adjustments.h"

// Independent Thrust Vector rate-PID loop (FEATURE_THRUST_VECTOR). Targets the
// same setpoint as the main roll/pitch/yaw loop but with its own gains/filters/
// iterm state, feeding the MIXER_IN_STABILIZED_TV_ROLL/PITCH/YAW mixer inputs.

// Kept as its own type (not flight/pid.h's pidAxisData_t) so this loop stays
// textually independent of the main loop -- see the "not sharing state"
// comment in tv_pid.c. Exposed here (rather than staying private to the .c)
// so blackbox.c can log it the same way it logs the main loop's pidData[].
typedef struct {
    float P;
    float I;
    float D;
    float F;
    float B;
    float pidSum;
    float axisError;
} tvPidAxisData_t;

void tvPidInit(const tvPidProfile_t *profile);
void tvPidLoadProfile(const tvPidProfile_t *profile);
void tvPidReset(void);

void tvPidController(timeUs_t currentTimeUs);

float tvPidGetOutput(int axis);
const tvPidAxisData_t * tvPidGetAxisData(void);

ADJFUN_DECLARE(TV_MASTER_GAIN_ROLL)
ADJFUN_DECLARE(TV_MASTER_GAIN_PITCH)
ADJFUN_DECLARE(TV_MASTER_GAIN_YAW)
ADJFUN_DECLARE(TV_ROLL_P_GAIN)
ADJFUN_DECLARE(TV_ROLL_I_GAIN)
ADJFUN_DECLARE(TV_ROLL_D_GAIN)
ADJFUN_DECLARE(TV_ROLL_F_GAIN)
ADJFUN_DECLARE(TV_ROLL_B_GAIN)
ADJFUN_DECLARE(TV_PITCH_P_GAIN)
ADJFUN_DECLARE(TV_PITCH_I_GAIN)
ADJFUN_DECLARE(TV_PITCH_D_GAIN)
ADJFUN_DECLARE(TV_PITCH_F_GAIN)
ADJFUN_DECLARE(TV_PITCH_B_GAIN)
ADJFUN_DECLARE(TV_YAW_P_GAIN)
ADJFUN_DECLARE(TV_YAW_I_GAIN)
ADJFUN_DECLARE(TV_YAW_D_GAIN)
ADJFUN_DECLARE(TV_YAW_F_GAIN)
ADJFUN_DECLARE(TV_YAW_B_GAIN)

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
#include "common/filter.h"
#include "common/axis.h"

#include "pg/pid.h"
#include "pg/adjustments.h"


#define PID_PROCESS_DENOM_DEFAULT   8
#define MAX_PID_PROCESS_DENOM       16

#define FILTER_PROCESS_DENOM_DEFAULT 0

#define PID_GAIN_MAX                1000

#define ROLL_P_TERM_SCALE           0.00000666666f
#define ROLL_I_TERM_SCALE           0.0002f
#define ROLL_D_TERM_SCALE           0.1e-6f
#define ROLL_F_TERM_SCALE           0.000025f
#define ROLL_B_TERM_SCALE           0.1e-6f

#define PITCH_P_TERM_SCALE          0.00000666666f
#define PITCH_I_TERM_SCALE          0.0002f
#define PITCH_D_TERM_SCALE          1.0e-6f
#define PITCH_F_TERM_SCALE          0.000025f
#define PITCH_B_TERM_SCALE          0.1e-6f

#define YAW_P_TERM_SCALE            0.00006666666f
#define YAW_I_TERM_SCALE            0.0005f
#define YAW_D_TERM_SCALE            1.0e-6f
#define YAW_F_TERM_SCALE            0.000025f
#define YAW_B_TERM_SCALE            1.0e-6f

typedef struct {
    float P;
    float I;
    float D;
    float F;
    float B;
    float pidSum;
    float setPoint;
    float gyroRate;
    float axisError;
} pidAxisData_t;

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float Kf;
    float Kb;
} pidAxisCoef_t;

typedef struct pid_s {
    float dT;
    float freq;

    uint8_t pidMode;

    float masterGain[PID_AXIS_COUNT];  // Live per-axis P/I/D/F scale (1.0 = unscaled) - see pidProfile_t.master_gain
    uint8_t gainCurveIndex[PID_AXIS_COUNT];  // 0=none, 1..GAIN_CURVE_COUNT = gainCurves(idx-1) - see pidProfile_t.gain_curve

    float fwTpaBreakpoint;
    float fwTpaRate;

    uint8_t itermRelaxType;
    uint8_t itermRelaxLevel[PID_AXIS_COUNT];
    float crossAxisRelaxStrength;
    float crossAxisRelaxPitchStrength;
    uint8_t crossAxisRelaxLevel;
    float crossAxisRelaxYawActivity;

    float itermDecayRate;
    float itermDecayLimit;

    float errorLimit[PID_AXIS_COUNT];

    pidAxisCoef_t coef[PID_ITEM_COUNT];
    pidAxisData_t data[PID_AXIS_COUNT];

    filter_t gyrorFilter[PID_AXIS_COUNT];

    pt1Filter_t relaxFilter[PID_AXIS_COUNT];
    pt1Filter_t crossAxisRelaxFilter;

    difFilter_t dtermFilter[PID_AXIS_COUNT];
    difFilter_t btermFilter[PID_AXIS_COUNT];

} pidData_t;


void pidController(const pidProfile_t *pidProfile, timeUs_t currentTimeUs);

void resetPidProfile(pidProfile_t *profile);

void pidResetAxisErrors(void);
void pidResetAxisError(int axis);

void pidInit(const pidProfile_t *pidProfile);
void pidLoadProfile(const pidProfile_t *pidProfile);
void pidChangeProfile(const pidProfile_t *pidProfile);

void pidCopyProfile(uint8_t dstPidProfileIndex, uint8_t srcPidProfileIndex);

float pidGetDT(void);
float pidGetPidFrequency(void);

float pidGetSetpoint(int axis);
float pidGetOutput(int axis);

const pidAxisData_t * pidGetAxisData(void);

ADJFUN_DECLARE(PID_PROFILE)
ADJFUN_DECLARE(MASTER_GAIN_PITCH)
ADJFUN_DECLARE(MASTER_GAIN_ROLL)
ADJFUN_DECLARE(MASTER_GAIN_YAW)
ADJFUN_DECLARE(PITCH_P_GAIN)
ADJFUN_DECLARE(ROLL_P_GAIN)
ADJFUN_DECLARE(YAW_P_GAIN)
ADJFUN_DECLARE(PITCH_I_GAIN)
ADJFUN_DECLARE(ROLL_I_GAIN)
ADJFUN_DECLARE(YAW_I_GAIN)
ADJFUN_DECLARE(PITCH_D_GAIN)
ADJFUN_DECLARE(ROLL_D_GAIN)
ADJFUN_DECLARE(YAW_D_GAIN)
ADJFUN_DECLARE(PITCH_F_GAIN)
ADJFUN_DECLARE(ROLL_F_GAIN)
ADJFUN_DECLARE(YAW_F_GAIN)
ADJFUN_DECLARE(PITCH_GYRO_CUTOFF)
ADJFUN_DECLARE(ROLL_GYRO_CUTOFF)
ADJFUN_DECLARE(YAW_GYRO_CUTOFF)
ADJFUN_DECLARE(PITCH_DTERM_CUTOFF)
ADJFUN_DECLARE(ROLL_DTERM_CUTOFF)
ADJFUN_DECLARE(YAW_DTERM_CUTOFF)
ADJFUN_DECLARE(PITCH_B_GAIN)
ADJFUN_DECLARE(ROLL_B_GAIN)
ADJFUN_DECLARE(YAW_B_GAIN)

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
#include <math.h>

#include "platform.h"

#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"

#include "pg/tv_pid.h"

#include "sensors/gyro.h"

#include "flight/pid.h"
#include "flight/mixer.h"

#include "tv_pid.h"

// Deliberately not sharing state/statics with flight/pid.c: the two loops stay
// textually independent so either can be tuned, debugged, or removed without
// touching the other.

typedef struct {
    float P;
    float I;
    float D;
    float F;
    float B;
    float pidSum;
    float axisError;
} tvPidAxisData_t;

typedef struct {
    pidAxisCoef_t coef[PID_AXIS_COUNT];
    tvPidAxisData_t data[PID_AXIS_COUNT];

    float masterGain[PID_AXIS_COUNT]; // Live per-axis P/I/D/F scale (1.0 = unscaled) - see tvPidProfile_t.master_gain

    uint8_t itermRelaxType;
    uint8_t itermRelaxLevel[PID_AXIS_COUNT];

    float itermDecayRate;
    float itermDecayLimit;

    float errorLimit[PID_AXIS_COUNT];

    filter_t gyrorFilter[PID_AXIS_COUNT];
    pt1Filter_t relaxFilter[PID_AXIS_COUNT];
    difFilter_t dtermFilter[PID_AXIS_COUNT];
    difFilter_t btermFilter[PID_AXIS_COUNT];
} tvPidData_t;

static FAST_DATA_ZERO_INIT tvPidData_t tvPid;


float tvPidGetOutput(int axis)
{
    return tvPid.data[axis].pidSum;
}

void tvPidReset(void)
{
    memset(tvPid.data, 0, sizeof(tvPid.data));
}


//// Adjustment functions

int get_ADJUSTMENT_TV_MASTER_GAIN_ROLL(void)
{
    return tvPidProfile()->master_gain[PID_ROLL];
}

void set_ADJUSTMENT_TV_MASTER_GAIN_ROLL(int value)
{
    tvPidProfileMutable()->master_gain[PID_ROLL] = value;
    tvPid.masterGain[PID_ROLL] = value * 0.01f;
}

int get_ADJUSTMENT_TV_MASTER_GAIN_PITCH(void)
{
    return tvPidProfile()->master_gain[PID_PITCH];
}

void set_ADJUSTMENT_TV_MASTER_GAIN_PITCH(int value)
{
    tvPidProfileMutable()->master_gain[PID_PITCH] = value;
    tvPid.masterGain[PID_PITCH] = value * 0.01f;
}

int get_ADJUSTMENT_TV_MASTER_GAIN_YAW(void)
{
    return tvPidProfile()->master_gain[PID_YAW];
}

void set_ADJUSTMENT_TV_MASTER_GAIN_YAW(int value)
{
    tvPidProfileMutable()->master_gain[PID_YAW] = value;
    tvPid.masterGain[PID_YAW] = value * 0.01f;
}


int get_ADJUSTMENT_TV_ROLL_P_GAIN(void)
{
    return tvPidProfile()->pid[PID_ROLL].P;
}

void set_ADJUSTMENT_TV_ROLL_P_GAIN(int value)
{
    tvPidProfileMutable()->pid[PID_ROLL].P = value;
    tvPid.coef[PID_ROLL].Kp = ROLL_P_TERM_SCALE * value;
}

int get_ADJUSTMENT_TV_ROLL_I_GAIN(void)
{
    return tvPidProfile()->pid[PID_ROLL].I;
}

void set_ADJUSTMENT_TV_ROLL_I_GAIN(int value)
{
    tvPidProfileMutable()->pid[PID_ROLL].I = value;
    tvPid.coef[PID_ROLL].Ki = ROLL_I_TERM_SCALE * value;
}

int get_ADJUSTMENT_TV_ROLL_D_GAIN(void)
{
    return tvPidProfile()->pid[PID_ROLL].D;
}

void set_ADJUSTMENT_TV_ROLL_D_GAIN(int value)
{
    tvPidProfileMutable()->pid[PID_ROLL].D = value;
    tvPid.coef[PID_ROLL].Kd = ROLL_D_TERM_SCALE * value;
}

int get_ADJUSTMENT_TV_ROLL_F_GAIN(void)
{
    return tvPidProfile()->pid[PID_ROLL].F;
}

void set_ADJUSTMENT_TV_ROLL_F_GAIN(int value)
{
    tvPidProfileMutable()->pid[PID_ROLL].F = value;
    tvPid.coef[PID_ROLL].Kf = ROLL_F_TERM_SCALE * value;
}

int get_ADJUSTMENT_TV_ROLL_B_GAIN(void)
{
    return tvPidProfile()->pid[PID_ROLL].B;
}

void set_ADJUSTMENT_TV_ROLL_B_GAIN(int value)
{
    tvPidProfileMutable()->pid[PID_ROLL].B = value;
    tvPid.coef[PID_ROLL].Kb = ROLL_B_TERM_SCALE * value;
}


int get_ADJUSTMENT_TV_PITCH_P_GAIN(void)
{
    return tvPidProfile()->pid[PID_PITCH].P;
}

void set_ADJUSTMENT_TV_PITCH_P_GAIN(int value)
{
    tvPidProfileMutable()->pid[PID_PITCH].P = value;
    tvPid.coef[PID_PITCH].Kp = PITCH_P_TERM_SCALE * value;
}

int get_ADJUSTMENT_TV_PITCH_I_GAIN(void)
{
    return tvPidProfile()->pid[PID_PITCH].I;
}

void set_ADJUSTMENT_TV_PITCH_I_GAIN(int value)
{
    tvPidProfileMutable()->pid[PID_PITCH].I = value;
    tvPid.coef[PID_PITCH].Ki = PITCH_I_TERM_SCALE * value;
}

int get_ADJUSTMENT_TV_PITCH_D_GAIN(void)
{
    return tvPidProfile()->pid[PID_PITCH].D;
}

void set_ADJUSTMENT_TV_PITCH_D_GAIN(int value)
{
    tvPidProfileMutable()->pid[PID_PITCH].D = value;
    tvPid.coef[PID_PITCH].Kd = PITCH_D_TERM_SCALE * value;
}

int get_ADJUSTMENT_TV_PITCH_F_GAIN(void)
{
    return tvPidProfile()->pid[PID_PITCH].F;
}

void set_ADJUSTMENT_TV_PITCH_F_GAIN(int value)
{
    tvPidProfileMutable()->pid[PID_PITCH].F = value;
    tvPid.coef[PID_PITCH].Kf = PITCH_F_TERM_SCALE * value;
}

int get_ADJUSTMENT_TV_PITCH_B_GAIN(void)
{
    return tvPidProfile()->pid[PID_PITCH].B;
}

void set_ADJUSTMENT_TV_PITCH_B_GAIN(int value)
{
    tvPidProfileMutable()->pid[PID_PITCH].B = value;
    tvPid.coef[PID_PITCH].Kb = PITCH_B_TERM_SCALE * value;
}


int get_ADJUSTMENT_TV_YAW_P_GAIN(void)
{
    return tvPidProfile()->pid[PID_YAW].P;
}

void set_ADJUSTMENT_TV_YAW_P_GAIN(int value)
{
    tvPidProfileMutable()->pid[PID_YAW].P = value;
    tvPid.coef[PID_YAW].Kp = YAW_P_TERM_SCALE * value;
}

int get_ADJUSTMENT_TV_YAW_I_GAIN(void)
{
    return tvPidProfile()->pid[PID_YAW].I;
}

void set_ADJUSTMENT_TV_YAW_I_GAIN(int value)
{
    tvPidProfileMutable()->pid[PID_YAW].I = value;
    tvPid.coef[PID_YAW].Ki = YAW_I_TERM_SCALE * value;
}

int get_ADJUSTMENT_TV_YAW_D_GAIN(void)
{
    return tvPidProfile()->pid[PID_YAW].D;
}

void set_ADJUSTMENT_TV_YAW_D_GAIN(int value)
{
    tvPidProfileMutable()->pid[PID_YAW].D = value;
    tvPid.coef[PID_YAW].Kd = YAW_D_TERM_SCALE * value;
}

int get_ADJUSTMENT_TV_YAW_F_GAIN(void)
{
    return tvPidProfile()->pid[PID_YAW].F;
}

void set_ADJUSTMENT_TV_YAW_F_GAIN(int value)
{
    tvPidProfileMutable()->pid[PID_YAW].F = value;
    tvPid.coef[PID_YAW].Kf = YAW_F_TERM_SCALE * value;
}

int get_ADJUSTMENT_TV_YAW_B_GAIN(void)
{
    return tvPidProfile()->pid[PID_YAW].B;
}

void set_ADJUSTMENT_TV_YAW_B_GAIN(int value)
{
    tvPidProfileMutable()->pid[PID_YAW].B = value;
    tvPid.coef[PID_YAW].Kb = YAW_B_TERM_SCALE * value;
}


static void tvPidInitFilters(const tvPidProfile_t *profile)
{
    const float freq = pidGetPidFrequency();

    for (int i = 0; i < PID_AXIS_COUNT; i++) {
        lowpassFilterInit(&tvPid.gyrorFilter[i], LPF_1ST_ORDER, profile->gyro_cutoff[i], freq, LPF_UPDATE);
        difFilterInit(&tvPid.dtermFilter[i], profile->dterm_cutoff[i], freq);
        difFilterInit(&tvPid.btermFilter[i], profile->bterm_cutoff[i], freq);
        pt1FilterInit(&tvPid.relaxFilter[i], 1, freq);
    }
}

void tvPidLoadProfile(const tvPidProfile_t *profile)
{
    // PID not initialised yet
    if (pidGetPidFrequency() <= 0)
        return;

    const float freq = pidGetPidFrequency();

    // Live per-axis P/I/D/F scale - applied at the point of use
    // (tvPidApplyAxis), not baked into tvPid.coef[], so it stays correct
    // regardless of which gain adjustment (including this one) last touched
    // the coefficients.
    for (int i = 0; i < PID_AXIS_COUNT; i++)
        tvPid.masterGain[i] = profile->master_gain[i] * 0.01f;

    tvPid.coef[PID_ROLL].Kp = ROLL_P_TERM_SCALE * profile->pid[PID_ROLL].P;
    tvPid.coef[PID_ROLL].Ki = ROLL_I_TERM_SCALE * profile->pid[PID_ROLL].I;
    tvPid.coef[PID_ROLL].Kd = ROLL_D_TERM_SCALE * profile->pid[PID_ROLL].D;
    tvPid.coef[PID_ROLL].Kf = ROLL_F_TERM_SCALE * profile->pid[PID_ROLL].F;
    tvPid.coef[PID_ROLL].Kb = ROLL_B_TERM_SCALE * profile->pid[PID_ROLL].B;

    tvPid.coef[PID_PITCH].Kp = PITCH_P_TERM_SCALE * profile->pid[PID_PITCH].P;
    tvPid.coef[PID_PITCH].Ki = PITCH_I_TERM_SCALE * profile->pid[PID_PITCH].I;
    tvPid.coef[PID_PITCH].Kd = PITCH_D_TERM_SCALE * profile->pid[PID_PITCH].D;
    tvPid.coef[PID_PITCH].Kf = PITCH_F_TERM_SCALE * profile->pid[PID_PITCH].F;
    tvPid.coef[PID_PITCH].Kb = PITCH_B_TERM_SCALE * profile->pid[PID_PITCH].B;

    tvPid.coef[PID_YAW].Kp = YAW_P_TERM_SCALE * profile->pid[PID_YAW].P;
    tvPid.coef[PID_YAW].Ki = YAW_I_TERM_SCALE * profile->pid[PID_YAW].I;
    tvPid.coef[PID_YAW].Kd = YAW_D_TERM_SCALE * profile->pid[PID_YAW].D;
    tvPid.coef[PID_YAW].Kf = YAW_F_TERM_SCALE * profile->pid[PID_YAW].F;
    tvPid.coef[PID_YAW].Kb = YAW_B_TERM_SCALE * profile->pid[PID_YAW].B;

    for (int i = 0; i < PID_AXIS_COUNT; i++)
        tvPid.errorLimit[i] = profile->error_limit[i];

    // Exponential I-term decay rate
    tvPid.itermDecayRate = (profile->iterm_decay_time) ? (10.0f / profile->iterm_decay_time) : 0;

    // Max I-term decay speed in degs/s (linear decay)
    tvPid.itermDecayLimit = (profile->iterm_decay_limit) ? profile->iterm_decay_limit : 3600;

    for (int i = 0; i < PID_AXIS_COUNT; i++) {
        filterUpdate(&tvPid.gyrorFilter[i], profile->gyro_cutoff[i], freq);
        difFilterUpdate(&tvPid.dtermFilter[i], profile->dterm_cutoff[i], freq);
        difFilterUpdate(&tvPid.btermFilter[i], profile->bterm_cutoff[i], freq);
    }

    tvPid.itermRelaxType = profile->iterm_relax_type;
    if (tvPid.itermRelaxType) {
        for (int i = 0; i < PID_AXIS_COUNT; i++) {
            const uint8_t cutoff = constrain(profile->iterm_relax_cutoff[i], 1, 100);
            pt1FilterUpdate(&tvPid.relaxFilter[i], cutoff, freq);
            tvPid.itermRelaxLevel[i] = constrain(profile->iterm_relax_level[i], 10, 250);
        }
    }
}

void tvPidInit(const tvPidProfile_t *profile)
{
    tvPidReset();
    tvPidInitFilters(profile);
    tvPidLoadProfile(profile);
}


static float tvApplyItermRelax(int axis, float itermError, float setpoint)
{
    if ((tvPid.itermRelaxType == ITERM_RELAX_RPY) ||
        (tvPid.itermRelaxType == ITERM_RELAX_RP && axis == PID_ROLL) ||
        (tvPid.itermRelaxType == ITERM_RELAX_RP && axis == PID_PITCH))
    {
        const float setpointLpf = pt1FilterApply(&tvPid.relaxFilter[axis], setpoint);
        const float setpointHpf = setpoint - setpointLpf;
        const float itermRelaxFactor = MAX(0, 1.0f - fabsf(setpointHpf) / tvPid.itermRelaxLevel[axis]);

        itermError *= itermRelaxFactor;
    }

    return itermError;
}

static void tvPidApplyAxis(uint8_t axis)
{
    // Reuse this tick's final main-loop setpoint (post rates-curve, post any
    // leveling-mode shaping). Only correct because tvPidController() is called
    // after pidController() within the same TASK_PID tick -- see fc/core.c.
    const float setpoint = pidGetSetpoint(axis);

    // Gyro rate through the TV loop's own filter, independent of the main loop
    const float gyroRate = filterApply(&tvPid.gyrorFilter[axis], gyro.gyroADCf[axis]);

    const float errorRate = setpoint - gyroRate;

    const float masterGain = tvPid.masterGain[axis];

  //// P-term

    tvPid.data[axis].P = tvPid.coef[axis].Kp * masterGain * errorRate;

  //// D-term (gyro only)

    const float dTerm = difFilterApply(&tvPid.dtermFilter[axis], -gyroRate);
    tvPid.data[axis].D = tvPid.coef[axis].Kd * masterGain * dTerm;

  //// I-term

    const float itermErrorRate = tvApplyItermRelax(axis, errorRate, setpoint);

    const bool saturation = (tvPidAxisSaturated(axis) && tvPid.data[axis].axisError * itermErrorRate > 0);
    const float itermDelta = saturation ? 0 : itermErrorRate * pidGetDT();

    tvPid.data[axis].axisError = limitf(tvPid.data[axis].axisError + itermDelta, tvPid.errorLimit[axis]);
    tvPid.data[axis].I = tvPid.coef[axis].Ki * masterGain * tvPid.data[axis].axisError;

    const float errorDecay = limitf(tvPid.data[axis].axisError * tvPid.itermDecayRate, tvPid.itermDecayLimit);
    tvPid.data[axis].axisError -= errorDecay * pidGetDT();

  //// Feedforward

    tvPid.data[axis].F = tvPid.coef[axis].Kf * masterGain * setpoint;

  //// Feedforward Boost (FF Derivative)

    const float bTerm = difFilterApply(&tvPid.btermFilter[axis], setpoint);
    tvPid.data[axis].B = tvPid.coef[axis].Kb * bTerm;

  //// PID Sum

    tvPid.data[axis].pidSum = tvPid.data[axis].P + tvPid.data[axis].I + tvPid.data[axis].D +
                               tvPid.data[axis].F + tvPid.data[axis].B;
}

void tvPidController(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    tvPidApplyAxis(PID_ROLL);
    tvPidApplyAxis(PID_PITCH);
    tvPidApplyAxis(PID_YAW);
}

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

#include "build/build_config.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/filter.h"

#include "config/config_reset.h"

#include "pg/pg.h"
#include "pg/pid.h"
#include "pg/adjustments.h"
#include "pg/pg_ids.h"

#include "drivers/pwm_output.h"
#include "drivers/dshot_command.h"
#include "drivers/sound_beeper.h"
#include "drivers/time.h"

#include "sensors/acceleration.h"
#include "sensors/battery.h"
#include "sensors/gyro.h"

#include "fc/core.h"
#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "fc/rc_rates.h"
#include "fc/runtime_config.h"

#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/trainer.h"
#include "flight/leveling.h"
#include "flight/autohover.h"
#include "flight/atthold.h"
#include "flight/rpm_filter.h"

#include "pid.h"

static FAST_DATA_ZERO_INIT pidData_t pid;


//// Access functions

float pidGetDT(void)
{
    return pid.dT;
}

float pidGetPidFrequency(void)
{
    return pid.freq;
}

float pidGetSetpoint(int axis)
{
    return pid.data[axis].setPoint;
}

float pidGetOutput(int axis)
{
    return pid.data[axis].pidSum;
}

const pidAxisData_t * pidGetAxisData(void)
{
    return pid.data;
}

void INIT_CODE pidReset(void)
{
    memset(pid.data, 0, sizeof(pid.data));
}

void INIT_CODE pidResetAxisError(int axis)
{
    pid.data[axis].I = 0;
    pid.data[axis].axisError = 0;
}

void INIT_CODE pidResetAxisErrors(void)
{
    for (int axis = 0; axis < 3; axis++) {
        pid.data[axis].I = 0;
        pid.data[axis].axisError = 0;
    }
}


//// Adjustment functions

int get_ADJUSTMENT_PID_PROFILE(void)
{
    return getCurrentPidProfileIndex() + 1;
}

void set_ADJUSTMENT_PID_PROFILE(int value)
{
    changePidProfile(value - 1);
}

int get_ADJUSTMENT_MASTER_GAIN_PITCH(void)
{
    return currentPidProfile->master_gain[PID_PITCH];
}

void set_ADJUSTMENT_MASTER_GAIN_PITCH(int value)
{
    currentPidProfile->master_gain[PID_PITCH] = value;
    pid.masterGain[PID_PITCH] = value * 0.01f;
}

int get_ADJUSTMENT_MASTER_GAIN_ROLL(void)
{
    return currentPidProfile->master_gain[PID_ROLL];
}

void set_ADJUSTMENT_MASTER_GAIN_ROLL(int value)
{
    currentPidProfile->master_gain[PID_ROLL] = value;
    pid.masterGain[PID_ROLL] = value * 0.01f;
}

int get_ADJUSTMENT_MASTER_GAIN_YAW(void)
{
    return currentPidProfile->master_gain[PID_YAW];
}

void set_ADJUSTMENT_MASTER_GAIN_YAW(int value)
{
    currentPidProfile->master_gain[PID_YAW] = value;
    pid.masterGain[PID_YAW] = value * 0.01f;
}

int get_ADJUSTMENT_PITCH_P_GAIN(void)
{
    return currentPidProfile->pid[PID_PITCH].P;
}

void set_ADJUSTMENT_PITCH_P_GAIN(int value)
{
    currentPidProfile->pid[PID_PITCH].P = value;
    pid.coef[PID_PITCH].Kp = PITCH_P_TERM_SCALE * value;
}

int get_ADJUSTMENT_ROLL_P_GAIN(void)
{
    return currentPidProfile->pid[PID_ROLL].P;
}

void set_ADJUSTMENT_ROLL_P_GAIN(int value)
{
    currentPidProfile->pid[PID_ROLL].P = value;
    pid.coef[PID_ROLL].Kp = ROLL_P_TERM_SCALE * value;
}

int get_ADJUSTMENT_YAW_P_GAIN(void)
{
    return currentPidProfile->pid[PID_YAW].P;
}

void set_ADJUSTMENT_YAW_P_GAIN(int value)
{
    currentPidProfile->pid[PID_YAW].P = value;
    pid.coef[PID_YAW].Kp = YAW_P_TERM_SCALE * value;
}

int get_ADJUSTMENT_PITCH_I_GAIN(void)
{
    return currentPidProfile->pid[PID_PITCH].I;
}

void set_ADJUSTMENT_PITCH_I_GAIN(int value)
{
    currentPidProfile->pid[PID_PITCH].I = value;
    pid.coef[PID_PITCH].Ki = PITCH_I_TERM_SCALE * value;
}

int get_ADJUSTMENT_ROLL_I_GAIN(void)
{
    return currentPidProfile->pid[PID_ROLL].I;
}

void set_ADJUSTMENT_ROLL_I_GAIN(int value)
{
    currentPidProfile->pid[PID_ROLL].I = value;
    pid.coef[PID_ROLL].Ki = ROLL_I_TERM_SCALE * value;
}

int get_ADJUSTMENT_YAW_I_GAIN(void)
{
    return currentPidProfile->pid[PID_YAW].I;
}

void set_ADJUSTMENT_YAW_I_GAIN(int value)
{
    currentPidProfile->pid[PID_YAW].I = value;
    pid.coef[PID_YAW].Ki = YAW_I_TERM_SCALE * value;
}

int get_ADJUSTMENT_PITCH_D_GAIN(void)
{
    return currentPidProfile->pid[PID_PITCH].D;
}

void set_ADJUSTMENT_PITCH_D_GAIN(int value)
{
    currentPidProfile->pid[PID_PITCH].D = value;
    pid.coef[PID_PITCH].Kd = PITCH_D_TERM_SCALE * value;
}

int get_ADJUSTMENT_ROLL_D_GAIN(void)
{
    return currentPidProfile->pid[PID_ROLL].D;
}

void set_ADJUSTMENT_ROLL_D_GAIN(int value)
{
    currentPidProfile->pid[PID_ROLL].D = value;
    pid.coef[PID_ROLL].Kd = ROLL_D_TERM_SCALE * value * (pid.pidMode == 4 ? 0.2f : 1.0f);
}

int get_ADJUSTMENT_YAW_D_GAIN(void)
{
    return currentPidProfile->pid[PID_YAW].D;
}

void set_ADJUSTMENT_YAW_D_GAIN(int value)
{
    currentPidProfile->pid[PID_YAW].D = value;
    pid.coef[PID_YAW].Kd = YAW_D_TERM_SCALE * value;
}

int get_ADJUSTMENT_PITCH_F_GAIN(void)
{
    return currentPidProfile->pid[PID_PITCH].F;
}

void set_ADJUSTMENT_PITCH_F_GAIN(int value)
{
    currentPidProfile->pid[PID_PITCH].F = value;
    pid.coef[PID_PITCH].Kf = PITCH_F_TERM_SCALE * value;
}

int get_ADJUSTMENT_ROLL_F_GAIN(void)
{
    return currentPidProfile->pid[PID_ROLL].F;
}

void set_ADJUSTMENT_ROLL_F_GAIN(int value)
{
    currentPidProfile->pid[PID_ROLL].F = value;
    pid.coef[PID_ROLL].Kf = ROLL_F_TERM_SCALE * value;
}

int get_ADJUSTMENT_YAW_F_GAIN(void)
{
    return currentPidProfile->pid[PID_YAW].F;
}

void set_ADJUSTMENT_YAW_F_GAIN(int value)
{
    currentPidProfile->pid[PID_YAW].F = value;
    pid.coef[PID_YAW].Kf = YAW_F_TERM_SCALE * value;
}

int get_ADJUSTMENT_PITCH_B_GAIN(void)
{
    return currentPidProfile->pid[PID_PITCH].B;
}

void set_ADJUSTMENT_PITCH_B_GAIN(int value)
{
    currentPidProfile->pid[PID_PITCH].B = value;
    pid.coef[PID_PITCH].Kb = PITCH_B_TERM_SCALE * value * (pid.pidMode == 4 ? 10 : 1);
}

int get_ADJUSTMENT_ROLL_B_GAIN(void)
{
    return currentPidProfile->pid[PID_ROLL].B;
}

void set_ADJUSTMENT_ROLL_B_GAIN(int value)
{
    currentPidProfile->pid[PID_ROLL].B = value;
    pid.coef[PID_ROLL].Kb = ROLL_B_TERM_SCALE * value * (pid.pidMode == 4 ? 0.2f : 1.0f);
}

int get_ADJUSTMENT_YAW_B_GAIN(void)
{
    return currentPidProfile->pid[PID_YAW].B;
}

void set_ADJUSTMENT_YAW_B_GAIN(int value)
{
    currentPidProfile->pid[PID_YAW].B = value;
    pid.coef[PID_YAW].Kb = YAW_B_TERM_SCALE * value;
}


int get_ADJUSTMENT_PITCH_GYRO_CUTOFF(void)
{
    return currentPidProfile->gyro_cutoff[PID_PITCH];
}

void set_ADJUSTMENT_PITCH_GYRO_CUTOFF(int value)
{
    currentPidProfile->gyro_cutoff[PID_PITCH] = value;
    filterUpdate(&pid.gyrorFilter[PID_PITCH], value, pid.freq);
}

int get_ADJUSTMENT_ROLL_GYRO_CUTOFF(void)
{
    return currentPidProfile->gyro_cutoff[PID_ROLL];
}

void set_ADJUSTMENT_ROLL_GYRO_CUTOFF(int value)
{
    currentPidProfile->gyro_cutoff[PID_ROLL] = value;
    filterUpdate(&pid.gyrorFilter[PID_ROLL], value, pid.freq);
}

int get_ADJUSTMENT_YAW_GYRO_CUTOFF(void)
{
    return currentPidProfile->gyro_cutoff[PID_YAW];
}

void set_ADJUSTMENT_YAW_GYRO_CUTOFF(int value)
{
    currentPidProfile->gyro_cutoff[PID_YAW] = value;
    filterUpdate(&pid.gyrorFilter[PID_YAW], value, pid.freq);
}

int get_ADJUSTMENT_PITCH_DTERM_CUTOFF(void)
{
    return currentPidProfile->dterm_cutoff[PID_PITCH];
}

void set_ADJUSTMENT_PITCH_DTERM_CUTOFF(int value)
{
    currentPidProfile->dterm_cutoff[PID_PITCH] = value;
    difFilterUpdate(&pid.dtermFilter[PID_PITCH], value, pid.freq);
}

int get_ADJUSTMENT_ROLL_DTERM_CUTOFF(void)
{
    return currentPidProfile->dterm_cutoff[PID_ROLL];
}

void set_ADJUSTMENT_ROLL_DTERM_CUTOFF(int value)
{
    currentPidProfile->dterm_cutoff[PID_ROLL] = value;
    difFilterUpdate(&pid.dtermFilter[PID_ROLL], value, pid.freq);
}

int get_ADJUSTMENT_YAW_DTERM_CUTOFF(void)
{
    return currentPidProfile->dterm_cutoff[PID_YAW];
}

void set_ADJUSTMENT_YAW_DTERM_CUTOFF(int value)
{
    currentPidProfile->dterm_cutoff[PID_YAW] = value;
    difFilterUpdate(&pid.dtermFilter[PID_YAW], value, pid.freq);
}


//// Internal functions

static void INIT_CODE pidSetLooptime(uint32_t pidLooptime)
{
    pid.dT = pidLooptime * 1e-6f;
    pid.freq = 1.0f / pid.dT;

#ifdef USE_DSHOT
    dshotSetPidLoopTime(pidLooptime);
#endif
}

static void INIT_CODE pidInitFilters(const pidProfile_t *pidProfile)
{
    // PID Filters
    for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
        lowpassFilterInit(&pid.gyrorFilter[i], LPF_1ST_ORDER, pidProfile->gyro_cutoff[i], pid.freq, LPF_UPDATE);
        difFilterInit(&pid.dtermFilter[i], pidProfile->dterm_cutoff[i], pid.freq);
        difFilterInit(&pid.btermFilter[i], pidProfile->bterm_cutoff[i], pid.freq);
        pt1FilterInit(&pid.relaxFilter[i], 1, pid.freq);
    }
    pt1FilterInit(&pid.crossAxisRelaxFilter, 1, pid.freq);

}

void INIT_CODE pidLoadProfile(const pidProfile_t *pidProfile)
{
    // PID not initialised yet
    if (pid.dT == 0)
      return;

    // PID algorithm
    pid.pidMode = pidProfile->pid_mode;

    // Live per-axis P/I/D/F scale - applied at the point of use
    // (pidApplyMode0/1), not baked into pid.coef[], so it stays correct
    // regardless of which gain adjustment (including this one) last touched
    // the coefficients.
    for (int i = 0; i < PID_AXIS_COUNT; i++)
        pid.masterGain[i] = pidProfile->master_gain[i] * 0.01f;

    // Optional per-axis curve that further scales master gain by |stick deflection|
    for (int i = 0; i < PID_AXIS_COUNT; i++)
        pid.gainCurveIndex[i] = pidProfile->gain_curve[i];

    // Fixed-wing throttle-based gain attenuation: baseline gain plus an
    // optional shaping curve, mirroring master_gain + gain_curve
    pid.fwTpaGain = pidProfile->fw_tpa_gain * 0.01f;
    pid.fwTpaCurveIndex = pidProfile->fw_tpa_curve;

    // Roll axis
    pid.coef[PID_ROLL].Kp = ROLL_P_TERM_SCALE * pidProfile->pid[PID_ROLL].P;
    pid.coef[PID_ROLL].Ki = ROLL_I_TERM_SCALE * pidProfile->pid[PID_ROLL].I;
    pid.coef[PID_ROLL].Kd = ROLL_D_TERM_SCALE * pidProfile->pid[PID_ROLL].D;
    pid.coef[PID_ROLL].Kf = ROLL_F_TERM_SCALE * pidProfile->pid[PID_ROLL].F;
    pid.coef[PID_ROLL].Kb = ROLL_B_TERM_SCALE * pidProfile->pid[PID_ROLL].B;

    // Pitch axis
    pid.coef[PID_PITCH].Kp = PITCH_P_TERM_SCALE * pidProfile->pid[PID_PITCH].P;
    pid.coef[PID_PITCH].Ki = PITCH_I_TERM_SCALE * pidProfile->pid[PID_PITCH].I;
    pid.coef[PID_PITCH].Kd = PITCH_D_TERM_SCALE * pidProfile->pid[PID_PITCH].D;
    pid.coef[PID_PITCH].Kf = PITCH_F_TERM_SCALE * pidProfile->pid[PID_PITCH].F;
    pid.coef[PID_PITCH].Kb = PITCH_B_TERM_SCALE * pidProfile->pid[PID_PITCH].B;

    // Yaw axis
    pid.coef[PID_YAW].Kp = YAW_P_TERM_SCALE * pidProfile->pid[PID_YAW].P;
    pid.coef[PID_YAW].Ki = YAW_I_TERM_SCALE * pidProfile->pid[PID_YAW].I;
    pid.coef[PID_YAW].Kd = YAW_D_TERM_SCALE * pidProfile->pid[PID_YAW].D;
    pid.coef[PID_YAW].Kf = YAW_F_TERM_SCALE * pidProfile->pid[PID_YAW].F;
    pid.coef[PID_YAW].Kb = YAW_B_TERM_SCALE * pidProfile->pid[PID_YAW].B;

    // Accumulated error limit
    for (int i = 0; i < XYZ_AXIS_COUNT; i++)
        pid.errorLimit[i] = pidProfile->error_limit[i];

    // Exponential I-term decay rate
    pid.itermDecayRate = (pidProfile->iterm_decay_time) ? (10.0f / pidProfile->iterm_decay_time) : 0;

    // Max I-term decay speed in degs/s (linear decay)
    pid.itermDecayLimit = (pidProfile->iterm_decay_limit) ? pidProfile->iterm_decay_limit : 3600;

    // Filters
    for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
        filterUpdate(&pid.gyrorFilter[i], pidProfile->gyro_cutoff[i], pid.freq);
        difFilterUpdate(&pid.dtermFilter[i], pidProfile->dterm_cutoff[i], pid.freq);
        difFilterUpdate(&pid.btermFilter[i], pidProfile->bterm_cutoff[i], pid.freq);
    }

    // Error relax
    pid.itermRelaxType = pidProfile->iterm_relax_type;
    if (pid.itermRelaxType) {
        for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
            uint8_t freq = constrain(pidProfile->iterm_relax_cutoff[i], 1, 100);
            pt1FilterUpdate(&pid.relaxFilter[i], freq, pid.freq);
            pid.itermRelaxLevel[i] = constrain(pidProfile->iterm_relax_level[i], 10, 250);
        }
    }

    // Fixed-wing cross-axis relax: yaw stick activity can soften roll feedback
    // and/or pitch feedback so rudder does not feel like an artificial hold.
    pid.crossAxisRelaxStrength = constrain(pidProfile->cross_axis_relax_strength, 0, 100) * 0.01f;
    pid.crossAxisRelaxPitchStrength = constrain(pidProfile->cross_axis_relax_pitch_strength, 0, 100) * 0.01f;
    pid.crossAxisRelaxLevel = constrain(pidProfile->cross_axis_relax_level, 10, 250);
    const uint8_t crossAxisRelaxCutoff = constrain(pidProfile->cross_axis_relax_cutoff, 1, 100);
    pt1FilterUpdate(&pid.crossAxisRelaxFilter, crossAxisRelaxCutoff, pid.freq);


    // Initialise sub-profiles
#ifdef USE_ACC
    levelingInit(pidProfile);
    autoHoverInit(pidProfile);
    attHoldInit(pidProfile);
#endif
#ifdef USE_ACRO_TRAINER
    acroTrainerInit(pidProfile);
#endif
}

void INIT_CODE pidChangeProfile(const pidProfile_t *pidProfile)
{
    pidLoadProfile(pidProfile);
    //pidResetAxisErrors();
}

void INIT_CODE pidInit(const pidProfile_t *pidProfile)
{
    pidReset();
    pidSetLooptime(gyro.targetLooptime);
    pidInitFilters(pidProfile);
    pidChangeProfile(pidProfile);
}

void INIT_CODE pidCopyProfile(uint8_t dstPidProfileIndex, uint8_t srcPidProfileIndex)
{
    if (dstPidProfileIndex < PID_PROFILE_COUNT && srcPidProfileIndex < PID_PROFILE_COUNT &&
        dstPidProfileIndex != srcPidProfileIndex) {
        memcpy(pidProfilesMutable(dstPidProfileIndex), pidProfilesMutable(srcPidProfileIndex), sizeof(pidProfile_t));
    }
}


/*
 * 2D Rotation matrix
 *
 *        | cos(r)   -sin(r) |
 *    R = |                  |
 *        | sin(r)    cos(r) |
 *
 *
 *               x³    x⁵    x⁷    x⁹
 * sin(x) = x - ――― + ――― - ――― + ――― - …
 *               3!    5!    7!    9!
 *
 *
 *               x²    x⁴    x⁶    x⁸
 * cos(x) = 1 - ――― + ――― - ――― + ――― - …
 *               2!    4!    6!    8!
 *
 *
 * For very small values of x, sin(x) ~= x and cos(x) ~= 1.
 *
 * In this use case, using two or three terms gives nearly 24bits of
 * resolution, which is what can be stored in a float.
 */

static inline void rotateAxisError(void)
{
      const float r = gyro.gyroADCf[Z] * RAD * pid.dT;

      const float t = r * r / 2;
      const float C = t * (1 - t / 6);
      const float S = r * (1 - t / 3);

      const float x = pid.data[PID_ROLL].axisError;
      const float y = pid.data[PID_PITCH].axisError;

      pid.data[PID_ROLL].axisError  -= x * C - y * S;
      pid.data[PID_PITCH].axisError -= y * C + x * S;
}


static float applyItermRelax(int axis, float itermError, float gyroRate, float setpoint)
{
    if ((pid.itermRelaxType == ITERM_RELAX_RPY) ||
        (pid.itermRelaxType == ITERM_RELAX_RP && axis == PID_ROLL) ||
        (pid.itermRelaxType == ITERM_RELAX_RP && axis == PID_PITCH))
    {
        const float setpointLpf = pt1FilterApply(&pid.relaxFilter[axis], setpoint);
        const float setpointHpf = setpoint - setpointLpf;

        const float itermRelaxFactor = MAX(0, 1.0f - fabsf(setpointHpf) / pid.itermRelaxLevel[axis]);

        itermError *= itermRelaxFactor;

        DEBUG_AXIS(ITERM_RELAX, axis, 0, setpoint * 1000);
        DEBUG_AXIS(ITERM_RELAX, axis, 1, gyroRate * 1000);
        DEBUG_AXIS(ITERM_RELAX, axis, 2, setpointLpf * 1000);
        DEBUG_AXIS(ITERM_RELAX, axis, 3, setpointHpf * 1000);
        DEBUG_AXIS(ITERM_RELAX, axis, 4, itermRelaxFactor * 1000);
        DEBUG_AXIS(ITERM_RELAX, axis, 5, itermError * 1000);
    }

    return itermError;
}

static void updateCrossAxisRelax(void)
{
    if (pid.crossAxisRelaxStrength <= 0 && pid.crossAxisRelaxPitchStrength <= 0) {
        pid.crossAxisRelaxYawActivity = 0;
        return;
    }

    pid.crossAxisRelaxYawActivity = pt1FilterApply(&pid.crossAxisRelaxFilter, fabsf(getSetpoint(PID_YAW)));
}

static float getCrossAxisRelaxFactor(int axis)
{
    const float strength = (axis == PID_ROLL) ? pid.crossAxisRelaxStrength :
                           (axis == PID_PITCH) ? pid.crossAxisRelaxPitchStrength : 0;

    if (strength <= 0) {
        return 1.0f;
    }

    const float relaxAmount = MIN(1.0f, pid.crossAxisRelaxYawActivity / pid.crossAxisRelaxLevel) * strength;

    return 1.0f - relaxAmount;
}


static float pidApplySetpoint(uint8_t axis)
{
    // Rate setpoint
    float setpoint = getSetpoint(axis);

#ifdef USE_ACC
    // Apply leveling modes
    if (FLIGHT_MODE(ANGLE_MODE | GPS_RESCUE_MODE | FAILSAFE_MODE)) {
        // Failsafe/GPS rescue take priority over AUTO HOVER/ATT HOLD and force recovery to
        // level, even while genuinely hovering or holding an off-level attitude -- a deliberate
        // safety choice.
        setpoint = angleModeApply(axis, setpoint);
    }
    else if (FLIGHT_MODE(AUTOHOVER_MODE)) {
        setpoint = autoHoverApply(axis, setpoint);
    }
    else if (FLIGHT_MODE(ATTHOLD_MODE)) {
        setpoint = attHoldApply(axis, setpoint);
    }
    else if (FLIGHT_MODE(HORIZON_MODE)) {
        setpoint = horizonModeApply(axis, setpoint);
    }
#ifdef USE_ACRO_TRAINER
    else if (FLIGHT_MODE(TRAINER_MODE)) {
        setpoint = acroTrainerApply(axis, setpoint);
    }
#endif
#endif

    // Save setpoint
    pid.data[axis].setPoint = setpoint;

    return setpoint;
}

static float pidApplyGyroRate(uint8_t axis)
{
    // Get gyro rate
    float gyroRate = gyro.gyroADCf[axis];

    // Bandwidth limiter
    gyroRate = filterApply(&pid.gyrorFilter[axis], gyroRate);

    // Save current rate
    pid.data[axis].gyroRate = gyroRate;

    return gyroRate;
}

/** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** **
 **
 ** MODE 0 - PASSTHROUGH
 **
 ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** **/

static void pidApplyMode0(uint8_t axis)
{
    // Rate setpoint
    float setpoint = pidApplySetpoint(axis);

  //// Unused term
    pid.data[axis].P = 0;
    pid.data[axis].I = 0;
    pid.data[axis].D = 0;

  //// F-term

    // Calculate feedforward component
    pid.data[axis].F = pid.coef[axis].Kf * pid.masterGain[axis] * setpoint;

  //// PID Sum

    // Calculate PID sum
    pid.data[axis].pidSum = pid.data[axis].F;
}


/** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** **
 **
 ** MODE 1 - FIXED-WING RATE PID
 **
 ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** ** **/

// Linear interpolation through a curve's (ascending-x) points, evaluated on
// a 0..1 magnitude -- |stick deflection| for per-axis gain curves, throttle
// for fw_tpa_curve. Mirrors mixerEvaluateCurve()'s algorithm but the domain
// is unipolar since the caller always passes a magnitude.
static float pidEvaluateGainCurve(const gainCurve_t *curve, float mag)
{
    const int n = curve->count;

    if (n < 2)
        return 1.0f;

    const float xs = mag * 1000.0f;

    if (xs <= curve->points[0].x)
        return curve->points[0].y * 0.01f;

    if (xs >= curve->points[n - 1].x)
        return curve->points[n - 1].y * 0.01f;

    for (int i = 0; i < n - 1; i++) {
        const gainCurvePoint_t *p0 = &curve->points[i];
        const gainCurvePoint_t *p1 = &curve->points[i + 1];

        if (xs >= p0->x && xs <= p1->x) {
            const float t = (p1->x != p0->x) ? (xs - p0->x) / (float)(p1->x - p0->x) : 0;
            return (p0->y + t * (p1->y - p0->y)) * 0.01f;
        }
    }

    return 1.0f;
}

static float pidAxisGainCurvePosition(uint8_t axis)
{
    return fminf(1.0f, fabsf(getRcDeflection(axis)));
}

static float pidAxisGainCurve(uint8_t axis)
{
    const uint8_t curveIdx = pid.gainCurveIndex[axis];
    return curveIdx > 0
        ? pidEvaluateGainCurve(gainCurves(curveIdx - 1), pidAxisGainCurvePosition(axis))
        : 1.0f;
}

static float pidThrottleAttenuation(void)
{
    // Throttle is a proxy for prop-wash dynamic pressure over the control
    // surfaces, not airspeed -- on aircraft that hover/harrier at or past
    // stall, surfaces stay authoritative at high throttle regardless of
    // airspeed, so gain is attenuated as throttle rises, not as it falls.
    // Mirrors masterGain + gain_curve: fwTpaGain is the baseline scale, an
    // optional curve from the same shared pool further shapes it by
    // throttle (0..1) instead of |stick deflection|.
    const float curveMult = pid.fwTpaCurveIndex > 0
        ? pidEvaluateGainCurve(gainCurves(pid.fwTpaCurveIndex - 1), getThrottle())
        : 1.0f;

    return pid.fwTpaGain * curveMult;
}

static uint32_t pidScaleToCentiPercent(float scale)
{
    return lrintf(fmaxf(0.0f, scale) * 10000.0f);
}

static uint32_t pidGainToCenti(float gain)
{
    return lrintf(fmaxf(0.0f, gain) * 100.0f);
}

void pidGetRuntimeGains(pidRuntimeGains_t *runtimeGains)
{
    memset(runtimeGains, 0, sizeof(*runtimeGains));

    const float fwTpa = pidThrottleAttenuation();
    runtimeGains->fwTpa = pidScaleToCentiPercent(fwTpa);

    for (int axis = 0; axis < PID_AXIS_COUNT; axis++) {
        const pidf_t *raw = &currentPidProfile->pid[axis];
        const float gainCurve = pidAxisGainCurve(axis);
        const float masterGain = pid.masterGain[axis] * gainCurve;

        runtimeGains->raw[axis] = *raw;
        runtimeGains->masterGain[axis] = currentPidProfile->master_gain[axis];
        runtimeGains->gainCurve[axis] = pidScaleToCentiPercent(gainCurve);
        runtimeGains->gainCurvePosition[axis] = pidScaleToCentiPercent(pidAxisGainCurvePosition(axis));

        runtimeGains->effective[axis].P = pidGainToCenti(raw->P * masterGain * fwTpa);
        runtimeGains->effective[axis].I = pidGainToCenti(raw->I * masterGain);
        runtimeGains->effective[axis].D = pidGainToCenti(raw->D * masterGain * fwTpa);
        runtimeGains->effective[axis].F = pidGainToCenti(raw->F * masterGain);
        runtimeGains->effective[axis].B = pidGainToCenti(raw->B);
    }
}

static void pidApplyMode1(uint8_t axis)
{
    // Rate setpoint
    const float setpoint = pidApplySetpoint(axis);

    // Get gyro rate
    const float gyroRate = pidApplyGyroRate(axis);

    // Calculate error rate
    const float errorRate = setpoint - gyroRate;

    // Throttle-based gain attenuation
    const float atten = pidThrottleAttenuation();

    // Cross-axis relax
    const float crossAxisRelax = getCrossAxisRelaxFactor(axis);

    // Optional per-axis curve scaling master gain by |stick deflection|
    const float curveMult = pidAxisGainCurve(axis);
    const float masterGain = pid.masterGain[axis] * curveMult;


  //// P-term

    // Calculate P-component
    pid.data[axis].P = pid.coef[axis].Kp * masterGain * atten * crossAxisRelax * errorRate;


  //// D-term (gyro only)

    // Calculate D-term with bandwidth limit
    const float dTerm = difFilterApply(&pid.dtermFilter[axis], -gyroRate);

    // Calculate D-component
    pid.data[axis].D = pid.coef[axis].Kd * masterGain * atten * crossAxisRelax * dTerm;


  //// I-term

    // Apply error relax
    const float itermErrorRate = applyItermRelax(axis, errorRate, gyroRate, setpoint) * crossAxisRelax;

    // Saturation
    const bool saturation = (pidAxisSaturated(axis) && pid.data[axis].axisError * itermErrorRate > 0);

    // I-term change
    const float itermDelta = saturation ? 0 : itermErrorRate * pid.dT;

    // Calculate I-component
    pid.data[axis].axisError = limitf(pid.data[axis].axisError + itermDelta, pid.errorLimit[axis]);
    pid.data[axis].I = pid.coef[axis].Ki * masterGain * crossAxisRelax * pid.data[axis].axisError;

    // Apply error decay (fixed rate -- no ground/airborne distinction; a plane
    // sitting on its wheels isn't at risk of tipping over from I-term windup
    // the way a loaded heli rotor disk is, so there's no need to decay faster
    // while landed)
    const float errorDecay = limitf(pid.data[axis].axisError * pid.itermDecayRate, pid.itermDecayLimit);

    pid.data[axis].axisError -= errorDecay * pid.dT;


  //// Feedforward

    // Calculate F component
    pid.data[axis].F = pid.coef[axis].Kf * masterGain * setpoint;


  //// Feedforward Boost (FF Derivative)

    // Calculate B-term with bandwidth limit
    const float bTerm = difFilterApply(&pid.btermFilter[axis], setpoint);

    // Calculate B-component
    pid.data[axis].B = pid.coef[axis].Kb * bTerm;


  //// PID Sum

    // Calculate sum of all terms (no Offset/HSI term -- heli-only)
    pid.data[axis].pidSum = pid.data[axis].P + pid.data[axis].I + pid.data[axis].D +
                            pid.data[axis].F + pid.data[axis].B;
}


void pidController(const pidProfile_t *pidProfile, timeUs_t currentTimeUs)
{
    UNUSED(pidProfile);
    UNUSED(currentTimeUs);

    // Rotate pitch/roll axis error with yaw rotation
    rotateAxisError();

    updateCrossAxisRelax();

    // Apply PID for each axis
    switch (pid.pidMode) {
        case 1:
            pidApplyMode1(PID_ROLL);
            pidApplyMode1(PID_PITCH);
            pidApplyMode1(PID_YAW);
            break;
        default:
            pidApplyMode0(PID_ROLL);
            pidApplyMode0(PID_PITCH);
            pidApplyMode0(PID_YAW);
            break;
    }

    // Reset PID control if gyro overflow detected
    if (gyroOverflowDetected())
        pidReset();
}

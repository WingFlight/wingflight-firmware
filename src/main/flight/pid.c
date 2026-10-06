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
#include "flight/atthold.h"
#include "flight/hold_engine.h"
#include "flight/position.h"
#include "flight/rpm_filter.h"
#include "flight/speed_atten.h"

#include "io/gps.h"

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

// Exposes the same feedforward computation pidApplyMode1's F-term uses (pid.coef[axis].Kf is
// otherwise file-static) -- lets a caller ask "what would stabilized flight command for this
// axis at this rate, with no gyro correction at all" without duplicating Kf's derivation or
// scale. See setpoint.c's getManualDeflection(), which uses this as MANUAL mode's whole output.
// Deliberately NOT attenuated by throttle/GPS speed: MANUAL is a bail-out mode, and its
// full-stick floor (MANUAL_MIN_THROW) must hold whatever the TPA/SPA curves do.
float pidGetFeedforward(int axis, float rate)
{
    return pid.coef[axis].Kf * rate;
}

const pidAxisData_t * pidGetAxisData(void)
{
    return pid.data;
}

void INIT_CODE pidReset(void)
{
    memset(pid.data, 0, sizeof(pid.data));

    memset(pid.snapAboveTime, 0, sizeof(pid.snapAboveTime));
    pid.snapActive = false;
    pid.snapHoldTimer = 0;
    pid.snapRelax = 0;
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


// Exponential I-term decay rate (1/s) for a decay time constant in 0.01 s. Clamped because MSP
// writes and CLI array values are not range-checked. Shared with the thrust-vector loop.
float pidItermDecayRate(uint8_t decayTime)
{
    return 100.0f / constrain(decayTime, ITERM_DECAY_TIME_MIN, ITERM_DECAY_TIME_MAX);
}

// I-term relax setpoint filter cutoff (Hz) for an I-term relax score. A lower cutoff
// treats more of each stick move as "fast" and suppresses I build-up for longer, so less
// bounce-back. Score 5 (default) is the long-standing 10 Hz; 5-9 cover the 10-5 Hz range wing
// pilots actually tune in, one Hz per step. The score is clamped because MSP writes and CLI array
// values are not range-checked. Shared with the thrust-vector loop.
uint8_t pidItermRelaxCutoff(uint8_t score)
{
    static const uint8_t cutoffHz[ITERM_RELAX_MAX] = { 50, 30, 20, 15, 10, 8, 7, 6, 5, 3 };

    return cutoffHz[constrain(score, ITERM_RELAX_MIN, ITERM_RELAX_MAX) - 1];
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

int get_ADJUSTMENT_ITERM_DECAY_TIME_ROLL(void)
{
    return currentPidProfile->iterm_decay_time[PID_ROLL];
}

void set_ADJUSTMENT_ITERM_DECAY_TIME_ROLL(int value)
{
    currentPidProfile->iterm_decay_time[PID_ROLL] = value;
    pid.itermDecayRate[PID_ROLL] = pidItermDecayRate(value);
}

int get_ADJUSTMENT_ITERM_DECAY_TIME_PITCH(void)
{
    return currentPidProfile->iterm_decay_time[PID_PITCH];
}

void set_ADJUSTMENT_ITERM_DECAY_TIME_PITCH(int value)
{
    currentPidProfile->iterm_decay_time[PID_PITCH] = value;
    pid.itermDecayRate[PID_PITCH] = pidItermDecayRate(value);
}

int get_ADJUSTMENT_ITERM_DECAY_TIME_YAW(void)
{
    return currentPidProfile->iterm_decay_time[PID_YAW];
}

void set_ADJUSTMENT_ITERM_DECAY_TIME_YAW(int value)
{
    currentPidProfile->iterm_decay_time[PID_YAW] = value;
    pid.itermDecayRate[PID_YAW] = pidItermDecayRate(value);
}

int get_ADJUSTMENT_ITERM_RELAX_ROLL(void)
{
    return currentPidProfile->iterm_relax[PID_ROLL];
}

void set_ADJUSTMENT_ITERM_RELAX_ROLL(int value)
{
    currentPidProfile->iterm_relax[PID_ROLL] = value;
    pt1FilterUpdate(&pid.relaxFilter[PID_ROLL], pidItermRelaxCutoff(value), pid.freq);
}

int get_ADJUSTMENT_ITERM_RELAX_PITCH(void)
{
    return currentPidProfile->iterm_relax[PID_PITCH];
}

void set_ADJUSTMENT_ITERM_RELAX_PITCH(int value)
{
    currentPidProfile->iterm_relax[PID_PITCH] = value;
    pt1FilterUpdate(&pid.relaxFilter[PID_PITCH], pidItermRelaxCutoff(value), pid.freq);
}

int get_ADJUSTMENT_ITERM_RELAX_YAW(void)
{
    return currentPidProfile->iterm_relax[PID_YAW];
}

void set_ADJUSTMENT_ITERM_RELAX_YAW(int value)
{
    currentPidProfile->iterm_relax[PID_YAW] = value;
    pt1FilterUpdate(&pid.relaxFilter[PID_YAW], pidItermRelaxCutoff(value), pid.freq);
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
    pid.coef[PID_ROLL].Kd = ROLL_D_TERM_SCALE * value;
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
    pid.coef[PID_PITCH].Kb = PITCH_B_TERM_SCALE * value;
}

int get_ADJUSTMENT_ROLL_B_GAIN(void)
{
    return currentPidProfile->pid[PID_ROLL].B;
}

void set_ADJUSTMENT_ROLL_B_GAIN(int value)
{
    currentPidProfile->pid[PID_ROLL].B = value;
    pid.coef[PID_ROLL].Kb = ROLL_B_TERM_SCALE * value;
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

    // Live per-axis P/I/D scale - applied at the point of use
    // (pidApplyMode0/1), not baked into pid.coef[], so it stays correct
    // regardless of which gain adjustment (including this one) last touched
    // the coefficients.
    for (int i = 0; i < PID_AXIS_COUNT; i++)
        pid.masterGain[i] = constrain(pidProfile->master_gain[i], MASTER_GAIN_MIN, MASTER_GAIN_MAX) * 0.01f;

    // Optional per-axis curve that further scales master gain by |stick deflection|
    for (int i = 0; i < PID_AXIS_COUNT; i++)
        pid.gainCurveIndex[i] = pidProfile->gain_curve[i];

    // Fixed-wing throttle-based gain attenuation: baseline gain plus an
    // optional shaping curve, mirroring master_gain + gain_curve
    pid.fwTpaGain = pidProfile->fw_tpa_gain * 0.01f;
    pid.fwTpaCurveIndex = pidProfile->fw_tpa_curve;

    // GPS speed attenuation (separate per-profile storage, see fwSpaConfig_t)
    speedAttenInit(fwSpaConfigs(getCurrentPidProfileIndex()), pid.freq);

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

    // Per-axis exponential I-term decay rate
    for (int i = 0; i < PID_AXIS_COUNT; i++)
        pid.itermDecayRate[i] = pidItermDecayRate(pidProfile->iterm_decay_time[i]);

    // Max I-term decay speed in degs/s (linear decay)
    pid.itermDecayLimit = (pidProfile->iterm_decay_limit) ? pidProfile->iterm_decay_limit : 3600;

    // Filters
    for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
        filterUpdate(&pid.gyrorFilter[i], pidProfile->gyro_cutoff[i], pid.freq);
        difFilterUpdate(&pid.dtermFilter[i], pidProfile->dterm_cutoff[i], pid.freq);
        difFilterUpdate(&pid.btermFilter[i], pidProfile->bterm_cutoff[i], pid.freq);
    }

    // Error relax -- always on for roll, pitch and yaw (a fixed-wing always wants bounce-back
    // suppression on every axis, so there is no type or off switch)
    for (int i = 0; i < XYZ_AXIS_COUNT; i++) {
        pt1FilterUpdate(&pid.relaxFilter[i], pidItermRelaxCutoff(pidProfile->iterm_relax[i]), pid.freq);
        pid.itermRelaxLevel[i] = constrain(pidProfile->iterm_relax_level[i], 10, 250);
    }

    // Fixed-wing cross-axis relax: yaw stick activity can soften roll feedback
    // and/or pitch feedback so rudder does not feel like an artificial hold.
    pid.crossAxisRelaxStrength = constrain(pidProfile->cross_axis_relax_strength, 0, 100) * 0.01f;
    pid.crossAxisRelaxPitchStrength = constrain(pidProfile->cross_axis_relax_pitch_strength, 0, 100) * 0.01f;
    pid.crossAxisRelaxLevel = constrain(pidProfile->cross_axis_relax_level, 10, 250);
    const uint8_t crossAxisRelaxCutoff = constrain(pidProfile->cross_axis_relax_cutoff, 1, 100);
    pt1FilterUpdate(&pid.crossAxisRelaxFilter, crossAxisRelaxCutoff, pid.freq);

    // Snap relax (separate per-profile storage, see snapRelaxConfig_t)
    const snapRelaxConfig_t *snapRelaxConfig = snapRelaxConfigs(getCurrentPidProfileIndex());
    pid.snapRelaxStrength = MIN(snapRelaxConfig->strength, 100) * 0.01f;
    pid.snapRelaxThreshold = constrain(snapRelaxConfig->threshold, SNAP_RELAX_THRESHOLD_MIN, 100) * 0.01f;
    pid.snapRelaxWindow = MIN(snapRelaxConfig->window, SNAP_RELAX_TIME_MAX) * 0.001f;
    pid.snapRelaxHold = MIN(snapRelaxConfig->hold, SNAP_RELAX_TIME_MAX) * 0.001f;

    // Prop-hang relax (separate per-profile storage, see propHangConfig_t)
    const propHangConfig_t *propHangConfig = propHangConfigs(getCurrentPidProfileIndex());
    pid.propHangStrength = MIN(propHangConfig->strength, 100) * 0.01f;
    pid.propHangCosAngle = cos_approx(DEGREES_TO_RADIANS(constrain(propHangConfig->angle, PROP_HANG_ANGLE_MIN, PROP_HANG_ANGLE_MAX)));
    pid.propHangFade = MIN(propHangConfig->fade, PROP_HANG_FADE_MAX) * 0.001f;


    // Initialise sub-profiles
#ifdef USE_ACC
    levelingInit(pidProfile);
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
    speedAttenReset();
    pidSetLooptime(gyro.targetLooptime);
    pidInitFilters(pidProfile);
    pidChangeProfile(pidProfile);
}

void INIT_CODE pidCopyProfile(uint8_t dstPidProfileIndex, uint8_t srcPidProfileIndex)
{
    if (dstPidProfileIndex < PID_PROFILE_COUNT && srcPidProfileIndex < PID_PROFILE_COUNT &&
        dstPidProfileIndex != srcPidProfileIndex) {
        memcpy(pidProfilesMutable(dstPidProfileIndex), pidProfilesMutable(srcPidProfileIndex), sizeof(pidProfile_t));
        memcpy(attitudeLimitsMutable(dstPidProfileIndex), attitudeLimits(srcPidProfileIndex), sizeof(attitudeLimits_t));
        memcpy(fwSpaConfigsMutable(dstPidProfileIndex), fwSpaConfigs(srcPidProfileIndex), sizeof(fwSpaConfig_t));
        memcpy(levelConfigsMutable(dstPidProfileIndex), levelConfigs(srcPidProfileIndex), sizeof(levelConfig_t));
        memcpy(snapRelaxConfigsMutable(dstPidProfileIndex), snapRelaxConfigs(srcPidProfileIndex), sizeof(snapRelaxConfig_t));
        memcpy(propHangConfigsMutable(dstPidProfileIndex), propHangConfigs(srcPidProfileIndex), sizeof(propHangConfig_t));
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
      // Not while a snap is relaxed: the airframe yaws 100-200 deg through a pop top, which
      // would carry roll I into pitch and back for no aerodynamic reason.
      const float r = gyro.gyroADCf[Z] * RAD * pid.dT * (1.0f - pid.snapRelax);

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

/*
 * Snap relax
 *
 * Pop tops, pinwheels and snaps are entered with roll, pitch and yaw slammed in
 * together. The airframe then stalls and autorotates well past the commanded rate
 * (logs show roll at 2-2.5x setpoint), and rate feedback reverses the surfaces
 * against a full stick: up to ~35 % opposite aileron, and the I-term it winds up
 * is left behind as a bump on the exit.
 *
 * A snap is detected when all three sticks pass the threshold within the window of
 * each other, so a slow rolling-harrier style build-up of the same inputs does not
 * count. While the sticks stay past the threshold, and fading out over the hold
 * time after, feedback is relaxed on all three axes only where it opposes the
 * direction the stick was snapped in: P/D pushing back and I winding against it.
 * Feedback that helps the rotation and F are untouched. Yaw mostly lags the stick
 * (logs show ~15 % of the commanded rate), so its feedback helps and is left alone;
 * it is relaxed only when the airframe out-yaws the stick. A heli port would need
 * its own gesture; this one is fixed-wing specific.
 */
static void updateSnapRelax(void)
{
    if (pid.snapRelaxStrength <= 0) {
        pid.snapActive = false;
        pid.snapHoldTimer = 0;
        pid.snapRelax = 0;
        return;
    }

    // Cap on the time-above counters, s. Any value well past the window will do.
    const float timeCap = 10.0f;

    bool allAbove = true;
    float first = 0, last = timeCap;

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        if (fabsf(getRcDeflection(axis)) >= pid.snapRelaxThreshold) {
            pid.snapAboveTime[axis] = MIN(pid.snapAboveTime[axis] + pid.dT, timeCap);
            first = MAX(first, pid.snapAboveTime[axis]);
            last = MIN(last, pid.snapAboveTime[axis]);
        }
        else {
            pid.snapAboveTime[axis] = 0;
            allAbove = false;
        }
    }

    if (!allAbove) {
        pid.snapActive = false;
    }
    else if (!pid.snapActive && first - last <= pid.snapRelaxWindow) {
        pid.snapActive = true;
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            // In setpoint sign: setpointUpdate() negates yaw, so yaw's stick sign is flipped.
            const float deflection = (axis == FD_YAW) ? -getRcDeflection(axis) : getRcDeflection(axis);
            pid.snapDirection[axis] = (deflection > 0) ? 1.0f : -1.0f;
        }
    }

    if (pid.snapActive) {
        pid.snapHoldTimer = pid.snapRelaxHold;
        pid.snapRelax = pid.snapRelaxStrength;
    }
    else if (pid.snapHoldTimer > 0) {
        pid.snapHoldTimer = MAX(pid.snapHoldTimer - pid.dT, 0);
        pid.snapRelax = pid.snapRelaxStrength * pid.snapHoldTimer / pid.snapRelaxHold;
    }
    else {
        pid.snapRelax = 0;
    }

    DEBUG(SNAP_RELAX, 0, lrintf(pid.snapRelax * 1000));
    DEBUG(SNAP_RELAX, 1, pid.snapActive);
    DEBUG(SNAP_RELAX, 2, lrintf((first - last) * 1000));
    DEBUG(SNAP_RELAX, 3, lrintf(getRcDeflection(FD_ROLL) * 1000));
    DEBUG(SNAP_RELAX, 4, lrintf(getRcDeflection(FD_PITCH) * 1000));
    DEBUG(SNAP_RELAX, 5, lrintf(getRcDeflection(FD_YAW) * 1000));
}

// Scale for an error term: below 1 only while a snap is relaxed and the
// error pushes against the snap direction. During the fade-out, a stick reversed
// against the snap gets full feedback back, so the pilot can stop the rotation.
static float getSnapRelaxFactor(int axis, float error, float setpoint)
{
    if (pid.snapRelax <= 0) {
        return 1.0f;
    }

    const float direction = pid.snapDirection[axis];

    if (error * direction >= 0 || setpoint * direction < 0) {
        return 1.0f;
    }

    return 1.0f - pid.snapRelax;
}


/*
 * Prop-hang relax
 *
 * In a prop hang the prop torque rolls the airframe, and on a 3D model that torque roll is
 * part of the flying. A rate gyro holding zero roll rate builds roll I until the ailerons
 * cancel the torque (logs show the roll rate held at ~0 deg/s with the gyro on, against a
 * natural 30-135 deg/s torque roll with it off).
 *
 * A hang is the nose within the configured angle of vertical, the vertical speed within
 * PROP_HANG_VARIO_MAX (an up-line has the nose just as vertical but climbs at 3-30 m/s), for
 * PROP_HANG_ENTRY_TIME, in plain rate flight. It needs an altitude estimate: without one an
 * up-line cannot be told from a hang, so nothing is relaxed. While hanging, roll I stops
 * building and what it holds bleeds off, so the prop is free to roll the airframe; P, D and F
 * are untouched, so the stick still rolls it and P still damps a gust. Roll only: in a hang
 * the rudder and elevator steer the nose. Once the hang ends, the relax fades out over the
 * fade time.
 */
#define PROP_HANG_VARIO_MAX     2.0f    // m/s
#define PROP_HANG_ENTRY_TIME    0.5f    // s
#define PROP_HANG_BLEED_TIME    0.5f    // s, time constant of the roll I bleed while hanging

static void resetPropHangRelax(void)
{
    pid.propHangTime = 0;
    pid.propHangFadeTimer = 0;
    pid.propHangRelax = 0;
}

static void updatePropHangRelax(void)
{
    if (pid.propHangStrength <= 0) {
        resetPropHangRelax();
        return;
    }

    // Leveling and hold layers need roll I to hold their target, so only in plain rate flight
    const bool rateFlight = !FLIGHT_MODE(ANGLE_MODE | GPS_RESCUE_MODE | FAILSAFE_MODE |
                                         LOITER_MODE | RTH_MODE | ATTHOLD_MODE | TRAINER_MODE);

    // rMat[2][0] is the nose-up component of the body X axis: 1 pointing straight up
    const float noseUp = rMat[2][0];
    const float vario = hasEstimatedAltitude() ? getVario() : 0;

    const bool hanging = rateFlight && hasEstimatedAltitude() &&
                         noseUp >= pid.propHangCosAngle &&
                         fabsf(vario) <= PROP_HANG_VARIO_MAX;

    pid.propHangTime = hanging ? MIN(pid.propHangTime + pid.dT, PROP_HANG_ENTRY_TIME) : 0;

    if (pid.propHangTime >= PROP_HANG_ENTRY_TIME) {
        pid.propHangFadeTimer = pid.propHangFade;
        pid.propHangRelax = pid.propHangStrength;
    }
    else if (pid.propHangFadeTimer > 0) {
        pid.propHangFadeTimer = MAX(pid.propHangFadeTimer - pid.dT, 0);
        pid.propHangRelax = pid.propHangStrength * pid.propHangFadeTimer / pid.propHangFade;
    }
    else {
        pid.propHangRelax = 0;
    }

    DEBUG(PROP_HANG, 0, lrintf(pid.propHangRelax * 1000));
    DEBUG(PROP_HANG, 1, lrintf(noseUp * 1000));
    DEBUG(PROP_HANG, 2, lrintf(vario * 100));
    DEBUG(PROP_HANG, 3, lrintf(pid.propHangTime * 1000));
    DEBUG(PROP_HANG, 4, hasEstimatedAltitude());
    DEBUG(PROP_HANG, 5, rateFlight);
}

static float pidApplySetpoint(uint8_t axis)
{
    // Rate setpoint
    float setpoint = getSetpoint(axis);

#ifdef USE_ACC
    // Apply leveling modes
    if (FLIGHT_MODE(ANGLE_MODE | GPS_RESCUE_MODE | FAILSAFE_MODE | LOITER_MODE | RTH_MODE)) {
        // Failsafe/GPS rescue/GPS nav take priority over ATT HOLD and force recovery to level,
        // even while holding an off-level attitude -- a deliberate safety choice.
        setpoint = angleModeApply(axis, setpoint);
    }
    else {
        // Next engagement starts the level target from the attitude at that moment
        angleModeReset();

        if (FLIGHT_MODE(ATTHOLD_MODE)) {
            setpoint = attHoldApply(axis, setpoint);
        }
#ifdef USE_ACRO_TRAINER
        else if (FLIGHT_MODE(TRAINER_MODE)) {
            setpoint = acroTrainerApply(axis, setpoint);
        }
#endif
    }
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
    pid.data[axis].F = pid.coef[axis].Kf * setpoint;

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

// Scale from a gain curve (1..GAIN_CURVE_COUNT, 0 = none) at this axis's
// |stick deflection|. Shared with the thrust-vector loop's tv_gain_curve, so
// both loops read the same curve pool the same way. Out-of-range indices (MSP
// and CLI array writes are unchecked) count as no curve.
float pidGetGainCurveScale(uint8_t curveIndex, uint8_t axis)
{
    return pidGetGainCurveScaleAt(curveIndex, pidAxisGainCurvePosition(axis));
}

// Same, at a caller-supplied 0..1 position (GPS speed attenuation's speed / speed_max).
float pidGetGainCurveScaleAt(uint8_t curveIndex, float position)
{
    return (curveIndex > 0 && curveIndex <= GAIN_CURVE_COUNT)
        ? pidEvaluateGainCurve(gainCurves(curveIndex - 1), position)
        : 1.0f;
}

static float pidAxisGainCurve(uint8_t axis)
{
    return pidGetGainCurveScale(pid.gainCurveIndex[axis], axis);
}

static float pidThrottleAttenuation(void)
{
    // Throttle is a proxy for prop-wash dynamic pressure over the control
    // surfaces, not airspeed -- on aircraft that hover/harrier at or past
    // stall, surfaces stay authoritative at high throttle regardless of
    // airspeed, so gain is attenuated as throttle rises, not as it falls.
    // Mirrors masterGain + gain_curve: fwTpaGain is the baseline scale, an
    // optional curve from the same shared pool further shapes it by
    // throttle (0..1) instead of |stick deflection|. Applied (with GPS speed) to P, D,
    // F and B through pidSurfaceAttenuation(); not to I or MANUAL.
    const float curveMult = pid.fwTpaCurveIndex > 0
        ? pidEvaluateGainCurve(gainCurves(pid.fwTpaCurveIndex - 1), getThrottle())
        : 1.0f;

    return pid.fwTpaGain * curveMult;
}

// Throttle and GPS speed attenuation together, as applied to the surface terms.
// Floored so a curve point near zero, or both baselines at their minimum, can
// never leave the surfaces without throw: F carries most of the stick authority,
// so a zero here would leave the sticks almost nothing to move.
static float pidSurfaceAttenuation(void)
{
    return fmaxf(pidThrottleAttenuation() * speedAttenGetScale(), PID_ATTENUATION_MIN);
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
    const float fwSpa = speedAttenGetScale();
    const float atten = pidSurfaceAttenuation();
    runtimeGains->fwTpa = pidScaleToCentiPercent(fwTpa);
    runtimeGains->fwSpa = pidScaleToCentiPercent(fwSpa);
    runtimeGains->fwSpaSpeed = lrintf(speedAttenGetSpeed() * 10.0f);
    runtimeGains->fwSpaEnabled = speedAttenIsEnabled();

    for (int axis = 0; axis < PID_AXIS_COUNT; axis++) {
        const pidf_t *raw = &currentPidProfile->pid[axis];
        const float gainCurve = pidAxisGainCurve(axis);
        const float masterGain = pid.masterGain[axis] * gainCurve;

        runtimeGains->raw[axis] = *raw;
        runtimeGains->masterGain[axis] = currentPidProfile->master_gain[axis];
        runtimeGains->gainCurve[axis] = pidScaleToCentiPercent(gainCurve);
        runtimeGains->gainCurvePosition[axis] = pidScaleToCentiPercent(pidAxisGainCurvePosition(axis));

        runtimeGains->effective[axis].P = pidGainToCenti(raw->P * masterGain * atten);
        runtimeGains->effective[axis].I = pidGainToCenti(raw->I * masterGain);
        runtimeGains->effective[axis].D = pidGainToCenti(raw->D * masterGain * atten);
        runtimeGains->effective[axis].F = pidGainToCenti(raw->F * atten);
        runtimeGains->effective[axis].B = pidGainToCenti(raw->B * atten);
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

    // Throttle- and GPS speed-based gain attenuation
    const float atten = pidSurfaceAttenuation();

    // Cross-axis relax
    const float crossAxisRelax = getCrossAxisRelaxFactor(axis);

    // Snap relax: feedback pushing against a snapped stick
    const float snapRelax = getSnapRelaxFactor(axis, errorRate, setpoint);

    // Optional per-axis curve scaling master gain by |stick deflection|
    const float curveMult = pidAxisGainCurve(axis);
    const float masterGain = pid.masterGain[axis] * curveMult;


  //// P-term

    // Calculate P-component
    pid.data[axis].P = pid.coef[axis].Kp * masterGain * atten * crossAxisRelax * snapRelax * errorRate;


  //// D-term (gyro only)

    // Calculate D-term with bandwidth limit
    const float dTerm = difFilterApply(&pid.dtermFilter[axis], -gyroRate);

    // Calculate D-component. Relaxed only while it pushes against the snap too.
    const float snapRelaxD = getSnapRelaxFactor(axis, dTerm, setpoint);
    pid.data[axis].D = pid.coef[axis].Kd * masterGain * atten * crossAxisRelax * snapRelaxD * dTerm;


  //// I-term

    // Apply error relax. Cross-axis relax slows the accumulation here and is NOT
    // applied again to the I output below: scaling the output as well made I drop
    // immediately when rudder was applied and jump back on release (#112).
    // Prop-hang relax holds roll I back so the prop torque can roll the airframe
    const float propHangRelax = (axis == PID_ROLL) ? pid.propHangRelax : 0;

    const float itermErrorRate = applyItermRelax(axis, errorRate, gyroRate, setpoint) * crossAxisRelax * snapRelax * (1.0f - propHangRelax);

    // Saturation
    const bool saturation = (pidAxisSaturated(axis) && pid.data[axis].axisError * itermErrorRate > 0);

    // While PASSTHROUGH or MANUAL drives the surfaces the gyro is not tracking this setpoint, so
    // integrating the error only winds I up, and it was released as a bump on switching back
    // (with a leveling mode also on, decay is suspended and nothing bled it off). Hold the
    // accumulation; decay below still applies as normal.
    const bool bypassed = mixerStabilizationBypassed();

    // I-term change
    const float itermDelta = (saturation || bypassed) ? 0 : itermErrorRate * pid.dT;

    // Calculate I-component
    pid.data[axis].axisError = limitf(pid.data[axis].axisError + itermDelta, pid.errorLimit[axis]);

    // Bleed off the roll I that was holding the airframe against the torque before the hang
    if (propHangRelax > 0) {
        pid.data[axis].axisError -= pid.data[axis].axisError * MIN(propHangRelax * pid.dT / PROP_HANG_BLEED_TIME, 1.0f);
    }
    // TRADITIONAL_MODE forces I output to zero without touching axisError's own bookkeeping, so
    // relax/decay keep behaving as configured and I resumes smoothly if the mode is switched off.
    pid.data[axis].I = FLIGHT_MODE(TRADITIONAL_MODE) ? 0.0f
        : pid.coef[axis].Ki * masterGain * pid.data[axis].axisError;

    // Apply error decay (fixed rate -- no ground/airborne distinction; a plane
    // sitting on its wheels isn't at risk of tipping over from I-term windup
    // the way a loaded heli rotor disk is, so there's no need to decay faster
    // while landed) -- but suspended while a leveling/attitude-hold layer is
    // actively shaping this axis's setpoint. Those layers (ANGLE/GPS rescue/
    // failsafe/loiter/RTH's shared angleModeApply on roll+pitch, the
    // acro trainer only while limiting, and ATTHOLD on an axis that is actually
    // holding a target) fundamentally need a sustained I-term to hold a
    // corrected attitude against a persistent disturbance once the rate error
    // itself has settled to ~0 -- an unconditional decay quietly erodes exactly
    // that contribution, which feels indistinguishable from the correction just
    // giving up after a couple of seconds even though the true attitude error
    // never went away. Plain acro/manual flight (and TRADITIONAL_MODE, which
    // only masks the I *output* above, not axisError itself) still decay
    // normally -- and so does an ATTHOLD axis that's free-tracking
    // (stick active, or still settling after release): there it's plain rate
    // flight, so it should bleed I exactly like normal mode rather than carry
    // stale I from an earlier maneuver into the next hold.
    //
    // An ATTHOLD axis that IS holding is the one exception to "suspended":
    // it decays at a small fraction of the normal rate instead of not at all.
    // With no bleed whatsoever, I left over from before the hold engaged (or
    // from a disturbance long gone) would keep the surfaces parked off-center
    // forever even with zero attitude error and zero motion -- e.g. sitting on
    // the bench, where nothing the hold does can ever move the aircraft. A
    // slow bleed still lets the hold carry a real steady disturbance (torque
    // roll): the outer attitude loop just re-grows whatever I is needed, at the
    // cost of a small sag -- while anything not actually needed drains away.
    const flightModeFlags_e rollPitchLevelingModes = ANGLE_MODE | GPS_RESCUE_MODE
        | FAILSAFE_MODE | LOITER_MODE | RTH_MODE;

    bool trainerLimitingThisAxis = false;
#ifdef USE_ACRO_TRAINER
    trainerLimitingThisAxis = FLIGHT_MODE(TRAINER_MODE) && acroTrainerIsLimiting(axis);
#endif

    float attHoldDecayScale = 1.0f;
#ifdef USE_ACC
    attHoldDecayScale = attHoldIDecayScale(axis);
#endif

    const bool isYaw = (axis == FD_YAW);
    const bool levelingModeShapingThisAxis = trainerLimitingThisAxis
        || (!isYaw && FLIGHT_MODE(rollPitchLevelingModes));

    if (!levelingModeShapingThisAxis) {
        const float errorDecay = limitf(pid.data[axis].axisError * pid.itermDecayRate[axis] * attHoldDecayScale, pid.itermDecayLimit * attHoldDecayScale);

        pid.data[axis].axisError -= errorDecay * pid.dT;
    }


  //// Feedforward

    // Calculate F component. Attenuated like P and D: a surface made more effective by
    // prop wash or airspeed needs less deflection for the same rate, and F sets that
    // deflection far more than the small P-term does.
    pid.data[axis].F = pid.coef[axis].Kf * atten * setpoint;


  //// Feedforward Boost (FF Derivative)

    // Calculate B-term with bandwidth limit
    const float bTerm = difFilterApply(&pid.btermFilter[axis], setpoint);

    // Calculate B-component
    pid.data[axis].B = pid.coef[axis].Kb * atten * bTerm;


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

    updateSnapRelax();

    updatePropHangRelax();

    speedAttenUpdate(pid.dT);

    DEBUG(GAIN_ATTEN, 0, lrintf(pidThrottleAttenuation() * 1000));
    DEBUG(GAIN_ATTEN, 1, lrintf(speedAttenGetScale() * 1000));
    DEBUG(GAIN_ATTEN, 2, lrintf(speedAttenGetSpeed() * 10));
#ifdef USE_GPS
    DEBUG(GAIN_ATTEN, 3, MAX(gpsSol.speed3d, gpsSol.groundSpeed));
    DEBUG(GAIN_ATTEN, 4, STATE(GPS_FIX) ? 1 : 0);
#endif

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

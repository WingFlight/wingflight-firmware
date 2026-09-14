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
#include <float.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/filter.h"

#include "config/config.h"
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

// Oscillation limiter tuning constants -- see docs/development/Oscillation Detection.md.
// Not exposed via CLI: these shape the detection window/gates, not the enable/sensitivity
// knobs a pilot needs to tune per-aircraft (those live in pidProfile_t).
#define OSC_LIMITER_WINDOW_MS               200.0f   // periodicity window length
#define OSC_LIMITER_MIN_CROSSINGS           6         // >= 3 full cycles required per window
#define OSC_LIMITER_ENERGY_ALPHA            0.05f     // leaky-integrator energy smoothing
#define OSC_LIMITER_RAMP_MS                 400.0f    // time to ease fully from 1.0 to the floor
#define OSC_LIMITER_TELEMETRY_HOLD_MS        3000.0f   // min time the reported 'active' flag stays asserted
#define OSC_LIMITER_SETPOINT_GATE_DEGS      300.0f    // freeze while commanding a large rate
#define OSC_LIMITER_SETPOINT_SLEW_GATE_DEGS 3000.0f    // freeze right after an abrupt stick step
#define OSC_LIMITER_GAIN_RISE_FAST_PER_SEC  0.5f      // combined-gain (masterGain*curveMult) rise
                                                        // rate, per second, treated as an actively-
                                                        // rising gain (e.g. a live adjustment
                                                        // function sweep) rather than a static tune
#define OSC_LIMITER_FAST_RISE_SCORE_STEP    3.0f       // score step on a candidate sample while the
                                                        // gain is rising fast (vs +1.0f normally) --
                                                        // confirms sooner, since a rising gain during
                                                        // a live candidate is itself corroborating
#define OSC_LIMITER_MIN_REF_GAIN            0.1f      // ignore an engaged sample's gain snapshot
                                                        // below this when pinning the ceiling -- a
                                                        // gain_curve dipping near zero at this exact
                                                        // stick position is not evidence that near-
                                                        // zero gain is "safe"; it would otherwise
                                                        // freeze the axis near-zero even at other,
                                                        // untested stick positions for the rest of
                                                        // the flight, which is worse than the bug
                                                        // this ceiling exists to fix


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

// Oscillation limiter note: clamped to pidOscLimiterBaseCeiling() so a live gain sweep can't
// climb -- or get saved -- past what this axis's limiter is already fighting this flight. See
// docs/development/Oscillation Detection.md.
void set_ADJUSTMENT_MASTER_GAIN_PITCH(int value)
{
    const float gain = fminf(value * 0.01f, pidOscLimiterBaseCeiling(PID_PITCH));
    currentPidProfile->master_gain[PID_PITCH] = lrintf(gain * 100.0f);
    pid.masterGain[PID_PITCH] = gain;
}

int get_ADJUSTMENT_MASTER_GAIN_ROLL(void)
{
    return currentPidProfile->master_gain[PID_ROLL];
}

void set_ADJUSTMENT_MASTER_GAIN_ROLL(int value)
{
    const float gain = fminf(value * 0.01f, pidOscLimiterBaseCeiling(PID_ROLL));
    currentPidProfile->master_gain[PID_ROLL] = lrintf(gain * 100.0f);
    pid.masterGain[PID_ROLL] = gain;
}

int get_ADJUSTMENT_MASTER_GAIN_YAW(void)
{
    return currentPidProfile->master_gain[PID_YAW];
}

void set_ADJUSTMENT_MASTER_GAIN_YAW(int value)
{
    const float gain = fminf(value * 0.01f, pidOscLimiterBaseCeiling(PID_YAW));
    currentPidProfile->master_gain[PID_YAW] = lrintf(gain * 100.0f);
    pid.masterGain[PID_YAW] = gain;
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

    // Live per-axis P/I/D scale - applied at the point of use
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


    // Oscillation limiter -- see docs/development/Oscillation Detection.md. Eases the combined
    // (masterGain*curveMult) gain down further each time a sustained oscillation is confirmed,
    // and holds (never eases back up) once it clears -- so a recurring oscillation gets cut
    // harder each time. Only a new arm clears it (pidResetOscLimiter(), called from tryArm());
    // reloading -- including switching to a different PID profile and back -- does NOT reset the
    // accumulated history below, only the tuning constants and this profile's filter coefficients.
    pid.oscLimiterEnabled = pidProfile->osc_limiter;
    const float oscMinHz = MAX(1, pidProfile->osc_limiter_min_hz);
    const float oscMaxHz = MAX(oscMinHz + 1, pidProfile->osc_limiter_max_hz);
    const float oscCenterHz = (oscMinHz + oscMaxHz) * 0.5f;
    const float oscQ = oscCenterHz / (oscMaxHz - oscMinHz);
    pid.oscLimiterThresholdSq = pidProfile->osc_limiter_threshold * pidProfile->osc_limiter_threshold;
    pid.oscLimiterFloor = constrainf(pidProfile->osc_limiter_floor * 0.01f, 0.1f, 1.0f);
    pid.oscLimiterScoreMax = fmaxf(1.0f, pidProfile->osc_limiter_engage_ms * 1e-3f * pid.freq);
    pid.oscLimiterWindowTicks = fmaxf(1.0f, OSC_LIMITER_WINDOW_MS * 1e-3f * pid.freq);
    pid.oscLimiterTelemetryHoldTicks = fmaxf(1.0f, OSC_LIMITER_TELEMETRY_HOLD_MS * 1e-3f * pid.freq);
    const float oscRampTicks = fmaxf(1.0f, OSC_LIMITER_RAMP_MS * 1e-3f * pid.freq);
    pid.oscLimiterRampPerLoop = (1.0f - pid.oscLimiterFloor) / oscRampTicks;
    // Only (re)initialise this profile's own filter coefficients -- other profiles' history/
    // filters are untouched so a profile switch and back finds its own state as it left it.
    const uint8_t oscProfileIndex = getCurrentPidProfileIndex();
    for (int i = 0; i < PID_AXIS_COUNT; i++) {
        biquadFilterInit(&pid.oscLimiter[oscProfileIndex][i].bandpass, oscCenterHz, pid.freq, oscQ, BIQUAD_BPF);
    }

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
    // One-time clean start for every profile's oscillation-limiter history; tryArm() repeats
    // this on every arm thereafter (a profile switch alone deliberately does not).
    pidResetOscLimiter();
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
    if (FLIGHT_MODE(ANGLE_MODE | GPS_RESCUE_MODE | FAILSAFE_MODE | LOITER_MODE | RTH_MODE)) {
        // Failsafe/GPS rescue/GPS nav take priority over AUTO HOVER/ATT HOLD and force recovery to
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
        runtimeGains->effective[axis].F = pidGainToCenti(raw->F);
        runtimeGains->effective[axis].B = pidGainToCenti(raw->B);
    }
}

// Resets one axis's detector + ratchet history to a clean, fully-unrestricted state.
static void oscLimiterResetAxis(oscLimiterAxis_t *osc)
{
    osc->energy = 0;
    osc->score = 0;
    osc->gainScale = 1.0f;
    osc->refGain = FLT_MAX;
    osc->baseGainCeiling = FLT_MAX;
    osc->prevSample = 0;
    osc->prevSetpoint = 0;
    osc->prevCombinedGain = 0;
    osc->crossCount = 0;
    osc->windowRemaining = pid.oscLimiterWindowTicks;
    osc->telemetryHold = 0;
    osc->reportedScale = 100;
    osc->periodicOk = false;
    osc->active = false;
}

// Oscillation limiter detector -- see docs/development/Oscillation Detection.md.
// Pure function of read-only per-axis signals; only ever writes pid.oscLimiter[profile][axis].
// Never touches pidProfile_t, never alters setpoint/gyroRate, and is fully disabled unless
// osc_limiter is turned on. Returns the combined (masterGain*curveMult) gain to actually apply,
// clamped to this axis's ratcheted ceiling if one has ever been established this flight.
static float updateOscLimiter(uint8_t axis, float setpoint, float errorRate, float combinedGain)
{
    oscLimiterAxis_t *osc = &pid.oscLimiter[getCurrentPidProfileIndex()][axis];

    if (!pid.oscLimiterEnabled || pid.pidMode != 1 || !isAirborne() || gyroOverflowDetected() ||
        FLIGHT_MODE(FAILSAFE_MODE) || FLIGHT_MODE(GPS_RESCUE_MODE)) {
        // Preconditions not met. Track setpoint/gain so the slew gates don't see a stale
        // value next time, but otherwise freeze -- don't reset ongoing state. A ceiling
        // already latched (osc->refGain) still applies -- it's enforced below regardless.
        osc->prevSetpoint = setpoint;
        osc->prevCombinedGain = combinedGain;
        return fminf(combinedGain, osc->refGain * osc->gainScale);
    }

    // Band-pass the tracking error to isolate the oscillation band
    const float bp = biquadFilterApplyDF1(&osc->bandpass, errorRate);

    // Smoothed energy estimate (mean square, leaky integrator)
    osc->energy += (bp * bp - osc->energy) * OSC_LIMITER_ENERGY_ALPHA;

    // Zero-crossing based periodicity check over a rolling window
    if ((bp >= 0) != (osc->prevSample >= 0)) {
        osc->crossCount++;
    }
    osc->prevSample = bp;

    if (osc->windowRemaining <= 1) {
        osc->periodicOk = (osc->crossCount >= OSC_LIMITER_MIN_CROSSINGS);
        osc->crossCount = 0;
        osc->windowRemaining = pid.oscLimiterWindowTicks;
    } else {
        osc->windowRemaining--;
    }

    // Gate out large/abrupt intentional stick inputs so they can't be scored as oscillation
    const float setpointRate = fabsf(setpoint - osc->prevSetpoint) * pid.freq;
    const bool gated = (fabsf(setpoint) > OSC_LIMITER_SETPOINT_GATE_DEGS) ||
                        (setpointRate > OSC_LIMITER_SETPOINT_SLEW_GATE_DEGS);
    osc->prevSetpoint = setpoint;

    // A combined gain that's actively climbing fast (e.g. a live master_gain adjustment
    // function sweep) while a candidate is otherwise forming is itself corroborating
    // evidence, not something to gate out -- confirm sooner instead of waiting it out.
    const float gainRiseRate = (combinedGain - osc->prevCombinedGain) * pid.freq;
    const bool gainRisingFast = gainRiseRate > OSC_LIMITER_GAIN_RISE_FAST_PER_SEC;
    osc->prevCombinedGain = combinedGain;

    const bool candidate = !gated && osc->periodicOk && (osc->energy > pid.oscLimiterThresholdSq);

    // Asymmetric hysteresis: slow to confirm oscillating, faster to confirm it has subsided.
    const float scoreStep = candidate ? (gainRisingFast ? OSC_LIMITER_FAST_RISE_SCORE_STEP : 1.0f) : -2.0f;
    osc->score = constrainf(osc->score + scoreStep, 0.0f, pid.oscLimiterScoreMax);

    const bool engaged = osc->score >= pid.oscLimiterScoreMax;

    if (engaged) {
        // Pin the ceiling to the LOWEST combined gain seen while engaged, times the floor --
        // never to whatever's live right now. This is what actually stops a gain that keeps
        // climbing afterwards (e.g. the pilot keeps raising master_gain) from outrunning the
        // cut: raising combinedGain can't raise refGain, since only fminf() ever applies.
        // Guarded by OSC_LIMITER_MIN_REF_GAIN: a gain_curve sample that happens to be near
        // zero at this exact stick position isn't evidence that near-zero gain is the fix --
        // pinning to it would freeze the axis near-zero at every OTHER stick position too,
        // for the rest of the flight.
        if (combinedGain > OSC_LIMITER_MIN_REF_GAIN) {
            osc->refGain = fminf(osc->refGain, combinedGain);
        }
        // Same ratchet, expressed against pid.masterGain[axis] alone (curve-independent) --
        // this is what gates the adjustment-function setters below, so a live sweep can't
        // creep the *persisted* base gain past what this axis is already fighting either.
        if (pid.masterGain[axis] > OSC_LIMITER_MIN_REF_GAIN) {
            osc->baseGainCeiling = fminf(osc->baseGainCeiling, pid.masterGain[axis]);
        }
        // Back off further each time -- never eased back up mid-flight, so a recurring
        // oscillation gets cut harder on every engagement, down to the floor at most.
        osc->gainScale = fmaxf(pid.oscLimiterFloor, osc->gainScale - pid.oscLimiterRampPerLoop);
        osc->telemetryHold = pid.oscLimiterTelemetryHoldTicks;
    } else if (osc->telemetryHold > 0) {
        osc->telemetryHold--;
    }

    // Reported flag only, decoupled from gainScale: stays asserted for a bit after the last
    // engagement so a brief drop-out doesn't flicker the blackbox/telemetry indication.
    osc->active = engaged || (osc->telemetryHold > 0);

    const float ceiling = osc->refGain * osc->gainScale;   // FLT_MAX while never engaged
    const float enforcedGain = fminf(combinedGain, ceiling);
    osc->reportedScale = (combinedGain > 0.0f)
        ? (uint8_t)lrintf(constrainf(enforcedGain / combinedGain, 0.0f, 1.0f) * 100.0f)
        : 100;

    DEBUG_AXIS(OSC_LIMITER, axis, 0, lrintf(osc->energy));
    DEBUG_AXIS(OSC_LIMITER, axis, 1, lrintf(osc->score));
    DEBUG_AXIS(OSC_LIMITER, axis, 2, osc->reportedScale);

    return enforcedGain;
}

bool pidOscLimiterActive(int axis)
{
    return pid.oscLimiter[getCurrentPidProfileIndex()][axis].active;
}

// Clears every profile's cut and telemetry hold for a new flight -- called from tryArm() only.
void pidResetOscLimiter(void)
{
    for (int p = 0; p < PID_PROFILE_COUNT; p++) {
        for (int i = 0; i < PID_AXIS_COUNT; i++) {
            oscLimiterResetAxis(&pid.oscLimiter[p][i]);
        }
    }
}

uint8_t pidOscLimiterScale(int axis)
{
    return pid.oscLimiter[getCurrentPidProfileIndex()][axis].reportedScale;
}

float pidOscLimiterBaseCeiling(int axis)
{
    const oscLimiterAxis_t *osc = &pid.oscLimiter[getCurrentPidProfileIndex()][axis];
    return osc->baseGainCeiling * osc->gainScale;
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
    const float combinedGain = pid.masterGain[axis] * curveMult;

    // Detect a sustained gain-induced oscillation on this axis and, if found, ease the
    // combined gain down (never up) further -- holds once it subsides, backing off more if
    // it recurs. The ceiling this enforces is pinned to the gain level it was first seen at,
    // so a gain that keeps climbing afterwards (a live master_gain adjustment function sweep,
    // say) cannot outrun the cut -- see docs/development/Oscillation Detection.md. Always
    // called (even in failsafe/rescue, which updateOscLimiter itself gates off internally) so
    // its state keeps freezing/tracking correctly and resumes cleanly once those modes clear.
    const float oscEnforcedGain = updateOscLimiter(axis, setpoint, errorRate, combinedGain);

    // Failsafe/GPS-rescue authority always wins over a stale cut, regardless of what the
    // detector above just returned.
    const float masterGain = (FLIGHT_MODE(FAILSAFE_MODE) || FLIGHT_MODE(GPS_RESCUE_MODE))
        ? combinedGain
        : oscEnforcedGain;


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
    pid.data[axis].F = pid.coef[axis].Kf * setpoint;


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

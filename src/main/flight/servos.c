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
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#ifdef USE_SERVOS

#include "build/build_config.h"

#include "common/curve.h"
#include "common/maths.h"

#include "config/config.h"
#include "config/config_reset.h"

#include "drivers/time.h"
#include "drivers/pwm_output.h"

#include "sensors/gyro.h"

#include "fc/runtime_config.h"

#include "flight/servos.h"
#include "flight/mixer.h"

#include "pg/servos.h"
#include "pg/servo_curve.h"
#include "pg/bus_servo.h"

#include "rx/rx.h"


static FAST_DATA_ZERO_INIT uint8_t      servoCount;

static FAST_DATA_ZERO_INIT float        servoInput[MAX_SUPPORTED_SERVOS];
static FAST_DATA_ZERO_INIT float        servoOutput[MAX_SUPPORTED_SERVOS];
static FAST_DATA_ZERO_INIT float        servoResolution[MAX_SUPPORTED_SERVOS];

static FAST_DATA_ZERO_INIT int16_t      servoOverride[MAX_SUPPORTED_SERVOS];

static FAST_DATA_ZERO_INIT float        servoAxisTrim[3];  // last commanded per-axis trim value in µs [ROLL=0, PITCH=1, YAW=2]
static FAST_DATA_ZERO_INIT float        servoRuntimeAxisTrim[3]; // runtime-only (continuous mode) axis trim in µs, never saved
static FAST_DATA_ZERO_INIT float        servoRuntimeTrim[MAX_SUPPORTED_SERVOS]; // the above, resolved per servo

static FAST_DATA_ZERO_INIT timerChannel_t servoChannel[MAX_SUPPORTED_SERVOS];


/*
 * Which way a stabilized axis's trim moves a servo: 0 if no mixer rule feeds that
 * servo from the axis, otherwise +1 or -1.
 *
 * mixerUpdateRules() scales the raw stabilized value by the axis input's rate
 * (mixer.input[src] * mixerInputs(src)->rate) before any rule sees it, so a
 * negative rate -- the per-axis "Invert" setting -- flips every rule fed by
 * this axis the same way a negative weight would. Three independent things can
 * flip a servo's direction relative to the stabilized axis: that Invert/rate
 * sign, a negative mixer rule weight (e.g. paired aileron servos driven from
 * the same input with opposite-signed weights instead of a flag), and the
 * per-servo SERVO_FLAG_REVERSED flag (servoUpdate() negates pos before applying
 * rpos/rneg/mid). All three must be folded into the trim direction so it stays
 * coordinated with other servos sharing the same axis, regardless of which
 * mechanism reverses which servo.
 */
static int axisTrimDirection(int axis, int servo)
{
    static const uint8_t axisInput[3] = {
        MIXER_IN_STABILIZED_ROLL, MIXER_IN_STABILIZED_PITCH, MIXER_IN_STABILIZED_YAW,
    };

    const bool rateReversed = mixerInputs(axisInput[axis])->rate < 0;

    for (int r = 0; r < MIXER_RULE_COUNT; r++) {
        const mixerRule_t *rule = mixerRules(r);
        if (rule->oper && rule->output == (uint8_t)(MIXER_SERVO_OFFSET + servo) &&
            rule->input == axisInput[axis]) {
            const bool flagReversed = servoParams(servo)->flags & SERVO_FLAG_REVERSED;
            const bool weightReversed = (rule->weight != 0 ? rule->weight : rule->weightNeg) < 0;
            return (rateReversed != (flagReversed != weightReversed)) ? -1 : 1;
        }
    }
    return 0;
}

/*
 * Apply the change (delta) in a stabilized axis's trim directly to servoParams()->mid
 * of every servo whose mixer rule is fed by that axis, so the new center point is
 * immediately visible (e.g. in the configurator's Servos tab) and persists like any
 * other live-adjusted config value. This is the relative path (switch-stepped
 * adjustment); the continuous path below never touches mid.
 */
static void applyServoAxisTrim(int axis, int newValue)
{
    const float delta = (float)newValue - servoAxisTrim[axis];
    servoAxisTrim[axis] = (float)newValue;

    if (delta == 0)
        return;

    const uint8_t count = getServoCount();
    for (int s = 0; s < count; s++) {
        const int dir = axisTrimDirection(axis, s);
        if (dir)
            servoParamsMutable(s)->mid += lrintf(dir * delta);
    }

    validateAndFixServoConfig();
}

/*
 * Runtime-only axis trim, for the continuous ("Absolute") adjustment mode where a
 * pot/channel position IS the trim. It is added at the servo output on top of the
 * center and never written to servoParams()->mid, so:
 *  - it can't be saved: if it were, the pot would apply itself again on top of its
 *    own saved result after every reboot (or save), walking the center away;
 *  - a bad channel reading (e.g. a channel that isn't valid yet at boot) can move a
 *    surface by at most SERVO_TRIM_LIMIT_PERCENT of the servo's scale, and leaves
 *    nothing behind once the reading is fixed.
 * It starts from zero at boot and follows the pot. The per-servo values are worked
 * out from the mixer rules as they are when the pot moves.
 */
int getServoAxisRuntimeTrim(int axis)
{
    return lrintf(servoRuntimeAxisTrim[axis]);
}

void setServoAxisRuntimeTrim(int axis, int value)
{
    servoRuntimeAxisTrim[axis] = value;

    for (int s = 0; s < MAX_SUPPORTED_SERVOS; s++) {
        float sum = 0;
        for (int a = 0; a < 3; a++) {
            sum += axisTrimDirection(a, s) * servoRuntimeAxisTrim[a];
        }
        servoRuntimeTrim[s] = sum;
    }
}

// What is actually added to the output, limited to its share of the servo's scale.
float getServoRuntimeTrim(uint8_t servo)
{
    const servoParam_t *param = servoParams(servo);
    const float limit = MAX(param->rneg, param->rpos) * SERVO_TRIM_LIMIT_PERCENT / 100;
    return constrainf(servoRuntimeTrim[servo], -limit, limit);
}

int get_ADJUSTMENT_SERVO_TRIM_ROLL(void)    { return lrintf(servoAxisTrim[0]); }
void set_ADJUSTMENT_SERVO_TRIM_ROLL(int v)  { applyServoAxisTrim(0, v); }

int get_ADJUSTMENT_SERVO_TRIM_PITCH(void)   { return lrintf(servoAxisTrim[1]); }
void set_ADJUSTMENT_SERVO_TRIM_PITCH(int v) { applyServoAxisTrim(1, v); }

int get_ADJUSTMENT_SERVO_TRIM_YAW(void)     { return lrintf(servoAxisTrim[2]); }
void set_ADJUSTMENT_SERVO_TRIM_YAW(int v)   { applyServoAxisTrim(2, v); }

/*
 * Re-baseline the runtime trim tracking to zero once the current servo
 * mid points have actually been persisted (written to EEPROM). The
 * adjustment range for SERVO_TRIM_* is clamped to +-200us of whatever
 * servoAxisTrim[] currently reads, so without this call that window would
 * stay relative to the value at boot rather than the last saved value,
 * letting repeated save cycles drift the servo center arbitrarily far.
 * The mid points themselves are already live-updated by applyServoAxisTrim(),
 * so this only resets the tracking, it does not touch servoParams()->mid.
 */
void servoTrimCommit(void)
{
    servoAxisTrim[0] = 0;
    servoAxisTrim[1] = 0;
    servoAxisTrim[2] = 0;
}

uint8_t getServoCount(void)
{
    return servoCount;
}

uint16_t getServoOutput(uint8_t servo)
{
#if defined(USE_SBUS_OUTPUT) || defined(USE_FBUS_MASTER)
    // Check if this is a bus servo (SBUS/FBUS)
    if (servo >= BUS_SERVO_OFFSET && servo < BUS_SERVO_OFFSET + BUS_SERVO_CHANNELS) {
        const uint8_t busServoIndex = servo - BUS_SERVO_OFFSET;
        return getBusServoOutput(busServoIndex);
    }
#endif
    // PWM servo
    if (servo >= MAX_SUPPORTED_SERVOS) {
        return 0;
    }
    return lrintf(servoOutput[servo]);
}

bool hasServoOverride(uint8_t servo)
{
    return (servoOverride[servo] >= SERVO_OVERRIDE_MIN && servoOverride[servo] <= SERVO_OVERRIDE_MAX);
}

int16_t getServoOverride(uint8_t servo)
{
    return servoOverride[servo];
}

int16_t setServoOverride(uint8_t servo, int16_t val)
{
    return servoOverride[servo] = val;
}

bool isServoOverrideActive(void)
{
    for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        if (hasServoOverride(i))
            return true;
    }
    return false;
}

void validateAndFixServoConfig(void)
{
    for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        volatile servoParam_t *servo = servoParamsMutable(i);
        const bool isBusServo = (i >= BUS_SERVO_OFFSET);
        const uint16_t minSignal = isBusServo ? BUS_SERVO_MIN_SIGNAL : PWM_SERVO_PULSE_MIN;
        const uint16_t maxSignal = isBusServo ? BUS_SERVO_MAX_SIGNAL : PWM_SERVO_PULSE_MAX;
        
#ifndef USE_SERVO_GEOMETRY_CORRECTION
        servo->flags &= ~SERVO_FLAG_GEO_CORR;
#endif

        // Constrain midpoint to the valid signal range.
        servo->mid = constrain(servo->mid, minSignal, maxSignal);

        // Constrain travel to valid offset limits first.
        servo->min = constrain(servo->min, SERVO_LIMIT_MIN, 0);
        servo->max = constrain(servo->max, 0, SERVO_LIMIT_MAX);

        // Ensure the resulting absolute signal stays within allowed range.
        const int16_t minAllowed = (int16_t)minSignal - (int16_t)servo->mid;
        const int16_t maxAllowed = (int16_t)maxSignal - (int16_t)servo->mid;

        if (servo->min < minAllowed) {
            servo->min = minAllowed;
        }
        if (servo->max > maxAllowed) {
            servo->max = maxAllowed;
        }
    }
}

void servoInit(void)
{
    const ioTag_t *ioTags = servoConfig()->ioTags;
    const timerHardware_t *timer[MAX_SUPPORTED_SERVOS];
    uint32_t rates[MAX_SUPPORTED_SERVOS];
    uint8_t index, jndex;

    for (index = 0; index < MAX_SUPPORTED_SERVOS; index++)
    {
        servoOutput[index] = servoParams(index)->mid;
        servoOverride[index] = SERVO_OVERRIDE_OFF;
    }

    for (index = 0; index < MAX_SUPPORTED_SERVOS && ioTags[index]; index++)
    {
        const ioTag_t tag = ioTags[index];
        const IO_t io = IOGetByTag(tag);

        timer[index] = timerAllocate(tag, OWNER_SERVO, RESOURCE_INDEX(index));

        if (!timer[index])
            break;

        IOInit(io, OWNER_SERVO, RESOURCE_INDEX(index));
#if defined(STM32F4) || defined(STM32F7) || defined(STM32H7) || defined(STM32G4)
        IOConfigGPIOAF(io, IOCFG_AF_PP, timer[index]->alternateFunction);
#else
        IOConfigGPIO(io, IOCFG_AF_PP);
#endif
    }

    servoCount = index;

    for (index = 0; index < servoCount; index++)
    {
        uint32_t update_rate = servoParams(index)->rate;

        for (jndex = 0; jndex < servoCount; jndex++) {
            if (timer[index]->tim == timer[jndex]->tim) {
                uint32_t maxpulse = servoParams(jndex)->mid + servoParams(jndex)->max;
                uint32_t maxrate = MIN(servoParams(jndex)->rate, 950000 / maxpulse);  // 1000000 / (maxpulse +5%)
                if (maxrate < update_rate)
                    update_rate = maxrate;
            }
        }

        rates[index] = constrain(update_rate, SERVO_RATE_MIN, SERVO_RATE_MAX);
    }

    for (index = 0; index < servoCount; index++)
    {
        const uint32_t timer_clock = timerClock(timer[index]->tim);
        const uint32_t update_rate = rates[index];
        uint32_t timebase;

        servoParamsMutable(index)->rate = update_rate;

        if (update_rate > 500) {
            const uint32_t timer_rate = update_rate * 64000;
            const uint32_t timer_div = (timer_clock + timer_rate - 1) / timer_rate;
            timebase = timer_clock / timer_div;
        }
        else if (update_rate > 154 && timer_clock % 10000000 == 0) {
            timebase = 10000000;
        }
        else if (update_rate > 124 && timer_clock % 8000000 == 0) {
            timebase = 8000000;
        }
        else if (update_rate > 77 && timer_clock % 5000000 == 0) {
            timebase = 5000000;
        }
        else if (update_rate > 62 && timer_clock % 4000000 == 0) {
            timebase = 4000000;
        }
        else if (update_rate > 31 && timer_clock % 2000000 == 0) {
            timebase = 2000000;
        }
        else {
            timebase = 1000000;
        }

        servoResolution[index] = timebase * 1e-6f;

        pwmOutConfig(&servoChannel[index], timer[index], timebase, timebase / update_rate, 0, 0);
    }
}

void servoShutdown(void)
{
    for (int index = 0; index < MAX_SUPPORTED_SERVOS; index++)
    {
        if (servoChannel[index].ccr) {
            *servoChannel[index].ccr = 0;
            servoChannel[index].ccr = NULL;
        }
    }

    delay(100);
}

static inline void servoSetOutput(uint8_t index, float pos)
{
    servoOutput[index] = pos;

    if (servoChannel[index].ccr)
        *servoChannel[index].ccr = lrintf(pos * servoResolution[index]);
}

static inline float limitTravel(uint8_t servo, float pos, float min, float max)
{
    if (pos > max) {
        mixerSaturateServoOutput(servo);
        return max;
    } else if (pos < min) {
        mixerSaturateServoOutput(servo);
        return min;
    }
    return pos;
}

static inline float limitSpeed(float old, float new, float speed)
{
    float rate = 1200 * pidGetDT() / speed;
    float diff = new - old;

    if (diff > rate)
        new = old + rate;
    else if (diff < -rate)
        new = old - rate;

    return new;
 }

 static inline float limitRatio(float old, float new, float ratio)
 {
    return old + (new - old) * ratio;
 }

#ifdef USE_SERVO_GEOMETRY_CORRECTION
float geometryCorrection(float pos)
{
    // 1.0 == 50° without correction
    float height = constrainf(pos * 0.7660444431f, -1, 1);

    // Scale 50° in rad => 1.0
    float rotation = asin_approx(height) * 1.14591559026f;

    return rotation;
}
#endif

void servoUpdate(void)
{
    float input[MAX_SUPPORTED_SERVOS];
    float cyclic_ratio = 1;

    for (int i = 0; i < servoCount; i++)
    {
        const servoParam_t *servo = servoParams(i);

        if (!ARMING_FLAG(ARMED) && hasServoOverride(i))
            input[i] = servoOverride[i] / 1000.0f;
        else
            input[i] = mixerGetServoOutput(i);

#ifdef USE_SERVO_GEOMETRY_CORRECTION
        if (servo->flags & SERVO_FLAG_GEO_CORR)
            input[i] = geometryCorrection(input[i]);
#endif

        // Servo balance curve: a small corrective delta added on top of the
        // servo's own output (not a full reshape), so two servos driving the
        // same surface can be trimmed to match each other's travel. An
        // unconfigured/disabled curve (count < 2) contributes 0 - the
        // fallback is a delta of nothing, not a passthrough of input[i].
        {
            const servoCurve_t *curve = servoCurves(i);
            input[i] += evaluateCurvePoints(curve->points, curve->count,
                                             input[i] * 1000.0f, 0.0f) / 1000.0f;
        }

        if (servo->speed && mixerIsCyclicServo(i)) {
            const float limit = 1200 * pidGetDT() / servo->speed;
            const float speed = fabsf(input[i] - servoInput[i]);
            if (speed > limit)
                cyclic_ratio = fminf(cyclic_ratio, limit / speed);
        }
    }

    for (int i = 0; i < servoCount; i++)
    {
        const servoParam_t *servo = servoParams(i);
        float pos = input[i];

        if (servo->speed > 0) {
            if (mixerIsCyclicServo(i))
                pos = limitRatio(servoInput[i], pos, cyclic_ratio);
            else
                pos = limitSpeed(servoInput[i], pos, servo->speed);
        }

        servoInput[i] = pos;

        if (servo->flags & SERVO_FLAG_REVERSED)
            pos = -pos;

        float scale = (pos > 0) ? servo->rpos : servo->rneg;

        // The runtime trim shifts the whole output but stays inside the servo's travel limits.
        pos = limitTravel(i, scale * pos + getServoRuntimeTrim(i), servo->min, servo->max);
        pos = servo->mid + pos;

        servoSetOutput(i, pos);
    }
}

#endif

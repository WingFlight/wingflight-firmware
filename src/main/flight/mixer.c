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
#include <ctype.h>
#include <math.h>

#include "platform.h"

#include "build/build_config.h"

#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"

#include "config/config.h"
#include "config/config_reset.h"

#include "fc/runtime_config.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/rc.h"

#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/logic_condition.h"

#include "rx/rx.h"

#include "pg/mixer.h"

#include "sensors/gyro.h"


/** Internal data **/

typedef struct {

    float           input[MIXER_INPUT_COUNT];
    float           output[MIXER_OUTPUT_COUNT];
    float           ruleOutput[MIXER_RULE_COUNT];  // per-rule slew state for mixerRule_t.speed

    bitmap_t        mapping[MIXER_OUTPUT_COUNT];
    int16_t         override[MIXER_INPUT_COUNT];
    uint16_t        saturation[MIXER_INPUT_COUNT];

    float           cyclicTotal;

    bitmap_t        cyclicMapping;

} mixerData_t;

static FAST_DATA_ZERO_INIT mixerData_t mixer;


/** Interface functions **/

float mixerGetInput(uint8_t index)
{
    return mixer.input[index];
}

float mixerGetOutput(uint8_t index)
{
    return mixer.output[index];
}

float getCyclicDeflection(void)
{
    return mixer.cyclicTotal;
}

bool mixerSaturated(uint8_t index)
{
    return (mixer.saturation[index] > 0);
}

void mixerSaturateInput(uint8_t index)
{
    mixer.saturation[index] = MIXER_SATURATION_TIME;
}

void mixerSaturateOutput(uint8_t index)
{
    for (int i = 1; i < MIXER_INPUT_COUNT; i++) {
        if (mixer.mapping[index] & BIT(i)) {
            mixerSaturateInput(i);
        }
    }
}

int16_t mixerGetOverride(uint8_t index)
{
    return mixer.override[index];
}

int16_t mixerSetOverride(uint8_t index, int16_t value)
{
    return mixer.override[index] = value;
}

bool isMixerOverrideActive(void)
{
    for (int i = 1; i < MIXER_INPUT_COUNT; i++) {
        const int16_t ovr = mixer.override[i];
        if ((ovr >= MIXER_OVERRIDE_MIN && ovr <= MIXER_OVERRIDE_MAX) || ovr == MIXER_OVERRIDE_PASSTHROUGH)
            return true;
    }
    return false;
}

bool mixerIsCyclicServo(uint8_t index)
{
    return (mixer.cyclicMapping & BIT(MIXER_SERVO_OFFSET + index));
}


/** Internal functions **/

static inline void mixerApplyInputLimit(int index, float value)
{
    const mixerInput_t *in = mixerInputs(index);

    // Input limits
    const float in_min = in->min / 1000.0f;
    const float in_max = in->max / 1000.0f;

    // Constrain and saturate
    if (value > in_max) {
        mixer.input[index] = in_max;
        mixerSaturateInput(index);
    }
    else if (value < in_min) {
        mixer.input[index] = in_min;
        mixerSaturateInput(index);
    }
    else {
        mixer.input[index] = value;
    }
}

/*
 * Check if the mixer index is one of the stabilized axes. If so,
 * return the overriden value (directly from RC). Otherwise, return original
 * value.
 */
static float mixerGetPassthroughInput(const int index,
                                      const float original_value)
{
    float rc = 0;
    switch (index) {
    case MIXER_IN_STABILIZED_ROLL:
        rc = getRcDeflection(ROLL);
        break;
    case MIXER_IN_STABILIZED_PITCH:
        rc = getRcDeflection(PITCH);
        break;
    case MIXER_IN_STABILIZED_YAW:
        // Normally, yaw command is reversed in setpoint.c (unlike other axes).
        // As we passthrough RC commands we want to keep the same reversal.
        rc = -getRcDeflection(YAW);
        break;
    default:
        return original_value;
    }

    // Scale rc by 120% for easier observing endpoints.
    rc *= 1.2f;

    if (rc > 0) {
        return scaleRangef(rc, 0, 1.0f, 0, mixerInputs(index)->max / 1000.0f);
    }
    return scaleRangef(rc, 0, -1.0f, 0, mixerInputs(index)->min / 1000.0f);
}

static void mixerSetInput(int index, float value)
{
    // Use override only if not armed
    if (!ARMING_FLAG(ARMED)) {
        if (mixer.override[index] >= MIXER_OVERRIDE_MIN && mixer.override[index] <= MIXER_OVERRIDE_MAX) {
            value = mixer.override[index] / 1000.0f;
        }
        else if (mixer.override[index] == MIXER_OVERRIDE_PASSTHROUGH) {
            value = mixerGetPassthroughInput(index, value);
        }
    }

    mixerApplyInputLimit(index, value);
}

static void mixerUpdateCyclic(void)
{
    const float SR = mixer.input[MIXER_IN_STABILIZED_ROLL];
    const float SP = mixer.input[MIXER_IN_STABILIZED_PITCH];

    // Total cyclic deflection (combined roll+pitch magnitude, used e.g. by
    // smartfuel's stick-load sag compensation)
    mixer.cyclicTotal = sqrtf(sq(SP) + sq(SR));
}

// Linear interpolation through a curve's (ascending-x) points. Points beyond
// either end clamp to that end's y. Curves have at most MIXER_CURVE_POINTS
// (9) points, so a linear scan is negligible cost.
static float mixerEvaluateCurve(const mixerCurve_t *curve, float x)
{
    const int n = curve->count;

    if (n < 2)
        return x;

    const float xs = x * 1000.0f;

    if (xs <= curve->points[0].x)
        return curve->points[0].y / 1000.0f;

    if (xs >= curve->points[n - 1].x)
        return curve->points[n - 1].y / 1000.0f;

    for (int i = 0; i < n - 1; i++) {
        const mixerCurvePoint_t *p0 = &curve->points[i];
        const mixerCurvePoint_t *p1 = &curve->points[i + 1];

        if (xs >= p0->x && xs <= p1->x) {
            const float t = (p1->x != p0->x) ? (xs - p0->x) / (float)(p1->x - p0->x) : 0;
            return (p0->y + t * (p1->y - p0->y)) / 1000.0f;
        }
    }

    return x;
}

static void mixerUpdateRules(void)
{
    for (int i = 0; i < MIXER_RULE_COUNT; i++) {
        if (mixerRules(i)->oper) {
            if (mixerRules(i)->condition > 0 &&
                !logicConditionGetValue(mixerRules(i)->condition - 1)) {
                continue;   // gated off - this rule contributes nothing this cycle
            }

            uint8_t src = mixerRules(i)->input;
            uint8_t dst = mixerRules(i)->output;
            float   val = mixer.input[src] * mixerInputs(src)->rate / 1000.0f;

            if (mixerRules(i)->curve > 0 && mixerRules(i)->curve <= MIXER_CURVE_COUNT) {
                val = mixerEvaluateCurve(mixerCurves(mixerRules(i)->curve - 1), val);
            }

            int16_t weight = (val >= 0) ? mixerRules(i)->weight : mixerRules(i)->weightNeg;
            float   out = (mixerRules(i)->offset + weight * val) / 1000.0f;

            if (mixerRules(i)->speed > 0) {
                out = slewLimit(mixer.ruleOutput[i], out, 1200.0f * pidGetDT() / mixerRules(i)->speed);
            }
            mixer.ruleOutput[i] = out;

            switch (mixerRules(i)->oper)
            {
                case MIXER_OP_SET:
                    mixer.output[dst] = out;
                    break;
                case MIXER_OP_ADD:
                    mixer.output[dst] += out;
                    break;
                case MIXER_OP_MUL:
                    mixer.output[dst] *= out;
                    break;
            }
        }
    }
}

static void mixerUpdateInputs(void)
{
    // Flight Dynamics
    mixerSetInput(MIXER_IN_RC_COMMAND_ROLL, getRcDeflection(ROLL));
    mixerSetInput(MIXER_IN_RC_COMMAND_PITCH, getRcDeflection(PITCH));
    mixerSetInput(MIXER_IN_RC_COMMAND_YAW, getRcDeflection(YAW));

    // Throttle input
    mixerSetInput(MIXER_IN_RC_COMMAND_THROTTLE, getThrottle());

    // RC channels
    for (int i = 0; i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++)
        mixerSetInput(MIXER_IN_RC_CHANNEL_ROLL + i, rcCommand[i] / 500);

    // Stabilised inputs
    mixerSetInput(MIXER_IN_STABILIZED_ROLL, pidGetOutput(PID_ROLL));
    mixerSetInput(MIXER_IN_STABILIZED_PITCH, pidGetOutput(PID_PITCH));
    mixerSetInput(MIXER_IN_STABILIZED_YAW, pidGetOutput(PID_YAW));

    // BOXPASSTHROUGH mode: replace stabilized inputs with raw RC channels
    if (IS_RC_MODE_ACTIVE(BOXPASSTHROUGH)) {
        mixer.input[MIXER_IN_STABILIZED_ROLL]  = mixer.input[MIXER_IN_RC_CHANNEL_ROLL];
        mixer.input[MIXER_IN_STABILIZED_PITCH] = mixer.input[MIXER_IN_RC_CHANNEL_PITCH];
        // Yaw command is reversed in setpoint.c relative to raw RC (unlike other axes);
        // keep the same reversal here so passthrough yaw direction matches stabilized.
        mixer.input[MIXER_IN_STABILIZED_YAW]   = -mixer.input[MIXER_IN_RC_CHANNEL_YAW];
    }

    // Calculate cyclic
    mixerUpdateCyclic();

    // Update throttle (no governor -- direct passthrough)
    mixerSetInput(MIXER_IN_STABILIZED_THROTTLE, getThrottle());
}

void mixerUpdate(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    // Reset saturation
    for (int i = 0; i < MIXER_INPUT_COUNT; i++) {
        if (mixer.saturation[i])
            mixer.saturation[i]--;
    }

    // Reset mixer outputs
    for (int i = 0; i < MIXER_OUTPUT_COUNT; i++) {
        mixer.output[i] = 0;
    }

    // Fetch input values
    mixerUpdateInputs();

    // Evaluate logic conditions used to gate mixer rules
    logicConditionUpdate();

    // Evaluate rule-based mixer
    mixerUpdateRules();
}

void INIT_CODE validateAndFixMixerConfig(void)
{
    for (int i = 0; i < MIXER_RULE_COUNT; i++)
    {
        mixerRule_t *rule = mixerRulesMutable(i);

        if (rule->oper) {
            rule->oper    = constrain(rule->oper, 0, MIXER_OP_COUNT - 1);
            rule->input   = constrain(rule->input, 0, MIXER_INPUT_COUNT - 1);
            rule->output  = constrain(rule->output, 0, MIXER_OUTPUT_COUNT - 1);
            rule->offset    = constrain(rule->offset, MIXER_INPUT_MIN, MIXER_INPUT_MAX);
            rule->weight    = constrain(rule->weight, MIXER_WEIGHT_MIN, MIXER_WEIGHT_MAX);
            rule->weightNeg = constrain(rule->weightNeg, MIXER_WEIGHT_MIN, MIXER_WEIGHT_MAX);
        }
        else {
            rule->oper      = 0;
            rule->input     = 0;
            rule->output    = 0;
            rule->offset    = 0;
            rule->weight    = 0;
            rule->weightNeg = 0;
        }
    }

}

static void INIT_CODE setMapping(uint8_t in, uint8_t out)
{
    mixer.mapping[out] = BIT(in);

    if (in == MIXER_IN_STABILIZED_ROLL || in == MIXER_IN_STABILIZED_PITCH ||
        in == MIXER_IN_RC_COMMAND_ROLL || in == MIXER_IN_RC_COMMAND_PITCH) {
        mixer.cyclicMapping |= BIT(out);
    }
}

static void INIT_CODE addMapping(uint8_t in, uint8_t out)
{
    mixer.mapping[out] |= BIT(in);

    if (in == MIXER_IN_STABILIZED_ROLL || in == MIXER_IN_STABILIZED_PITCH ||
        in == MIXER_IN_RC_COMMAND_ROLL || in == MIXER_IN_RC_COMMAND_PITCH) {
        mixer.cyclicMapping |= BIT(out);
    }
}

#define addServoMapping(INDEX,SERVO)    addMapping((INDEX), MIXER_SERVO_OFFSET + (SERVO))
#define addMotorMapping(INDEX,MOTOR)    addMapping((INDEX), MIXER_MOTOR_OFFSET + (MOTOR))

void INIT_CODE mixerInit(void)
{
    for (int i = 0; i < MIXER_OUTPUT_COUNT; i++) {
        mixer.output[i] = 0;
        mixer.mapping[i] = 0;
    }

    for (int i = 1; i < MIXER_INPUT_COUNT; i++) {
        mixer.override[i] = MIXER_OVERRIDE_OFF;
    }

    for (int i = 0; i < MIXER_RULE_COUNT; i++)
    {
        const mixerRule_t *rule = mixerRules(i);

        switch (rule->oper)
        {
            case MIXER_OP_SET:
                setMapping(rule->input, rule->output);
                break;
            case MIXER_OP_ADD:
            case MIXER_OP_MUL:
                addMapping(rule->input, rule->output);
                break;
        }
    }
}

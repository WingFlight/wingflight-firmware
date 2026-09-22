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
#include "common/curve.h"
#include "common/filter.h"
#include "common/maths.h"

#include "config/config.h"
#include "config/config_reset.h"
#include "config/feature.h"

#include "fc/runtime_config.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/rc.h"

#include "flight/autohover.h"
#include "flight/pid.h"
#include "flight/tv_pid.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/wiggle.h"
#include "flight/logic_condition.h"
#include "flight/governor.h"
#include "flight/setpoint.h"

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

/** Internal functions **/

static inline void mixerApplyInputLimit(int index, float value)
{
    const mixerInput_t *in = mixerInputs(index);

    // Input limits
    const float in_min = in->min / 1000.0f;
    const float in_max = in->max / 1000.0f;

    // Constrain and saturate. +-Inf are still correctly caught below even
    // under -ffast-math (see constrainf() in common/maths.h). A NaN input
    // (a bad sensor read, a PID/setpoint computation gone wrong, a
    // misconfigured rc.c deadband/deflection pair, ...) fails every
    // comparison and would otherwise fall through to the final `else` and
    // reach every servo/motor fed from this input untouched -- isfinitef()
    // catches that remaining case without changing how Inf is already
    // handled. isnan() itself can't be used for this; see isfinitef().
    if (value > in_max) {
        mixer.input[index] = in_max;
        mixerSaturateInput(index);
    }
    else if (value < in_min) {
        mixer.input[index] = in_min;
        mixerSaturateInput(index);
    }
    else if (!isfinitef(value)) {
        mixer.input[index] = 0;
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
    // Use override or wiggle only if not armed
    if (!ARMING_FLAG(ARMED)) {
        if (mixer.override[index] >= MIXER_OVERRIDE_MIN && mixer.override[index] <= MIXER_OVERRIDE_MAX) {
            value = mixer.override[index] / 1000.0f;
        }
        else if (mixer.override[index] == MIXER_OVERRIDE_PASSTHROUGH) {
            value = mixerGetPassthroughInput(index, value);
        }
        else if (wiggleActive()) {
            if (index >= MIXER_IN_STABILIZED_ROLL && index <= MIXER_IN_STABILIZED_YAW)
                value = wiggleGetAxis(index - MIXER_IN_STABILIZED_ROLL);
        }
    }

    mixerApplyInputLimit(index, value);
}

// Curves have at most MIXER_CURVE_POINTS (9) points, so a linear scan is
// negligible cost.
static float mixerEvaluateCurve(const mixerCurve_t *curve, float x)
{
    return evaluateCurvePoints(curve->points, curve->count, x * 1000.0f, x * 1000.0f) / 1000.0f;
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

            // Guard before this feeds back into ruleOutput: a NaN stored here
            // would poison every future slewLimit() call on this rule forever
            // (NaN - NaN is still NaN), and would otherwise reach mixer.output[]
            // -- and from there every servo/motor -- untouched.
            if (!isfinitef(out))
                out = 0;
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

    // Independent Thrust Vector stabilised inputs. Left at their zero default
    // when the feature is disabled, so any mixer rule referencing them is a
    // harmless no-op.
    if (featureIsEnabled(FEATURE_THRUST_VECTOR)) {
        mixerSetInput(MIXER_IN_STABILIZED_TV_ROLL, tvPidGetOutput(PID_ROLL));
        mixerSetInput(MIXER_IN_STABILIZED_TV_PITCH, tvPidGetOutput(PID_PITCH));
        mixerSetInput(MIXER_IN_STABILIZED_TV_YAW, tvPidGetOutput(PID_YAW));
    }

    // BOXPASSTHROUGH mode: replace stabilized inputs with raw RC channels, bypassing the
    // rates/expo curve as well as PID - direct radio to surfaces. Takes priority over MANUAL
    // if both happen to be active at once.
    if (IS_RC_MODE_ACTIVE(BOXPASSTHROUGH)) {
        mixer.input[MIXER_IN_STABILIZED_ROLL]  = mixer.input[MIXER_IN_RC_CHANNEL_ROLL];
        mixer.input[MIXER_IN_STABILIZED_PITCH] = mixer.input[MIXER_IN_RC_CHANNEL_PITCH];
        // Yaw command is reversed in setpoint.c relative to raw RC (unlike other axes);
        // keep the same reversal here so passthrough yaw direction matches stabilized.
        mixer.input[MIXER_IN_STABILIZED_YAW]   = -mixer.input[MIXER_IN_RC_CHANNEL_YAW];
        // No raw RC channel is mapped to the independent TV axes, so the only safe
        // bypass is neutral: zero the TV stabilized inputs rather than leave any
        // TV-driven actuator still under PID stabilization during a passthrough bailout.
        mixer.input[MIXER_IN_STABILIZED_TV_ROLL]  = 0;
        mixer.input[MIXER_IN_STABILIZED_TV_PITCH] = 0;
        mixer.input[MIXER_IN_STABILIZED_TV_YAW]   = 0;
    }
    // BOXMANUAL mode: replace stabilized inputs with the same rates/expo-shaped setpoint the
    // PID rate loop targets, but skip the gyro-corrected PID output itself - same stick feel as
    // stabilized flight, no stabilization. getManualDeflection() already matches the stabilized
    // sign convention (yaw included), so no extra reversal is needed here.
    else if (IS_RC_MODE_ACTIVE(BOXMANUAL)) {
        mixer.input[MIXER_IN_STABILIZED_ROLL]  = getManualDeflection(FD_ROLL);
        mixer.input[MIXER_IN_STABILIZED_PITCH] = getManualDeflection(FD_PITCH);
        mixer.input[MIXER_IN_STABILIZED_YAW]   = getManualDeflection(FD_YAW);
    }

    // Update throttle (governor holds RPM/throttle per its configured mode when BOXGOVERNOR is engaged)
    float throttle = getThrottle();
#ifdef USE_ACC
    // AUTOHOVER's optional throttle assist (disabled by default) is added here, before governorApply,
    // so any governor-side slew/ceiling still applies on top as a second layer of limiting. It's a
    // no-op (returns 0) whenever the mode is inactive or the assist isn't configured/triggered.
    throttle = constrainf(throttle + autoHoverThrottleBoost(), 0.0f, 1.0f);
#endif
    mixerSetInput(MIXER_IN_STABILIZED_THROTTLE, governorApply(throttle));
}

void mixerUpdate(timeUs_t currentTimeUs)
{
    // Reset saturation
    for (int i = 0; i < MIXER_INPUT_COUNT; i++) {
        if (mixer.saturation[i])
            mixer.saturation[i]--;
    }

    // Reset mixer outputs
    for (int i = 0; i < MIXER_OUTPUT_COUNT; i++) {
        mixer.output[i] = 0;
    }

    // Update wiggles
    wiggleUpdate(currentTimeUs);

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
            rule->role      = constrain(rule->role, 0, MIXER_RULE_ROLE_COUNT - 1);
            mixerCaptureRuleSign(i);
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

/*
 * Remembers the pilot-configured sign of each rule's weight (Reverse in the
 * mixer table, or a negative weight via CLI), independent of mixerRule_t
 * itself -- applyRoleWeight() below needs this because it continuously
 * overwrites weight with a live-scaled *magnitude*, and that magnitude
 * legitimately passes through exactly 0 (the low end of an adjustment's
 * range, or a freshly wizard-generated compensation rule before it's
 * tuned). 0 has no sign, so deriving "positive or negative" from weight's
 * own live value -- as an earlier version of this did -- loses the
 * pilot's configured polarity the moment it crosses zero, and silently
 * defaults back to positive on the next nonzero write, with no user
 * action at all. Captured fresh from the actual configurator/CLI-supplied
 * value every time one writes a rule (see msp.c's MSP_SET_MIXER_RULE and
 * cli.c's `mixer rule` handlers), plus once at boot here for whatever was
 * last persisted to EEPROM -- never from applyRoleWeight()'s own writes.
 */
static int8_t mixerRuleSign[MIXER_RULE_COUNT];

void mixerCaptureRuleSign(uint8_t index)
{
    if (index < MIXER_RULE_COUNT) {
        mixerRuleSign[index] = (mixerRules(index)->weight >= 0) ? 1 : -1;
    }
}

/*
 * Reads or writes the weight *magnitude* of every active rule tagged with a
 * given mixerRuleRole_e, for RC adjustment functions (fc/rc_adjustments.c)
 * that need to live-tune a rule without a fixed index -- nothing in this
 * codebase reserves fixed rule slots (pg/mixer.h), and the rule table is
 * freely reordered by the configurator's rule editor, so a tag is the
 * only stable handle. `oper` gates "active" the same way it does
 * everywhere else a rule's liveness is checked (mixerUpdateRules(),
 * configurator's isNullRule(), the LUA suite's isEmpty()).
 *
 * *value is a signed scale applied on top of each matching rule's
 * configured polarity (via mixerRuleSign[], not weight's own live value --
 * see that comment): weight = sign * value. Positive values keep the
 * configured direction, negative values flip it, so one adjustment can
 * sweep a rule through both directions while Reverse in the mixer table
 * still means what it says. An earlier version of this wrote the
 * adjustment's own raw value straight into the first matching rule,
 * silently overwriting whatever sign the pilot had configured on the very
 * next tick; a later one only ever scaled |weight|, which fixed that but
 * made negative values impossible. Per-rule sign is still what a
 * differential-thrust-yaw pair needs (the two rules are tagged the same
 * role but opposite sign by design -- one motor speeds up, the other
 * slows down) and what several same-signed flap-compensation rules on a
 * v-tail/flying-wing need: a negative value flips every match together
 * and their relative polarity is untouched.
 */
static bool applyRoleWeight(uint8_t role, int *value, bool write)
{
    bool found = false;

    for (int i = 0; i < MIXER_RULE_COUNT; i++) {
        mixerRule_t *rule = mixerRulesMutable(i);
        if (!rule->oper || rule->role != role) {
            continue;
        }

        if (write) {
            rule->weight    = (mixerRuleSign[i] >= 0) ? *value : -*value;
            rule->weightNeg = rule->weight;
        } else if (!found) {
            *value = (mixerRuleSign[i] >= 0) ? rule->weight : -rule->weight;
        }

        found = true;
    }

    return found;
}

int get_ADJUSTMENT_FLAP_COMPENSATION_GAIN(void)
{
    int value = 0;
    applyRoleWeight(MIXER_RULE_ROLE_FLAP_COMPENSATION, &value, false);
    return value;
}

void set_ADJUSTMENT_FLAP_COMPENSATION_GAIN(int value)
{
    applyRoleWeight(MIXER_RULE_ROLE_FLAP_COMPENSATION, &value, true);
}

int get_ADJUSTMENT_DIFF_THRUST_YAW_GAIN(void)
{
    int value = 0;
    applyRoleWeight(MIXER_RULE_ROLE_DIFFERENTIAL_THRUST_YAW, &value, false);
    return value;
}

void set_ADJUSTMENT_DIFF_THRUST_YAW_GAIN(int value)
{
    applyRoleWeight(MIXER_RULE_ROLE_DIFFERENTIAL_THRUST_YAW, &value, true);
}

static void INIT_CODE setMapping(uint8_t in, uint8_t out)
{
    mixer.mapping[out] = BIT(in);
}

static void INIT_CODE addMapping(uint8_t in, uint8_t out)
{
    mixer.mapping[out] |= BIT(in);
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

    wiggleInit();
}

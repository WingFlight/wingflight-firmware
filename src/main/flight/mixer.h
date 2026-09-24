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
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "pg/mixer.h"
#include "pg/adjustments.h"

#include "drivers/io_types.h"
#include "drivers/pwm_output.h"

#include "flight/servos.h"
#include "flight/motors.h"


/** Mixer constants **/

#define MIXER_OUTPUT_COUNT    (1 + MAX_SUPPORTED_SERVOS + MAX_SUPPORTED_MOTORS)

// Mixer output numbers are stored in mixer rules, CLI diffs and backups, so
// they must not move. S1-S26 come first (outputs 1-26), then the motors
// (27-30), then the bus servos added for 24-channel F.Bus, S27-S32 (31-36).
#define MIXER_SERVO_LOW_MAX   26
#if MAX_SUPPORTED_SERVOS > MIXER_SERVO_LOW_MAX
#define MIXER_SERVO_LOW_COUNT MIXER_SERVO_LOW_MAX
#else
#define MIXER_SERVO_LOW_COUNT MAX_SUPPORTED_SERVOS
#endif

#define MIXER_SERVO_OFFSET    1
#define MIXER_MOTOR_OFFSET    (MIXER_SERVO_OFFSET + MIXER_SERVO_LOW_COUNT)
#define MIXER_SERVO_HIGH_OFFSET (MIXER_MOTOR_OFFSET + MAX_SUPPORTED_MOTORS)

#define MIXER_RATE_MIN       -10000
#define MIXER_RATE_MAX        10000

#define MIXER_WEIGHT_MIN     -10000
#define MIXER_WEIGHT_MAX      10000

#define MIXER_INPUT_MIN      -2500
#define MIXER_INPUT_MAX       2500

#define MIXER_CURVE_MIN      -1000
#define MIXER_CURVE_MAX       1000

#define MIXER_OVERRIDE_MIN   -2500
#define MIXER_OVERRIDE_MAX    2500
#define MIXER_OVERRIDE_OFF    (MIXER_OVERRIDE_MAX + 1)
#define MIXER_OVERRIDE_PASSTHROUGH  (MIXER_OVERRIDE_MAX + 2)

#define MIXER_SATURATION_TIME 5


/** Interface function **/

void mixerInit(void);

void validateAndFixMixerConfig(void);

// Re-latches mixerRuleSign[index] from mixerRules(index)->weight's current
// sign -- call this whenever a rule's weight is written from the
// configurator/CLI (see flight/mixer.c's own comment on why this can't just
// be inferred from weight's live value at adjustment time).
void mixerCaptureRuleSign(uint8_t index);

ADJFUN_DECLARE(FLAP_COMPENSATION_GAIN)
ADJFUN_DECLARE(DIFF_THRUST_YAW_GAIN)

void mixerUpdate(timeUs_t currentTimeUs);

float mixerGetInput(uint8_t index);

float mixerGetOutput(uint8_t index);

bool mixerSaturated(uint8_t index);
void mixerSaturateInput(uint8_t index);
void mixerSaturateOutput(uint8_t index);

int16_t mixerGetOverride(uint8_t index);
int16_t mixerSetOverride(uint8_t index, int16_t value);
bool    isMixerOverrideActive(void);


/** Inline functions **/

static inline uint8_t mixerServoOutputIndex(uint8_t servo)
{
    return (servo < MIXER_SERVO_LOW_COUNT) ?
        MIXER_SERVO_OFFSET + servo :
        MIXER_SERVO_HIGH_OFFSET + (servo - MIXER_SERVO_LOW_COUNT);
}

static inline float mixerGetServoOutput(uint8_t index)
{
    return mixerGetOutput(mixerServoOutputIndex(index));
}

static inline float mixerGetMotorOutput(uint8_t index)
{
    return mixerGetOutput(MIXER_MOTOR_OFFSET + index);
}

static inline float getYawDeflection(void)
{
    return mixerGetInput(MIXER_IN_STABILIZED_YAW);
}

static inline float getYawDeflectionAbs(void)
{
    return fabsf(mixerGetInput(MIXER_IN_STABILIZED_YAW));
}

static inline float mixerGetThrottle(void)
{
    return mixerGetInput(MIXER_IN_RC_COMMAND_THROTTLE);
}

static inline bool pidAxisSaturated(uint8_t index)
{
    return mixerSaturated(MIXER_IN_STABILIZED_ROLL + index);
}

static inline bool tvPidAxisSaturated(uint8_t index)
{
    return mixerSaturated(MIXER_IN_STABILIZED_TV_ROLL + index);
}

static inline void mixerSaturateServoOutput(uint8_t index)
{
    mixerSaturateOutput(mixerServoOutputIndex(index));
}

static inline void mixerSaturateMotorOutput(uint8_t index)
{
    mixerSaturateOutput(index + MIXER_MOTOR_OFFSET);
}

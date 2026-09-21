/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#ifdef USE_ACRO_TRAINER

#include "build/build_config.h"
#include "build/debug.h"

#include "config/config.h"

#include "common/axis.h"
#include "common/filter.h"
#include "common/maths.h"

#include "fc/core.h"
#include "fc/runtime_config.h"

#include "sensors/acceleration.h"
#include "sensors/gyro.h"

#include "flight/imu.h"
#include "flight/trainer.h"


#define ACRO_TRAINER_LOOKAHEAD_RATE_LIMIT 500.0f    // Max gyro rate for lookahead time scaling
#define ACRO_TRAINER_SETPOINT_LIMIT       1000.0f   // Limit the correcting setpoint

typedef int8_t sign_t;

typedef struct {
    bool        Active;
    bool        Limiting[2];
    float       Gain;
    float       AngleLimit[2];
    float       LookaheadTime;
} acroTrainer_t;

static FAST_DATA_ZERO_INIT acroTrainer_t acroTrainer;

int get_ADJUSTMENT_ACRO_TRAINER_GAIN(void)
{
    return currentPidProfile->trainer.gain;
}

void set_ADJUSTMENT_ACRO_TRAINER_GAIN(int value)
{
    currentPidProfile->trainer.gain = value;
    acroTrainer.Gain = value / 10.0f;
}

INIT_CODE void acroTrainerInit(const pidProfile_t *pidProfile)
{
    memset(acroTrainer.Limiting, 0, sizeof(acroTrainer.Limiting));
    acroTrainer.Gain = pidProfile->trainer.gain / 10.0f;
    const attitudeLimits_t *limits = attitudeLimits(getCurrentPidProfileIndex());
    acroTrainer.AngleLimit[FD_ROLL] = attitudeLimitDegrees(limits->trainer_roll, pidProfile->trainer.angle_limit, FD_ROLL);
    acroTrainer.AngleLimit[FD_PITCH] = attitudeLimitDegrees(limits->trainer_pitch, pidProfile->trainer.angle_limit, FD_PITCH);
    acroTrainer.LookaheadTime = pidProfile->trainer.lookahead_ms / 1000.0f;
}

void acroTrainerSetState(bool state)
{
    acroTrainer.Active = state;
    if (!state) {
        memset(acroTrainer.Limiting, 0, sizeof(acroTrainer.Limiting));
    }
}

bool acroTrainerIsLimiting(int axis)
{
    return acroTrainer.Active && (axis == FD_ROLL || axis == FD_PITCH)
        && acroTrainer.Limiting[axis];
}

static inline sign_t Sign(float x)
{
    return (x > 0) ? 1 : -1;
}

//
// Acro Trainer - Manipulate the setPoint to limit axis angle while in acro mode
//
// 1. Current angle has exceeded limit
//    Apply proportional correction to return to limit, while allowing
//    pilot input that helps return (stateless, no latch)
// 2. Current angle is within limits but projected angle exceeds limit
//    Limit the setPoint to control the gyro rate as the angle
//    approaches the limit (try to prevent overshoot)
// 3. No correction needed, return the original setPoint
//

float acroTrainerApply(int axis, float setPoint)
{
    if (acroTrainer.Active && (axis == FD_ROLL || axis == FD_PITCH))
    {
        const rollAndPitchTrims_t *angleTrim = &accelerometerConfig()->accelerometerTrims;
        const float currentAngle = (attitude.raw[axis] - angleTrim->raw[axis]) / 10.0f;
        const float angleLimit = acroTrainer.AngleLimit[axis];
        const float angleExcess = fabsf(currentAngle) - angleLimit;
        float projectedAngle = 0;
        const float pilotSetPoint = setPoint;

        if (angleExcess > 0) {
            // Angle exceeds the limit: apply correction proportional to excess angle.
            // The correction is always directed back toward the limit (stateless).
            const sign_t angleSign = Sign(currentAngle);
            const float correction = limitf((angleLimit * angleSign - currentAngle) * acroTrainer.Gain, ACRO_TRAINER_SETPOINT_LIMIT);

            // Allow pilot input that helps return, block input that drives further out
            if (angleSign > 0) {
                setPoint = fminf(setPoint, correction);
            } else {
                setPoint = fmaxf(setPoint, correction);
            }
        }
        else {
            // Within limits: project the angle based on current angle and gyro rate.
            // If the projected angle exceeds the limit then apply limiting to minimize overshoot.
            const float checkInterval = constrainf(fabsf(gyro.gyroADCf[axis]) / ACRO_TRAINER_LOOKAHEAD_RATE_LIMIT, 0.0f, 1.0f) * acroTrainer.LookaheadTime;
            projectedAngle = (gyro.gyroADCf[axis] * checkInterval) + currentAngle;

            const sign_t projectedAngleSign = Sign(projectedAngle);
            
            if ((fabsf(projectedAngle) > angleLimit) && (projectedAngleSign == Sign(setPoint))) {
                setPoint = limitf(((angleLimit * projectedAngleSign) - projectedAngle) * acroTrainer.Gain, ACRO_TRAINER_SETPOINT_LIMIT);
            }
        }

        // Helping stick input can pass through even outside the envelope. Only
        // retain I while the limiter actually changes this axis's rate command.
        acroTrainer.Limiting[axis] = setPoint != pilotSetPoint;

        DEBUG_AXIS(ACRO_TRAINER, axis, 0, currentAngle * 10);
        DEBUG_AXIS(ACRO_TRAINER, axis, 1, acroTrainer.Limiting[axis]);
        DEBUG_AXIS(ACRO_TRAINER, axis, 2, setPoint);
        DEBUG_AXIS(ACRO_TRAINER, axis, 3, projectedAngle * 10);
    }

    return setPoint;
}

#endif

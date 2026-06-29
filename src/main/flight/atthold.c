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
#include <math.h>

#include "platform.h"

#ifdef USE_ACC

#include "build/build_config.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"

#include "config/config.h"

#include "fc/rc.h"

#include "flight/imu.h"
#include "flight/airborne.h"

#include "atthold.h"

typedef struct {
    float Gain;
    float Deadband;
    float target[XYZ_AXIS_COUNT];
} attHold_t;

static FAST_DATA_ZERO_INIT attHold_t attHold;


INIT_CODE void attHoldModeInit(const pidProfile_t *pidProfile)
{
    attHold.Gain = pidProfile->atthold.gain / 10.0f;
    attHold.Deadband = pidProfile->atthold.deadband / 100.0f;
}

int get_ADJUSTMENT_ATTHOLD_GAIN(void)
{
    return currentPidProfile->atthold.gain;
}

void set_ADJUSTMENT_ATTHOLD_GAIN(int value)
{
    currentPidProfile->atthold.gain = value;
    attHold.Gain = value / 10.0f;
}

// Called once on the rising edge of ATTHOLD_MODE so a stick that's already
// centered at the moment the mode is switched on doesn't lock onto stale data.
void attHoldModeSetState(bool state)
{
    if (state) {
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            attHold.target[axis] = attitude.raw[axis] / 10.0f;
        }
    }
}

// Shortest signed distance from `from` to `to` in degrees, wrapped to (-180,180].
static float wrap180(float to, float from)
{
    float delta = fmodf(to - from + 180.0f, 360.0f);
    if (delta < 0)
        delta += 360.0f;
    return delta - 180.0f;
}

float attHoldModeApply(int axis, float pidSetpoint)
{
    const float currentAngle = attitude.raw[axis] / 10.0f;
    const float deflection = fabsf(getRcDeflection(axis));

    if (!isAirborne() || deflection > attHold.Deadband) {
        // Pilot is actively commanding this axis (or not airborne yet) --
        // keep the hold target tracking attitude so a future re-lock is
        // seamless: error is ~0 the instant the stick returns to center.
        attHold.target[axis] = currentAngle;

        DEBUG_AXIS(ATTHOLD, axis, 0, currentAngle * 10);
        DEBUG_AXIS(ATTHOLD, axis, 1, attHold.target[axis] * 10);
        DEBUG_AXIS(ATTHOLD, axis, 2, 0);
        DEBUG_AXIS(ATTHOLD, axis, 3, pidSetpoint);

        return pidSetpoint;
    }

    const float error = wrap180(attHold.target[axis], currentAngle);
    const float correction = error * attHold.Gain;

    DEBUG_AXIS(ATTHOLD, axis, 0, currentAngle * 10);
    DEBUG_AXIS(ATTHOLD, axis, 1, attHold.target[axis] * 10);
    DEBUG_AXIS(ATTHOLD, axis, 2, error * 10);
    DEBUG_AXIS(ATTHOLD, axis, 3, pidSetpoint + correction);

    return pidSetpoint + correction;
}

#endif // USE_ACC

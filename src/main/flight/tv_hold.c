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

#include "common/maths.h"

#include "flight/airborne.h"
#include "flight/imu.h"
#include "flight/pid.h"
#include "flight/setpoint.h"

#include "tv_hold.h"

// Independent quaternion-based attitude/heading hold for the Thrust Vector loop
// (FEATURE_THRUST_VECTOR) -- engaged by its own switch (BOXTVHOLD), completely
// decoupled from the main loop's ANGLE/AUTOHOVER/ATTHOLD chain in pid.c. This is
// what makes it possible to hold heading/attitude on the vectored nozzle while
// the aerodynamic control surfaces stay in plain rate/acro under the pilot's
// stick.
//
// Deliberately a second, independent instance of atthold.c's track/freeze
// algorithm rather than shared state or a call into atthold.c -- same "not
// sharing state" reasoning tv_pid.c already documents for itself: this hold
// engine needs to run (or not) regardless of what the main loop's flight mode
// is doing, and must stay tunable/removable on its own. See atthold.c for the
// full rationale behind the quaternion track/freeze model and the singularity
// handling below; this is a straight port of that algorithm onto its own state.

typedef struct {
    bool        Active;
    bool        Tracking;
    float       Gain;
    float       Deadband;
    float       MaxRate;
    quaternion  qTarget;
} tvHold_t;

static FAST_DATA_ZERO_INIT tvHold_t tvHold;

INIT_CODE void tvHoldInit(const tvPidProfile_t *profile)
{
    tvHold.Gain = profile->hold.gain / 10.0f;
    tvHold.Deadband = profile->hold.deadband / 100.0f;
    tvHold.MaxRate = profile->hold.max_rate;
}

// Called once on the rising edge of BOXTVHOLD so a stale target from a
// previous engagement can never linger -- mirrors attHoldSetState.
void tvHoldSetState(bool state)
{
    if (state && !tvHold.Active) {
        getQuaternion(&tvHold.qTarget);
    }

    tvHold.Active = state;
}

float tvHoldApply(int axis, float pidSetpoint)
{
    static float rate[3];

    if (!tvHold.Active) {
        return pidSetpoint;
    }

    // Shared, cross-axis work only needs computing once per PID loop iteration --
    // do it on the first axis touched each iteration and cache it, same pattern
    // attHoldApply uses.
    if (axis == PID_ROLL) {
        const bool sticksActive = !isAirborne()
            || fabsf(getDeflection(PID_ROLL))  > tvHold.Deadband
            || fabsf(getDeflection(PID_PITCH)) > tvHold.Deadband
            || fabsf(getDeflection(PID_YAW))   > tvHold.Deadband;

        if (sticksActive) {
            getQuaternion(&tvHold.qTarget);
            tvHold.Tracking = true;
        } else {
            tvHold.Tracking = false;

            quaternion qCurrent;
            getQuaternion(&qCurrent);

            quaternion qCurrentConj = { .w = qCurrent.w, .x = -qCurrent.x, .y = -qCurrent.y, .z = -qCurrent.z };

            quaternion qError;
            imuQuaternionMultiplication(&qCurrentConj, &tvHold.qTarget, &qError);

            // Shortest-path sign correction -- see atthold.c for why this is needed.
            if (qError.w < 0.0f) {
                qError.w = -qError.w;
                qError.x = -qError.x;
                qError.y = -qError.y;
                qError.z = -qError.z;
            }

            const float errorDeg[3] = {
                (2.0f * qError.x) / M_RADf,
                (2.0f * qError.y) / M_RADf,
                (2.0f * qError.z) / M_RADf,
            };

            float magnitude = 0.0f;
            for (int i = 0; i < 3; i++) {
                rate[i] = errorDeg[i] * tvHold.Gain;
                magnitude += sq(rate[i]);
            }
            magnitude = sqrtf(magnitude);

            if (magnitude > tvHold.MaxRate && magnitude > 0.0f) {
                const float scale = tvHold.MaxRate / magnitude;
                rate[0] *= scale;
                rate[1] *= scale;
                rate[2] *= scale;
            }
        }
    }

    if (tvHold.Tracking) {
        DEBUG_AXIS(TVHOLD, axis, 0, pidSetpoint);
        return pidSetpoint;
    }

    DEBUG_AXIS(TVHOLD, axis, 0, rate[axis]);

    return rate[axis];
}

#endif

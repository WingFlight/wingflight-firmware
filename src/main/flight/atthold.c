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
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#ifdef USE_ACC

#include "build/build_config.h"
#include "build/debug.h"

#include "config/config.h"

#include "common/axis.h"
#include "common/maths.h"

#include "flight/airborne.h"
#include "flight/imu.h"
#include "flight/setpoint.h"

#include "atthold.h"

// Quaternion-based hold of *whatever attitude the aircraft was in* -- any orientation, not just
// level or vertical. Generalizes autohover.c's quaternion approach (needed here too, for the same
// reason: this must work right through inverted/knife-edge/harrier attitudes where Euler roll/pitch
// error terms couple and leveling.c's Euler math would misbehave) but replaces autohover's
// always-on bounded-stick-offset model with a deadband-gated track/freeze model, so this mode
// gives full, zero-lag stick authority while the pilot is actively flying and only locks onto an
// attitude the instant every stick returns to center. That behavioral model (continuously
// recapture the hold target while a stick is deflected so a future freeze is seamless; freeze and
// correct only inside a deadband) is carried over from an older Euler-angle-based attitude hold
// (removed in commit e10a57cd4) -- this file reimplements that behavior on autohover's quaternion
// math instead, since the old Euler implementation could not survive attitudes away from level.
//
// Unlike autohover (which deliberately frees the roll axis, since roll coincides with the world
// vertical axis exactly at hover and is the pilot's pirouette control there), this mode holds all
// three axes. There's no equivalent "free spin axis" at a generic captured attitude, so leaving
// any axis unheld would just let the aircraft drift on that axis.
//
// Known limitations (documented, not fixed here):
// - Like autohover, does not subtract accelerometerConfig()->accelerometerTrims, so a pilot with
//   board-mount trim dialled in will hold systematically off from where they released the sticks.
// - The activity gate is a single global OR-of-per-axis-deflection check, not a per-axis one --
//   deliberate. Once frozen, the correction is a single coupled 3D quaternion-error vector across
//   all 3 axes; thawing/freezing axes independently would reintroduce exactly the kind of
//   Euler-style axis coupling autohover.c's header warns about.

typedef struct {
    bool        Active;
    bool        Tracking;      // true while the target is free-tracking (sticks active / not airborne)
    float       Gain;
    float       Deadband;      // fraction (0..1) of stick deflection that keeps the target tracking
    float       MaxRate;
    quaternion  qTarget;
} attHold_t;

static FAST_DATA_ZERO_INIT attHold_t attHold;

int get_ADJUSTMENT_ATTHOLD_GAIN(void)
{
    return currentPidProfile->atthold.gain;
}

void set_ADJUSTMENT_ATTHOLD_GAIN(int value)
{
    currentPidProfile->atthold.gain = value;
    attHold.Gain = value / 10.0f;
}

INIT_CODE void attHoldInit(const pidProfile_t *pidProfile)
{
    attHold.Gain = pidProfile->atthold.gain / 10.0f;
    attHold.Deadband = pidProfile->atthold.deadband / 100.0f;
    attHold.MaxRate = pidProfile->atthold.max_rate;
}

// Called once on the rising edge of ATTHOLD_MODE so a stale target from a previous engagement
// can never linger -- mirrors autoHoverSetState's rising-edge capture.
void attHoldSetState(bool state)
{
    if (state && !attHold.Active) {
        getQuaternion(&attHold.qTarget);
    }

    attHold.Active = state;
}

float attHoldApply(int axis, float pidSetpoint)
{
    static float rate[3];

    if (!attHold.Active) {
        return pidSetpoint;
    }

    // The activity gate and the (shared, cross-axis) correction only need computing once per PID
    // loop iteration, not once per axis call -- do that work on the first axis touched each
    // iteration and cache it, same pattern autoHoverApply uses for its pitch/yaw correction.
    if (axis == FD_ROLL) {
        const bool sticksActive = !isAirborne()
            || fabsf(getDeflection(FD_ROLL))  > attHold.Deadband
            || fabsf(getDeflection(FD_PITCH)) > attHold.Deadband
            || fabsf(getDeflection(FD_YAW))   > attHold.Deadband;

        if (sticksActive) {
            // Pilot is actively commanding the aircraft (or it's not airborne yet) -- keep the
            // hold target tracking reality so a future freeze is seamless: error is ~0 the
            // instant every stick returns to center. No correction is applied here at all.
            getQuaternion(&attHold.qTarget);
            attHold.Tracking = true;
        } else {
            attHold.Tracking = false;

            quaternion qCurrent;
            getQuaternion(&qCurrent);

            quaternion qCurrentConj = { .w = qCurrent.w, .x = -qCurrent.x, .y = -qCurrent.y, .z = -qCurrent.z };

            quaternion qError;
            imuQuaternionMultiplication(&qCurrentConj, &attHold.qTarget, &qError);

            // Shortest-path sign correction -- q and -q represent the same physical rotation, but
            // without this the error can decompose onto the "long way around" axis instead of the
            // direct one (see autoHoverApply for the same fix).
            if (qError.w < 0.0f) {
                qError.w = -qError.w;
                qError.x = -qError.x;
                qError.y = -qError.y;
                qError.z = -qError.z;
            }

            // Standard geometric attitude-control error term (2 * vector part), singularity-free
            // across the full 0-180 degree range -- see autoHoverApply for why this is preferred
            // over an axis-angle/acos decomposition. All three axes feed the corrective vector
            // here (unlike autohover, which leaves roll/index 0 unused).
            const float errorDeg[3] = {
                (2.0f * qError.x) / M_RADf,
                (2.0f * qError.y) / M_RADf,
                (2.0f * qError.z) / M_RADf,
            };

            float magnitude = 0.0f;
            for (int i = 0; i < 3; i++) {
                rate[i] = errorDeg[i] * attHold.Gain;
                magnitude += sq(rate[i]);
            }
            magnitude = sqrtf(magnitude);

            // Clamp the vector's magnitude, not each axis independently -- per-axis clamping
            // would distort the rotation axis mid-correction (see autoHoverApply).
            if (magnitude > attHold.MaxRate && magnitude > 0.0f) {
                const float scale = attHold.MaxRate / magnitude;
                rate[0] *= scale;
                rate[1] *= scale;
                rate[2] *= scale;
            }
        }
    }

    if (attHold.Tracking) {
        DEBUG_AXIS(ATTHOLD, axis, 0, pidSetpoint);
        return pidSetpoint;
    }

    DEBUG_AXIS(ATTHOLD, axis, 0, rate[axis]);

    return rate[axis];
}

#endif

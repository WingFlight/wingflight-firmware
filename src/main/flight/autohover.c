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

#include "autohover.h"

// Quaternion-based vertical (90 degree pitch) attitude + heading hold, for 3D "prop hang" hover.
// Deliberately NOT built on leveling.c's Euler-angle approach -- that computes roll/pitch error
// per axis independently and hits gimbal lock exactly at 90 degrees pitch, the one attitude this
// feature lives at. Roll and yaw become degenerate in Euler terms right at the moment control
// matters most, so leveling.c's angleModeApply/horizonModeApply cannot simply be re-aimed here.
//
// Known limitations (documented, not fixed here):
// - Does not subtract accelerometerConfig()->accelerometerTrims like leveling.c/trainer.c do, so a
//   pilot with board-mount trim dialled in will hover systematically off-vertical by that amount.
// - The held heading is captured from attitude.values.yaw, which is itself derived from an atan2
//   that degrades in conditioning as pitch->90 degrees -- re-engaging while already near-vertical
//   can capture a noisy heading. Heading is not well-defined exactly vertical; this is inherent.
// - Attitude/heading hold only -- no GPS or optical-flow position lock. Horizontal drift is the
//   pilot's responsibility via normal stick input, same as deflecting away from ANGLE_MODE's level
//   target and letting go to spring back.
// - Manual throttle only. This commands an attitude, not a maneuver profile -- it has no awareness
//   of airspeed/energy state or whether the airframe has enough thrust to sustain a vertical hover.
//   If it doesn't, engaging this commands a hard (rate-clamped) pitch-up and the aircraft will
//   likely stall/tumble rather than hover.

typedef struct {
    bool    Active;
    float   Gain;
    float   MaxAngle;   // degrees the stick may deflect the target off vertical/held heading
    float   MaxRate;    // deg/s clamp on the commanded attitude-capture rate (safety limit)
    int16_t HeadingTargetDecidegrees;
} autoHover_t;

static FAST_DATA_ZERO_INIT autoHover_t autoHover;

int get_ADJUSTMENT_AUTOHOVER_GAIN(void)
{
    return currentPidProfile->autohover.gain;
}

void set_ADJUSTMENT_AUTOHOVER_GAIN(int value)
{
    currentPidProfile->autohover.gain = value;
    autoHover.Gain = value / 10.0f;
}

INIT_CODE void autoHoverInit(const pidProfile_t *pidProfile)
{
    autoHover.Gain = pidProfile->autohover.gain / 10.0f;
    autoHover.MaxAngle = pidProfile->autohover.max_angle;
    autoHover.MaxRate = pidProfile->autohover.max_rate;
}

// Called once on the rising edge of AUTOHOVER_MODE so the held heading is captured fresh each
// time the mode engages -- mirrors how the (now-removed) attitude-hold mode used to capture its
// target on activation.
void autoHoverSetState(bool state)
{
    if (state && !autoHover.Active) {
        autoHover.HeadingTargetDecidegrees = attitude.values.yaw;
    }

    autoHover.Active = state;
}

float autoHoverApply(int axis, float pidSetpoint)
{
    static float rate[3];

    if (!autoHover.Active) {
        return pidSetpoint;
    }

    if (axis == FD_ROLL) {
        // Held target: vertical, at the captured heading, plus the pilot's stick deflection as a
        // small local (body-frame) rotation offset -- same "deflect away from the hold and spring
        // back when centred" feel as angleModeApply, just centred on vertical instead of level.
        quaternion qBase;
        imuEulerToQuaternion(0, 900, autoHover.HeadingTargetDecidegrees, &qBase);

        const float rollOffset  = DEGREES_TO_RADIANS(autoHover.MaxAngle * getDeflection(FD_ROLL));
        const float pitchOffset = DEGREES_TO_RADIANS(autoHover.MaxAngle * getDeflection(FD_PITCH));
        const float yawOffset   = DEGREES_TO_RADIANS(autoHover.MaxAngle * getDeflection(FD_YAW));

        quaternion qStickOffset = {
            .w = 1.0f,
            .x = rollOffset * 0.5f,
            .y = pitchOffset * 0.5f,
            .z = yawOffset * 0.5f,
        };
        const float stickNormRecip = 1.0f / sqrtf(sq(qStickOffset.w) + sq(qStickOffset.x) + sq(qStickOffset.y) + sq(qStickOffset.z));
        qStickOffset.w *= stickNormRecip;
        qStickOffset.x *= stickNormRecip;
        qStickOffset.y *= stickNormRecip;
        qStickOffset.z *= stickNormRecip;

        // Right-multiply: the stick offset is a perturbation local to the base target, not the
        // world frame. This is the deliberate choice -- it keeps stick response feeling the same
        // regardless of which way the held heading points. The other composition order would make
        // roll-stick response depend on the held heading, which would feel wrong.
        quaternion qTarget;
        imuQuaternionMultiplication(&qBase, &qStickOffset, &qTarget);

        // Defensive renormalize -- qBase and qStickOffset are each unit, so qTarget should already
        // be unit up to floating point drift, but this is cheap and matches how imu.c renormalizes
        // its own quaternion every iteration.
        const float targetNormRecip = 1.0f / sqrtf(sq(qTarget.w) + sq(qTarget.x) + sq(qTarget.y) + sq(qTarget.z));
        qTarget.w *= targetNormRecip;
        qTarget.x *= targetNormRecip;
        qTarget.y *= targetNormRecip;
        qTarget.z *= targetNormRecip;

        quaternion qCurrent;
        getQuaternion(&qCurrent);

        quaternion qCurrentConj = { .w = qCurrent.w, .x = -qCurrent.x, .y = -qCurrent.y, .z = -qCurrent.z };

        quaternion qError;
        imuQuaternionMultiplication(&qCurrentConj, &qTarget, &qError);

        // Shortest-path sign correction -- q and -q represent the same physical rotation, but
        // without this the error can decompose onto the "long way around" axis instead of the
        // direct one.
        if (qError.w < 0.0f) {
            qError.w = -qError.w;
            qError.x = -qError.x;
            qError.y = -qError.y;
            qError.z = -qError.z;
        }

        // Standard geometric attitude-control error term (2 * vector part) -- valid and
        // singularity-free across the full 0-180 degree range, unlike an acos/axis-angle
        // decomposition (which needs its own shortest-path check plus a division that blows up as
        // the error angle approaches zero). Magnitude saturates smoothly toward 2.0 rad as the true
        // error approaches 180 degrees, rather than growing unbounded.
        float errorDeg[3] = {
            (2.0f * qError.x) / M_RADf,
            (2.0f * qError.y) / M_RADf,
            (2.0f * qError.z) / M_RADf,
        };

        // Same pre-airborne attenuation angleModeApply/horizonModeApply use, so the switch can't
        // be armed/tested on the ground and snap violently.
        if (!isAirborne()) {
            errorDeg[0] *= 0.25f;
            errorDeg[1] *= 0.25f;
            errorDeg[2] *= 0.25f;
        }

        float magnitude = 0.0f;
        for (int i = 0; i < 3; i++) {
            rate[i] = errorDeg[i] * autoHover.Gain;
            magnitude += sq(rate[i]);
        }
        magnitude = sqrtf(magnitude);

        // Clamp the vector's magnitude, not each axis independently -- per-axis clamping would
        // distort the rotation axis mid-maneuver (e.g. pitch saturating before roll), turning a
        // clean single-axis snap-to-vertical into a curved one.
        if (magnitude > autoHover.MaxRate && magnitude > 0.0f) {
            const float scale = autoHover.MaxRate / magnitude;
            rate[0] *= scale;
            rate[1] *= scale;
            rate[2] *= scale;
        }
    }

    DEBUG_AXIS(AUTOHOVER, axis, 0, rate[axis]);

    return rate[axis];
}

#endif

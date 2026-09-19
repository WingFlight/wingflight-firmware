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

#include "common/axis.h"
#include "common/maths.h"

#include "flight/airborne.h"
#include "flight/imu.h"
#include "flight/pid.h"
#include "flight/setpoint.h"

#include "hold_engine.h"

// Quaternion-based hold of *whatever attitude the aircraft was in* -- any orientation, not just
// level or vertical. Generalizes autohover.c's quaternion approach (needed here too, for the same
// reason: this must work right through inverted/knife-edge/harrier attitudes where Euler roll/pitch
// error terms couple and leveling.c's Euler math would misbehave) but replaces autohover's
// always-on bounded-stick-offset model with a deadband-gated track/freeze model, so a hold gives
// full, zero-lag stick authority while the pilot is actively flying and locks onto an attitude
// once a stick returns to center and that axis has stopped rotating (see HOLD_SETTLE_RATE
// below -- capturing mid-rotation would snap the aircraft back). That behavioral model
// (continuously recapture the hold target while a stick is deflected so a future freeze is
// seamless; freeze and correct only inside a deadband) is carried over from an older Euler-angle-
// based attitude hold (removed in commit e10a57cd4) -- this file reimplements that behavior on
// autohover's quaternion math instead, since the old Euler implementation could not survive
// attitudes away from level.
//
// Unlike autohover (which deliberately frees the roll axis, since roll coincides with the world
// vertical axis exactly at hover and is the pilot's pirouette control there), the engine can hold
// any subset of the three axes at once -- each axis tracks or freezes independently based on its
// own stick, e.g. holding pitch/yaw attitude while aileron alone is worked (a clean axial roll or
// rifle roll), or holding roll/yaw while riding the elevator through a high-alpha attitude. There
// is no fixed reference attitude to decompose into independent per-axis offsets the way autohover
// does for roll (that always-vertical target IS the reference), so per-axis freezing is done
// directly in quaternion space instead (see the qKeep construction below), never by extracting or
// composing Euler angles -- that's what keeps this safe through inverted/knife-edge attitudes
// where an Euler decomposition would couple axes or hit gimbal lock.
//
// Known limitation (documented, not fixed here):
// - Like autohover, does not subtract accelerometerConfig()->accelerometerTrims, so a pilot with
//   board-mount trim dialled in will hold systematically off from where they released the sticks.

// After an axis's stick returns to center, its target keeps free-tracking until the aircraft has
// actually stopped rotating on that axis (below HOLD_SETTLE_RATE) or HOLD_SETTLE_MAX_S has
// passed, whichever comes first. Freezing the target at the instant of release instead would pin
// it to an attitude the aircraft is still rotating through, so the hold would haul it back past
// where the pilot actually stopped -- a rubber-band snap-back, unlike plain acro/rate flight,
// where the rate loop just brakes and the attitude stays put. The time cap keeps a persistent
// disturbance rotation (torque roll, spin-up) from ever holding the axis in tracking forever.
#define HOLD_SETTLE_RATE     15.0f   // deg/s
#define HOLD_SETTLE_MAX_S    0.4f

// If a frozen axis has a sizeable attitude error (above HOLD_STALL_ERROR_DEG) yet has not moved
// (below HOLD_STALL_RATE) for HOLD_STALL_TIME_S, the correction isn't achieving anything --
// the aircraft is pinned, sitting on the bench, or the surface has no authority. A hold that keeps
// pushing forever against something it can't move isn't holding anything useful and leaves the
// surfaces pegged, so the axis gives up and re-captures its target at the current attitude (which
// lets the surfaces re-center). A small steady error held in balance by the I term -- e.g.
// sagging a couple of degrees against torque roll -- stays under the error threshold and keeps
// holding indefinitely.
#define HOLD_STALL_ERROR_DEG 5.0f
#define HOLD_STALL_RATE      5.0f    // deg/s
#define HOLD_STALL_TIME_S    3.0f

void quatHoldInit(quatHold_t *hold, float gain, float deadband, float maxRate)
{
    hold->Gain = gain;
    // Constrained here, not just at the CLI (settings.c) or MSP boundary -- MSP's
    // SET_PID_PROFILE handler writes the raw wire byte with no clamping of its own, and an
    // out-of-range deadband (>100%) would make fabsf(getDeflection()) > Deadband never true for
    // normal [-1, 1] stick input, so the hold would keep correcting instead of passing through
    // even at full stick.
    hold->Deadband = constrainf(deadband, 0.0f, 1.0f);
    hold->MaxRate = maxRate;
}

void quatHoldSetGain(quatHold_t *hold, float gain)
{
    hold->Gain = gain;
}

// Called once on the rising edge of the mode so a stale target from a previous engagement can
// never linger -- mirrors autoHoverSetState's rising-edge capture.
void quatHoldSetState(quatHold_t *hold, bool state)
{
    if (state && !hold->Active) {
        getQuaternion(&hold->qTarget);

        for (int i = 0; i < 3; i++) {
            hold->Tracking[i] = false;
            hold->SettleTime[i] = 0.0f;
            hold->StallTime[i] = 0.0f;
        }
    }

    hold->Active = state;
}

bool quatHoldIsHolding(const quatHold_t *hold, int axis)
{
    return hold->Active && !hold->Tracking[axis];
}

float quatHoldApply(quatHold_t *hold, int axis, float pidSetpoint)
{
    if (!hold->Active) {
        return pidSetpoint;
    }

    // The per-axis activity gate and the shared quaternion-error work only need computing once
    // per PID loop iteration, not once per axis call -- do that work on the first axis touched
    // each iteration and cache it, same pattern autoHoverApply uses for its pitch/yaw correction.
    if (axis == FD_ROLL) {
        // Deliberately not gated on isAirborne() -- see the pre-airborne attenuation below
        // instead. Forcing tracking (pure passthrough) whenever grounded, as this used to, meant
        // a hold gave zero correction authority on the bench no matter how long you sat there,
        // unlike angleModeApply/horizonModeApply/autoHoverApply's pitch+yaw, which all still
        // correct pre-airborne, just at reduced strength.
        const pidAxisData_t *pidData = pidGetAxisData();
        const float dT = pidGetDT();

        for (int i = 0; i < 3; i++) {
            if (fabsf(getDeflection(i)) > hold->Deadband) {
                hold->Tracking[i] = true;
                hold->SettleTime[i] = 0.0f;
            } else if (hold->Tracking[i]) {
                hold->SettleTime[i] += dT;

                if (fabsf(pidData[i].gyroRate) < HOLD_SETTLE_RATE || hold->SettleTime[i] >= HOLD_SETTLE_MAX_S) {
                    hold->Tracking[i] = false;
                }
            }
        }

        quaternion qCurrent;
        getQuaternion(&qCurrent);

        quaternion qCurrentConj = { .w = qCurrent.w, .x = -qCurrent.x, .y = -qCurrent.y, .z = -qCurrent.z };

        quaternion qError;
        imuQuaternionMultiplication(&qCurrentConj, &hold->qTarget, &qError);

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
        // over an axis-angle/acos decomposition. All three axes feed this vector here (unlike
        // autohover, which leaves roll/index 0 unused).
        float errorDeg[3] = {
            (2.0f * qError.x) / M_RADf,
            (2.0f * qError.y) / M_RADf,
            (2.0f * qError.z) / M_RADf,
        };

        // Stall detection uses the raw error (before the pre-airborne attenuation below), since
        // that's the true distance from the held attitude. Sends a stalled axis back through the
        // tracking path -- which re-aims just that axis's target at the current attitude -- and
        // lets the normal settle logic above hand it back to frozen a loop or two later.
        for (int i = 0; i < 3; i++) {
            if (!hold->Tracking[i] && fabsf(errorDeg[i]) > HOLD_STALL_ERROR_DEG
                && fabsf(pidData[i].gyroRate) < HOLD_STALL_RATE) {
                hold->StallTime[i] += dT;

                if (hold->StallTime[i] >= HOLD_STALL_TIME_S) {
                    hold->Tracking[i] = true;
                    hold->SettleTime[i] = 0.0f;
                    hold->StallTime[i] = 0.0f;
                }
            } else {
                hold->StallTime[i] = 0.0f;
            }
        }

        // Same pre-airborne attenuation angleModeApply/horizonModeApply/autoHoverApply's pitch+yaw
        // use, so the mode can be armed/tested on the ground without snapping at full strength --
        // reduced authority, not the zero authority a hard isAirborne() gate on tracking used to
        // give (see above). Applied to all three axes uniformly, unlike autohover.c, since here
        // there's no single "always-on" axis to treat differently -- all three go through the same
        // track/freeze machinery.
        if (!isAirborne()) {
            errorDeg[0] *= 0.25f;
            errorDeg[1] *= 0.25f;
            errorDeg[2] *= 0.25f;
        }

        // Correction rates: only the axes that are actually frozen this loop are being asked to
        // move anything, so only they take part in the magnitude clamp below -- a tracking axis
        // isn't part of the correction at all (it's a pure pidSetpoint passthrough, same as
        // before), and folding its raw error into the clamp vector would distort the frozen axes'
        // rotation direction for no reason.
        float magnitude = 0.0f;
        for (int i = 0; i < 3; i++) {
            if (!hold->Tracking[i]) {
                hold->Rate[i] = errorDeg[i] * hold->Gain;
                magnitude += sq(hold->Rate[i]);
            }
        }
        magnitude = sqrtf(magnitude);

        if (magnitude > hold->MaxRate && magnitude > 0.0f) {
            const float scale = hold->MaxRate / magnitude;
            for (int i = 0; i < 3; i++) {
                if (!hold->Tracking[i]) {
                    hold->Rate[i] *= scale;
                }
            }
        }

        // Advance qTarget so a tracking axis's error is ~0 again next loop (seamless future
        // freeze) while a frozen axis's held offset is carried forward completely unchanged.
        // This is a per-axis partial version of "getQuaternion(&hold->qTarget)" (the old
        // all-tracking capture): qError's vector part is, to the small-per-loop-step accuracy
        // this runs at, an axis-separable measure of how far qTarget currently sits from
        // qCurrent along each body axis. Zeroing a tracking axis's component before recomposing
        // says "close that gap"; leaving a frozen axis's component untouched says "keep exactly
        // the offset already held on that axis". Re-deriving qTarget = qCurrent (x) qKeep this
        // way (rather than integrating a running per-axis offset, which is how autohover.c
        // tracks its one live axis) avoids ever constructing an intermediate Euler triple, which
        // is what would reintroduce gimbal lock/coupling here since -- unlike autohover's fixed
        // vertical reference -- any of this mode's 3 axes can legitimately end up frozen near a
        // 90 degree offset from the other two.
        if (hold->Tracking[FD_ROLL] || hold->Tracking[FD_PITCH] || hold->Tracking[FD_YAW]) {
            quaternion qKeep = {
                .x = hold->Tracking[FD_ROLL]  ? 0.0f : qError.x,
                .y = hold->Tracking[FD_PITCH] ? 0.0f : qError.y,
                .z = hold->Tracking[FD_YAW]   ? 0.0f : qError.z,
            };
            qKeep.w = sqrtf(fmaxf(0.0f, 1.0f - sq(qKeep.x) - sq(qKeep.y) - sq(qKeep.z)));

            quaternion qTargetNew;
            imuQuaternionMultiplication(&qCurrent, &qKeep, &qTargetNew);

            const float targetNormRecip = 1.0f / sqrtf(sq(qTargetNew.w) + sq(qTargetNew.x) + sq(qTargetNew.y) + sq(qTargetNew.z));
            qTargetNew.w *= targetNormRecip;
            qTargetNew.x *= targetNormRecip;
            qTargetNew.y *= targetNormRecip;
            qTargetNew.z *= targetNormRecip;

            hold->qTarget = qTargetNew;
        }
        // else: every axis is frozen -- leave qTarget completely untouched rather than
        // recomputing an identical result through the quaternion algebra above, so a long hold
        // can't accumulate floating-point drift loop after loop.
    }

    if (hold->Tracking[axis]) {
        return pidSetpoint;
    }

    return hold->Rate[axis];
}

#endif

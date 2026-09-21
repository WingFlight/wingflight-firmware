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

#include "drivers/time.h"

#include "fc/rc.h"

#include "flight/airborne.h"
#include "flight/imu.h"
#include "flight/pid.h"
#include "flight/setpoint.h"

#include "rx/rx.h"

#include "autohover.h"

// Absolute, firmware-enforced backstop on the throttle assist ceiling, independent of whatever
// value autohover.throttle_assist_max happens to hold. The CLI settings table clamps `set` inputs
// to this same 0-50 range, but MSP's SET_PID_PROFILE handler writes the raw wire byte with no
// clamping of its own -- this constant is what actually prevents a stray/malicious/corrupted
// profile value from raising the ceiling past a sane bound, not just the CLI or configurator UI.
#define AUTOHOVER_THROTTLE_ASSIST_MAX_CEILING 0.50f

// Quaternion-based vertical (90 degree pitch) attitude + heading hold, for 3D "prop hang" hover.
// Deliberately NOT built on leveling.c's Euler-angle approach -- that computes roll/pitch error
// per axis independently and hits gimbal lock exactly at 90 degrees pitch, the one attitude this
// feature lives at. Roll and yaw become degenerate in Euler terms right at the moment control
// matters most, so leveling.c's angleModeApply/horizonModeApply cannot simply be re-aimed here.
//
// Only pitch and yaw are attitude-held (they reposition the nose horizontally while vertical).
// Roll is a free rate pass-through, not held -- in this vertical attitude the aircraft's roll
// axis coincides with the world vertical axis, so roll is the pilot's spin/pirouette control,
// the same role yaw plays in normal nose-level flight (compare angleModeApply/horizonModeApply,
// which likewise leave yaw unheld).
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
// - Manual throttle only by default. This commands an attitude, not a maneuver profile -- it has
//   no awareness of airspeed/energy state or whether the airframe has enough thrust to sustain a
//   vertical hover. If it doesn't, engaging this commands a hard (rate-clamped) pitch-up and the
//   aircraft will likely stall/tumble rather than hover. An optional, profile-gated throttle assist
//   (throttle_assist_gain/_max/_trigger_ms, 0 gain = disabled/default) can nudge throttle up when
//   the pitch correction below is pegged at MaxRate for a sustained period -- see the assist block
//   in autoHoverApply and autoHoverThrottleBoost. It is a bounded nudge, not a fix for a genuinely
//   underpowered airframe: it ramps in slowly, is capped hard at throttle_assist_max, and is purely
//   additive on top of the pilot's own throttle stick, never a substitute for it.

typedef struct {
    bool    Active;
    float   Gain;
    float   MaxAngle;   // degrees the stick may deflect the target off vertical/held heading
    float   MaxRate;    // deg/s clamp on the commanded attitude-capture rate (safety limit)
    int16_t HeadingTargetDecidegrees;
    float   ThrottleAssistGain;     // fraction-of-throttle-range added per second while pitch
                                     // correction is saturated (0 = feature disabled)
    float   ThrottleAssistMax;      // hard ceiling, fraction of throttle range, on the added boost
    uint16_t ThrottleAssistTriggerMs; // ms the pitch correction must stay saturated before the
                                     // boost starts ramping in
    timeMs_t PitchSaturatedSinceMs; // 0 when not currently saturated; set to millis() on the rising
                                     // edge of saturation, same edge-timer pattern LOGIC_CONDITION_DELAY
                                     // uses in logic_condition.c
    float   ThrottleAssistPercent;  // live boost value, 0..ThrottleAssistMax, fraction of throttle range
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
    autoHover.ThrottleAssistGain = pidProfile->autohover.throttle_assist_gain / 100.0f;
    autoHover.ThrottleAssistMax = fminf(pidProfile->autohover.throttle_assist_max / 100.0f,
        AUTOHOVER_THROTTLE_ASSIST_MAX_CEILING);
    autoHover.ThrottleAssistTriggerMs = pidProfile->autohover.throttle_assist_trigger_ms;
}

// Called once on the rising edge of AUTOHOVER_MODE so the held heading is captured fresh each
// time the mode engages -- mirrors how the (now-removed) attitude-hold mode used to capture its
// target on activation.
void autoHoverSetState(bool state)
{
    if (state && !autoHover.Active) {
        autoHover.HeadingTargetDecidegrees = attitude.values.yaw;
    }

    if (!state) {
        // Hard reset on disengage, not just a decay to zero -- a stale boost must never carry
        // over into the next engagement, and the pilot's throttle stick regains sole authority
        // the instant the mode drops.
        autoHover.ThrottleAssistPercent = 0.0f;
        autoHover.PitchSaturatedSinceMs = 0;
    }

    autoHover.Active = state;
}

// True while this axis is actively holding a target, as opposed to being under free stick control.
// Pitch and yaw always hold while the mode is active; roll is always free. pid.c uses this to decide
// whether I-term decay should be suspended -- see pidApplyMode1.
bool autoHoverIsHolding(int axis)
{
    return autoHover.Active && axis != FD_ROLL;
}

float autoHoverApply(int axis, float pidSetpoint)
{
    static float rate[3];

    if (!autoHover.Active) {
        return pidSetpoint;
    }

    if (axis == FD_ROLL) {
        // Roll (aileron) is deliberately NOT held here. In a 90-degree nose-up hover the
        // aircraft's roll axis coincides with the world vertical axis, so roll is the
        // "spin about vertical" (pirouette) axis, not a position-hold axis -- physically the
        // same role yaw plays in normal (nose-level) flight. angleModeApply/horizonModeApply
        // leave yaw as a raw pass-through for exactly this reason (see leveling.c); roll gets
        // the same treatment here. Passing pidSetpoint straight through lets a held aileron
        // deflection produce continuous rotation instead of converging on a bounded +-MaxAngle
        // offset and fighting the stick.
        rate[FD_ROLL] = pidSetpoint;

        // Held target: vertical, at the captured heading, plus the pilot's pitch/yaw stick
        // deflection as a small local (body-frame) rotation offset -- same "deflect away from
        // the hold and spring back when centred" feel as angleModeApply, just centred on
        // vertical instead of level. Roll is intentionally omitted from this offset (see above).
        // Bench-confirmed: +900 here drives the elevator toward nose-down, not nose-up (the
        // stabilisation loop itself is correct -- verified via blackbox, axisP/axisF go strongly
        // positive on engage exactly as intended -- it was just chasing the wrong target). -900
        // is the physically-vertical, nose-up target.
        quaternion qBase;
        imuEulerToQuaternion(0, -900, autoHover.HeadingTargetDecidegrees, &qBase);

        const float pitchOffset = DEGREES_TO_RADIANS(autoHover.MaxAngle * getDeflection(FD_PITCH));
        const float yawOffset   = DEGREES_TO_RADIANS(autoHover.MaxAngle * getDeflection(FD_YAW));

        quaternion qStickOffset = {
            .w = 1.0f,
            .x = 0.0f,
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
        // error approaches 180 degrees, rather than growing unbounded. Only pitch/yaw (indices 1
        // and 2) feed the corrective loop -- errorDeg[0] (roll) is intentionally left unused since
        // roll is free-running (see above); referencing it here would just reintroduce the P-loop
        // fighting the pilot's spin.
        float errorDeg[3] = {
            0.0f,
            (2.0f * qError.y) / M_RADf,
            (2.0f * qError.z) / M_RADf,
        };

        // Same pre-airborne attenuation angleModeApply/horizonModeApply use, so the switch can be
        // armed/tested on the ground without snapping at full strength.
        if (!isAirborne()) {
            errorDeg[1] *= 0.25f;
            errorDeg[2] *= 0.25f;
        }

        float magnitude = 0.0f;
        for (int i = FD_PITCH; i <= FD_YAW; i++) {
            rate[i] = errorDeg[i] * autoHover.Gain;
            magnitude += sq(rate[i]);
        }
        magnitude = sqrtf(magnitude);

        // Captured pre-clamp -- the vector clamp below rescales rate[FD_PITCH], but the throttle
        // assist below that needs the raw, unscaled commanded pitch effort to judge saturation.
        const float pitchEffort = fabsf(rate[FD_PITCH]);

        // Clamp the vector's magnitude, not each axis independently -- per-axis clamping would
        // distort the rotation axis mid-maneuver (e.g. pitch saturating before yaw), turning a
        // clean single-axis snap-to-vertical into a curved one. Roll is excluded -- it's the
        // pilot's free-running rate command, not part of this corrective vector.
        if (magnitude > autoHover.MaxRate && magnitude > 0.0f) {
            const float scale = autoHover.MaxRate / magnitude;
            rate[FD_PITCH] *= scale;
            rate[FD_YAW] *= scale;
        }

        // Throttle assist: optional, profile-gated nudge (ThrottleAssistGain 0 = disabled, the
        // default) for when the pitch correction above has been pegged at MaxRate long enough to
        // suggest the airframe can't out-thrust the hold on the pilot's current throttle, not just
        // ride out a single gust. Pitch only, not the combined pitch+yaw vector above -- pitch is
        // the axis fighting gravity in this vertical attitude, so sustained pitch saturation is a
        // more specific proxy for thrust deficiency than a yaw/heading disturbance would be. Gated
        // on isAirborne() for the same reason the pre-airborne attenuation above exists -- ground
        // pitch error (e.g. sitting nose-up on a bench stand) must never drive throttle up.
        // MaxRate > 0.0f is required, not just ThrottleAssistGain -- with MaxRate at 0 (a valid
        // CLI/MSP value that effectively disables attitude correction), pitchEffort >= 0.0f is
        // true on every loop regardless of actual pitch error, which would trigger the assist
        // continuously even though no real correction is being commanded.
        if (autoHover.ThrottleAssistGain > 0.0f && autoHover.MaxRate > 0.0f && isAirborne() && pitchEffort >= autoHover.MaxRate) {
            if (autoHover.PitchSaturatedSinceMs == 0) {
                autoHover.PitchSaturatedSinceMs = millis();
            }
        } else {
            autoHover.PitchSaturatedSinceMs = 0;
        }

        const bool assistTriggered = autoHover.PitchSaturatedSinceMs != 0
            && (millis() - autoHover.PitchSaturatedSinceMs) >= autoHover.ThrottleAssistTriggerMs;

        // Ramped, not stepped, in both directions -- even an instantly-detected trigger can't jump
        // straight to the ceiling in one loop tick, and releasing decays back out over the same
        // timescale instead of latching high. slewLimit is the same bounded-rate-of-change helper
        // governor.c uses for its own throttle target; ThrottleAssistGain is fraction-of-range per
        // second, so scaling it by pidGetDT() gives the max change allowed this loop tick.
        const float assistTarget = assistTriggered ? autoHover.ThrottleAssistMax : 0.0f;
        autoHover.ThrottleAssistPercent = constrainf(
            slewLimit(autoHover.ThrottleAssistPercent, assistTarget, autoHover.ThrottleAssistGain * pidGetDT()),
            0.0f, autoHover.ThrottleAssistMax);

        DEBUG_AXIS(AUTOHOVER, axis, 1, lrintf(autoHover.ThrottleAssistPercent * 1000.0f));
    }

    DEBUG_AXIS(AUTOHOVER, axis, 0, rate[axis]);

    return rate[axis];
}

// 0..1 fraction of throttle range to add on top of the pilot's own throttle command -- 0 whenever
// the mode is inactive or the assist is disabled/not currently triggered. mixer.c adds this before
// governorApply() so any governor-side slew/ceiling still applies on top as a second layer.
//
// The assist is additive on the pilot's throttle, never a substitute for it, so it is also off
// whenever the throttle stick is at or below the off-throttle threshold, or the receiver has no
// signal (a held-on AUTOHOVER switch plus a centred/held stick must not spin the motor up, and
// the flight-controller failsafe stage 2 is disabled, so nothing else clears it). The ramp is reset, not merely
// masked, so the assist re-ramps from zero once the stick comes back up instead of stepping in.
float autoHoverThrottleBoost(void)
{
    if (!autoHover.Active) {
        return 0.0f;
    }

    if (isThrottleOff() || !rxIsReceivingSignal()) {
        autoHover.ThrottleAssistPercent = 0.0f;
        autoHover.PitchSaturatedSinceMs = 0;
        return 0.0f;
    }

    return autoHover.ThrottleAssistPercent;
}

#endif

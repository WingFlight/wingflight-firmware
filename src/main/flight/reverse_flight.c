/*
 * This file is part of Wingflight.
 *
 * Wingflight is free software. You can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Wingflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software. If not, see <https://www.gnu.org/licenses/>.
 */

#include <math.h>

#include "platform.h"

#include "build/debug.h"

#include "common/maths.h"
#include "common/utils.h"

#include "reverse_flight.h"

// 4D / reverse flight.
//
// A 4D shockflyer has a reversible motor on a pitch/yaw gimbal at the nose. The
// throttle is neutral at mid stick: forward above, reverse below. Two kinds of
// control reverse their effect at different moments when the pilot reverses:
//
// - The gimbal pushes the nose with the thrust, so its effect flips when the
//   thrust flips. That is not when the stick crosses: the ESC first brakes the
//   prop, then spins it up the other way. The thrust is estimated with a first
//   order lag of the motor command (reverse_motor_time), and the gimbal factor
//   follows the sign of that estimate and fades with its size.
// - The surfaces work with the air flowing over them, so they flip when the
//   aircraft's motion reverses, after it has stopped. The speed is estimated
//   with a first order lag of the motor command (reverse_time_constant),
//   normalised so that steady-state speed equals the thrust command.
//
// The stick is not reversed: it moves every surface and the gimbal the same way in
// both directions, as without a gyro, and the pilot flies the reversed effect as he
// would without one. The factors here reverse and fade the *gyro correction* only
// (pid.c and tv_pid.c turn that into "stick unchanged, gyro times factor").
//
// While thrust and motion disagree (the handover, after a reversal until the
// aircraft has stopped) the surface factor is zero: the gyro doesn't know which way
// the surfaces work then, and a correction with the wrong sign is positive feedback.
// The stick still moves the surfaces; the gimbal's gyro carries the aircraft through
// the stop.
//
// On some airframes one servo moves both a surface and one gimbal axis (the rudder
// linked to the gimbal's yaw tilt). Its two effects reverse at different moments,
// so a coupled axis (reverse_coupled_axes) gets a blend: the gimbal factor while
// there is thrust, handing over to the surface factor as the thrust fades out.
// Forward and reverse it is +-1 like the others; while stopping the gimbal, which
// has the authority then, sets the sign; with the motor stopped the surface does.
// That servo is driven by the main loop (its surface's mixer rule); the TV input
// of that axis stays unmapped.
//
// Pure apart from the debug output: the config comes in through
// reverseFlightInit() and the flight state through reverseFlightUpdate(), so it
// can be unit tested on its own.

#define SPEED_STOPPED   0.01f   // |speed estimate| below this counts as standing still

typedef struct {
    bool    enabled;

    // configuration, converted
    float   deadbandUs;
    float   flipUs;
    float   reverseLimit;
    float   motorTau;
    float   speedTau;
    float   surfaceGain;
    float   fade;
    float   dwell;
    uint8_t coupledAxes;

    // stick stage
    int8_t  stickDir;

    // control stage
    int8_t  dir;            // motor direction latch
    float   throttle;       // motor command after the dwell
    float   zeroTime;       // time the motor command has been zero
    float   thrust;         // estimated real thrust
    float   speed;          // estimated speed
    float   surfaceFactor;
    float   vectorFactor;
    float   coupledFactor;
    int8_t  surfaceSign;    // sign of each factor, holding the last non-zero one
    int8_t  vectorSign;
    int8_t  coupledSign;
    bool    reversed;
    reverseFlightState_e state;
} reverseFlight_t;

static FAST_DATA_ZERO_INIT reverseFlight_t rf;

static inline float signf(float x)
{
    return (x > 0) ? 1.0f : (x < 0) ? -1.0f : 0.0f;
}

static inline void holdSign(int8_t *sign, float factor)
{
    if (factor > 0)
        *sign = 1;
    else if (factor < 0)
        *sign = -1;
}

static inline float fadeIn(float magnitude)
{
    return constrainf(magnitude / rf.fade, 0.0f, 1.0f);
}

// Resets the control stage only. The stick latch belongs to the RC stage and keeps
// following the stick while disarmed.
void reverseFlightReset(void)
{
    rf.dir = 1;
    rf.throttle = 0;
    rf.zeroTime = rf.dwell;     // the motor has been at rest: no dwell before the first reversal
    rf.thrust = 0;
    rf.speed = 0;
    rf.surfaceFactor = 1;
    rf.vectorFactor = 1;
    rf.coupledFactor = 1;
    rf.surfaceSign = 1;
    rf.vectorSign = 1;
    rf.coupledSign = 1;
    rf.reversed = false;
    rf.state = rf.enabled ? REVERSE_STATE_DISARMED : REVERSE_STATE_OFF;
}

void reverseFlightInit(const reverseFlightConfig_t *config)
{
    rf.enabled = (config->throttle_mode == THROTTLE_MODE_BIDIRECTIONAL);

    rf.deadbandUs = MIN(config->throttle_deadband, THROTTLE_DEADBAND_MAX);
    rf.flipUs = rf.deadbandUs * (1.0f + MIN(config->throttle_reverse_hysteresis, THROTTLE_REVERSE_HYSTERESIS_MAX) / 100.0f);
    rf.reverseLimit = constrain(config->throttle_reverse_limit, THROTTLE_REVERSE_LIMIT_MIN, 100) / 100.0f;
    rf.motorTau = constrain(config->reverse_motor_time, REVERSE_MOTOR_TIME_MIN, REVERSE_MOTOR_TIME_MAX) / 1000.0f;
    rf.speedTau = constrain(config->reverse_time_constant, REVERSE_TIME_CONSTANT_MIN, REVERSE_TIME_CONSTANT_MAX) / 1000.0f;
    rf.surfaceGain = MIN(config->reverse_surface_gain, REVERSE_SURFACE_GAIN_MAX) / 100.0f;
    rf.fade = constrain(config->reverse_fade, REVERSE_FADE_MIN, REVERSE_FADE_MAX) / 100.0f;
    rf.dwell = config->reverse_motor_dwell / 1000.0f;
    rf.coupledAxes = config->reverse_coupled_axes & REVERSE_COUPLED_AXES_MASK;

    rf.stickDir = 1;
    reverseFlightReset();
}

bool reverseFlightIsEnabled(void)
{
    return rf.enabled;
}

bool reverseFlightStickInDeadband(float offsetUs)
{
    return fabsf(offsetUs) <= rf.deadbandUs;
}

float reverseFlightMapStick(float offsetUs, float halfRangeUs)
{
    // The direction only flips once the stick is clearly past the far side of the
    // deadband, so a stick parked near an edge can't make the direction chatter.
    if (offsetUs > rf.flipUs)
        rf.stickDir = 1;
    else if (offsetUs < -rf.flipUs)
        rf.stickDir = -1;

    // Thrust starts from zero at the deadband edge on the latched side. On the
    // other side it stays zero until the latch flips.
    float thrust = 0;
    if (offsetUs * rf.stickDir > rf.deadbandUs) {
        const float span = fmaxf(halfRangeUs - rf.deadbandUs, 1.0f);
        thrust = rf.stickDir * constrainf((fabsf(offsetUs) - rf.deadbandUs) / span, 0.0f, 1.0f);
    }

    if (thrust < 0)
        thrust *= rf.reverseLimit;

    return thrust;
}

void reverseFlightUpdate(float stickThrottle, bool armed, bool airborne, float dT)
{
    rf.reversed = false;

    if (!rf.enabled) {
        rf.throttle = stickThrottle;
        rf.surfaceFactor = 1;
        rf.vectorFactor = 1;
        rf.coupledFactor = 1;
        rf.state = REVERSE_STATE_OFF;
        return;
    }

    if (!armed) {
        // Start every flight forward and at rest. Full authority on the bench.
        reverseFlightReset();
        return;
    }

    // Motor direction, with an optional rest at zero before reversing
    float command = stickThrottle;
    if (command != 0 && signf(command) != rf.dir) {
        if (rf.zeroTime < rf.dwell) {
            command = 0;
        }
        else {
            rf.dir = signf(command);
            rf.reversed = true;
        }
    }
    rf.throttle = command;
    rf.zeroTime = (command == 0) ? rf.zeroTime + dT : 0;

    // Thrust and speed estimates
    rf.thrust += (command - rf.thrust) * (dT / (rf.motorTau + dT));
    rf.speed  += (command - rf.speed)  * (dT / (rf.speedTau + dT));

    if (!airborne) {
        // On the ground the aircraft hasn't moved yet: keep full authority in the
        // latched direction, so the reversed gyro can be checked before takeoff.
        rf.vectorFactor = rf.dir;
        rf.surfaceFactor = rf.dir;
        rf.state = REVERSE_STATE_GROUND;
    }
    else {
        // Gimbal: follows the real thrust, fades out as the thrust passes zero
        rf.vectorFactor = signf(rf.thrust) * fadeIn(fabsf(rf.thrust));

        // Surfaces: follow the motion, gyro off while thrust and motion disagree
        const float motion = (fabsf(rf.speed) > SPEED_STOPPED) ? signf(rf.speed) : rf.dir;
        if (command != 0 && motion != rf.dir) {
            rf.surfaceFactor = 0;
            rf.state = REVERSE_STATE_HANDOVER;
        }
        else {
            rf.surfaceFactor = motion * fadeIn(fabsf(rf.speed));
            if (command == 0)
                rf.state = REVERSE_STATE_MOTOR_STOPPED;
            else
                rf.state = (rf.dir > 0) ? REVERSE_STATE_FORWARD : REVERSE_STATE_REVERSE;
        }
    }

    // Separate gyro gain for flying tail first
    if (rf.surfaceFactor < 0)
        rf.surfaceFactor *= rf.surfaceGain;

    // Surface and gimbal on one servo: the gimbal while there is thrust, the surface as it fades
    rf.coupledFactor = rf.vectorFactor + (1.0f - fabsf(rf.vectorFactor)) * rf.surfaceFactor;

    // The direction the stick works in, for the PID loops (see reverseFlightGetAxisSign()).
    // Held through a zero factor, where the gyro has no say anyway.
    holdSign(&rf.surfaceSign, rf.surfaceFactor);
    holdSign(&rf.vectorSign, rf.vectorFactor);
    holdSign(&rf.coupledSign, rf.coupledFactor);

    DEBUG(REVERSE_FLIGHT, 0, lrintf(stickThrottle * 1000));
    DEBUG(REVERSE_FLIGHT, 1, lrintf(rf.throttle * 1000));
    DEBUG(REVERSE_FLIGHT, 2, lrintf(rf.thrust * 1000));
    DEBUG(REVERSE_FLIGHT, 3, lrintf(rf.speed * 1000));
    DEBUG(REVERSE_FLIGHT, 4, lrintf(rf.vectorFactor * 1000));
    DEBUG(REVERSE_FLIGHT, 5, lrintf(rf.surfaceFactor * 1000));
    DEBUG(REVERSE_FLIGHT, 6, rf.dir);
    DEBUG(REVERSE_FLIGHT, 7, rf.state);
}

float reverseFlightGetThrottle(void)
{
    return rf.throttle;
}

// The factors are exactly 1 whenever reverse flight is off, also before
// reverseFlightInit() has run: the PID loops multiply by them unconditionally.
float reverseFlightGetSurfaceFactor(void)
{
    return rf.enabled ? rf.surfaceFactor : 1.0f;
}

float reverseFlightGetVectorFactor(void)
{
    return rf.enabled ? rf.vectorFactor : 1.0f;
}

float reverseFlightGetCoupledFactor(void)
{
    return rf.enabled ? rf.coupledFactor : 1.0f;
}

float reverseFlightGetAxisFactor(int axis)
{
    if (!rf.enabled)
        return 1.0f;
    return (rf.coupledAxes & BIT(axis)) ? rf.coupledFactor : rf.surfaceFactor;
}

int8_t reverseFlightGetAxisSign(int axis)
{
    if (!rf.enabled)
        return 1;
    return (rf.coupledAxes & BIT(axis)) ? rf.coupledSign : rf.surfaceSign;
}

int8_t reverseFlightGetVectorSign(void)
{
    return rf.enabled ? rf.vectorSign : 1;
}

bool reverseFlightJustReversed(void)
{
    return rf.enabled && rf.reversed;
}

int8_t reverseFlightGetDirection(void)
{
    return rf.dir;
}

float reverseFlightGetThrustEstimate(void)
{
    return rf.thrust;
}

float reverseFlightGetSpeedEstimate(void)
{
    return rf.speed;
}

reverseFlightState_e reverseFlightGetState(void)
{
    return rf.state;
}

bool reverseFlightSurfacesForward(void)
{
    return rf.surfaceFactor > 0.5f;
}

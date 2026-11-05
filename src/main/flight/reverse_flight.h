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

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "pg/reverse_flight.h"

// Below this |factor| a loop holds its I-term: the actuator has too little
// (or uncertain) authority for the integrated error to mean anything.
#define REVERSE_FLIGHT_ITERM_HOLD   0.5f

typedef enum {
    REVERSE_STATE_OFF = 0,          // throttle_mode NORMAL, factors are 1
    REVERSE_STATE_DISARMED,         // factors 1, so bench checks behave as always
    REVERSE_STATE_GROUND,           // armed, not yet airborne: full authority in the latched direction
    REVERSE_STATE_MOTOR_STOPPED,    // airborne, motor command zero (in the deadband or failsafe)
    REVERSE_STATE_FORWARD,          // thrust and motion forward
    REVERSE_STATE_HANDOVER,         // thrust reversed, aircraft still moving the old way: surfaces off
    REVERSE_STATE_REVERSE,          // thrust and motion backwards
} reverseFlightState_e;

void reverseFlightInit(const reverseFlightConfig_t *config);
void reverseFlightReset(void);

bool reverseFlightIsEnabled(void);

// Stick stage, at RC rate. offsetUs is the throttle channel's distance from
// the neutral point. Returns the signed thrust command -1..1 (0 inside the
// deadband, and between the deadband and the flip point on the side the
// direction latch is not on). Updates the stick direction latch.
float reverseFlightMapStick(float offsetUs, float halfRangeUs);
bool  reverseFlightStickInDeadband(float offsetUs);

// Control stage, once per PID cycle before the PID loops. stickThrottle is the
// signed command from reverseFlightMapStick() (0 in failsafe).
void reverseFlightUpdate(float stickThrottle, bool armed, bool airborne, float dT);

float reverseFlightGetThrottle(void);        // motor command -1..1, after the reversal dwell
float reverseFlightGetSurfaceFactor(void);   // scales the main loop / MANUAL surface output
float reverseFlightGetAxisFactor(int axis);  // the main loop's factor for one axis: the surface factor, or the coupled factor
float reverseFlightGetCoupledFactor(void);   // for a servo that moves both a surface and the gimbal
float reverseFlightGetVectorFactor(void);    // scales the thrust vector loop / MANUAL gimbal output
bool  reverseFlightJustReversed(void);       // true for the one cycle the motor direction flipped
int8_t reverseFlightGetDirection(void);      // +1 forward, -1 reverse (motor direction latch)
float reverseFlightGetThrustEstimate(void);  // estimated real thrust -1..1
float reverseFlightGetSpeedEstimate(void);   // estimated speed, normalised to steady-state thrust
reverseFlightState_e reverseFlightGetState(void);

// True while the surfaces work the normal way round at more than half authority.
// Captures that average surface positions (autotrim) only make sense then.
bool  reverseFlightSurfacesForward(void);

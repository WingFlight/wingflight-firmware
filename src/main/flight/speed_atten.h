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

#include "pg/pid.h"

// Low-pass cutoff on GPS speed. GPS arrives at 5-10 Hz; this turns its steps into a ramp.
#define FW_SPA_SPEED_CUTOFF_HZ    1.0f

// After the fix is lost, keep the last scale this long, then ease back to 1.0.
#define FW_SPA_HOLD_TIME_S        3.0f

// Scale change per second while easing: back to 1.0 after the hold, or onto the curve when
// SPA (re)engages. 0.2/s takes a 50% attenuation back to 100% in 2.5 s.
#define FW_SPA_SLEW_RATE          0.2f

void speedAttenReset(void);
void speedAttenInit(const fwSpaConfig_t *config, float sampleRate);

// Once per PID loop. speedAttenUpdate() reads the GPS itself; speedAttenUpdateFrom() takes
// the fix state and speed (km/h) directly so the logic can be unit tested.
void speedAttenUpdate(float dT);
void speedAttenUpdateFrom(bool haveFix, float speedKmh, float dT);

bool speedAttenIsEnabled(void);
float speedAttenGetScale(void);
float speedAttenGetSpeed(void);

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

// The feedback (P + D) and the stick are high-passed at this cutoff, so trim, a steady
// correction and a slow stick input do not count as a swing.
#define AUTO_GAIN_HPF_CUTOFF_HZ     1.5f

// A swing is the feedback going from one side of the threshold to the other. It counts as
// oscillation when it takes this long: 20..250 ms per half cycle is 2..25 Hz.
#define AUTO_GAIN_HALF_PERIOD_MIN   0.020f
#define AUTO_GAIN_HALF_PERIOD_MAX   0.250f

// Swings in a row before the gain is cut. 4 half cycles = 2 full cycles, so one gust and its
// rebound do not cut it.
#define AUTO_GAIN_SWINGS            4

// Gain is multiplied by this on every swing from the AUTO_GAIN_SWINGS'th on
#define AUTO_GAIN_ATTACK            0.85f

// After the last cut, the gain is held this long before it starts to recover
#define AUTO_GAIN_HOLD_TIME_S       2.0f

// High-passed stick deflection (0..1) above this is the pilot working the stick: the swings
// that follow are commanded, so the count restarts and stays at zero for the hold time.
#define AUTO_GAIN_STICK_MOTION      0.10f
#define AUTO_GAIN_STICK_HOLD_S      0.30f

void autoGainReset(void);
void autoGainInit(const autoGainConfig_t *config, float sampleRate);

// AUTO GAIN mode switch: while on, every axis runs auto gain, whatever its auto_gain setting
void autoGainSetSwitch(bool on);

// Once per PID loop and axis, after P and D were calculated. feedback is P + D as sent to the
// mixer (1.0 = full surface travel), stick is the deflection (-1..1). While not active (on the
// ground, PASSTHROUGH/MANUAL) the scale is 1.0 and nothing is counted.
void autoGainUpdate(int axis, float feedback, float stick, bool active, float dT);

bool autoGainIsEnabled(int axis);
float autoGainGetScale(int axis);
float autoGainGetFeedback(int axis);

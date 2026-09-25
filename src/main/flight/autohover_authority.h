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

#pragma once

#include <stdint.h>

// Factor by which the stabilised-yaw mixer input clamp is widened while AUTOHOVER is holding.
//
// The mixer applies a mixer input's `rate` AFTER clamping the input to min/max (see
// mixerUpdateRules()), so a stabilised-yaw rate of, say, 350 (35%) is not just a gain: it is a hard
// ceiling of 35% rudder travel however hard the yaw PID pushes. That is a sensible cruise setting
// (it keeps the rudder from being twitchy), but a prop hang has no airspeed and gets all of its
// rudder authority from prop wash, and the yaw correction is what pins first -- a hover blackbox
// showed the rudder at that ceiling in 96% of the samples where the attitude correction was
// saturated, while the elevator had headroom.
//
// Widening the clamp by 1000/|rate| keeps `rate` acting as the same gain (rate * clamp == the
// min/max limit) but lets the input travel until the *output* reaches the limit min/max define,
// instead of stopping at min/max * rate. With the default min/max of +-1000 that is full travel.
// Servo travel limits and every other output stage still apply after this.
//
// Only ever widens: rates of 0 (axis disabled) and |rate| >= 1000 (already at or above unity)
// return 1.0, so this can never reduce authority relative to the normal path. A negative rate
// (reversed axis) widens by the same factor as its magnitude.
static inline float autoHoverYawAuthorityScale(int16_t mixerInputRate)
{
    const int32_t rate = (mixerInputRate < 0) ? -(int32_t)mixerInputRate : (int32_t)mixerInputRate;

    if (rate <= 0 || rate >= 1000) {
        return 1.0f;
    }

    return 1000.0f / (float)rate;
}

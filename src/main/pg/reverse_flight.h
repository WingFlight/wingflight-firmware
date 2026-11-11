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

#include "types.h"
#include "platform.h"

#include "pg/pg.h"

// Throttle stick mapping. NORMAL: 0..1 from rc_min_throttle to rc_max_throttle.
// BIDIRECTIONAL (4D): zero thrust at mid stick, forward above, reverse below.
typedef enum {
    THROTTLE_MODE_NORMAL = 0,
    THROTTLE_MODE_BIDIRECTIONAL,
    THROTTLE_MODE_COUNT
} throttleMode_e;

#define THROTTLE_DEADBAND_MAX               200     // us each side of centre
#define THROTTLE_REVERSE_HYSTERESIS_MAX     200     // % of the deadband
#define THROTTLE_REVERSE_LIMIT_MIN          10      // %
#define REVERSE_MOTOR_TIME_MIN              10      // ms
#define REVERSE_MOTOR_TIME_MAX              1000    // ms
#define REVERSE_TIME_CONSTANT_MIN           50      // ms
#define REVERSE_TIME_CONSTANT_MAX           5000    // ms
#define REVERSE_SURFACE_GAIN_MAX            300     // %
#define REVERSE_FADE_MIN                    1       // %
#define REVERSE_FADE_MAX                    50      // %
#define REVERSE_COUPLED_AXES_MASK           0x07    // bit 0 roll, 1 pitch, 2 yaw

// 4D / reverse flight: bidirectional throttle and the direction estimate that
// flips the stabilisation (flight/reverse_flight.c).
typedef struct reverseFlightConfig_s {
    uint8_t  throttle_mode;                 // throttleMode_e
    uint8_t  throttle_deadband;             // us each side of centre, motor stopped inside
    uint8_t  throttle_reverse_hysteresis;   // % of the deadband the stick must pass beyond it to flip direction
    uint8_t  throttle_reverse_limit;        // % of full thrust allowed in reverse
    uint16_t reverse_motor_time;            // ms, time constant of the ESC/motor/prop reversing
    uint16_t reverse_time_constant;         // ms, time constant of the aircraft's speed change
    uint16_t reverse_surface_gain;          // % surface authority while flying backwards
    uint8_t  reverse_fade;                  // % of full thrust/speed over which a factor fades in
    uint8_t  reverse_motor_dwell;           // ms the motor must rest at zero before it may reverse
    uint8_t  reverse_coupled_axes;          // axes where one servo moves both the surface and the gimbal (bit 0 roll, 1 pitch, 2 yaw)
} reverseFlightConfig_t;

PG_DECLARE(reverseFlightConfig_t, reverseFlightConfig);

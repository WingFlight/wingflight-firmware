/*
 * This file is part of Wingflight.
 *
 * Wingflight is free software. You can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#pragma once

#include <stdint.h>

#include "pg/pg.h"

typedef struct autolaunchConfig_s {
    uint8_t auto_throttle;       // 0 = skip throw detection after throttle confirmation, 1 = wait for throw detection
    uint8_t launch_throttle;     // percent
    uint8_t climb_angle;         // degrees nose-up target
    uint8_t stick_threshold;     // percent roll/pitch/yaw deflection needed for pilot takeover
    uint16_t accel_threshold;    // cm/s/s forward acceleration threshold
    uint16_t detect_time;        // ms that launch detection must be continuously true
    uint16_t motor_delay;        // ms to wait after throw detection before spinning the motor
    uint16_t timeout;            // ms before giving control back if no throw is detected
} autolaunchConfig_t;

PG_DECLARE(autolaunchConfig_t, autolaunchConfig);

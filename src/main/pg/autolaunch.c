/*
 * This file is part of Wingflight.
 *
 * Wingflight is free software. You can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include <stdint.h>

#include "platform.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "autolaunch.h"

PG_REGISTER_WITH_RESET_TEMPLATE(autolaunchConfig_t, autolaunchConfig, PG_AUTOLAUNCH_CONFIG, 1);

PG_RESET_TEMPLATE(autolaunchConfig_t, autolaunchConfig,
    .auto_throttle = 1,
    .launch_throttle = 60,
    .climb_angle = 18,
    .stick_threshold = 20,
    .accel_threshold = 1900,
    .detect_time = 40,
    .motor_delay = 500,
    .timeout = 5000,
);

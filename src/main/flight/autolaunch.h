/*
 * This file is part of Wingflight.
 *
 * Wingflight is free software. You can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#pragma once

#include <stdbool.h>

#include "drivers/time.h"

void autolaunchUpdate(timeUs_t currentTimeUs);
bool autolaunchIsActive(void);
bool autolaunchHasThrottleOverride(void);
float autolaunchGetThrottle(void);
float autolaunchApplyAxis(int axis, float setpoint);

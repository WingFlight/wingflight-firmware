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

#include "common/axis.h"

// Samples are block means of the PID loops in each block.
#define TRIM_MONITOR_SAMPLE_HZ      50

// Smoothing time constant of the I-term, s.
#define TRIM_MONITOR_TAU            2.0f

// Qualifying flight before the first report, s. Lets I settle after takeoff.
#define TRIM_MONITOR_SETTLE         3.0f

// Out of trim when the smoothed I is above ON, back in trim below OFF.
// Surface units: 1.0 = full travel.
#define TRIM_MONITOR_ON             0.04f
#define TRIM_MONITOR_OFF            0.02f

// An axis only counts while its stick is near center, it isn't rotating and its
// surfaces aren't at a limit.
#define TRIM_MONITOR_STICK_MAX      0.05f   // |deflection|
#define TRIM_MONITOR_RATE_MAX       30.0f   // deg/s

// Only near-level flight counts (when an accelerometer gives the attitude), so
// knife edge, hover and steep climbs aren't taken for trim.
#define TRIM_MONITOR_ANGLE_MAX      30      // degrees, roll and pitch

// Which way an axis needs trimming. Sent in TELEM_SYSTEM_CONFIG, see telemetry/status.h.
typedef enum {
    TRIM_MONITOR_IN_TRIM = 0,
    TRIM_MONITOR_RIGHT_UP,          // roll/yaw: trim right; pitch: trim nose up
    TRIM_MONITOR_LEFT_DOWN,         // roll/yaw: trim left;  pitch: trim nose down
} trimMonitorState_e;

typedef struct {
    bool valid;                             // armed, airborne, plain rate flight, near level
    float iterm[XYZ_AXIS_COUNT];            // I output, surface units
    float deflection[XYZ_AXIS_COUNT];       // stick -1..1
    float gyro[XYZ_AXIS_COUNT];             // deg/s
    bool saturated[XYZ_AXIS_COUNT];
} trimMonitorSample_t;

void trimMonitorReset(void);

// Start a new measurement (on arming). Each axis keeps its last report until the new
// measurement has settled.
void trimMonitorRestart(void);

// One sample of dT seconds. Exposed for unit tests; the flight code calls trimMonitorUpdate().
void trimMonitorProcessSample(const trimMonitorSample_t *sample, float dT);

// Every PID loop, after pidController()
void trimMonitorUpdate(float dT);

trimMonitorState_e trimMonitorGetState(int axis);

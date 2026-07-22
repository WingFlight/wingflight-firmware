/*
 * This file is part of Wingflight.
 *
 * Wingflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Wingflight is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "common/time.h"

#ifdef USE_HIL_SENSOR_OVERRIDE

// See hitl/bridge/PROTOCOL.md for the wire format and rationale.

void hilSensorInit(void);
void hilSensorUpdate(timeUs_t currentTimeUs);

// True if a valid HIL sensor packet has been received within the last
// HIL_SENSOR_TIMEOUT_MS. Used both by the override hooks below and to
// decide whether it is safe to treat gyro/accel data as HIL-driven.
bool hilSensorIsActive(void);

// Called from sensors/gyro.c (gyroUpdateSensor) and sensors/acceleration.c
// (accUpdate), immediately after the real driver's readFn() has produced a
// fresh sample and the firmware's own per-board alignment/rotation has
// already been applied. Overrides gyroADC/accADC in place with the latest
// injected value (zero-order hold) when hilSensorIsActive(); a no-op
// otherwise, so real sensor data passes through unmodified if the bridge is
// not connected.
void hilSensorOverrideGyro(float gyroADC[3], float scale);
void hilSensorOverrideAcc(float accADC[3], uint16_t acc1G);

#endif // USE_HIL_SENSOR_OVERRIDE

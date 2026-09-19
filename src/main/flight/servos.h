/*
 * This file is part of Rotorflight.
 *
 * Rotorflight is free software. You can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Rotorflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software. If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include "config/config.h"

#include "pg/servos.h"
#include "pg/servo_curve.h"
#include "pg/adjustments.h"

#define DEFAULT_SERVO_FLAGS      0
#define DEFAULT_SERVO_CENTER  1500
#define DEFAULT_SERVO_MIN     -700
#define DEFAULT_SERVO_MAX      700
#define DEFAULT_SERVO_SCALE    500
#define DEFAULT_SERVO_RATE      50
#define DEFAULT_SERVO_SPEED      0

#define SERVO_LIMIT_MIN      -1000
#define SERVO_LIMIT_MAX       1000
#define SERVO_SCALE_MIN         50
#define SERVO_SCALE_MAX       1000
#define SERVO_RATE_MIN          25
#define SERVO_RATE_MAX        5000
#define SERVO_SPEED_MIN          0
#define SERVO_SPEED_MAX      60000
#define SERVO_OVERRIDE_MIN   -2000
#define SERVO_OVERRIDE_MAX    2000
#define SERVO_OVERRIDE_OFF   (SERVO_OVERRIDE_MAX + 1)

// A servo's trim may move its output by at most servo_trim_limit percent of its scale
// (rneg/rpos, whichever is larger). The trim is separate from the center so that
// whatever drives it (adjustment channel, auto trim, configurator) can never move a
// surface further than this from where the center puts it. The limit is a CLI-only
// setting: the default suits nearly everything, and a larger one has to be a
// deliberate choice.
#define SERVO_TRIM_LIMIT_PERCENT_DEFAULT  20
#define SERVO_TRIM_LIMIT_PERCENT_MAX      50
// Largest value the SERVO_TRIM_* adjustments can take; the per-servo limit above is what
// actually bounds the trim. A continuous (pot) adjustment is absolute, so a range wider
// than the limit just has dead travel at the ends.
#define SERVO_TRIM_ADJUSTMENT_MAX         200

enum {
    SERVO_FLAG_REVERSED     = BIT(0),
    SERVO_FLAG_GEO_CORR     = BIT(1),
    SERVO_FLAGS_ALL         = BIT(2) - 1,
};

void servoInit(void);
void servoUpdate(void);
void servoShutdown(void);

void validateAndFixServoConfig(void);
int16_t getServoTrim(uint8_t servo);
void setServoTrim(uint8_t servo, int16_t trim);
int16_t getServoTrimLimit(uint8_t servo);
int16_t getServoRuntimeTrim(uint8_t servo);
int getServoAxisRuntimeTrim(int axis);
void setServoAxisRuntimeTrim(int axis, int value);

ADJFUN_DECLARE(SERVO_TRIM_ROLL)
ADJFUN_DECLARE(SERVO_TRIM_PITCH)
ADJFUN_DECLARE(SERVO_TRIM_YAW)

uint8_t getServoCount(void);
uint16_t getServoOutput(uint8_t servo);

int16_t getServoOverride(uint8_t servo);
int16_t setServoOverride(uint8_t servo, int16_t val);
bool    hasServoOverride(uint8_t servo);
bool    isServoOverrideActive(void);

#ifdef USE_SERVO_GEOMETRY_CORRECTION
float geometryCorrection(float pos);
#endif


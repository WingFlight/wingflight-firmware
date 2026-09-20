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

#include "types.h"
#include "platform.h"

#include "common/curve.h"

#include "pg/pg.h"
#include "pg/servos.h"

// Same point count/UX as MIXER_CURVE_POINTS - one curve per physical
// servo (not a shared pool), added on top of a servo's own output (see
// servoUpdate()) so it can be trimmed to track another servo driving the
// same control surface (e.g. dual ailerons). Unlike mixerCurve_t, this is
// a corrective delta, not a reshape: x is the servo's own output
// (-1000..1000, full range), but y is a small offset added to it - the
// configurator constrains y to roughly -100..100 (+-10%), matching the
// intended use as a fine trim, not a general-purpose curve.
#define SERVO_CURVE_POINTS   9

typedef struct
{
    uint8_t       count;                       // active points, 2..SERVO_CURVE_POINTS
    curvePoint_t  points[SERVO_CURVE_POINTS];   // ascending by x, x: -1000..1000, y: delta, ~-100..100
} servoCurve_t;

PG_DECLARE_ARRAY(servoCurve_t, MAX_SUPPORTED_SERVOS, servoCurves);

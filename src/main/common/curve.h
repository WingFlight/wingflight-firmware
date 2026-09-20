/*
 * This file is part of Rotorflight.
 *
 * Rotorflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Rotorflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdint.h>

// Shared by mixerCurve_t and servoCurve_t - both use the same
// ascending-x, -1000..1000 point format.
typedef struct {
    int16_t   x;
    int16_t   y;
} curvePoint_t;

// Linear interpolation through a curve's (ascending-x) points. `xs` is on
// the same x1000 integer scale as the stored points (i.e. already
// multiplied by 1000 by the caller). Points beyond either end clamp to
// that end's y. Returns `fallback` unchanged if the curve has fewer than
// 2 active points (i.e. is unconfigured/disabled).
float evaluateCurvePoints(const curvePoint_t *points, uint8_t count, float xs, float fallback);

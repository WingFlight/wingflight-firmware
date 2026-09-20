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

#include <stdint.h>

#include "common/curve.h"

float evaluateCurvePoints(const curvePoint_t *points, uint8_t count, float xs, float fallback)
{
    const int n = count;

    if (n < 2)
        return fallback;

    if (xs <= points[0].x)
        return points[0].y;

    if (xs >= points[n - 1].x)
        return points[n - 1].y;

    for (int i = 0; i < n - 1; i++) {
        const curvePoint_t *p0 = &points[i];
        const curvePoint_t *p1 = &points[i + 1];

        if (xs >= p0->x && xs <= p1->x) {
            const float t = (p1->x != p0->x) ? (xs - p0->x) / (float)(p1->x - p0->x) : 0;
            return p0->y + t * (p1->y - p0->y);
        }
    }

    return fallback;
}

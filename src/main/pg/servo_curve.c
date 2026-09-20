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

#include "types.h"
#include "platform.h"

#ifdef USE_SERVOS

#include "pg/pg_ids.h"
#include "pg/servo_curve.h"

PG_REGISTER_ARRAY_WITH_RESET_FN(servoCurve_t, MAX_SUPPORTED_SERVOS, servoCurves, PG_SERVO_CURVES, 0);

void pgResetFn_servoCurves(servoCurve_t *curve)
{
    // Flat zero-delta per servo, so an unconfigured curve adds no
    // correction to servo output (see servoUpdate() - the curve is added
    // to the servo's own output, not substituted for it).
    for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        curve[i].count = 2;
        curve[i].points[0] = (curvePoint_t){ -1000, 0 };
        curve[i].points[1] = (curvePoint_t){  1000, 0 };
    }
}

#endif

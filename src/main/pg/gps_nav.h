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

#include "pg/pg.h"

typedef enum {
    NAV_LOITER_CW = 0,
    NAV_LOITER_CCW,
} navLoiterDirection_e;

typedef struct {
    uint16_t    loiterRadiusM;      // meters
    uint8_t     loiterDirection;    // navLoiterDirection_e
    uint16_t    rthAltitudeM;       // meters, above the altitude recorded at arm
    uint8_t     minSats;
    uint8_t     maxBankAngleDeg;
    uint8_t     maxPitchAngleDeg;
    uint16_t    bearingKp;          // centidegrees of bank per degree of bearing error
    uint16_t    altitudeKp;         // centidegrees of pitch per meter of altitude error
} gpsNavConfig_t;

PG_DECLARE(gpsNavConfig_t, gpsNavConfig);

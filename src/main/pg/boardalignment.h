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

#include "common/sensor_alignment.h"

#include "pg/pg.h"

typedef struct {
    int32_t rollDegrees;
    int32_t pitchDegrees;
    int32_t yawDegrees;

    // Fine correction for a mounting surface that isn't perfectly level
    // relative to the airframe's true reference, composed AFTER (outermost
    // to) rollDegrees/pitchDegrees/yawDegrees above -- see
    // initBoardAlignment()/alignBoard() in sensors/boardalignment.c. Kept
    // deliberately separate from the discrete alignment above: composing a
    // fine trim into the same fixed roll->pitch->yaw sequence as a large
    // 90-degree-step correction makes the trim's effective axis depend on
    // whatever the discrete correction happens to be (e.g. a "roll" trim
    // shows up entirely as pitch once yaw=270 -- verified algebraically).
    // Reuses sensorAlignment_t (decidegrees, already used for gyro/mag chip
    // alignment) rather than inventing a new type.
    sensorAlignment_t mountTrim;
} boardAlignment_t;

PG_DECLARE(boardAlignment_t, boardAlignment);

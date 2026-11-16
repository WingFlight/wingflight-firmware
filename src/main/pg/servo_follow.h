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
#include "pg/servos.h"

// Servos that drive the same control surface (e.g. two servos on one aileron).
// A follower takes the mixer output of the servo it follows, its primary, and
// then applies its own reverse, geometry correction, balance curve, scale, trim,
// center and limits, so it can be mounted the other way round and balanced to
// track the primary. It needs no mixer rules of its own; any it has are ignored.
// Its speed limit is the primary's, so a surface never has servos fighting at
// different speeds. See servoUpdate() in flight/servos.c.
typedef struct {
    uint8_t primary;    // 0 = drives itself from its own mixer rules, N = follows servo N (1-based)
} servoFollow_t;

PG_DECLARE_ARRAY(servoFollow_t, MAX_SUPPORTED_SERVOS, servoFollows);

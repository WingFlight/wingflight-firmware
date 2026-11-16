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

#include <stdbool.h>
#include <stdint.h>

#include "types.h"
#include "platform.h"

#include "pg/pg.h"
#include "pg/servos.h"

// A short label per servo ("L AIL", "ELEV"), for reference in the Configurator, the
// CLI and the Lua suites. Nothing in flight reads it.
#define SERVO_NAME_LENGTH 8

typedef struct {
    char name[SERVO_NAME_LENGTH + 1];   // always NUL-terminated, empty by default
} servoName_t;

PG_DECLARE_ARRAY(servoName_t, MAX_SUPPORTED_SERVOS, servoNames);

// Sets servo `index`'s name from `len` bytes of `name` (not necessarily NUL-terminated).
// Leading and trailing spaces are dropped. Refuses (returns false, name unchanged) a name
// longer than SERVO_NAME_LENGTH or with a character outside printable ASCII. An empty
// name clears it.
bool setServoName(uint8_t index, const char *name, uint8_t len);

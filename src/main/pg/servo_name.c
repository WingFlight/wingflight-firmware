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


#include <string.h>

#include "types.h"
#include "platform.h"

#ifdef USE_SERVOS

#include "pg/pg_ids.h"
#include "pg/servo_name.h"

// Zero-filled by default: no names.
PG_REGISTER_ARRAY(servoName_t, MAX_SUPPORTED_SERVOS, servoNames, PG_SERVO_NAMES, 0);

bool setServoName(uint8_t index, const char *name, uint8_t len)
{
    if (index >= MAX_SUPPORTED_SERVOS) {
        return false;
    }

    while (len > 0 && name[0] == ' ') {
        name++;
        len--;
    }
    while (len > 0 && name[len - 1] == ' ') {
        len--;
    }

    if (len > SERVO_NAME_LENGTH) {
        return false;
    }
    for (uint8_t i = 0; i < len; i++) {
        if (name[i] < 0x20 || name[i] > 0x7E) {
            return false;
        }
    }

    servoName_t *entry = servoNamesMutable(index);
    memset(entry->name, 0, sizeof(entry->name));
    memcpy(entry->name, name, len);
    return true;
}

#endif

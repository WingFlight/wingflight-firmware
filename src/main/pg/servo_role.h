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

/*
 * What each servo output drives (servoRoles PG), for the configurator and the
 * radio scripts to show and group servos by surface. Descriptive only: the
 * firmware never reads it, so it can't change how anything flies.
 *
 * One byte per servo: the role in the upper 5 bits, and which servo of that
 * role it is (1-8, stored as 0-7) in the lower 3, e.g. the second servo on the
 * left aileron. The numbers are part of the MSP and CLI interface: add new roles
 * in the reserved slots, never renumber.
 */
typedef enum {
    SERVO_ROLE_NONE = 0,
    SERVO_ROLE_AILERON,
    SERVO_ROLE_AILERON_LEFT,
    SERVO_ROLE_AILERON_RIGHT,
    SERVO_ROLE_ELEVATOR,
    SERVO_ROLE_ELEVATOR_LEFT,
    SERVO_ROLE_ELEVATOR_RIGHT,
    SERVO_ROLE_RUDDER,
    SERVO_ROLE_VTAIL_LEFT,
    SERVO_ROLE_VTAIL_RIGHT,
    SERVO_ROLE_ELEVON_LEFT,
    SERVO_ROLE_ELEVON_RIGHT,
    SERVO_ROLE_FLAP,
    SERVO_ROLE_FLAP_LEFT,
    SERVO_ROLE_FLAP_RIGHT,
    SERVO_ROLE_SPOILER,
    SERVO_ROLE_SPOILER_LEFT,
    SERVO_ROLE_SPOILER_RIGHT,
    SERVO_ROLE_THROTTLE,
    SERVO_ROLE_THRUST_VECTOR_PITCH,
    SERVO_ROLE_THRUST_VECTOR_YAW,
    SERVO_ROLE_THRUST_VECTOR_ROLL,
    SERVO_ROLE_STEERING,
    SERVO_ROLE_GEAR,
    SERVO_ROLE_WHEEL_BRAKE,
    SERVO_ROLE_TOW_RELEASE,
    SERVO_ROLE_PAYLOAD_RELEASE,
    SERVO_ROLE_SMOKE,
    SERVO_ROLE_CHOKE,
    // 29-30 reserved for new roles
    SERVO_ROLE_OTHER = 31,
    SERVO_ROLE_COUNT = 32,
} servoRole_e;

#define SERVO_ROLE_NUMBER_MAX   8

typedef struct {
    uint8_t value;      // role << 3 | (number - 1)
} servoRole_t;

PG_DECLARE_ARRAY(servoRole_t, MAX_SUPPORTED_SERVOS, servoRoles);

static inline uint8_t servoRoleValue(servoRole_e role, uint8_t number)
{
    const uint8_t n = (number < 1) ? 1 : (number > SERVO_ROLE_NUMBER_MAX) ? SERVO_ROLE_NUMBER_MAX : number;
    return (uint8_t)(((role & 0x1F) << 3) | (n - 1));
}

static inline servoRole_e servoRoleOf(uint8_t value)
{
    return (servoRole_e)(value >> 3);
}

// Which servo of its role this is, 1-8.
static inline uint8_t servoRoleNumber(uint8_t value)
{
    return (value & 0x07) + 1;
}

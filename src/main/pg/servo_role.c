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
#include "pg/servo_role.h"

PG_REGISTER_ARRAY_WITH_RESET_FN(servoRole_t, MAX_SUPPORTED_SERVOS, servoRoles, PG_SERVO_ROLES, 0);

// Matches the default mixer rules (pg/mixer.c): S1 left aileron, S2 right
// aileron, S3 elevator, S4 rudder. The rest have no role.
void pgResetFn_servoRoles(servoRole_t *instance)
{
    for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
        instance[i].value = servoRoleValue(SERVO_ROLE_NONE, 1);
    }
    instance[0].value = servoRoleValue(SERVO_ROLE_AILERON_LEFT, 1);
    instance[1].value = servoRoleValue(SERVO_ROLE_AILERON_RIGHT, 1);
    instance[2].value = servoRoleValue(SERVO_ROLE_ELEVATOR, 1);
    instance[3].value = servoRoleValue(SERVO_ROLE_RUDDER, 1);
}

#endif

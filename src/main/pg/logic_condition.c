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

#include "pg/pg_ids.h"
#include "pg/logic_condition.h"

#include "config/config_reset.h"


PG_REGISTER_ARRAY_WITH_RESET_FN(logicCondition_t, LOGIC_CONDITION_COUNT, logicConditions, PG_GENERIC_LOGIC_CONDITIONS, 0);

void pgResetFn_logicConditions(logicCondition_t *condition)
{
    for (int i = 0; i < LOGIC_CONDITION_COUNT; i++) {
        condition[i].enabled = 0;
        condition[i].operation = LOGIC_CONDITION_TRUE;
    }
}

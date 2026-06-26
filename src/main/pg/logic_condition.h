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

#define LOGIC_CONDITION_COUNT   16

enum {
    LOGIC_CONDITION_TRUE = 0,
    LOGIC_CONDITION_EQUAL,
    LOGIC_CONDITION_GREATER_THAN,
    LOGIC_CONDITION_LOWER_THAN,
    LOGIC_CONDITION_AND,
    LOGIC_CONDITION_OR,
    LOGIC_CONDITION_XOR,
    LOGIC_CONDITION_NOT,
    LOGIC_CONDITION_STICKY,        // operandA=set trigger, operandB=reset trigger
    LOGIC_CONDITION_DELAY,         // operandA=trigger, operandB=hold time (ms) before true
    LOGIC_CONDITION_EDGE,          // operandA=trigger, operandB=pulse duration (ms) from rising edge
    LOGIC_CONDITION_OPERATION_COUNT
};

enum {
    LOGIC_CONDITION_OPERAND_TYPE_VALUE = 0,    // literal constant
    LOGIC_CONDITION_OPERAND_TYPE_RC_CHANNEL,   // 0-based index into rcInput[] (0-3=Roll/Pitch/Yaw/Throttle, 4+=AUX1..)
    LOGIC_CONDITION_OPERAND_TYPE_FLIGHT_MODE,  // a boxId_e - evaluates to 1/0 via IS_RC_MODE_ACTIVE
    LOGIC_CONDITION_OPERAND_TYPE_CONDITION,    // another condition's own last result (0-based index)
    LOGIC_CONDITION_OPERAND_TYPE_COUNT
};

typedef struct
{
    uint8_t   enabled;
    uint8_t   operation;
    uint8_t   operandAType;
    int16_t   operandAValue;
    uint8_t   operandBType;
    int16_t   operandBValue;
} logicCondition_t;

PG_DECLARE_ARRAY(logicCondition_t, LOGIC_CONDITION_COUNT, logicConditions);

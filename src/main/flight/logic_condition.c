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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "drivers/time.h"

#include "fc/rc_modes.h"

#include "pg/rx.h"
#include "pg/logic_condition.h"

#include "rx/rx.h"

#include "flight/logic_condition.h"

// Runtime-only state for the stateful operators (STICKY/DELAY/EDGE) - not
// persisted, mirrors how mixer.ruleOutput[] holds the per-rule slew state
// alongside the persisted mixerRule_t array.
typedef struct {
    bool        lastInput;
    bool        latched;
    timeMs_t    eventTime;
} logicConditionState_t;

static logicConditionState_t conditionState[LOGIC_CONDITION_COUNT];
static bool conditionValue[LOGIC_CONDITION_COUNT];

static float logicConditionGetOperandValue(uint8_t type, int16_t value)
{
    switch (type) {
        case LOGIC_CONDITION_OPERAND_TYPE_RC_CHANNEL:
            if (value >= 0 && value < MAX_SUPPORTED_RC_CHANNEL_COUNT) {
                return rcInput[value];
            }
            return 0;

        case LOGIC_CONDITION_OPERAND_TYPE_FLIGHT_MODE:
            if (value >= 0 && value < CHECKBOX_ITEM_COUNT) {
                return IS_RC_MODE_ACTIVE((boxId_e)value) ? 1 : 0;
            }
            return 0;

        case LOGIC_CONDITION_OPERAND_TYPE_CONDITION:
            if (value >= 0 && value < LOGIC_CONDITION_COUNT) {
                return conditionValue[value] ? 1 : 0;
            }
            return 0;

        case LOGIC_CONDITION_OPERAND_TYPE_VALUE:
        default:
            return value;
    }
}

static bool logicConditionEvaluate(int index)
{
    const logicCondition_t *condition = logicConditions(index);
    logicConditionState_t  *state = &conditionState[index];

    const float a = logicConditionGetOperandValue(condition->operandAType, condition->operandAValue);
    const float b = logicConditionGetOperandValue(condition->operandBType, condition->operandBValue);

    switch (condition->operation) {
        case LOGIC_CONDITION_EQUAL:
            return a == b;

        case LOGIC_CONDITION_GREATER_THAN:
            return a > b;

        case LOGIC_CONDITION_LOWER_THAN:
            return a < b;

        case LOGIC_CONDITION_AND:
            return (a != 0) && (b != 0);

        case LOGIC_CONDITION_OR:
            return (a != 0) || (b != 0);

        case LOGIC_CONDITION_XOR:
            return (a != 0) != (b != 0);

        case LOGIC_CONDITION_NOT:
            return (a == 0);

        case LOGIC_CONDITION_STICKY: {
            // operandA sets the latch, operandB resets it - stays on/off until the other fires
            if (b != 0) {
                state->latched = false;
            } else if (a != 0) {
                state->latched = true;
            }
            return state->latched;
        }

        case LOGIC_CONDITION_DELAY: {
            // True only once operandA has held continuously for operandB ms; drops the instant it goes false
            const bool input = (a != 0);
            if (input && !state->lastInput) {
                state->eventTime = millis();
            }
            state->lastInput = input;
            return input && (millis() - state->eventTime) >= (timeMs_t)b;
        }

        case LOGIC_CONDITION_EDGE: {
            // Pulses true for operandB ms starting at operandA's rising edge, then auto-resets
            const bool input = (a != 0);
            if (input && !state->lastInput) {
                state->eventTime = millis();
                state->latched = true;
            }
            state->lastInput = input;
            if (state->latched && (millis() - state->eventTime) >= (timeMs_t)b) {
                state->latched = false;
            }
            return state->latched;
        }

        case LOGIC_CONDITION_TRUE:
        default:
            return true;
    }
}

void logicConditionUpdate(void)
{
    // Evaluated in index order - a condition referencing a later-indexed one
    // via OPERAND_TYPE_CONDITION sees that condition's value from last cycle.
    for (int i = 0; i < LOGIC_CONDITION_COUNT; i++) {
        conditionValue[i] = logicConditions(i)->enabled && logicConditionEvaluate(i);
    }
}

bool logicConditionGetValue(int index)
{
    if (index < 0 || index >= LOGIC_CONDITION_COUNT) {
        return false;
    }
    return conditionValue[index];
}

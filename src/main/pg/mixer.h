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

enum {
    TAIL_MODE_VARIABLE,
    TAIL_MODE_MOTORIZED,
    TAIL_MODE_BIDIRECTIONAL,
};

enum {
    MIXER_IN_NONE = 0,
    MIXER_IN_STABILIZED_ROLL,
    MIXER_IN_STABILIZED_PITCH,
    MIXER_IN_STABILIZED_YAW,
    MIXER_IN_STABILIZED_THROTTLE,
    MIXER_IN_RC_COMMAND_ROLL,
    MIXER_IN_RC_COMMAND_PITCH,
    MIXER_IN_RC_COMMAND_YAW,
    MIXER_IN_RC_COMMAND_THROTTLE,
    MIXER_IN_RC_CHANNEL_ROLL,
    MIXER_IN_RC_CHANNEL_PITCH,
    MIXER_IN_RC_CHANNEL_YAW,
    MIXER_IN_RC_CHANNEL_THROTTLE,
    MIXER_IN_RC_CHANNEL_AUX1,
    MIXER_IN_RC_CHANNEL_AUX2,
    MIXER_IN_RC_CHANNEL_AUX3,
    MIXER_IN_RC_CHANNEL_8,
    MIXER_IN_RC_CHANNEL_9,
    MIXER_IN_RC_CHANNEL_10,
    MIXER_IN_RC_CHANNEL_11,
    MIXER_IN_RC_CHANNEL_12,
    MIXER_IN_RC_CHANNEL_13,
    MIXER_IN_RC_CHANNEL_14,
    MIXER_IN_RC_CHANNEL_15,
    MIXER_IN_RC_CHANNEL_16,
    MIXER_IN_RC_CHANNEL_17,
    MIXER_IN_RC_CHANNEL_18,
    MIXER_IN_COUNT
};

enum {
    MIXER_OP_NUL = 0,
    MIXER_OP_SET,
    MIXER_OP_ADD,
    MIXER_OP_MUL,
    MIXER_OP_COUNT
};

#define MIXER_RULE_COUNT      32
#define MIXER_INPUT_COUNT     MIXER_IN_COUNT
#define MIXER_CURVE_COUNT     8
#define MIXER_CURVE_POINTS    9

typedef struct
{
    uint8_t   tail_rotor_mode;      // Tail motor vs. variable pitch tail
} mixerConfig_t;

PG_DECLARE(mixerConfig_t, mixerConfig);

typedef struct
{
    int16_t   rate;             // multiplier
    int16_t   min;              // minimum input value
    int16_t   max;              // maximum input value
} mixerInput_t;

PG_DECLARE_ARRAY(mixerInput_t, MIXER_INPUT_COUNT, mixerInputs);

typedef struct
{
    int16_t   x;                // -1000..1000, normalized like weight/offset
    int16_t   y;                // -1000..1000
} mixerCurvePoint_t;

typedef struct
{
    uint8_t            count;                          // active points, 2..MIXER_CURVE_POINTS
    mixerCurvePoint_t   points[MIXER_CURVE_POINTS];     // ascending by x
} mixerCurve_t;

PG_DECLARE_ARRAY(mixerCurve_t, MIXER_CURVE_COUNT, mixerCurves);

typedef struct
{
    uint8_t   oper;             // rule operation
    uint8_t   input;            // input channel
    uint8_t   output;           // output channel
    int16_t   offset;           // addition
    int16_t   weight;           // signed multiplier applied when the input is >= 0
    int16_t   weightNeg;        // signed multiplier applied when the input is < 0 (for differential and/or polarity)
    uint16_t  speed;            // slew rate limit on this rule's contribution (0=unlimited, same units/scale as servo speed)
    uint8_t   curve;            // 0=none, 1..MIXER_CURVE_COUNT = mixerCurves(curve-1), applied before weight selection
    uint8_t   condition;        // 0=always active, 1..LOGIC_CONDITION_COUNT = logicConditions(condition-1) gates this rule
} mixerRule_t;

PG_DECLARE_ARRAY(mixerRule_t, MIXER_RULE_COUNT, mixerRules);


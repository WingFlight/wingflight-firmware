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
#include "pg/mixer.h"

#include "config/config_reset.h"
#include "flight/mixer.h"


PG_REGISTER_WITH_RESET_TEMPLATE(mixerConfig_t, mixerConfig, PG_GENERIC_MIXER_CONFIG, 0);

PG_RESET_TEMPLATE(mixerConfig_t, mixerConfig,
    .main_rotor_dir = DIR_CW,
    .tail_rotor_mode = TAIL_MODE_VARIABLE,
    .tail_motor_idle = 0,
    .tail_center_trim = 0,
    .swash_type = SWASH_TYPE_NONE,
    .swash_ring = 100,
    .swash_phase = 0,
    .swash_pitch_limit = 0,
    .swash_trim = { 0, 0, 0 },
    .swash_tta_precomp = 0,
    .swash_geo_correction = 0,
    .collective_tilt_correction_pos = 0,
    .collective_tilt_correction_neg = 10,
);

// v1: added weightNeg (second weight applied when a rule's input is negative,
// for differential mixing). v2: added reverse (inverts the rule's polarity).
// Existing saved rules reset to these defaults rather than being
// reinterpreted at the new, wider per-rule layout.
PG_REGISTER_ARRAY_WITH_RESET_FN(mixerRule_t, MIXER_RULE_COUNT, mixerRules, PG_GENERIC_MIXER_RULES, 2);

void pgResetFn_mixerRules(mixerRule_t *rule)
{
    // Default standard glider mixer rules (2 ail + elev + rud + motor)
    // S1: Left aileron (STABILIZED_ROLL, weight +1000)
    rule[0].oper   = MIXER_OP_SET;
    rule[0].input  = MIXER_IN_STABILIZED_ROLL;
    rule[0].output = MIXER_SERVO_OFFSET + 0;
    rule[0].offset = 0;
    rule[0].weight = 1000;
    rule[0].weightNeg = 1000;

    // S2: Right aileron (STABILIZED_ROLL, weight -1000 for differential)
    rule[1].oper   = MIXER_OP_SET;
    rule[1].input  = MIXER_IN_STABILIZED_ROLL;
    rule[1].output = MIXER_SERVO_OFFSET + 1;
    rule[1].offset = 0;
    rule[1].weight = -1000;
    rule[1].weightNeg = -1000;

    // S3: Elevator (STABILIZED_PITCH, weight +1000)
    rule[2].oper   = MIXER_OP_SET;
    rule[2].input  = MIXER_IN_STABILIZED_PITCH;
    rule[2].output = MIXER_SERVO_OFFSET + 2;
    rule[2].offset = 0;
    rule[2].weight = 1000;
    rule[2].weightNeg = 1000;

    // S4: Rudder (STABILIZED_YAW, weight +1000)
    rule[3].oper   = MIXER_OP_SET;
    rule[3].input  = MIXER_IN_STABILIZED_YAW;
    rule[3].output = MIXER_SERVO_OFFSET + 3;
    rule[3].offset = 0;
    rule[3].weight = 1000;
    rule[3].weightNeg = 1000;

    // M1: Motor/Throttle (RC_CHANNEL_THROTTLE, weight +1000)
    rule[4].oper   = MIXER_OP_SET;
    rule[4].input  = MIXER_IN_RC_CHANNEL_THROTTLE;
    rule[4].output = MIXER_MOTOR_OFFSET + 0;
    rule[4].offset = 0;
    rule[4].weight = 1000;
    rule[4].weightNeg = 1000;

    // Clear remaining rules
    for (int i = 5; i < MIXER_RULE_COUNT; i++) {
        rule[i].oper = MIXER_OP_NUL;
    }
}

PG_REGISTER_ARRAY_WITH_RESET_FN(mixerInput_t, MIXER_INPUT_COUNT, mixerInputs, PG_GENERIC_MIXER_INPUTS, 0);

void pgResetFn_mixerInputs(mixerInput_t *input)
{
    // Wing: stabilized roll/pitch/yaw at full rate with symmetric limits
    for (int i = MIXER_IN_STABILIZED_ROLL; i <= MIXER_IN_STABILIZED_YAW; i++) {
        input[i].rate =  1000;
        input[i].min  = -1000;
        input[i].max  =  1000;
    }

    // Collective is unused for fixed-wing; keep at zero-rate so it has no effect
    input[MIXER_IN_STABILIZED_COLLECTIVE].rate = 0;
    input[MIXER_IN_STABILIZED_COLLECTIVE].min  = -1000;
    input[MIXER_IN_STABILIZED_COLLECTIVE].max  =  1000;

    input[MIXER_IN_STABILIZED_THROTTLE].rate =  1000;
    input[MIXER_IN_STABILIZED_THROTTLE].min  =  0;
    input[MIXER_IN_STABILIZED_THROTTLE].max  =  1000;

    for (int i = MIXER_IN_RC_COMMAND_ROLL; i <= MIXER_IN_RC_CHANNEL_18; i++) {
        input[i].rate =  1000;
        input[i].min  = -1000;
        input[i].max  =  1000;
    }
}


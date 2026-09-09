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

#include "common/utils.h"
#include "pg/pg.h"

// SBUS has 18 channels, FBUS has 16 channels

// Bus servo defaults (S9-S26) - constrained to BUS_SERVO_MIN/MAX range
#define DEFAULT_BUS_SERVO_MIN     -500
#define DEFAULT_BUS_SERVO_MAX      500
#define DEFAULT_BUS_SERVO_SCALE    1000
#define BUS_SERVO_MAX_SIGNAL       2000
#define BUS_SERVO_MIN_SIGNAL       1000

// S1-S8 (indices 0-7) are PWM servos
// S9-S26 (indices 8-25) are BUS servos for SBUS/FBUS
#define BUS_SERVO_OFFSET 8

// Bus servo output functions
void setBusServoOutput(uint8_t channel, float value);
uint16_t getBusServoOutput(uint8_t channel);

// Bus servo configuration helpers
bool hasBusServosConfigured(void);

typedef struct busServoConfig_s {
    // When ON (default), bus channel N mirrors PWM servo output S(N+1)
    // one-to-one (PWM1->BUS1, PWM2->BUS2, ...) for every bus channel that
    // has a physical PWM counterpart, so mixer setups built by the
    // configurator's PWM-only wizards (named model types) drive bus servos
    // too without any extra mixer rules. Bus channels beyond the physical
    // PWM servo count always keep their own independent mixer rule
    // regardless of this setting, since there's no PWM output to mirror.
    //
    // Turn this OFF for a custom model where the bus servos need mixer
    // rules of their own that differ from the PWM outputs (e.g. a
    // different surface layout, or extra bus-only channels interleaved
    // with the PWM-mirrored ones).
    uint8_t cloneFromPwm;
} busServoConfig_t;

PG_DECLARE(busServoConfig_t, busServoConfig);

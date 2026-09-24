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
#include "pg/bus_servo.h"

#define SBUS_OUT_CHANNELS 18
typedef struct sbusOutConfig_s {

    // SBus output frame rate in Hz, typically 50Hz. Your receiver may support
    // faster updates.
    uint8_t frameRate;

    uint8_t pinSwap;

    // When ON, the UART output is electrically inverted (S.Bus signal uses
    // inverted logic). When OFF, the output is non-inverted.
    uint8_t inverted;

    // Channel count, busOutChannels_e (pg/bus_servo.h): 8, 12 or 16. The frame
    // is always the 16-channel SBUS frame; channels past the count are sent
    // at center.
    uint8_t channels;

} sbusOutConfig_t;

PG_DECLARE(sbusOutConfig_t, sbusOutConfig);

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

// SBUS carries 16 analog + 2 digital channels. F.Bus carries 8 analog (8-channel
// frame), 16 + 2 digital (16-channel frame) or 24 analog (24-channel frame).

// Bus output channel count setting (sbus_out_channels, fbus_master_channels).
// SBUS offers 8, 12 and 16 only.
typedef enum {
    BUS_OUT_CHANNELS_8 = 0,
    BUS_OUT_CHANNELS_12,
    BUS_OUT_CHANNELS_16,
    BUS_OUT_CHANNELS_24,
    BUS_OUT_CHANNELS_COUNT
} busOutChannels_e;

// Channel count (8, 12, 16 or 24) for a busOutChannels_e setting
uint8_t busOutChannelCount(uint8_t setting);
// busOutChannels_e setting for a channel count, BUS_OUT_CHANNELS_COUNT if invalid
uint8_t busOutChannelSetting(uint8_t count);
// Analog bus servo channels the configured bus outputs drive (the larger if
// SBUS and F.Bus are both set up), 0 without a bus output
uint8_t getBusServoOutputCount(void);

// Servo N is channel N on the wire: a bus output sends the same finished servo
// output (flight/servos.c) that goes to PWM output N on boards that have one.
// The bus carries 1000-2000us, so a servo that is only on the bus is limited
// to that range, and one that is also on a PWM output is clipped to it on the
// bus only.
#define BUS_SERVO_MAX_SIGNAL       2000
#define BUS_SERVO_MIN_SIGNAL       1000

// Default travel of the servos past the PWM outputs (S9-S24), which can only
// ever be on a bus
#define DEFAULT_BUS_SERVO_MIN     -500
#define DEFAULT_BUS_SERVO_MAX      500

// Servos a bus output of this many analog channels takes its values from.
// The 16-channel frame also has two digital channels (CH17-18), on at 1500us
// and above, taken from S17 and S18.
uint8_t busOutServoCount(uint8_t channels);
// Servos the configured bus outputs take their values from (the larger if
// SBUS and F.Bus are both set up), 0 without a bus output
uint8_t getBusServoCount(void);

// Whether a bus output (SBUS or F.Bus) is set up on a serial port
bool hasBusServosConfigured(void);

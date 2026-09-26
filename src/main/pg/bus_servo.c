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
#include <math.h>

#include "platform.h"

#if defined(USE_SBUS_OUTPUT) || defined(USE_FBUS_MASTER) || defined(USE_BUS_SERVO)

#include "common/maths.h"
#include "config/config_reset.h"
#include "io/serial.h"
#include "pg/bus_servo.h"
#include "pg/fbus_master.h"
#include "pg/sbus_output.h"
#include "pg/pg_ids.h"

PG_REGISTER_WITH_RESET_TEMPLATE(busServoConfig_t, busServoConfig, PG_BUS_SERVO_CONFIG, 0);

PG_RESET_TEMPLATE(busServoConfig_t, busServoConfig,
    .cloneFromPwm = 1,
);

bool hasBusServosConfigured(void)
{
    return findSerialPortConfig(FUNCTION_SBUS_OUT) || findSerialPortConfig(FUNCTION_FBUS_MASTER);
}

static const uint8_t busOutChannelCounts[BUS_OUT_CHANNELS_COUNT] = { 8, 12, 16, 24 };

uint8_t busOutChannelCount(uint8_t setting)
{
    return busOutChannelCounts[MIN(setting, BUS_OUT_CHANNELS_COUNT - 1)];
}

uint8_t busOutChannelSetting(uint8_t count)
{
    for (int i = 0; i < BUS_OUT_CHANNELS_COUNT; i++) {
        if (busOutChannelCounts[i] == count) {
            return i;
        }
    }
    return BUS_OUT_CHANNELS_COUNT;
}

uint8_t getBusServoOutputCount(void)
{
    // SBUS and F.Bus output can run at the same time; bus servo N is channel
    // N on both, so the count is the larger of the two.
    uint8_t count = 0;
#ifdef USE_FBUS_MASTER
    if (findSerialPortConfig(FUNCTION_FBUS_MASTER)) {
        count = MAX(count, busOutChannelCount(fbusMasterConfig()->channels));
    }
#endif
#ifdef USE_SBUS_OUTPUT
    if (findSerialPortConfig(FUNCTION_SBUS_OUT)) {
        count = MAX(count, MIN(busOutChannelCount(sbusOutConfig()->channels), 16));
    }
#endif
    return count;
}

// Storage for bus servo outputs (SBUS/FBUS)
static float busServoOutput[BUS_SERVO_CHANNELS];

void setBusServoOutput(uint8_t channel, float value)
{
    if (channel < BUS_SERVO_CHANNELS) {
        busServoOutput[channel] = value;
    }
}

uint16_t getBusServoOutput(uint8_t channel)
{
    if (channel < BUS_SERVO_CHANNELS) {
        long value = lrintf(busServoOutput[channel]);
        if (value < 0) return 0;
        if (value > UINT16_MAX) return UINT16_MAX;
        return (uint16_t)value;
    }
    return 0;
}
#endif

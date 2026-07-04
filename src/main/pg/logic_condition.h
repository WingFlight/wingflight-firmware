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
    LOGIC_CONDITION_APPROX_EQUAL,  // true when operandA is within LOGIC_CONDITION_APPROX_TOLERANCE of operandB
    LOGIC_CONDITION_OPERATION_COUNT
};

enum {
    LOGIC_CONDITION_OPERAND_TYPE_VALUE = 0,    // literal constant
    LOGIC_CONDITION_OPERAND_TYPE_RC_CHANNEL,   // 0-based index into rcInput[] (0-3=Roll/Pitch/Yaw/Throttle, 4+=AUX1..)
    LOGIC_CONDITION_OPERAND_TYPE_FLIGHT_MODE,  // a boxId_e - evaluates to 1/0 via IS_RC_MODE_ACTIVE
    LOGIC_CONDITION_OPERAND_TYPE_CONDITION,    // another condition's own last result (0-based index)
    LOGIC_CONDITION_OPERAND_TYPE_SENSOR,       // a logicSensor_e - live reading from a common sensor
    LOGIC_CONDITION_OPERAND_TYPE_PROFILE,      // a logicProfile_e - which profile of that kind is active
    LOGIC_CONDITION_OPERAND_TYPE_COUNT
};

// Common sensor readings usable as a logic condition operand, selected the same way
// OPERAND_TYPE_FLIGHT_MODE selects a boxId_e - value is the index below, not the reading itself.
enum {
    LOGIC_SENSOR_ALTITUDE = 0,  // 0.1m units (decimeters), getAltitude() * 10
    LOGIC_SENSOR_VOLTAGE,       // 0.1V units, getBatteryVoltage()
    LOGIC_SENSOR_CURRENT,       // 0.1A units, getBatteryCurrent()
    LOGIC_SENSOR_RPM,           // motor 1 RPM, getMotorRPMf(0)
    LOGIC_SENSOR_RSSI,          // %, getRssiPercent()
    LOGIC_SENSOR_BATTERY_PERCENT, // %, calculateBatteryPercentageRemaining()
    LOGIC_SENSOR_MAH_DRAWN,     // mAh, getBatteryCapacityUsed()
    LOGIC_SENSOR_GPS_SPEED,     // 0.1m/s units, gpsSol.groundSpeed (0 without a GPS fix)
    LOGIC_SENSOR_COUNT
};

// Which profile of each kind is currently active, selected the same way
// OPERAND_TYPE_SENSOR selects a logicSensor_e. All three kinds support 6 slots.
enum {
    LOGIC_PROFILE_PID = 0,      // 0-based index, getCurrentPidProfileIndex()
    LOGIC_PROFILE_RATE,         // 0-based index, getCurrentControlRateProfileIndex()
    LOGIC_PROFILE_BATTERY,      // 0-based index, getCurrentBatteryProfileIndex()
    LOGIC_PROFILE_COUNT
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

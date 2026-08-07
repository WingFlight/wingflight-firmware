/*
 * This file is part of Wingflight.
 *
 * Wingflight is free software. You can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Wingflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software. If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define KINGTECH_COMPACT_PAYLOAD_LENGTH  0x0C
#define KINGTECH_CONTROL_PAYLOAD_LENGTH  0x30
#define KINGTECH_EXTENDED_PAYLOAD_LENGTH 0x52
#define KINGTECH_MAX_FRAME_LENGTH        (KINGTECH_EXTENDED_PAYLOAD_LENGTH + 7)

typedef enum {
    KINGTECH_DECODE_NONE = 0,
    KINGTECH_DECODE_FRAME,
    KINGTECH_DECODE_INVALID_LENGTH,
    KINGTECH_DECODE_UNSUPPORTED_TYPE,
    KINGTECH_DECODE_INVALID_CHECKSUM,
    KINGTECH_DECODE_INVALID_TRAILER,
} kingtechDecodeResult_e;

typedef enum {
    KINGTECH_STATUS_TRIM_LOW = 0,
    KINGTECH_STATUS_READY = 1,
    KINGTECH_STATUS_STICK_LOW = 2,
    KINGTECH_STATUS_GLOW_TEST = 3,
    KINGTECH_STATUS_START_ON = 4,
    KINGTECH_STATUS_IGNITION = 5,
    KINGTECH_STATUS_PREHEAT = 6,
    KINGTECH_STATUS_FUEL_RAMP = 7,
    KINGTECH_STATUS_RUNNING = 8,
    KINGTECH_STATUS_STOP = 9,
    KINGTECH_STATUS_COOLING = 10,
    KINGTECH_STATUS_GLOW_BAD = 11,
    KINGTECH_STATUS_START_BAD = 12,
    KINGTECH_STATUS_LOW_RPM = 13,
    KINGTECH_STATUS_TEMPERATURE_HIGH = 14,
    KINGTECH_STATUS_FLAMEOUT = 15,
    KINGTECH_STATUS_SET_IDLE = 16,
    KINGTECH_STATUS_BURNER_ON = 17,
    KINGTECH_STATUS_START_ON_SECONDARY = 18,
    KINGTECH_STATUS_IGNITION_FAILURE = 19,
    KINGTECH_STATUS_IGNITOR_BAD = 20,
    KINGTECH_STATUS_BATTERY_LOW = 21,
    KINGTECH_STATUS_UNKNOWN = 22,
    KINGTECH_STATUS_UNKNOWN_SECONDARY = 23,
    KINGTECH_STATUS_STAGE_1 = 24,
    KINGTECH_STATUS_STAGE_2 = 25,
    KINGTECH_STATUS_STAGE_3 = 26,
    KINGTECH_STATUS_PRIME_VAPORIZER = 27,
    KINGTECH_STATUS_WEAK_GAS = 28,
} kingtechStatus_e;

typedef struct {
    uint8_t payloadLength;
    uint32_t rpm;
    int16_t egtCelsius;
    uint16_t turbineVoltageMv;
    uint16_t ecuVoltageMv;
    uint16_t pumpPower;
    int16_t throttlePercent;
    uint16_t status;
    bool rpmPresent;
    bool egtPresent;
    bool turbineVoltagePresent;
    bool ecuVoltagePresent;
    bool pumpPresent;
    bool throttlePresent;
    bool statusPresent;
} kingtechTelemetryData_t;

typedef struct {
    uint8_t buffer[KINGTECH_MAX_FRAME_LENGTH];
    uint8_t count;
    uint8_t expectedLength;
} kingtechTelemetryParser_t;

void kingtechTelemetryParserReset(kingtechTelemetryParser_t *parser);
kingtechDecodeResult_e kingtechTelemetryDecodeByte(kingtechTelemetryParser_t *parser,
    uint8_t byte, kingtechTelemetryData_t *telemetry);

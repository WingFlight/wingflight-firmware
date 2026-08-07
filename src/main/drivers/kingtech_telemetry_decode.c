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

#include "platform.h"

#include <limits.h>
#include <string.h>

#include "drivers/kingtech_telemetry_decode.h"

#define KINGTECH_HEADER_1       0xFC
#define KINGTECH_HEADER_2       0xC5
#define KINGTECH_SUPPORTED_TYPE 0x00
#define KINGTECH_TRAILER        0x5A
#define KINGTECH_PAYLOAD_OFFSET 4

typedef struct {
    const char *name;
    uint16_t status;
} kingtechStatusMap_t;

static const kingtechStatusMap_t kingtechStatusMap[] = {
    { "ignitionfailure", KINGTECH_STATUS_IGNITION_FAILURE },
    { "ignitorbad", KINGTECH_STATUS_IGNITOR_BAD },
    { "batterylow", KINGTECH_STATUS_BATTERY_LOW },
    { "primevaporizer", KINGTECH_STATUS_PRIME_VAPORIZER },
    { "primevap", KINGTECH_STATUS_PRIME_VAPORIZER },
    { "weakgas", KINGTECH_STATUS_WEAK_GAS },
    { "stage1", KINGTECH_STATUS_STAGE_1 },
    { "stage2", KINGTECH_STATUS_STAGE_2 },
    { "stage3", KINGTECH_STATUS_STAGE_3 },
    { "trimlow", KINGTECH_STATUS_TRIM_LOW },
    { "ready", KINGTECH_STATUS_READY },
    { "sticklow", KINGTECH_STATUS_STICK_LOW },
    { "sticklo", KINGTECH_STATUS_STICK_LOW },
    { "glowtest", KINGTECH_STATUS_GLOW_TEST },
    { "starton", KINGTECH_STATUS_START_ON },
    { "ignition", KINGTECH_STATUS_IGNITION },
    { "preheat", KINGTECH_STATUS_PREHEAT },
    { "fuelramp", KINGTECH_STATUS_FUEL_RAMP },
    { "running", KINGTECH_STATUS_RUNNING },
    { "cooling", KINGTECH_STATUS_COOLING },
    { "glowbad", KINGTECH_STATUS_GLOW_BAD },
    { "startbad", KINGTECH_STATUS_START_BAD },
    { "lowrpm", KINGTECH_STATUS_LOW_RPM },
    { "hightemp", KINGTECH_STATUS_TEMPERATURE_HIGH },
    { "temphigh", KINGTECH_STATUS_TEMPERATURE_HIGH },
    { "flameout", KINGTECH_STATUS_FLAMEOUT },
    { "setidle", KINGTECH_STATUS_SET_IDLE },
    { "burneron", KINGTECH_STATUS_BURNER_ON },
    { "stop", KINGTECH_STATUS_STOP },
};

static uint16_t readUint16LE(const uint8_t *data)
{
    return data[0] | ((uint16_t)data[1] << 8);
}

static bool isDigit(uint8_t value)
{
    return value >= '0' && value <= '9';
}

static char lowerAscii(uint8_t value)
{
    if (value >= 'A' && value <= 'Z') {
        return value + ('a' - 'A');
    }
    return value;
}

static void copyTrimmed(const uint8_t *source, size_t length, char *output, size_t capacity)
{
    size_t begin = 0;
    while (begin < length && source[begin] == ' ') {
        begin++;
    }
    while (length > begin && (source[length - 1] == ' ' || source[length - 1] == '\0')) {
        length--;
    }

    const size_t available = length - begin;
    const size_t count = available < capacity - 1 ? available : capacity - 1;
    for (size_t i = 0; i < count; i++) {
        const uint8_t value = source[begin + i];
        output[i] = value >= 0x20 && value <= 0x7E ? value : '?';
    }
    output[count] = '\0';
}

static void normalizeText(const char *input, char *output, size_t capacity)
{
    size_t count = 0;
    while (*input != '\0' && count + 1 < capacity) {
        const uint8_t value = *input++;
        if ((value >= 'a' && value <= 'z') || (value >= 'A' && value <= 'Z') || isDigit(value)) {
            output[count++] = lowerAscii(value);
        }
    }
    output[count] = '\0';
}

static bool parseSigned(const uint8_t *source, size_t length, int32_t *result)
{
    size_t index = 0;
    while (index < length && source[index] == ' ') {
        index++;
    }

    bool negative = false;
    if (index < length && (source[index] == '-' || source[index] == '+')) {
        negative = source[index++] == '-';
    }
    if (index >= length || !isDigit(source[index])) {
        return false;
    }

    int32_t value = 0;
    while (index < length && isDigit(source[index])) {
        value = value * 10 + source[index++] - '0';
    }
    *result = negative ? -value : value;
    return true;
}

static bool parseThrottle(const uint8_t *source, size_t length, int16_t *result)
{
    char text[9];
    char normalized[9];
    copyTrimmed(source, length, text, sizeof(text));
    normalizeText(text, normalized, sizeof(normalized));

    if (strcmp(normalized, "thoff") == 0 || strcmp(normalized, "thidle") == 0) {
        *result = 0;
        return true;
    }
    if (strcmp(normalized, "th") == 0 || strstr(text, "----") != NULL) {
        return false;
    }

    const char *colon = strchr(text, ':');
    int32_t value;
    if (!colon || !parseSigned((const uint8_t *)colon + 1, strlen(colon + 1), &value) || value < -100 || value > 100) {
        return false;
    }
    *result = value;
    return true;
}

static bool parseVoltageMv(const uint8_t *source, size_t length, uint16_t *result)
{
    size_t index = 0;
    while (index < length && source[index] == ' ') {
        index++;
    }
    if (index >= length || !isDigit(source[index])) {
        return false;
    }

    uint32_t whole = 0;
    while (index < length && isDigit(source[index])) {
        whole = whole * 10 + source[index++] - '0';
    }

    uint32_t fraction = 0;
    uint32_t scale = 1;
    if (index < length && source[index] == '.') {
        index++;
        while (index < length && isDigit(source[index]) && scale < 1000) {
            fraction = fraction * 10 + source[index++] - '0';
            scale *= 10;
        }
    }
    while (index < length && source[index] == ' ') {
        index++;
    }
    if (index >= length || (source[index] != 'V' && source[index] != 'v')) {
        return false;
    }

    const uint32_t millivolts = whole * 1000 + fraction * 1000 / scale;
    if (millivolts > UINT16_MAX) {
        return false;
    }
    *result = millivolts;
    return true;
}

static bool parseStatus(const uint8_t *source, size_t length, uint16_t *result)
{
    char text[17];
    char normalized[24];
    copyTrimmed(source, length, text, sizeof(text));
    if (text[0] == '\0') {
        return false;
    }
    normalizeText(text, normalized, sizeof(normalized));

    for (size_t i = 0; i < sizeof(kingtechStatusMap) / sizeof(kingtechStatusMap[0]); i++) {
        if (strstr(normalized, kingtechStatusMap[i].name) != NULL) {
            *result = kingtechStatusMap[i].status;
            return true;
        }
    }
    *result = KINGTECH_STATUS_UNKNOWN;
    return true;
}

static bool payloadLengthSupported(uint8_t length)
{
    return length == KINGTECH_COMPACT_PAYLOAD_LENGTH ||
        length == KINGTECH_CONTROL_PAYLOAD_LENGTH ||
        length == KINGTECH_EXTENDED_PAYLOAD_LENGTH;
}

static void restartAtByte(kingtechTelemetryParser_t *parser, uint8_t byte)
{
    kingtechTelemetryParserReset(parser);
    if (byte == KINGTECH_HEADER_1) {
        parser->buffer[0] = byte;
        parser->count = 1;
    }
}

void kingtechTelemetryParserReset(kingtechTelemetryParser_t *parser)
{
    parser->count = 0;
    parser->expectedLength = 0;
}

static kingtechDecodeResult_e decodeFrame(kingtechTelemetryParser_t *parser, kingtechTelemetryData_t *telemetry)
{
    const uint8_t payloadLength = parser->buffer[2];
    kingtechDecodeResult_e result = KINGTECH_DECODE_FRAME;

    if (parser->buffer[3] != KINGTECH_SUPPORTED_TYPE) {
        result = KINGTECH_DECODE_UNSUPPORTED_TYPE;
    } else if (parser->buffer[payloadLength + 6] != KINGTECH_TRAILER) {
        result = KINGTECH_DECODE_INVALID_TRAILER;
    } else {
        uint16_t checksum = 0;
        for (size_t i = 2; i < (size_t)payloadLength + 4; i++) {
            checksum += parser->buffer[i];
        }
        const uint16_t receivedChecksum = readUint16LE(&parser->buffer[payloadLength + 4]);
        if (checksum != receivedChecksum) {
            result = KINGTECH_DECODE_INVALID_CHECKSUM;
        }
    }

    if (result == KINGTECH_DECODE_FRAME) {
        memset(telemetry, 0, sizeof(*telemetry));
        telemetry->payloadLength = payloadLength;
        const uint8_t *payload = &parser->buffer[KINGTECH_PAYLOAD_OFFSET];

        if (payloadLength == KINGTECH_COMPACT_PAYLOAD_LENGTH || payloadLength == KINGTECH_EXTENDED_PAYLOAD_LENGTH) {
            telemetry->pumpPower = readUint16LE(&payload[4]);
            telemetry->pumpPresent = true;

            const uint16_t egt = readUint16LE(&payload[8]);
            if (egt <= INT16_MAX) {
                telemetry->egtCelsius = egt;
                telemetry->egtPresent = true;
            }

            telemetry->rpm = (uint32_t)readUint16LE(&payload[10]) * 100;
            telemetry->rpmPresent = true;
        }

        if (payloadLength == KINGTECH_EXTENDED_PAYLOAD_LENGTH) {
            telemetry->statusPresent = parseStatus(&payload[26], 16, &telemetry->status);
            telemetry->throttlePresent = parseThrottle(&payload[42], 8, &telemetry->throttlePercent);
            telemetry->turbineVoltagePresent = parseVoltageMv(&payload[50], 8, &telemetry->turbineVoltageMv);
            telemetry->ecuVoltagePresent = parseVoltageMv(&payload[58], 8, &telemetry->ecuVoltageMv);
        }
    }

    const uint8_t lastByte = parser->buffer[parser->count - 1];
    restartAtByte(parser, lastByte);
    return result;
}

kingtechDecodeResult_e kingtechTelemetryDecodeByte(kingtechTelemetryParser_t *parser,
    uint8_t byte, kingtechTelemetryData_t *telemetry)
{
    if (parser->count == 0) {
        if (byte == KINGTECH_HEADER_1) {
            parser->buffer[0] = byte;
            parser->count = 1;
        }
        return KINGTECH_DECODE_NONE;
    }

    if (parser->count == 1) {
        if (byte == KINGTECH_HEADER_2) {
            parser->buffer[parser->count++] = byte;
        } else {
            restartAtByte(parser, byte);
        }
        return KINGTECH_DECODE_NONE;
    }

    if (parser->count == 2) {
        if (!payloadLengthSupported(byte)) {
            restartAtByte(parser, byte);
            return KINGTECH_DECODE_INVALID_LENGTH;
        }
        parser->buffer[parser->count++] = byte;
        parser->expectedLength = byte + 7;
        return KINGTECH_DECODE_NONE;
    }

    parser->buffer[parser->count++] = byte;
    if (parser->count < parser->expectedLength) {
        return KINGTECH_DECODE_NONE;
    }
    return decodeFrame(parser, telemetry);
}

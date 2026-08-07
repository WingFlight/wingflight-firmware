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

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

extern "C" {
#include "drivers/kingtech_telemetry_decode.h"
}

#include "gtest/gtest.h"

namespace {

std::vector<uint8_t> makeFrame(const std::vector<uint8_t>& payload, uint8_t type = 0)
{
    std::vector<uint8_t> frame = { 0xFC, 0xC5, static_cast<uint8_t>(payload.size()), type };
    frame.insert(frame.end(), payload.begin(), payload.end());

    uint16_t checksum = frame[2] + frame[3];
    for (const uint8_t value : payload) {
        checksum += value;
    }
    frame.push_back(checksum & 0xFF);
    frame.push_back(checksum >> 8);
    frame.push_back(0x5A);
    return frame;
}

void putUint16LE(std::vector<uint8_t>& payload, size_t offset, uint16_t value)
{
    payload[offset] = value & 0xFF;
    payload[offset + 1] = value >> 8;
}

void putText(std::vector<uint8_t>& payload, size_t offset, size_t width, const std::string& text)
{
    std::fill(payload.begin() + offset, payload.begin() + offset + width, ' ');
    std::copy_n(text.begin(), std::min(width, text.size()), payload.begin() + offset);
}

kingtechDecodeResult_e feedFrame(kingtechTelemetryParser_t& parser,
    const std::vector<uint8_t>& frame, kingtechTelemetryData_t& telemetry)
{
    kingtechDecodeResult_e result = KINGTECH_DECODE_NONE;
    for (const uint8_t value : frame) {
        result = kingtechTelemetryDecodeByte(&parser, value, &telemetry);
    }
    return result;
}

TEST(KingtechTelemetryDecode, DecodesCompactTurbineFrame)
{
    std::vector<uint8_t> payload(KINGTECH_COMPACT_PAYLOAD_LENGTH);
    putUint16LE(payload, 4, 435);
    putUint16LE(payload, 8, 544);
    putUint16LE(payload, 10, 1408);

    kingtechTelemetryParser_t parser = {};
    kingtechTelemetryData_t telemetry = {};
    EXPECT_EQ(KINGTECH_DECODE_FRAME, feedFrame(parser, makeFrame(payload), telemetry));
    EXPECT_EQ(KINGTECH_COMPACT_PAYLOAD_LENGTH, telemetry.payloadLength);
    EXPECT_TRUE(telemetry.pumpPresent);
    EXPECT_EQ(435, telemetry.pumpPower);
    EXPECT_TRUE(telemetry.egtPresent);
    EXPECT_EQ(544, telemetry.egtCelsius);
    EXPECT_TRUE(telemetry.rpmPresent);
    EXPECT_EQ(140800u, telemetry.rpm);
    EXPECT_FALSE(telemetry.throttlePresent);
    EXPECT_FALSE(telemetry.turbineVoltagePresent);
    EXPECT_FALSE(telemetry.ecuVoltagePresent);
    EXPECT_FALSE(telemetry.statusPresent);
}

TEST(KingtechTelemetryDecode, DecodesExtendedTextFields)
{
    std::vector<uint8_t> payload(KINGTECH_EXTENDED_PAYLOAD_LENGTH);
    putUint16LE(payload, 4, 317);
    putUint16LE(payload, 8, 433);
    putUint16LE(payload, 10, 1265);
    putText(payload, 26, 16, "Stage3");
    putText(payload, 42, 8, "Th: 99");
    putText(payload, 50, 8, "10.1Vb");
    putText(payload, 58, 8, "6.5Vr");

    kingtechTelemetryParser_t parser = {};
    kingtechTelemetryData_t telemetry = {};
    EXPECT_EQ(KINGTECH_DECODE_FRAME, feedFrame(parser, makeFrame(payload), telemetry));
    EXPECT_EQ(126500u, telemetry.rpm);
    EXPECT_EQ(433, telemetry.egtCelsius);
    EXPECT_EQ(317, telemetry.pumpPower);
    EXPECT_TRUE(telemetry.throttlePresent);
    EXPECT_EQ(99, telemetry.throttlePercent);
    EXPECT_TRUE(telemetry.turbineVoltagePresent);
    EXPECT_EQ(10100, telemetry.turbineVoltageMv);
    EXPECT_TRUE(telemetry.ecuVoltagePresent);
    EXPECT_EQ(6500, telemetry.ecuVoltageMv);
    EXPECT_TRUE(telemetry.statusPresent);
    EXPECT_EQ(KINGTECH_STATUS_STAGE_3, telemetry.status);
}

TEST(KingtechTelemetryDecode, PublishesIdleThrottleAsZero)
{
    std::vector<uint8_t> payload(KINGTECH_EXTENDED_PAYLOAD_LENGTH);
    putUint16LE(payload, 4, 0);
    putUint16LE(payload, 8, 30);
    putUint16LE(payload, 10, 0);
    putText(payload, 42, 8, "Th:Idle");

    kingtechTelemetryParser_t parser = {};
    kingtechTelemetryData_t telemetry = {};
    EXPECT_EQ(KINGTECH_DECODE_FRAME, feedFrame(parser, makeFrame(payload), telemetry));
    EXPECT_TRUE(telemetry.throttlePresent);
    EXPECT_EQ(0, telemetry.throttlePercent);
}

TEST(KingtechTelemetryDecode, RejectsBadChecksumAndResynchronizes)
{
    std::vector<uint8_t> payload(KINGTECH_COMPACT_PAYLOAD_LENGTH);
    putUint16LE(payload, 8, 250);
    putUint16LE(payload, 10, 500);
    std::vector<uint8_t> corruptFrame = makeFrame(payload);
    corruptFrame[corruptFrame.size() - 3] ^= 0x01;

    kingtechTelemetryParser_t parser = {};
    kingtechTelemetryData_t telemetry = {};
    EXPECT_EQ(KINGTECH_DECODE_INVALID_CHECKSUM, feedFrame(parser, corruptFrame, telemetry));
    EXPECT_EQ(KINGTECH_DECODE_FRAME, feedFrame(parser, makeFrame(payload), telemetry));
    EXPECT_EQ(50000u, telemetry.rpm);
}

TEST(KingtechTelemetryDecode, RejectsUnsupportedLengthAndType)
{
    kingtechTelemetryParser_t parser = {};
    kingtechTelemetryData_t telemetry = {};
    EXPECT_EQ(KINGTECH_DECODE_NONE, kingtechTelemetryDecodeByte(&parser, 0xFC, &telemetry));
    EXPECT_EQ(KINGTECH_DECODE_NONE, kingtechTelemetryDecodeByte(&parser, 0xC5, &telemetry));
    EXPECT_EQ(KINGTECH_DECODE_INVALID_LENGTH, kingtechTelemetryDecodeByte(&parser, 0x04, &telemetry));

    std::vector<uint8_t> payload(KINGTECH_COMPACT_PAYLOAD_LENGTH);
    EXPECT_EQ(KINGTECH_DECODE_UNSUPPORTED_TYPE, feedFrame(parser, makeFrame(payload, 1), telemetry));
}

} // namespace

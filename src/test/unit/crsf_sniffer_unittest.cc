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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software. If not, see <https://www.gnu.org/licenses/>.
 */

#include <vector>

#include "gtest/gtest.h"

extern "C" {
#include "platform.h"

#include "drivers/crsf_sniffer.h"

uint8_t atomic_BASEPRI;
}

static const uint8_t BATTERY = 0x08;
static const uint8_t GPS = 0x02;

// A frame of `type` with `payload` bytes of payload: address, length, type, payload, CRC
static std::vector<uint8_t> frame(uint8_t type, uint8_t payload, uint8_t fill = 0x55)
{
    std::vector<uint8_t> f = { 0xC8, (uint8_t)(payload + 2), type };
    f.insert(f.end(), payload, fill);
    f.push_back(0xAA);
    return f;
}

static void record(timeMs_t nowMs, uint8_t flags, const std::vector<uint8_t> &f)
{
    crsfSnifferRecord(nowMs, flags, f.data(), (uint8_t)f.size());
}

class CrsfSnifferTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        crsfSnifferRequest(0, 0, 0);
    }
};

TEST_F(CrsfSnifferTest, RecordsNothingWhenOff)
{
    record(10, CRSF_SNIFFER_CRC_OK | CRSF_SNIFFER_DECODED, frame(BATTERY, 8));
    EXPECT_EQ(0, crsfSnifferGetFlags(10));
    EXPECT_EQ(0, crsfSnifferGetCount());
}

TEST_F(CrsfSnifferTest, ReadsWholeFramesOldestFirst)
{
    crsfSnifferRequest(CRSF_SNIFFER_FLAG_ON, 0, 100);
    const auto battery = frame(BATTERY, 8, 0x11);
    const auto gps = frame(GPS, 15, 0x22);
    record(101, CRSF_SNIFFER_CRC_OK | CRSF_SNIFFER_DECODED, battery);
    record(105, CRSF_SNIFFER_CRC_OK | CRSF_SNIFFER_SKIPPED, gps);

    uint8_t out[256];
    uint8_t frames = 0;
    const uint16_t written = crsfSnifferRead(out, sizeof(out), &frames);
    ASSERT_EQ(2, frames);
    ASSERT_EQ(2 * CRSF_SNIFFER_FRAME_HEADER + battery.size() + gps.size(), written);

    // time, flags, length, then the frame
    EXPECT_EQ(101, out[0] | (out[1] << 8) | (out[2] << 16) | (out[3] << 24));
    EXPECT_EQ(CRSF_SNIFFER_CRC_OK | CRSF_SNIFFER_DECODED, out[4]);
    EXPECT_EQ(battery.size(), out[5]);
    EXPECT_EQ(0, memcmp(&out[6], battery.data(), battery.size()));

    const uint8_t *second = &out[CRSF_SNIFFER_FRAME_HEADER + battery.size()];
    EXPECT_EQ(105, second[0]);
    EXPECT_EQ(CRSF_SNIFFER_CRC_OK | CRSF_SNIFFER_SKIPPED, second[4]);
    EXPECT_EQ(0, memcmp(&second[6], gps.data(), gps.size()));
    EXPECT_EQ(0, crsfSnifferGetCount());
}

TEST_F(CrsfSnifferTest, ReadsOnlyWholeFramesThatFit)
{
    crsfSnifferRequest(CRSF_SNIFFER_FLAG_ON, 0, 100);
    const auto battery = frame(BATTERY, 8);
    record(101, CRSF_SNIFFER_CRC_OK, battery);
    record(102, CRSF_SNIFFER_CRC_OK, battery);

    uint8_t out[64];
    uint8_t frames = 0;
    // room for one and a half frames: one is read, the other stays
    const uint16_t one = CRSF_SNIFFER_FRAME_HEADER + battery.size();
    EXPECT_EQ(one, crsfSnifferRead(out, one + one / 2, &frames));
    EXPECT_EQ(1, frames);
    EXPECT_EQ(1, crsfSnifferGetCount());
    EXPECT_EQ(one, crsfSnifferRead(out, sizeof(out), &frames));
    EXPECT_EQ(102, out[0]);
}

TEST_F(CrsfSnifferTest, FiltersBadCrcAndFrameType)
{
    crsfSnifferRequest(CRSF_SNIFFER_FLAG_ON, 0, 100);
    record(101, 0, frame(BATTERY, 8));      // bad CRC: left out by default
    EXPECT_EQ(0, crsfSnifferGetCount());

    crsfSnifferRequest(CRSF_SNIFFER_FLAG_ON | CRSF_SNIFFER_FLAG_BAD_CRC, BATTERY, 200);
    record(201, 0, frame(BATTERY, 8));
    record(202, CRSF_SNIFFER_CRC_OK, frame(GPS, 15));  // another type
    record(203, CRSF_SNIFFER_CRC_OK, frame(BATTERY, 8));
    EXPECT_EQ(2, crsfSnifferGetCount());
}

TEST_F(CrsfSnifferTest, CountsFramesDroppedWhenFullAndWrapsAround)
{
    crsfSnifferRequest(CRSF_SNIFFER_FLAG_ON, 0, 100);
    const auto big = frame(GPS, 58);      // the largest CRSF frame: 64 bytes
    const unsigned size = CRSF_SNIFFER_FRAME_HEADER + big.size();
    const unsigned fit = CRSF_SNIFFER_BUFFER_BYTES / size;
    for (unsigned i = 0; i < fit + 2; i++) {
        record(100 + i, CRSF_SNIFFER_CRC_OK, big);
    }
    EXPECT_EQ(fit, crsfSnifferGetCount());
    EXPECT_EQ(2, crsfSnifferTakeDropped());
    EXPECT_EQ(0, crsfSnifferTakeDropped());

    // read one, add one: the new frame wraps around the end of the ring
    uint8_t out[CRSF_SNIFFER_BUFFER_BYTES];
    uint8_t frames = 0;
    crsfSnifferRead(out, size, &frames);
    ASSERT_EQ(1, frames);
    const auto last = frame(GPS, 58, 0x77);
    record(500, CRSF_SNIFFER_CRC_OK, last);
    EXPECT_EQ(fit, crsfSnifferGetCount());

    const uint16_t written = crsfSnifferRead(out, sizeof(out), &frames);
    ASSERT_EQ(fit, frames);
    const uint8_t *newest = &out[written - size];
    EXPECT_EQ(500, newest[0] | (newest[1] << 8));
    EXPECT_EQ(0, memcmp(&newest[6], last.data(), last.size()));
}

TEST_F(CrsfSnifferTest, StopsWhenNoRequestKeepsItOn)
{
    crsfSnifferRequest(CRSF_SNIFFER_FLAG_ON, 0, 1000);
    record(1000 + CRSF_SNIFFER_TIMEOUT_MS, CRSF_SNIFFER_CRC_OK, frame(BATTERY, 8));
    EXPECT_EQ(1, crsfSnifferGetCount());
    record(1001 + CRSF_SNIFFER_TIMEOUT_MS, CRSF_SNIFFER_CRC_OK, frame(BATTERY, 8));
    EXPECT_EQ(1, crsfSnifferGetCount());
    EXPECT_EQ(0, crsfSnifferGetFlags(1001 + CRSF_SNIFFER_TIMEOUT_MS));

    // a new capture starts empty
    crsfSnifferRequest(CRSF_SNIFFER_FLAG_ON, 0, 20000);
    EXPECT_EQ(0, crsfSnifferGetCount());
}

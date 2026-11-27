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

#include "gtest/gtest.h"

extern "C" {
#include "platform.h"

#include "drivers/fbus_sniffer.h"

uint8_t atomic_BASEPRI;
}

static const uint8_t PRIM_NULL = 0x00;
static const uint8_t PRIM_DATA = 0x10;
static const uint8_t PRIM_READ = 0x30;
static const uint8_t PRIM_RESPONSE = 0x32;

static const uint8_t SERVO = 0x05;
static const uint16_t SERVO_APP_ID = 0x6806;

class FbusSnifferTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        fbusSnifferRequest(0, 0);
    }
};

TEST_F(FbusSnifferTest, RecordsNothingWhenOff)
{
    fbusSnifferRecord(10, FBUS_SNIFFER_FROM_RADIO, 0x1B, PRIM_READ, SERVO_APP_ID, 0x07);

    EXPECT_EQ(0, fbusSnifferGetFlags(10));
    EXPECT_EQ(0, fbusSnifferGetCount());
}

TEST_F(FbusSnifferTest, ReadsFramesOldestFirstAndRemovesThem)
{
    fbusSnifferRequest(FBUS_SNIFFER_FLAG_ON, 100);
    fbusSnifferRecord(101, FBUS_SNIFFER_FROM_RADIO, 0x1B, PRIM_READ, SERVO_APP_ID, 0x07);
    fbusSnifferRecord(102, FBUS_SNIFFER_TO_DEVICE, SERVO, PRIM_READ, SERVO_APP_ID, 0x07);
    fbusSnifferRecord(110, FBUS_SNIFFER_FROM_DEVICE, SERVO, PRIM_RESPONSE, SERVO_APP_ID, 0x0207);

    fbusSnifferFrame_t frames[2];
    ASSERT_EQ(2, fbusSnifferRead(frames, 2));
    EXPECT_EQ(101u, frames[0].timeMs);
    EXPECT_EQ(FBUS_SNIFFER_FROM_RADIO, frames[0].direction);
    EXPECT_EQ(0x1B, frames[0].phyId);
    EXPECT_EQ(PRIM_READ, frames[0].prim);
    EXPECT_EQ(SERVO_APP_ID, frames[0].appId);
    EXPECT_EQ(0x07u, frames[0].data);
    EXPECT_EQ(FBUS_SNIFFER_TO_DEVICE, frames[1].direction);
    EXPECT_EQ(SERVO, frames[1].phyId);

    EXPECT_EQ(1, fbusSnifferGetCount());
    ASSERT_EQ(1, fbusSnifferRead(frames, 2));
    EXPECT_EQ(FBUS_SNIFFER_FROM_DEVICE, frames[0].direction);
    EXPECT_EQ(0x0207u, frames[0].data);
    EXPECT_EQ(0, fbusSnifferRead(frames, 2));
}

TEST_F(FbusSnifferTest, LeavesOutTelemetryUnlessAskedForAll)
{
    fbusSnifferRequest(FBUS_SNIFFER_FLAG_ON, 100);
    fbusSnifferRecord(101, FBUS_SNIFFER_TO_DEVICE, SERVO, PRIM_DATA, 0, 0);                 // telemetry poll
    fbusSnifferRecord(102, FBUS_SNIFFER_FROM_DEVICE, SERVO, PRIM_DATA, SERVO_APP_ID, 1234); // telemetry
    fbusSnifferRecord(103, FBUS_SNIFFER_TO_RADIO, SERVO, PRIM_DATA, SERVO_APP_ID, 1234);    // forwarded
    fbusSnifferRecord(104, FBUS_SNIFFER_TO_DEVICE, 0, PRIM_NULL, 0, 0);
    EXPECT_EQ(0, fbusSnifferGetCount());

    // the poll collecting the answer to a read carries the request's App ID
    fbusSnifferRecord(105, FBUS_SNIFFER_TO_DEVICE, SERVO, PRIM_DATA, SERVO_APP_ID, 0);
    EXPECT_EQ(1, fbusSnifferGetCount());

    fbusSnifferRequest(FBUS_SNIFFER_FLAG_ON | FBUS_SNIFFER_FLAG_ALL, 200);
    fbusSnifferRecord(201, FBUS_SNIFFER_TO_DEVICE, SERVO, PRIM_DATA, 0, 0);
    fbusSnifferRecord(202, FBUS_SNIFFER_FROM_DEVICE, SERVO, PRIM_DATA, SERVO_APP_ID, 1234);
    fbusSnifferRecord(203, FBUS_SNIFFER_TO_DEVICE, 0, PRIM_NULL, 0, 0);
    EXPECT_EQ(3, fbusSnifferGetCount());
}

TEST_F(FbusSnifferTest, CountsFramesDroppedWhenFull)
{
    fbusSnifferRequest(FBUS_SNIFFER_FLAG_ON, 100);
    for (int i = 0; i < FBUS_SNIFFER_BUFFER_FRAMES + 3; i++) {
        fbusSnifferRecord(100 + i, FBUS_SNIFFER_TO_DEVICE, SERVO, PRIM_READ, SERVO_APP_ID, i);
    }

    EXPECT_EQ(FBUS_SNIFFER_BUFFER_FRAMES, fbusSnifferGetCount());
    EXPECT_EQ(3, fbusSnifferTakeDropped());
    EXPECT_EQ(0, fbusSnifferTakeDropped());

    // the frames kept are the first ones
    fbusSnifferFrame_t frame;
    ASSERT_EQ(1, fbusSnifferRead(&frame, 1));
    EXPECT_EQ(0u, frame.data);

    // and there is room again, across the end of the buffer
    fbusSnifferRecord(500, FBUS_SNIFFER_TO_DEVICE, SERVO, PRIM_READ, SERVO_APP_ID, 99);
    EXPECT_EQ(FBUS_SNIFFER_BUFFER_FRAMES, fbusSnifferGetCount());
    fbusSnifferFrame_t frames[FBUS_SNIFFER_BUFFER_FRAMES];
    ASSERT_EQ(FBUS_SNIFFER_BUFFER_FRAMES, fbusSnifferRead(frames, FBUS_SNIFFER_BUFFER_FRAMES));
    EXPECT_EQ(1u, frames[0].data);
    EXPECT_EQ(99u, frames[FBUS_SNIFFER_BUFFER_FRAMES - 1].data);
}

TEST_F(FbusSnifferTest, StopsWhenNoRequestKeepsItOn)
{
    fbusSnifferRequest(FBUS_SNIFFER_FLAG_ON, 1000);
    fbusSnifferRecord(1000 + FBUS_SNIFFER_TIMEOUT_MS, FBUS_SNIFFER_TO_DEVICE, SERVO, PRIM_READ, SERVO_APP_ID, 1);
    EXPECT_EQ(1, fbusSnifferGetCount());

    // a request keeps it on
    fbusSnifferRequest(FBUS_SNIFFER_FLAG_ON, 5000);
    fbusSnifferRecord(9000, FBUS_SNIFFER_TO_DEVICE, SERVO, PRIM_READ, SERVO_APP_ID, 2);
    EXPECT_EQ(2, fbusSnifferGetCount());

    fbusSnifferRecord(5001 + FBUS_SNIFFER_TIMEOUT_MS, FBUS_SNIFFER_TO_DEVICE, SERVO, PRIM_READ, SERVO_APP_ID, 3);
    EXPECT_EQ(2, fbusSnifferGetCount());
    EXPECT_EQ(0, fbusSnifferGetFlags(5001 + FBUS_SNIFFER_TIMEOUT_MS));

    // what was captured can still be read; a new capture starts empty
    fbusSnifferFrame_t frames[4];
    EXPECT_EQ(2, fbusSnifferRead(frames, 4));
    fbusSnifferRecord(20000, FBUS_SNIFFER_TO_DEVICE, SERVO, PRIM_READ, SERVO_APP_ID, 4);
    fbusSnifferRequest(FBUS_SNIFFER_FLAG_ON, 30000);
    EXPECT_EQ(FBUS_SNIFFER_FLAG_ON, fbusSnifferGetFlags(30000));
    EXPECT_EQ(0, fbusSnifferGetCount());
}

TEST_F(FbusSnifferTest, RequestWithoutOnStopsAndEmpties)
{
    fbusSnifferRequest(FBUS_SNIFFER_FLAG_ON, 100);
    fbusSnifferRecord(101, FBUS_SNIFFER_FROM_RADIO | FBUS_SNIFFER_NOT_RELAYED, 0x1B, PRIM_READ, SERVO_APP_ID, 7);
    EXPECT_EQ(1, fbusSnifferGetCount());

    fbusSnifferRequest(0, 102);
    EXPECT_EQ(0, fbusSnifferGetFlags(102));
    EXPECT_EQ(0, fbusSnifferGetCount());
    fbusSnifferRecord(103, FBUS_SNIFFER_FROM_RADIO, 0x1B, PRIM_READ, SERVO_APP_ID, 7);
    EXPECT_EQ(0, fbusSnifferGetCount());
}

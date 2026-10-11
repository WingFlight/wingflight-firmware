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

#include <stdint.h>
#include <string.h>

extern "C" {
#include "platform.h"

#include "drivers/fbus_ota.h"
#include "rx/frsky_crc.h"
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

static uint8_t frame[FBUS_OTA_MAX_FRAME_SIZE];
static uint8_t image[300];
static timeUs_t now;

static size_t prepare(timeDelta_t advanceUs = 2000)
{
    now += advanceUs;
    return fbusOtaPrepareFrame(frame, now);
}

// The device answers by echoing the downlink frame at the end of the frame
static void echo(size_t frameSize)
{
    fbusOtaProcessReply(&frame[frameSize - 10]);
}

static fbusOtaStatus_t status(void)
{
    fbusOtaStatus_t s;
    fbusOtaGetStatus(&s);
    return s;
}

static void startUpdate(uint8_t phyId, uint32_t size)
{
    fbusOtaAbort();
    for (unsigned i = 0; i < sizeof(image); i++) {
        image[i] = (uint8_t)(i * 7 + 3);
    }
    now = 1000000;
    ASSERT_TRUE(fbusOtaStart(phyId, size));
}

TEST(FbusOtaUnittest, StartFrame)
{
    startUpdate(0x03, 100);
    EXPECT_TRUE(fbusOtaIsActive());

    const size_t n = prepare();
    ASSERT_EQ(24u + 3 + 10, n);
    EXPECT_EQ(0x18, frame[0]);
    EXPECT_EQ(0xF0, frame[1]);
    EXPECT_EQ(0, frame[2]);  // Data1: reply after 50 us
    EXPECT_EQ(50, frame[3]); // Data2: reply window 5 ms
    EXPECT_EQ(frskyCheckSum(&frame[1], 25), frame[26]);

    const uint8_t *downlink = &frame[27];
    EXPECT_EQ(0x08, downlink[0]);
    EXPECT_EQ(0x83, downlink[1]); // physical ID 3 with S.Port check bits
    EXPECT_EQ(0xF0, downlink[2]);
    EXPECT_EQ(frskyCheckSum((uint8_t *)&downlink[1], 8), downlink[9]);
}

TEST(FbusOtaUnittest, RepeatsUnansweredFrames)
{
    startUpdate(0x16, 100);
    ASSERT_GT(prepare(), 0u);
    EXPECT_EQ(0u, prepare(5000));   // still waiting for the answer
    EXPECT_GT(prepare(8000), 0u);   // repeated after 12 ms
    EXPECT_EQ(1, status().retries);
    EXPECT_EQ(FBUS_OTA_STATE_STARTING, status().state);
}

TEST(FbusOtaUnittest, CompleteUpdate)
{
    const uint32_t size = 100; // three full chunks and a short one
    startUpdate(0x16, size);

    size_t n = prepare();
    echo(n);
    EXPECT_EQ(0u, prepare());       // no image data yet
    EXPECT_EQ(FBUS_OTA_STATE_TRANSFER, status().state);

    ASSERT_TRUE(fbusOtaWrite(0, image, 64));
    ASSERT_TRUE(fbusOtaWrite(64, image + 64, 36));
    EXPECT_EQ(size, status().received);
    EXPECT_EQ(0, status().bufferFree);

    for (uint32_t offset = 0; offset < size; offset += 32) {
        n = prepare();
        ASSERT_EQ(32u + 3 + 10, n);
        EXPECT_EQ(0x20, frame[0]);
        EXPECT_EQ(0xF1, frame[1]);
        const uint8_t *downlink = &frame[35];
        EXPECT_EQ(0xF1, downlink[2]);
        EXPECT_EQ(offset, (uint32_t)(downlink[5] | downlink[6] << 8 | downlink[7] << 16 | downlink[8] << 24));
        for (unsigned i = 0; i < 32; i++) {
            const uint8_t expected = offset + i < size ? image[offset + i] : 0xFF;
            EXPECT_EQ(expected, frame[2 + i]);
        }
        echo(n);
    }

    n = prepare();
    EXPECT_EQ(FBUS_OTA_STATE_ENDING, status().state);
    EXPECT_EQ(size, status().acked);
    ASSERT_EQ(24u + 3 + 10, n);
    EXPECT_EQ(0xF2, frame[1]);
    echo(n);
    EXPECT_EQ(0u, prepare());
    EXPECT_EQ(FBUS_OTA_STATE_DONE, status().state);
    EXPECT_EQ(FBUS_OTA_ERROR_NONE, status().error);
    EXPECT_FALSE(fbusOtaIsActive());
}

TEST(FbusOtaUnittest, IgnoresWrongAnswers)
{
    startUpdate(0x16, 32);
    size_t n = prepare();

    uint8_t reply[10];
    memcpy(reply, &frame[n - 10], 10);
    reply[2] = 0xF1; // other PRIM
    reply[9] = frskyCheckSum(&reply[1], 8);
    fbusOtaProcessReply(reply);
    prepare();
    EXPECT_EQ(FBUS_OTA_STATE_STARTING, status().state);

    // a plain physical ID without check bits is accepted
    memcpy(reply, &frame[n - 10], 10);
    reply[1] = 0x16;
    reply[9] = frskyCheckSum(&reply[1], 8);
    fbusOtaProcessReply(reply);
    prepare();
    EXPECT_EQ(FBUS_OTA_STATE_TRANSFER, status().state);
}

TEST(FbusOtaUnittest, WriteChecks)
{
    startUpdate(0x16, 2000);
    EXPECT_FALSE(fbusOtaWrite(4, image, 4));          // gap
    EXPECT_TRUE(fbusOtaWrite(0, image, 128));
    EXPECT_TRUE(fbusOtaWrite(0, image, 128));         // repeat
    EXPECT_FALSE(fbusOtaWrite(128, image, 0));
    EXPECT_FALSE(fbusOtaWrite(128, image, FBUS_OTA_MAX_WRITE + 1));
    for (uint32_t offset = 128; offset < FBUS_OTA_BUFFER_SIZE; offset += 128) {
        EXPECT_TRUE(fbusOtaWrite(offset, image, 128));
    }
    EXPECT_EQ(0, status().bufferFree);
    EXPECT_FALSE(fbusOtaWrite(FBUS_OTA_BUFFER_SIZE, image, 4)); // buffer full
    EXPECT_FALSE(fbusOtaStart(0x05, 100));                      // already running

    startUpdate(0x16, 10);
    EXPECT_FALSE(fbusOtaWrite(0, image, 11));                   // beyond the image
    EXPECT_FALSE(fbusOtaStart(0x1F, 10));
}

TEST(FbusOtaUnittest, Timeouts)
{
    // no answer to the start frame
    startUpdate(0x16, 100);
    for (int i = 0; i < 3000 && fbusOtaIsActive(); i++) {
        prepare();
    }
    EXPECT_EQ(FBUS_OTA_STATE_FAILED, status().state);
    EXPECT_EQ(FBUS_OTA_ERROR_NO_RESPONSE, status().error);

    // no image data from the Configurator
    startUpdate(0x16, 100);
    echo(prepare());
    for (int i = 0; i < 3000 && fbusOtaIsActive(); i++) {
        prepare();
    }
    EXPECT_EQ(FBUS_OTA_ERROR_STALLED, status().error);

    // device stops answering data frames
    startUpdate(0x16, 100);
    echo(prepare());
    ASSERT_TRUE(fbusOtaWrite(0, image, 100));
    for (int i = 0; i < 3000 && fbusOtaIsActive(); i++) {
        prepare();
    }
    EXPECT_EQ(FBUS_OTA_ERROR_TIMEOUT, status().error);
    EXPECT_GT(status().retries, 100);

    // device refuses the image: no answer to the end frame
    startUpdate(0x16, 32);
    echo(prepare());
    ASSERT_TRUE(fbusOtaWrite(0, image, 32));
    echo(prepare());
    for (int i = 0; i < 3000 && fbusOtaIsActive(); i++) {
        prepare();
    }
    EXPECT_EQ(FBUS_OTA_ERROR_REJECTED, status().error);
}

TEST(FbusOtaUnittest, Abort)
{
    startUpdate(0x16, 100);
    prepare();
    fbusOtaAbort();
    EXPECT_FALSE(fbusOtaIsActive());
    EXPECT_EQ(FBUS_OTA_ERROR_ABORTED, status().error);
    EXPECT_EQ(0u, prepare());
    EXPECT_FALSE(fbusOtaWrite(0, image, 4));
}

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

/*
 * Runtime diagnostics over MSP (msp/msp_runtime.c).
 *
 * The part worth testing is the task list's paging: a client loops on
 * nextTaskId until it reaches taskCount, so a page that makes no progress
 * hangs the configurator, and a page that skips a task loses it silently.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <vector>

extern "C" {
    #include "platform.h"

    #include "common/streambuf.h"
    #include "fc/runtime_config.h"
    #include "msp/msp.h"
    #include "msp/msp_protocol.h"
    #include "msp/msp_runtime.h"
    #include "pg/system.h"
    #include "scheduler/scheduler.h"

    uint8_t armingFlags;
    systemConfig_t systemConfig_System;

    // A handful of tasks, some disabled, so the list has gaps to skip.
    typedef struct {
        const char *name;
        uint32_t maxUs;
    } fakeTask_t;

    static fakeTask_t fakeTasks[TASK_COUNT];
    static int resetMaxCalls[TASK_COUNT];
    static int resetCheckFuncCalls;

    static bool receivingSignal;
    static float averageRxRefreshRate;

    void getTaskInfo(taskId_e taskId, taskInfo_t *taskInfo)
    {
        memset(taskInfo, 0, sizeof(*taskInfo));
        taskInfo->isEnabled = fakeTasks[taskId].name != NULL;
        taskInfo->taskName = fakeTasks[taskId].name;
        taskInfo->maxExecutionTimeUs = fakeTasks[taskId].maxUs;
        taskInfo->averageDeltaTime10thUs = 10000 + taskId;
        taskInfo->averageExecutionTime10thUs = 200 + taskId;
        taskInfo->totalExecutionTimeUs = 3000 + taskId;
    }

    void getCheckFuncInfo(cfCheckFuncInfo_t *checkFuncInfo)
    {
        checkFuncInfo->maxExecutionTimeUs = 11;
        checkFuncInfo->averageExecutionTimeUs = 22;
        checkFuncInfo->totalExecutionTimeUs = 33;
        checkFuncInfo->averageDeltaTimeUs = 44;
    }

    void schedulerResetTaskMaxExecutionTime(taskId_e taskId) { resetMaxCalls[taskId]++; }
    void schedulerResetCheckFunctionMaxExecutionTime(void) { resetCheckFuncCalls++; }

    bool rxIsReceivingSignal(void) { return receivingSignal; }
    float getAverageRxRefreshRate(void) { return averageRxRefreshRate; }
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

// Header: flags, taskCount, nextTaskId, three u32 check-function counters.
static const size_t HEADER_SIZE = 3 + 3 * 4;
// Per entry, before the name: id and name length; after it, seven u32.
static size_t entrySize(const char *name) { return 2 + strlen(name) + 7 * 4; }

class MspRuntimeTest : public ::testing::Test
{
protected:
    uint8_t requestBuffer[8];
    uint8_t replyBuffer[512];
    sbuf_t request;
    sbuf_t reply;

    void SetUp() override
    {
        armingFlags = 0;
        memset(&systemConfig_System, 0, sizeof(systemConfig_System));
        memset(fakeTasks, 0, sizeof(fakeTasks));
        memset(resetMaxCalls, 0, sizeof(resetMaxCalls));
        resetCheckFuncCalls = 0;
        receivingSignal = false;
        averageRxRefreshRate = 0;

        fakeTasks[0] = { "SYSTEM", 100 };
        fakeTasks[2] = { "GYRO", 200 };
        fakeTasks[5] = { "RX", 300 };
    }

    mspResult_e call(int16_t cmd, const uint8_t *payload, size_t length, size_t replyCapacity = sizeof(replyBuffer))
    {
        memcpy(requestBuffer, payload, length);
        sbufInit(&request, requestBuffer, requestBuffer + length);
        sbufInit(&reply, replyBuffer, replyBuffer + replyCapacity);

        mspResult_e result = MSP_RESULT_ERROR;
        EXPECT_TRUE(mspRuntimeCommand(cmd, &request, &reply, &result));
        return result;
    }

    mspResult_e taskPage(uint8_t first, size_t replyCapacity = sizeof(replyBuffer))
    {
        return call(MSP2_WING_TASK_INFO, &first, 1, replyCapacity);
    }

    size_t replyLength() const { return reply.ptr - replyBuffer; }

    static uint32_t get32(const uint8_t *at)
    {
        return at[0] | (at[1] << 8) | (at[2] << 16) | ((uint32_t)at[3] << 24);
    }

    /** The task ids in the current reply, in order. */
    std::vector<uint8_t> taskIds() const
    {
        std::vector<uint8_t> ids;
        const uint8_t *at = replyBuffer + HEADER_SIZE;
        while (at < reply.ptr) {
            ids.push_back(at[0]);
            at += 2 + at[1] + 7 * 4;
        }
        return ids;
    }
};

// --- dispatch ---------------------------------------------------------------

TEST_F(MspRuntimeTest, IgnoresOpcodesItDoesNotOwn)
{
    sbufInit(&request, requestBuffer, requestBuffer);
    sbufInit(&reply, replyBuffer, replyBuffer + sizeof(replyBuffer));
    mspResult_e result = MSP_RESULT_ERROR;

    EXPECT_FALSE(mspRuntimeCommand(MSP2_WING_PARAM_READ, &request, &reply, &result));
    EXPECT_FALSE(mspRuntimeCommand(MSP_API_VERSION, &request, &reply, &result));
}

// --- tasks ------------------------------------------------------------------

TEST_F(MspRuntimeTest, ListsOnlyEnabledTasksInOnePage)
{
    systemConfig_System.task_statistics = 1;

    ASSERT_EQ(MSP_RESULT_ACK, taskPage(0));

    EXPECT_EQ(MSP_TASK_INFO_FLAG_STATISTICS, replyBuffer[0] & MSP_TASK_INFO_FLAG_STATISTICS);
    EXPECT_EQ(TASK_COUNT, replyBuffer[1]);
    EXPECT_EQ(TASK_COUNT, replyBuffer[2]);  // last page
    EXPECT_EQ(11u, get32(replyBuffer + 3));
    EXPECT_EQ(22u, get32(replyBuffer + 7));
    EXPECT_EQ(33u, get32(replyBuffer + 11));

    EXPECT_EQ(std::vector<uint8_t>({ 0, 2, 5 }), taskIds());
    EXPECT_EQ(HEADER_SIZE + entrySize("SYSTEM") + entrySize("GYRO") + entrySize("RX"), replyLength());
}

TEST_F(MspRuntimeTest, EncodesATaskEntry)
{
    ASSERT_EQ(MSP_RESULT_ACK, taskPage(2));

    const uint8_t *entry = replyBuffer + HEADER_SIZE;
    EXPECT_EQ(2, entry[0]);
    ASSERT_EQ(4, entry[1]);
    EXPECT_EQ(0, memcmp(entry + 2, "GYRO", 4));

    const uint8_t *counters = entry + 6;
    EXPECT_EQ(10002u, get32(counters + 0));   // averageDeltaTime10thUs
    EXPECT_EQ(200u, get32(counters + 4));     // maxExecutionTimeUs
    EXPECT_EQ(202u, get32(counters + 8));     // averageExecutionTime10thUs
    EXPECT_EQ(3002u, get32(counters + 12));   // totalExecutionTimeUs
}

TEST_F(MspRuntimeTest, StatisticsFlagFollowsConfig)
{
    systemConfig_System.task_statistics = 0;
    ASSERT_EQ(MSP_RESULT_ACK, taskPage(0));
    EXPECT_EQ(0, replyBuffer[0] & MSP_TASK_INFO_FLAG_STATISTICS);
}

TEST_F(MspRuntimeTest, RequestWithoutPayloadStartsAtTheFirstTask)
{
    ASSERT_EQ(MSP_RESULT_ACK, call(MSP2_WING_TASK_INFO, NULL, 0));
    EXPECT_EQ(std::vector<uint8_t>({ 0, 2, 5 }), taskIds());
}

TEST_F(MspRuntimeTest, PagesWithoutLosingOrRepeatingATask)
{
    // Room for the header and exactly one entry, so every page holds one task.
    const size_t capacity = HEADER_SIZE + entrySize("SYSTEM");

    std::vector<uint8_t> seen;
    uint8_t next = 0;
    int pages = 0;
    while (next < TASK_COUNT) {
        ASSERT_EQ(MSP_RESULT_ACK, taskPage(next, capacity));
        const std::vector<uint8_t> ids = taskIds();
        seen.insert(seen.end(), ids.begin(), ids.end());

        ASSERT_GT(replyBuffer[2], next) << "a page must make progress";
        next = replyBuffer[2];
        ASSERT_LT(++pages, 10);
    }

    EXPECT_EQ(std::vector<uint8_t>({ 0, 2, 5 }), seen);
}

TEST_F(MspRuntimeTest, ResetsMaxTimesOnlyForWhatWasReported)
{
    const size_t capacity = HEADER_SIZE + entrySize("SYSTEM");

    ASSERT_EQ(MSP_RESULT_ACK, taskPage(0, capacity));
    EXPECT_EQ(1, resetMaxCalls[0]);
    EXPECT_EQ(0, resetMaxCalls[2]);
    EXPECT_EQ(0, resetMaxCalls[1]);        // disabled
    EXPECT_EQ(0, resetCheckFuncCalls);     // not the last page yet

    ASSERT_EQ(MSP_RESULT_ACK, taskPage(replyBuffer[2]));
    EXPECT_EQ(1, resetMaxCalls[2]);
    EXPECT_EQ(1, resetMaxCalls[5]);
    EXPECT_EQ(1, resetCheckFuncCalls);
}

TEST_F(MspRuntimeTest, RefusesAFirstTaskOutOfRange)
{
    EXPECT_EQ(MSP_RESULT_ERROR, taskPage(TASK_COUNT));
    EXPECT_EQ(MSP_RESULT_ERROR, taskPage(0xff));
}

TEST_F(MspRuntimeTest, RefusesAPageThatCannotHoldOneTask)
{
    // Returning an empty page with nextTaskId unchanged would loop forever.
    EXPECT_EQ(MSP_RESULT_ERROR, taskPage(0, HEADER_SIZE + 4));
    EXPECT_EQ(0, resetMaxCalls[0]);
}

TEST_F(MspRuntimeTest, TrailingDisabledTasksEndTheList)
{
    // From 6 onwards nothing is enabled: an empty last page, not an error.
    ASSERT_EQ(MSP_RESULT_ACK, taskPage(6));
    EXPECT_EQ(TASK_COUNT, replyBuffer[2]);
    EXPECT_TRUE(taskIds().empty());
}

// --- gyro registers ---------------------------------------------------------

TEST_F(MspRuntimeTest, RefusesGyroRegistersWhileArmed)
{
    ENABLE_ARMING_FLAG(ARMED);
    EXPECT_EQ(MSP_RESULT_ERROR, call(MSP2_WING_GYRO_REGISTERS, NULL, 0));
    EXPECT_EQ(0u, replyLength());
}

TEST_F(MspRuntimeTest, ReportsGyroRegisterCount)
{
    ASSERT_EQ(MSP_RESULT_ACK, call(MSP2_WING_GYRO_REGISTERS, NULL, 0));
    ASSERT_GE(replyLength(), 1u);
    // Every sensor reported is four bytes after the count.
    EXPECT_EQ(1u + replyBuffer[0] * 4u, replyLength());
}

// --- setpoint info ----------------------------------------------------------

TEST_F(MspRuntimeTest, ReportsRxFrameTiming)
{
    receivingSignal = true;
    averageRxRefreshRate = 6666.4f;

    ASSERT_EQ(MSP_RESULT_ACK, call(MSP2_WING_SETPOINT_INFO, NULL, 0));
    ASSERT_EQ(3u, replyLength());
    EXPECT_EQ(1, replyBuffer[0]);
    EXPECT_EQ(6666, replyBuffer[1] | (replyBuffer[2] << 8));
}

TEST_F(MspRuntimeTest, SaturatesAnImplausibleFrameTime)
{
    averageRxRefreshRate = 1e6f;

    ASSERT_EQ(MSP_RESULT_ACK, call(MSP2_WING_SETPOINT_INFO, NULL, 0));
    EXPECT_EQ(0, replyBuffer[0]);
    EXPECT_EQ(0xffff, replyBuffer[1] | (replyBuffer[2] << 8));
}

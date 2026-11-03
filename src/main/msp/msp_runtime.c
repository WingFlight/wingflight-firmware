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
 * Runtime diagnostics over MSP.
 *
 * Removing the CLI took three commands with it that had no MSP equivalent:
 * `tasks`, `gyroregisters` and `setpoint_info`. These opcodes bring the data
 * back. As with everything else that left the CLI, the firmware reports raw
 * values and the configurator does the formatting -- the printf-ing is what
 * cost flash, not the numbers.
 *
 * None of this is configuration, so none of it goes through the manifest, and
 * it is not part of the MSP catalogue being deleted. It lives in its own
 * translation unit for the same reason msp_param.c does. See §8.2 and §11.2
 * of parameter-addressing-design.md.
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include <string.h>

#include "platform.h"

#include "common/streambuf.h"
#include "common/utils.h"

#include "fc/rc.h"
#include "fc/runtime_config.h"

#include "msp/msp.h"
#include "msp/msp_protocol.h"
#include "msp/msp_runtime.h"

#include "pg/system.h"

#include "rx/rx.h"

#include "scheduler/scheduler.h"

#ifdef USE_GYRO_REGISTER_DUMP
#include "drivers/accgyro/accgyro_mpu.h"
#include "pg/gyro.h"
#include "sensors/gyro_init.h"
#endif

// Fixed part of one task entry: id, name length, and seven u32 counters.
#define TASK_ENTRY_FIXED_SIZE (1 + 1 + 7 * 4)

/*
 * MSP2_WING_TASK_INFO
 *
 * Request:  [u8 firstTaskId]  (optional, default 0)
 * Reply:    u8  flags                (MSP_TASK_INFO_FLAG_*)
 *           u8  taskCount            (TASK_COUNT; ids run 0..taskCount-1)
 *           u8  nextTaskId           (taskCount when this is the last page)
 *           u32 checkFunc maxExecutionTimeUs
 *           u32 checkFunc averageExecutionTimeUs
 *           u32 checkFunc totalExecutionTimeUs
 *           then, per enabled task, as many as fit:
 *           u8  taskId
 *           u8  nameLength, name bytes (no terminator)
 *           u32 averageDeltaTime10thUs
 *           u32 maxExecutionTimeUs
 *           u32 averageExecutionTime10thUs
 *           u32 totalExecutionTimeUs
 *           u32 runCount, u32 lateCount, u32 execTime  (0 without
 *               USE_LATE_TASK_STATISTICS)
 *
 * The list is paged because the out buffer can be as small as 320 bytes and a
 * full table does not fit. A client repeats the request from nextTaskId until
 * it equals taskCount.
 *
 * Like the CLI command it replaces, reading a task resets its max execution
 * time, and completing the last page resets the check function's. That makes
 * each read report the worst case since the previous one.
 */
static mspResult_e mspRuntimeTaskInfo(sbuf_t *src, sbuf_t *dst)
{
    const uint8_t firstTaskId = sbufBytesRemaining(src) >= 1 ? sbufReadU8(src) : 0;
    if (firstTaskId >= TASK_COUNT) {
        return MSP_RESULT_ERROR;
    }

    uint8_t flags = 0;
    if (systemConfig()->task_statistics) {
        flags |= MSP_TASK_INFO_FLAG_STATISTICS;
    }
#if defined(USE_LATE_TASK_STATISTICS)
    flags |= MSP_TASK_INFO_FLAG_LATE_STATS;
#endif

    cfCheckFuncInfo_t checkFuncInfo;
    getCheckFuncInfo(&checkFuncInfo);

    sbufWriteU8(dst, flags);
    sbufWriteU8(dst, TASK_COUNT);
    // Patched below, once it is known how many entries fit.
    uint8_t *nextTaskIdField = sbufPtr(dst);
    sbufWriteU8(dst, TASK_COUNT);
    sbufWriteU32(dst, checkFuncInfo.maxExecutionTimeUs);
    sbufWriteU32(dst, checkFuncInfo.averageExecutionTimeUs);
    sbufWriteU32(dst, checkFuncInfo.totalExecutionTimeUs);

    taskId_e taskId;
    for (taskId = firstTaskId; taskId < TASK_COUNT; taskId++) {
        taskInfo_t taskInfo;
        getTaskInfo(taskId, &taskInfo);
        if (!taskInfo.isEnabled) {
            continue;
        }

        const size_t nameLength = MIN(strlen(taskInfo.taskName), (size_t)UINT8_MAX);
        if (TASK_ENTRY_FIXED_SIZE + nameLength > (size_t)sbufBytesRemaining(dst)) {
            break;
        }

        sbufWriteU8(dst, taskId);
        sbufWriteU8(dst, nameLength);
        sbufWriteData(dst, taskInfo.taskName, nameLength);
        sbufWriteU32(dst, taskInfo.averageDeltaTime10thUs);
        sbufWriteU32(dst, taskInfo.maxExecutionTimeUs);
        sbufWriteU32(dst, taskInfo.averageExecutionTime10thUs);
        sbufWriteU32(dst, taskInfo.totalExecutionTimeUs);
#if defined(USE_LATE_TASK_STATISTICS)
        sbufWriteU32(dst, taskInfo.runCount);
        sbufWriteU32(dst, taskInfo.lateCount);
        sbufWriteU32(dst, taskInfo.execTime);
#else
        sbufWriteU32(dst, 0);
        sbufWriteU32(dst, 0);
        sbufWriteU32(dst, 0);
#endif

        schedulerResetTaskMaxExecutionTime(taskId);
    }

    if (taskId == firstTaskId && taskId < TASK_COUNT) {
        // Not even one entry fit: the client would request this page forever.
        return MSP_RESULT_ERROR;
    }

    *nextTaskIdField = taskId;
    if (taskId == TASK_COUNT) {
        schedulerResetCheckFunctionMaxExecutionTime();
    }

    return MSP_RESULT_ACK;
}

/*
 * MSP2_WING_GYRO_REGISTERS
 *
 * Reply:    u8  sensorCount
 *           per sensor: u8 sensor (1 or 2), u8 WHO_AM_I, u8 CONFIG,
 *                       u8 GYRO_CONFIG
 *
 * sensorCount is 0 on a build without USE_GYRO_REGISTER_DUMP. Refused while
 * armed: these are bus transactions against the gyro the control loop is
 * reading, which the CLI could never issue in flight either.
 */
#ifdef USE_GYRO_REGISTER_DUMP
static void mspRuntimeWriteGyroRegisters(sbuf_t *dst, uint8_t whichSensor)
{
    sbufWriteU8(dst, whichSensor + 1);
    sbufWriteU8(dst, gyroReadRegister(whichSensor, MPU_RA_WHO_AM_I));
    sbufWriteU8(dst, gyroReadRegister(whichSensor, MPU_RA_CONFIG));
    sbufWriteU8(dst, gyroReadRegister(whichSensor, MPU_RA_GYRO_CONFIG));
}
#endif

static mspResult_e mspRuntimeGyroRegisters(sbuf_t *dst)
{
    if (ARMING_FLAG(ARMED)) {
        return MSP_RESULT_ERROR;
    }

#ifdef USE_GYRO_REGISTER_DUMP
#ifdef USE_MULTI_GYRO
    const uint8_t gyroToUse = gyroConfig()->gyro_to_use;
    const bool useGyro1 = gyroToUse == GYRO_CONFIG_USE_GYRO_1 || gyroToUse == GYRO_CONFIG_USE_GYRO_BOTH;
    const bool useGyro2 = gyroToUse == GYRO_CONFIG_USE_GYRO_2 || gyroToUse == GYRO_CONFIG_USE_GYRO_BOTH;

    sbufWriteU8(dst, useGyro1 + useGyro2);
    if (useGyro1) {
        mspRuntimeWriteGyroRegisters(dst, GYRO_CONFIG_USE_GYRO_1);
    }
    if (useGyro2) {
        mspRuntimeWriteGyroRegisters(dst, GYRO_CONFIG_USE_GYRO_2);
    }
#else
    sbufWriteU8(dst, 1);
    mspRuntimeWriteGyroRegisters(dst, GYRO_CONFIG_USE_GYRO_1);
#endif
#else
    sbufWriteU8(dst, 0);
#endif

    return MSP_RESULT_ACK;
}

/*
 * MSP2_WING_SETPOINT_INFO
 *
 * Reply:    u8  receivingSignal
 *           u16 averageRxFrameUs  (saturated at 65535)
 */
static mspResult_e mspRuntimeSetpointInfo(sbuf_t *dst)
{
    const float averageRxFrameUs = getAverageRxRefreshRate();

    sbufWriteU8(dst, rxIsReceivingSignal());
    sbufWriteU16(dst, averageRxFrameUs >= UINT16_MAX ? UINT16_MAX : (uint16_t)lrintf(averageRxFrameUs));

    return MSP_RESULT_ACK;
}

bool mspRuntimeCommand(int16_t cmdMSP, sbuf_t *src, sbuf_t *dst, mspResult_e *result)
{
    switch (cmdMSP) {
    case MSP2_WING_TASK_INFO:
        *result = mspRuntimeTaskInfo(src, dst);
        return true;

    case MSP2_WING_GYRO_REGISTERS:
        *result = mspRuntimeGyroRegisters(dst);
        return true;

    case MSP2_WING_SETPOINT_INFO:
        *result = mspRuntimeSetpointInfo(dst);
        return true;

    default:
        return false;
    }
}

/*
 * This file is part of Rotorflight.
 */

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "platform.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/utils.h"

#include "config/config.h"

#include "drivers/time.h"

#include "fc/runtime_config.h"

#include "pg/boardalignment.h"

#include "sensors/acceleration.h"
#include "sensors/boardalignment.h"
#include "sensors/boardmounttrim_auto.h"

#define BOARD_MOUNT_TRIM_AUTO_TIMEOUT_US (8 * 1000000)
#define BOARD_MOUNT_TRIM_AUTO_STABLE_US (750 * 1000)
#define BOARD_MOUNT_TRIM_AUTO_STABILITY_DEG 1.0f
#define BOARD_MOUNT_TRIM_AUTO_MAX_RESIDUAL_DECIDEGREES 300

typedef struct boardMountTrimAutoRuntime_s {
    boardMountTrimAutoStatus_t status;
    timeUs_t startedAtUs;
    timeUs_t windowStartedAtUs;
    float sum[XYZ_AXIS_COUNT];
    uint32_t sampleCount;
} boardMountTrimAutoRuntime_t;

static boardMountTrimAutoRuntime_t boardMountTrimAutoRuntime;

static bool normalizeVector(const float *src, float *dst)
{
    const float mag = sqrtf((src[X] * src[X]) + (src[Y] * src[Y]) + (src[Z] * src[Z]));
    if (mag < 1e-6f) {
        return false;
    }

    dst[X] = src[X] / mag;
    dst[Y] = src[Y] / mag;
    dst[Z] = src[Z] / mag;
    return true;
}

static void resetAccumulator(const float *seedSample, timeUs_t now)
{
    boardMountTrimAutoRuntime.sum[X] = seedSample[X];
    boardMountTrimAutoRuntime.sum[Y] = seedSample[Y];
    boardMountTrimAutoRuntime.sum[Z] = seedSample[Z];
    boardMountTrimAutoRuntime.sampleCount = 1;
    boardMountTrimAutoRuntime.windowStartedAtUs = now;
    boardMountTrimAutoRuntime.status.stabilityPercent = 0;
}

void boardMountTrimAutoProcessSample(const float *correctedAccVector)
{
    if (!correctedAccVector || boardMountTrimAutoRuntime.status.state != BOARD_MOUNT_TRIM_AUTO_SAMPLING) {
        return;
    }

    if (ARMING_FLAG(ARMED)) {
        boardMountTrimAutoRuntime.status.state = BOARD_MOUNT_TRIM_AUTO_REJECTED_ARMED;
        return;
    }

    const timeUs_t now = micros();
    if ((now - boardMountTrimAutoRuntime.startedAtUs) > BOARD_MOUNT_TRIM_AUTO_TIMEOUT_US) {
        boardMountTrimAutoRuntime.status.state = BOARD_MOUNT_TRIM_AUTO_TIMEOUT;
        return;
    }

    float sampleUnit[XYZ_AXIS_COUNT];
    if (!normalizeVector(correctedAccVector, sampleUnit)) {
        return; // degenerate (near-zero) sample, ignore and keep waiting
    }

    // A sample that has drifted too far from the running mean means the
    // aircraft was disturbed mid-sample -- restart the stability window
    // seeded from this new sample, mirroring the "reset to 1, adopt new
    // candidate" pattern boardalignment_auto.c uses for its own consensus
    // counter (there: discrete-candidate agreement; here: continuous
    // angular agreement against a running mean).
    if (boardMountTrimAutoRuntime.sampleCount > 0) {
        const float mean[XYZ_AXIS_COUNT] = {
            boardMountTrimAutoRuntime.sum[X] / boardMountTrimAutoRuntime.sampleCount,
            boardMountTrimAutoRuntime.sum[Y] / boardMountTrimAutoRuntime.sampleCount,
            boardMountTrimAutoRuntime.sum[Z] / boardMountTrimAutoRuntime.sampleCount,
        };
        float meanUnit[XYZ_AXIS_COUNT];
        if (normalizeVector(mean, meanUnit)) {
            const float dot = meanUnit[X] * sampleUnit[X] + meanUnit[Y] * sampleUnit[Y] + meanUnit[Z] * sampleUnit[Z];
            if (dot < cos_approx(BOARD_MOUNT_TRIM_AUTO_STABILITY_DEG * RAD)) {
                resetAccumulator(correctedAccVector, now);
                return;
            }
        }
    } else {
        boardMountTrimAutoRuntime.windowStartedAtUs = now;
    }

    boardMountTrimAutoRuntime.sum[X] += correctedAccVector[X];
    boardMountTrimAutoRuntime.sum[Y] += correctedAccVector[Y];
    boardMountTrimAutoRuntime.sum[Z] += correctedAccVector[Z];
    boardMountTrimAutoRuntime.sampleCount++;

    const timeUs_t stableElapsedUs = now - boardMountTrimAutoRuntime.windowStartedAtUs;
    boardMountTrimAutoRuntime.status.stabilityPercent = (uint8_t)MIN(100u, (100u * stableElapsedUs) / BOARD_MOUNT_TRIM_AUTO_STABLE_US);

    if (stableElapsedUs < BOARD_MOUNT_TRIM_AUTO_STABLE_US) {
        return;
    }

    const float avg[XYZ_AXIS_COUNT] = {
        boardMountTrimAutoRuntime.sum[X] / boardMountTrimAutoRuntime.sampleCount,
        boardMountTrimAutoRuntime.sum[Y] / boardMountTrimAutoRuntime.sampleCount,
        boardMountTrimAutoRuntime.sum[Z] / boardMountTrimAutoRuntime.sampleCount,
    };
    float avgUnit[XYZ_AXIS_COUNT];
    if (!normalizeVector(avg, avgUnit)) {
        boardMountTrimAutoRuntime.status.state = BOARD_MOUNT_TRIM_AUTO_TIMEOUT;
        return;
    }

    // Mirrors flight/imu.c:311-312's attitude.values.roll/pitch formulas
    // (atan2_approx(rMat[2][1], rMat[2][2]) and (pi/2)-acos_approx(-rMat[2][0])),
    // applied to our own fully-corrected stationary sample instead of the
    // fused rMat estimate -- imu.c's own accelerometer correction term
    // (imu.c:254-257) treats a normalized stationary accel reading as
    // directly comparable to rMat[2], so the same decomposition applies.
    const float rollResidualRad = atan2_approx(avgUnit[Y], avgUnit[Z]);
    const float pitchResidualRad = (0.5f * M_PIf) - acos_approx(-avgUnit[X]);

    const int16_t rollResidualDecidegrees = lrintf(rollResidualRad * (1800.0f / M_PIf));
    const int16_t pitchResidualDecidegrees = lrintf(pitchResidualRad * (1800.0f / M_PIf));

    if (ABS(rollResidualDecidegrees) > BOARD_MOUNT_TRIM_AUTO_MAX_RESIDUAL_DECIDEGREES
        || ABS(pitchResidualDecidegrees) > BOARD_MOUNT_TRIM_AUTO_MAX_RESIDUAL_DECIDEGREES) {
        boardMountTrimAutoRuntime.status.state = BOARD_MOUNT_TRIM_AUTO_OUT_OF_RANGE;
        return;
    }

    const int16_t newRoll = constrain(boardAlignment()->mountTrim.roll + rollResidualDecidegrees, -3600, 3600);
    const int16_t newPitch = constrain(boardAlignment()->mountTrim.pitch + pitchResidualDecidegrees, -3600, 3600);

    boardAlignmentMutable()->mountTrim.roll = newRoll;
    boardAlignmentMutable()->mountTrim.pitch = newPitch;
    initBoardAlignment(boardAlignment());
    setConfigDirty();

    boardMountTrimAutoRuntime.status.rollTrimDecidegrees = newRoll;
    boardMountTrimAutoRuntime.status.pitchTrimDecidegrees = newPitch;
    boardMountTrimAutoRuntime.status.stabilityPercent = 100;
    boardMountTrimAutoRuntime.status.state = BOARD_MOUNT_TRIM_AUTO_SUCCESS;
}

bool boardMountTrimAutoStart(void)
{
    if (ARMING_FLAG(ARMED)) {
        boardMountTrimAutoRuntime.status.state = BOARD_MOUNT_TRIM_AUTO_REJECTED_ARMED;
        return false;
    }

    if (!accHasBeenCalibrated()) {
        boardMountTrimAutoRuntime.status.state = BOARD_MOUNT_TRIM_AUTO_REJECTED_UNCALIBRATED;
        return false;
    }

    boardMountTrimAutoRuntime.sampleCount = 0;
    boardMountTrimAutoRuntime.startedAtUs = micros();
    boardMountTrimAutoRuntime.status.state = BOARD_MOUNT_TRIM_AUTO_SAMPLING;
    boardMountTrimAutoRuntime.status.rollTrimDecidegrees = boardAlignment()->mountTrim.roll;
    boardMountTrimAutoRuntime.status.pitchTrimDecidegrees = boardAlignment()->mountTrim.pitch;
    boardMountTrimAutoRuntime.status.stabilityPercent = 0;
    return true;
}

boardMountTrimAutoStatus_t boardMountTrimAutoGetStatus(void)
{
    boardMountTrimAutoStatus_t status = boardMountTrimAutoRuntime.status;

    if (status.state == BOARD_MOUNT_TRIM_AUTO_IDLE) {
        status.rollTrimDecidegrees = boardAlignment()->mountTrim.roll;
        status.pitchTrimDecidegrees = boardAlignment()->mountTrim.pitch;
    }

    return status;
}

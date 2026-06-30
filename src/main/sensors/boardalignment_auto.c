/*
 * This file is part of Rotorflight.
 */

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/utils.h"

#include "config/config.h"

#include "drivers/time.h"

#include "fc/runtime_config.h"

#include "pg/boardalignment.h"

#include "sensors/boardalignment.h"
#include "sensors/boardalignment_auto.h"

#define BOARD_AUTO_ALIGN_TIMEOUT_US (15 * 1000000)
#define BOARD_AUTO_ALIGN_MIN_MATCHED_SAMPLES 12
#define BOARD_AUTO_ALIGN_MIN_TILT_ANGLE_DEG 20.0f

static const int16_t rightAngleSteps[] = { 0, 90, 180, 270 };

typedef struct boardAutoAlignRuntime_s {
    boardAutoAlignStatus_t status;
    timeUs_t startedAtUs;
    bool sampleValid;
    float latestSample[XYZ_AXIS_COUNT];
    float baseline[XYZ_AXIS_COUNT];
    int16_t candidateRoll;
    int16_t candidatePitch;
    int16_t candidateYaw;
} boardAutoAlignRuntime_t;

static boardAutoAlignRuntime_t boardAutoAlignRuntime;

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

static void rotateVectorByBoardAlignment(const float *src, int16_t rollDeg, int16_t pitchDeg, int16_t yawDeg, float *dst)
{
    fp_angles_t rotationAngles;
    fp_rotationMatrix_t rotationMatrix;

    rotationAngles.angles.roll = degreesToRadians(rollDeg);
    rotationAngles.angles.pitch = degreesToRadians(pitchDeg);
    rotationAngles.angles.yaw = degreesToRadians(yawDeg);

    buildRotationMatrix(&rotationAngles, &rotationMatrix);

    dst[X] = src[X];
    dst[Y] = src[Y];
    dst[Z] = src[Z];
    applyMatrixRotation(dst, &rotationMatrix);
}

static bool findBestRightAngleAlignment(const float *baseline, const float *current, int16_t *rollOut, int16_t *pitchOut, int16_t *yawOut)
{
    const float minTiltSin = sin_approx(BOARD_AUTO_ALIGN_MIN_TILT_ANGLE_DEG * RAD);

    float baselineUnit[XYZ_AXIS_COUNT];
    float currentUnit[XYZ_AXIS_COUNT];
    if (!normalizeVector(baseline, baselineUnit) || !normalizeVector(current, currentUnit)) {
        return false;
    }

    const float dot = baselineUnit[X] * currentUnit[X] + baselineUnit[Y] * currentUnit[Y] + baselineUnit[Z] * currentUnit[Z];
    if (dot > cos_approx(BOARD_AUTO_ALIGN_MIN_TILT_ANGLE_DEG * RAD)) {
        return false;
    }

    bool found = false;
    float bestScore = 1e9f;

    for (size_t i = 0; i < ARRAYLEN(rightAngleSteps); i++) {
        for (size_t j = 0; j < ARRAYLEN(rightAngleSteps); j++) {
            for (size_t k = 0; k < ARRAYLEN(rightAngleSteps); k++) {
                const int16_t roll = rightAngleSteps[i];
                const int16_t pitch = rightAngleSteps[j];
                const int16_t yaw = rightAngleSteps[k];

                float baselineRotated[XYZ_AXIS_COUNT];
                float currentRotated[XYZ_AXIS_COUNT];
                rotateVectorByBoardAlignment(baselineUnit, roll, pitch, yaw, baselineRotated);
                rotateVectorByBoardAlignment(currentUnit, roll, pitch, yaw, currentRotated);

                const float deltaX = currentRotated[X] - baselineRotated[X];
                const float deltaY = currentRotated[Y] - baselineRotated[Y];

                if (fabsf(deltaX) < minTiltSin) {
                    continue;
                }

                // Allow substantial roll coupling, but gate out movements dominated by roll.
                if (fabsf(deltaY) > 0.90f || fabsf(deltaY) > (fabsf(deltaX) * 1.10f)) {
                    continue;
                }

                const float score = fabsf(baselineRotated[X])
                    + fabsf(baselineRotated[Y])
                    + fabsf(deltaY)
                    + fabsf((1.0f - fabsf(baselineRotated[Z])));

                if (!found || score < bestScore) {
                    found = true;
                    bestScore = score;
                    *rollOut = roll;
                    *pitchOut = pitch;
                    *yawOut = yaw;
                }
            }
        }
    }

    return found;
}

void boardAutoAlignProcessSample(const float *sensorVector)
{
    if (!sensorVector) {
        return;
    }

    boardAutoAlignRuntime.latestSample[X] = sensorVector[X];
    boardAutoAlignRuntime.latestSample[Y] = sensorVector[Y];
    boardAutoAlignRuntime.latestSample[Z] = sensorVector[Z];
    boardAutoAlignRuntime.sampleValid = true;

    if (boardAutoAlignRuntime.status.state != BOARD_AUTO_ALIGN_WAITING_FOR_TAIL_LIFT) {
        return;
    }

    if (ARMING_FLAG(ARMED)) {
        boardAutoAlignRuntime.status.state = BOARD_AUTO_ALIGN_REJECTED_ARMED;
        boardAutoAlignRuntime.status.matchedSamples = 0;
        return;
    }

    if ((micros() - boardAutoAlignRuntime.startedAtUs) > BOARD_AUTO_ALIGN_TIMEOUT_US) {
        boardAutoAlignRuntime.status.state = BOARD_AUTO_ALIGN_TIMEOUT;
        boardAutoAlignRuntime.status.matchedSamples = 0;
        return;
    }

    int16_t roll;
    int16_t pitch;
    int16_t yaw;
    if (!findBestRightAngleAlignment(boardAutoAlignRuntime.baseline, boardAutoAlignRuntime.latestSample, &roll, &pitch, &yaw)) {
        boardAutoAlignRuntime.status.state = BOARD_AUTO_ALIGN_WAITING_FOR_TAIL_LIFT;
        boardAutoAlignRuntime.status.matchedSamples = 0;
        return;
    }

    if (boardAutoAlignRuntime.status.matchedSamples == 0
        || (roll == boardAutoAlignRuntime.candidateRoll
            && pitch == boardAutoAlignRuntime.candidatePitch
            && yaw == boardAutoAlignRuntime.candidateYaw)) {
        boardAutoAlignRuntime.status.matchedSamples++;
    } else {
        boardAutoAlignRuntime.status.matchedSamples = 1;
    }

    boardAutoAlignRuntime.candidateRoll = roll;
    boardAutoAlignRuntime.candidatePitch = pitch;
    boardAutoAlignRuntime.candidateYaw = yaw;

    if (boardAutoAlignRuntime.status.matchedSamples < BOARD_AUTO_ALIGN_MIN_MATCHED_SAMPLES) {
        boardAutoAlignRuntime.status.state = BOARD_AUTO_ALIGN_WAITING_FOR_TAIL_LIFT;
        return;
    }

    boardAlignmentMutable()->rollDegrees = roll;
    boardAlignmentMutable()->pitchDegrees = pitch;
    boardAlignmentMutable()->yawDegrees = yaw;
    initBoardAlignment(boardAlignment());
    setConfigDirty();

    boardAutoAlignRuntime.status.rollDegrees = roll;
    boardAutoAlignRuntime.status.pitchDegrees = pitch;
    boardAutoAlignRuntime.status.yawDegrees = yaw;
    boardAutoAlignRuntime.status.state = BOARD_AUTO_ALIGN_SUCCESS;
}

bool boardAutoAlignStart(void)
{
    boardAutoAlignRuntime.status.matchedSamples = 0;

    if (ARMING_FLAG(ARMED)) {
        boardAutoAlignRuntime.status.state = BOARD_AUTO_ALIGN_REJECTED_ARMED;
        return false;
    }

    if (!boardAutoAlignRuntime.sampleValid) {
        boardAutoAlignRuntime.status.state = BOARD_AUTO_ALIGN_NO_MATCH;
        return false;
    }

    memcpy(boardAutoAlignRuntime.baseline, boardAutoAlignRuntime.latestSample, sizeof(boardAutoAlignRuntime.baseline));
    boardAutoAlignRuntime.startedAtUs = micros();
    boardAutoAlignRuntime.status.state = BOARD_AUTO_ALIGN_WAITING_FOR_TAIL_LIFT;
    boardAutoAlignRuntime.status.rollDegrees = boardAlignment()->rollDegrees;
    boardAutoAlignRuntime.status.pitchDegrees = boardAlignment()->pitchDegrees;
    boardAutoAlignRuntime.status.yawDegrees = boardAlignment()->yawDegrees;
    boardAutoAlignRuntime.candidateRoll = 0;
    boardAutoAlignRuntime.candidatePitch = 0;
    boardAutoAlignRuntime.candidateYaw = 0;
    return true;
}

boardAutoAlignStatus_t boardAutoAlignGetStatus(void)
{
    boardAutoAlignStatus_t status = boardAutoAlignRuntime.status;

    if (status.state == BOARD_AUTO_ALIGN_IDLE) {
        status.rollDegrees = boardAlignment()->rollDegrees;
        status.pitchDegrees = boardAlignment()->pitchDegrees;
        status.yawDegrees = boardAlignment()->yawDegrees;
    }

    return status;
}

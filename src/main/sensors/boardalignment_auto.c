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

#include "sensors/acceleration.h"
#include "sensors/boardalignment.h"
#include "sensors/boardalignment_auto.h"

#define BOARD_AUTO_ALIGN_TIMEOUT_US (15 * 1000000)
#define BOARD_AUTO_ALIGN_MIN_MATCHED_SAMPLES 12
#define BOARD_AUTO_ALIGN_MIN_TILT_ANGLE_DEG 20.0f

// The accelerometer reads the *reaction* force, not gravity itself, so at
// rest (level) it reads +1g along the body's up axis -- imu.c's Mahony
// correction treats the raw (unnegated) acc reading as directly comparable
// to rMat[2], which is (0,0,1) at identity/level attitude. So baseline
// (captured at rest) already IS the body "up" direction, no sign flip.
//
// The pitch-axis sign below is the one assumption this file can't verify
// without hardware: it assumes the wizard's "raise the tail" instruction
// produces a *negative* rotation about the standard body pitch axis (nose
// tips down as the tail comes up). If a bench test comes back consistently
// wrong by a fixed rotation (not randomly wrong -- that would mean this
// fix didn't take), flip this single constant.
#define BOARD_AUTO_ALIGN_TAIL_LIFT_SIGN (-1.0f)

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

static void crossProduct(const float *a, const float *b, float *dst)
{
    dst[X] = a[Y] * b[Z] - a[Z] * b[Y];
    dst[Y] = a[Z] * b[X] - a[X] * b[Z];
    dst[Z] = a[X] * b[Y] - a[Y] * b[X];
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

// Derives a full right-handed orthonormal reference frame (bodyX/Y/Z, in
// raw sensor coordinates) from just the resting baseline and one in-gesture
// sample -- no second gesture needed:
//   - bodyZ (the body "up" axis) is the baseline itself: a rotation about
//     any axis through the origin never changes a vector's own direction if
//     it doesn't move, and gravity read at rest already *is* the up axis
//     (see the sign-convention comment above BOARD_AUTO_ALIGN_TAIL_LIFT_SIGN).
//   - bodyY (the pitch axis the tail-lift rotates about) is
//     cross(baseline, current): a pure single-axis rotation only ever moves
//     a vector within the plane perpendicular to that axis, so the cross
//     product of a before/during-gesture pair of samples points along the
//     rotation axis itself, regardless of how far the user tilted.
//   - bodyX completes the right-handed frame via cross(bodyY, bodyZ) --
//     algebraically guaranteed perpendicular to both, no matter how sloppy
//     the physical gesture was (that imprecision instead shows up as
//     candidate jitter across samples, which the caller's consecutive-match
//     counter already filters out).
static bool computeReferenceAxes(const float *baseline, const float *current,
    float *bodyXOut, float *bodyYOut, float *bodyZOut)
{
    float baselineUnit[XYZ_AXIS_COUNT];
    float currentUnit[XYZ_AXIS_COUNT];
    if (!normalizeVector(baseline, baselineUnit) || !normalizeVector(current, currentUnit)) {
        return false;
    }

    const float dot = baselineUnit[X] * currentUnit[X] + baselineUnit[Y] * currentUnit[Y] + baselineUnit[Z] * currentUnit[Z];
    if (dot > cos_approx(BOARD_AUTO_ALIGN_MIN_TILT_ANGLE_DEG * RAD)) {
        return false; // not enough tilt yet to trust the cross product's direction
    }

    bodyZOut[X] = baselineUnit[X];
    bodyZOut[Y] = baselineUnit[Y];
    bodyZOut[Z] = baselineUnit[Z];

    float pitchAxis[XYZ_AXIS_COUNT];
    crossProduct(baselineUnit, currentUnit, pitchAxis);
    if (!normalizeVector(pitchAxis, bodyYOut)) {
        return false; // baseline/current parallel (shouldn't happen once past the tilt gate above)
    }
    bodyYOut[X] *= BOARD_AUTO_ALIGN_TAIL_LIFT_SIGN;
    bodyYOut[Y] *= BOARD_AUTO_ALIGN_TAIL_LIFT_SIGN;
    bodyYOut[Z] *= BOARD_AUTO_ALIGN_TAIL_LIFT_SIGN;

    crossProduct(bodyYOut, bodyZOut, bodyXOut);

    return true;
}

// Finds the one of the 24 physically-possible right-angle board mountings
// that rotates the observed (bodyX,bodyY,bodyZ) frame onto the standard
// aircraft body frame -- i.e. scores every candidate by *signed* agreement
// (no fabsf) against the known target directions, so it can actually tell
// right-side-up from upside-down and "tail going up" from "tail going
// down", unlike the fabsf()-based heuristic this replaced.
static bool findBestRightAngleAlignment(const float *baseline, const float *current, int16_t *rollOut, int16_t *pitchOut, int16_t *yawOut)
{
    float bodyX[XYZ_AXIS_COUNT];
    float bodyY[XYZ_AXIS_COUNT];
    float bodyZ[XYZ_AXIS_COUNT];
    if (!computeReferenceAxes(baseline, current, bodyX, bodyY, bodyZ)) {
        return false;
    }

    bool found = false;
    float bestScore = -1e9f;

    for (size_t i = 0; i < ARRAYLEN(rightAngleSteps); i++) {
        for (size_t j = 0; j < ARRAYLEN(rightAngleSteps); j++) {
            for (size_t k = 0; k < ARRAYLEN(rightAngleSteps); k++) {
                const int16_t roll = rightAngleSteps[i];
                const int16_t pitch = rightAngleSteps[j];
                const int16_t yaw = rightAngleSteps[k];

                float xRotated[XYZ_AXIS_COUNT];
                float yRotated[XYZ_AXIS_COUNT];
                float zRotated[XYZ_AXIS_COUNT];
                rotateVectorByBoardAlignment(bodyX, roll, pitch, yaw, xRotated);
                rotateVectorByBoardAlignment(bodyY, roll, pitch, yaw, yRotated);
                rotateVectorByBoardAlignment(bodyZ, roll, pitch, yaw, zRotated);

                // Signed dot-product agreement against the standard basis
                // (1,0,0)/(0,1,0)/(0,0,1) -- higher is better, and unlike
                // fabsf() this rewards the correctly-signed candidate only,
                // so a sign-flipped (e.g. upside-down) candidate now scores
                // clearly worse instead of tying with the correct one.
                const float score = xRotated[X] + yRotated[Y] + zRotated[Z];

                if (!found || score > bestScore) {
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

    if (!accHasBeenCalibrated()) {
        boardAutoAlignRuntime.status.state = BOARD_AUTO_ALIGN_REJECTED_UNCALIBRATED;
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

/*
 * This file is part of Rotorflight.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    BOARD_AUTO_ALIGN_IDLE = 0,
    BOARD_AUTO_ALIGN_WAITING_FOR_TAIL_LIFT = 1,
    BOARD_AUTO_ALIGN_SUCCESS = 2,
    BOARD_AUTO_ALIGN_REJECTED_ARMED = 3,
    BOARD_AUTO_ALIGN_TIMEOUT = 4,
    BOARD_AUTO_ALIGN_NO_MATCH = 5,
    // Every sample this feature ever sees is raw, uncalibrated accADC (see
    // accelerometer.c: applyAccelerationTrims() runs after
    // boardAutoAlignProcessSample(), not before) -- an uncalibrated per-axis
    // bias skews both the "up" baseline and the cross-product-derived tilt
    // axis. Refuse to start until accHasBeenCalibrated() is true.
    BOARD_AUTO_ALIGN_REJECTED_UNCALIBRATED = 6,
} boardAutoAlignState_e;

typedef struct boardAutoAlignStatus_s {
    uint8_t state;
    int16_t rollDegrees;
    int16_t pitchDegrees;
    int16_t yawDegrees;
    uint8_t matchedSamples;
} boardAutoAlignStatus_t;

void boardAutoAlignProcessSample(const float *sensorVector);
bool boardAutoAlignStart(void);
boardAutoAlignStatus_t boardAutoAlignGetStatus(void);

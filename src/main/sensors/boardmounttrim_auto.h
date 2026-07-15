/*
 * This file is part of Rotorflight.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    BOARD_MOUNT_TRIM_AUTO_IDLE = 0,
    BOARD_MOUNT_TRIM_AUTO_SAMPLING = 1,
    BOARD_MOUNT_TRIM_AUTO_SUCCESS = 2,
    BOARD_MOUNT_TRIM_AUTO_REJECTED_ARMED = 3,
    // Every sample this feature sees is the fully-corrected acc.accADC
    // (board alignment + mount trim rotation applied, calibration bias
    // subtracted -- see boardMountTrimAutoProcessSample()'s call site in
    // acceleration.c, deliberately *after* applyAccelerationTrims(), unlike
    // boardalignment_auto.c's pre-rotation/pre-bias hook). An uncalibrated
    // bias would otherwise be read as a mounting tilt. Refuse to start
    // until accHasBeenCalibrated() is true.
    BOARD_MOUNT_TRIM_AUTO_REJECTED_UNCALIBRATED = 4,
    BOARD_MOUNT_TRIM_AUTO_TIMEOUT = 5,
    // Residual tilt exceeded the sanity cap -- almost certainly means the
    // discrete board alignment above is wrong, not that the mounting
    // surface is tilted. Mount trim is a *fine* correction and should never
    // be used to paper over a wrong coarse alignment.
    BOARD_MOUNT_TRIM_AUTO_OUT_OF_RANGE = 6,
} boardMountTrimAutoState_e;

typedef struct boardMountTrimAutoStatus_s {
    uint8_t state;
    int16_t rollTrimDecidegrees;
    int16_t pitchTrimDecidegrees;
    uint8_t stabilityPercent;
} boardMountTrimAutoStatus_t;

void boardMountTrimAutoProcessSample(const float *correctedAccVector);
bool boardMountTrimAutoStart(void);
boardMountTrimAutoStatus_t boardMountTrimAutoGetStatus(void);

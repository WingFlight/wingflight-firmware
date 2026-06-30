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

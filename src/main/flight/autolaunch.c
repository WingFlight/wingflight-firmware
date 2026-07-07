/*
 * This file is part of Wingflight.
 *
 * Wingflight is free software. You can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 */

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "platform.h"

#include "common/axis.h"
#include "common/maths.h"

#include "config/feature.h"

#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/imu.h"

#include "pg/autolaunch.h"

#include "sensors/acceleration.h"

#include "autolaunch.h"

#define AUTOLAUNCH_START_THROTTLE_THRESHOLD 0.05f
#define AUTOLAUNCH_MOTOR_SPINUP_TIME_US     500000
#define AUTOLAUNCH_MAX_LAUNCH_ANGLE_DEG     45.0f

typedef enum {
    AUTOLAUNCH_IDLE = 0,
    AUTOLAUNCH_WAIT_THROTTLE,
    AUTOLAUNCH_WAIT_THROW,
    AUTOLAUNCH_MOTOR_DELAY,
    AUTOLAUNCH_MOTOR_SPINUP,
    AUTOLAUNCH_LAUNCHING,
    AUTOLAUNCH_DONE,
} autolaunchState_e;

typedef struct autolaunchRuntime_s {
    autolaunchState_e state;
    bool armedWithMode;
    timeUs_t stateStartUs;
    timeUs_t detectStartUs;
} autolaunchRuntime_t;

static FAST_DATA_ZERO_INIT autolaunchRuntime_t autolaunch;

static void setState(autolaunchState_e state, timeUs_t currentTimeUs)
{
    autolaunch.state = state;
    autolaunch.stateStartUs = currentTimeUs;
    autolaunch.detectStartUs = 0;
}

static bool pilotTookOver(void)
{
    const float threshold = autolaunchConfig()->stick_threshold / 100.0f;
    return fabsf(getDeflection(FD_ROLL)) > threshold ||
           fabsf(getDeflection(FD_PITCH)) > threshold ||
           fabsf(getDeflection(FD_YAW)) > threshold;
}

static bool throwDetected(timeUs_t currentTimeUs)
{
    const bool attitudeOk = fabsf(attitude.values.roll / 10.0f) <= AUTOLAUNCH_MAX_LAUNCH_ANGLE_DEG &&
                            fabsf(attitude.values.pitch / 10.0f) <= AUTOLAUNCH_MAX_LAUNCH_ANGLE_DEG;
    const float forwardAccelCms = acc.accADC[X] * acc.dev.acc_1G_rec * 981.0f;
    const bool accelHigh = attitudeOk && forwardAccelCms > autolaunchConfig()->accel_threshold;

    if (!accelHigh) {
        autolaunch.detectStartUs = 0;
        return false;
    }

    if (autolaunch.detectStartUs == 0) {
        autolaunch.detectStartUs = currentTimeUs;
    }

    return cmpTimeUs(currentTimeUs, autolaunch.detectStartUs + autolaunchConfig()->detect_time * 1000) >= 0;
}

void autolaunchUpdate(timeUs_t currentTimeUs)
{
    if (!featureIsEnabled(FEATURE_AUTOLAUNCH) || !ARMING_FLAG(ARMED)) {
        autolaunch.armedWithMode = false;
        setState(AUTOLAUNCH_IDLE, currentTimeUs);
        return;
    }

    if (autolaunch.state == AUTOLAUNCH_IDLE) {
        autolaunch.armedWithMode = IS_RC_MODE_ACTIVE(BOXAUTOLAUNCH);
        if (!autolaunch.armedWithMode) {
            setState(AUTOLAUNCH_DONE, currentTimeUs);
            return;
        }

        setState(AUTOLAUNCH_WAIT_THROTTLE, currentTimeUs);
    }

    if (!autolaunch.armedWithMode || pilotTookOver()) {
        setState(AUTOLAUNCH_DONE, currentTimeUs);
        return;
    }

    if (autolaunch.state == AUTOLAUNCH_WAIT_THROTTLE) {
        if (getThrottle() > AUTOLAUNCH_START_THROTTLE_THRESHOLD) {
            setState(autolaunchConfig()->auto_throttle ? AUTOLAUNCH_WAIT_THROW : AUTOLAUNCH_MOTOR_SPINUP, currentTimeUs);
        }
    } else if (autolaunch.state == AUTOLAUNCH_WAIT_THROW) {
        if (throwDetected(currentTimeUs)) {
            setState(AUTOLAUNCH_MOTOR_DELAY, currentTimeUs);
        } else if (cmpTimeUs(currentTimeUs, autolaunch.stateStartUs + autolaunchConfig()->timeout * 1000) >= 0) {
            setState(AUTOLAUNCH_DONE, currentTimeUs);
        }
    } else if (autolaunch.state == AUTOLAUNCH_MOTOR_DELAY) {
        if (cmpTimeUs(currentTimeUs, autolaunch.stateStartUs + autolaunchConfig()->motor_delay * 1000) >= 0) {
            setState(AUTOLAUNCH_MOTOR_SPINUP, currentTimeUs);
        }
    } else if (autolaunch.state == AUTOLAUNCH_MOTOR_SPINUP) {
        if (cmpTimeUs(currentTimeUs, autolaunch.stateStartUs + AUTOLAUNCH_MOTOR_SPINUP_TIME_US) >= 0) {
            setState(AUTOLAUNCH_LAUNCHING, currentTimeUs);
        }
    }
}

bool autolaunchIsActive(void)
{
    return autolaunch.state == AUTOLAUNCH_WAIT_THROTTLE ||
           autolaunch.state == AUTOLAUNCH_WAIT_THROW ||
           autolaunch.state == AUTOLAUNCH_MOTOR_DELAY ||
           autolaunch.state == AUTOLAUNCH_MOTOR_SPINUP ||
           autolaunch.state == AUTOLAUNCH_LAUNCHING;
}

bool autolaunchHasThrottleOverride(void)
{
    return autolaunchIsActive();
}

float autolaunchGetThrottle(void)
{
    if (autolaunch.state == AUTOLAUNCH_WAIT_THROTTLE || autolaunch.state == AUTOLAUNCH_WAIT_THROW || autolaunch.state == AUTOLAUNCH_MOTOR_DELAY) {
        return 0.0f;
    }

    const float targetThrottle = MAX(getThrottle(), autolaunchConfig()->launch_throttle / 100.0f);

    if (autolaunch.state == AUTOLAUNCH_MOTOR_SPINUP) {
        const float ramp = constrainf((float)cmpTimeUs(micros(), autolaunch.stateStartUs) / AUTOLAUNCH_MOTOR_SPINUP_TIME_US, 0.0f, 1.0f);
        return targetThrottle * ramp;
    }

    if (autolaunch.state == AUTOLAUNCH_LAUNCHING) {
        return targetThrottle;
    }

    return getThrottle();
}

float autolaunchApplyAxis(int axis, float setpoint)
{
    if (autolaunch.state != AUTOLAUNCH_MOTOR_DELAY && autolaunch.state != AUTOLAUNCH_MOTOR_SPINUP && autolaunch.state != AUTOLAUNCH_LAUNCHING) {
        return setpoint;
    }

    float error;

    if (axis == FD_ROLL) {
        error = -(attitude.values.roll / 10.0f);
    } else if (axis == FD_PITCH) {
        error = autolaunchConfig()->climb_angle - (attitude.values.pitch / 10.0f);
    } else {
        return setpoint;
    }

    return constrainf(error * 6.0f, -300.0f, 300.0f);
}

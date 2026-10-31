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
 * Out-of-trim monitor.
 *
 * In rate flight the I-term holds whatever steady surface deflection the airframe needs,
 * so a model that is out of trim still flies straight, and a trim press is soaked up by
 * I and seems to do nothing. MANUAL and PASSTHROUGH drop I, so the missing trim shows up
 * only there. This watches the I-term during hands-off, near-level rate flight and
 * reports, per axis, which way the saved trim should move. The radio announces it and
 * shows it on the dashboard (TELEM_SYSTEM_CONFIG, telemetry/status.h).
 *
 * It only reports. It never changes a trim.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#ifdef USE_SERVOS

#include "common/maths.h"

#include "trim_monitor.h"

typedef struct {
    float load;                 // smoothed I, surface units
    float settled;              // qualifying time so far, s
    trimMonitorState_e state;
} trimMonitorAxis_t;

static struct {
    trimMonitorAxis_t axis[XYZ_AXIS_COUNT];

    // Block mean of the PID loops, see trimMonitorUpdate()
    trimMonitorSample_t acc;
    int accCount;
    float accTime;
    bool wasArmed;
} tm;

void trimMonitorReset(void)
{
    memset(tm.axis, 0, sizeof(tm.axis));
    tm.accCount = 0;
    tm.accTime = 0;
}

// A new measurement that keeps each axis's last report until it has settled again.
void trimMonitorRestart(void)
{
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        tm.axis[axis].load = 0;
        tm.axis[axis].settled = 0;
    }
    tm.accCount = 0;
    tm.accTime = 0;
}

trimMonitorState_e trimMonitorGetState(int axis)
{
    return tm.axis[axis].state;
}

/*
 * Which way the trim has to move for a positive I. I has the sign of the stabilized axis
 * input, and so does a positive SERVO_TRIM_* step (servos.c, axisTrimDirection()), and a
 * positive stabilized input moves the surfaces like:
 *  - roll:  stick right (raw RC sign)
 *  - pitch: stick forward, nose down (raw RC sign; attitude and rate pitch are positive
 *           nose-down, see the Flight Dynamics reference)
 *  - yaw:   stick left (setpoint.c negates raw RC yaw, which is positive to the right)
 */
static trimMonitorState_e directionOf(int axis, float load)
{
    const bool positiveIsRightUp = (axis == FD_ROLL);
    return ((load > 0) == positiveIsRightUp) ? TRIM_MONITOR_RIGHT_UP : TRIM_MONITOR_LEFT_DOWN;
}

void trimMonitorProcessSample(const trimMonitorSample_t *s, float dT)
{
    const float k = MIN(dT / TRIM_MONITOR_TAU, 1.0f);

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        trimMonitorAxis_t *ax = &tm.axis[axis];

        const bool counts = s->valid && !s->saturated[axis] &&
            fabsf(s->deflection[axis]) <= TRIM_MONITOR_STICK_MAX &&
            fabsf(s->gyro[axis]) <= TRIM_MONITOR_RATE_MAX &&
            isfinitef(s->iterm[axis]);

        // Outside qualifying flight the estimate and the report are held as they are.
        if (!counts)
            continue;

        ax->load += (s->iterm[axis] - ax->load) * k;
        ax->settled += dT;

        if (ax->settled < TRIM_MONITOR_SETTLE)
            continue;

        const float magnitude = fabsf(ax->load);

        if (magnitude >= TRIM_MONITOR_ON)
            ax->state = directionOf(axis, ax->load);
        else if (magnitude < TRIM_MONITOR_OFF)
            ax->state = TRIM_MONITOR_IN_TRIM;
    }
}

#ifndef UNIT_TEST

#include "fc/rc.h"
#include "fc/runtime_config.h"

#include "flight/airborne.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/pid.h"

#include "sensors/sensors.h"

static bool qualifyingFlight(void)
{
    if (!ARMING_FLAG(ARMED) || !isAirborne() || mixerStabilizationBypassed())
        return false;

    // Leveling, holds and navigation carry I for their own reasons; I is forced to zero in
    // TRADITIONAL mode.
    if (FLIGHT_MODE(ANGLE_MODE | ATTHOLD_MODE | TRAINER_MODE | GPS_RESCUE_MODE | FAILSAFE_MODE |
                    LOITER_MODE | RTH_MODE | PASSTHROUGH_MODE | MANUAL_MODE | TRADITIONAL_MODE))
        return false;

#ifdef USE_ACC
    if (sensors(SENSOR_ACC) &&
        (ABS(attitude.values.roll) > TRIM_MONITOR_ANGLE_MAX * 10 ||
         ABS(attitude.values.pitch) > TRIM_MONITOR_ANGLE_MAX * 10))
        return false;
#endif

    return true;
}

void trimMonitorUpdate(float dT)
{
    const bool armed = ARMING_FLAG(ARMED);

    // Each flight measures afresh, so the report is about the trim being flown now. The last
    // report stays up after landing, so the radio still shows what to fix, and until this
    // flight has settled, so arming alone never reads as "trim OK".
    if (armed && !tm.wasArmed)
        trimMonitorRestart();
    tm.wasArmed = armed;

    if (!armed)
        return;

    const pidAxisData_t *pd = pidGetAxisData();
    trimMonitorSample_t *acc = &tm.acc;

    if (tm.accCount == 0) {
        memset(acc, 0, sizeof(*acc));
        acc->valid = true;
    }

    acc->valid = acc->valid && qualifyingFlight();
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        acc->iterm[axis] += pd[axis].I;
        acc->gyro[axis] += pd[axis].gyroRate;
        acc->deflection[axis] += getRcDeflection(axis);
        acc->saturated[axis] |= pidAxisSaturated(axis);
    }
    tm.accCount++;
    tm.accTime += dT;

    if (tm.accTime < 1.0f / TRIM_MONITOR_SAMPLE_HZ)
        return;

    const float n = 1.0f / tm.accCount;
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        acc->iterm[axis] *= n;
        acc->gyro[axis] *= n;
        acc->deflection[axis] *= n;
    }

    trimMonitorProcessSample(acc, tm.accTime);

    tm.accCount = 0;
    tm.accTime = 0;
}

#endif // !UNIT_TEST

#endif // USE_SERVOS

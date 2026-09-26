/*
 * This file is part of Rotorflight.
 *
 * Rotorflight is free software. You can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Rotorflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software. If not, see <https://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"
#include "build/debug.h"
#include "common/axis.h"
#include "common/maths.h"
#include "common/filter.h"
#include "drivers/time.h"
#include "flight/pid.h"
#include "flight/imu.h"
#include "fc/runtime_config.h"
#include "fc/rc.h"
#include "sensors/gyro.h"
#include "airborne.h"

#define FILTER_CUTOFF                       5.0f
#define PEAK_UP_CUTOFF                     20.0f
#define PEAK_DN_CUTOFF                      0.5f

// Evidence, not proof, of flight: a meaningful pilot command accompanied by
// sustained roll/pitch rotation in the same direction. These initial thresholds
// need flight validation. Yaw is excluded because ground steering can follow it.
#define FLIGHT_INPUT_THRESHOLD             0.10f
#define FLIGHT_RESPONSE_RATE               15.0f  // degrees/second
#define FLIGHT_RESPONSE_TIME_US          250000

typedef enum {
    AIRBORNE_STATE_INIT = 0,
    AIRBORNE_STATE_LANDED,
    AIRBORNE_STATE_AIRBORNE,
} airborneState_e;

typedef struct {
    airborneState_e state;
    float handsOnThreshold[XYZ_AXIS_COUNT];
    pt1Filter_t filter[XYZ_AXIS_COUNT];
    peakFilter_t peakDeflection[XYZ_AXIS_COUNT];
    int8_t responseDirection[2];
    timeUs_t responseSince[2];
} airborneData_t;

static FAST_DATA_ZERO_INIT airborneData_t airborne;

static void resetResponse(void)
{
    for (int axis = FD_ROLL; axis <= FD_PITCH; axis++) {
        airborne.responseDirection[axis] = 0;
        airborne.responseSince[axis] = 0;
    }
}

INIT_CODE void airborneInit(void)
{
    airborne.state = AIRBORNE_STATE_LANDED;
    resetResponse();
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        pt1FilterInit(&airborne.filter[axis], FILTER_CUTOFF, pidGetPidFrequency());
        peakFilterInit(&airborne.peakDeflection[axis], PEAK_UP_CUTOFF, PEAK_DN_CUTOFF, pidGetPidFrequency());
        airborne.handsOnThreshold[axis] = rcControlsConfig()->rc_threshold[axis] / 1000.0f;
    }
}

static bool updateFlightResponse(const float rc[XYZ_AXIS_COUNT], timeUs_t now)
{
    bool confirmed = false;
    for (int axis = FD_ROLL; axis <= FD_PITCH; axis++) {
        const float input = rc[axis];
        const float rate = gyro.gyroADCf[axis];
        const int8_t direction = input > 0 ? 1 : -1;
        // Use current pilot input, not the decaying hands-on peak or an attitude
        // controller's correction: neither is evidence of current pilot intent.
        if (fabsf(input) >= FLIGHT_INPUT_THRESHOLD && rate * direction >= FLIGHT_RESPONSE_RATE) {
            if (airborne.responseDirection[axis] != direction) {
                airborne.responseDirection[axis] = direction;
                airborne.responseSince[axis] = now;
            }
            if ((timeUs_t)(now - airborne.responseSince[axis]) >= FLIGHT_RESPONSE_TIME_US) {
                confirmed = true;
            }
        } else {
            airborne.responseDirection[axis] = 0;
            airborne.responseSince[axis] = 0;
        }
    }
    return confirmed;
}

void airborneUpdate(const float rc[XYZ_AXIS_COUNT])
{
    // Preserve hands-on detection, including yaw, independently of flight state.
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        const float stick = pt1FilterApply(&airborne.filter[axis], rc[axis]);
        peakFilterApply(&airborne.peakDeflection[axis], fabsf(stick));
    }

    const timeUs_t now = micros();
    bool flightEvidence = false;
    if (!ARMING_FLAG(ARMED)) {
        airborne.state = AIRBORNE_STATE_LANDED;
        resetResponse();
    } else if (airborne.state != AIRBORNE_STATE_AIRBORNE) {
        // Preserve the existing rescue/failsafe override. Loss of pilot response
        // must never weaken a recovery, even if takeoff was not detected first.
        if (FLIGHT_MODE(GPS_RESCUE_MODE | FAILSAFE_MODE)) {
            flightEvidence = true;
        } else if (rxIsReceivingSignal()) {
            flightEvidence = updateFlightResponse(rc, now);
        } else {
            resetResponse();
        }
        if (flightEvidence) {
            airborne.state = AIRBORNE_STATE_AIRBORNE;
            resetResponse();
        }
    }
    // Flight is latched until disarm. Quiet sticks, low throttle, lack of gyro
    // response, and loss of altitude evidence cannot distinguish a landing from
    // a glide or low control authority. Armed ground handling after landing thus
    // retains flight authority; disarm to restore the ground reduction.

    DEBUG(AIRBORNE, 0, peakFilterOutput(&airborne.peakDeflection[FD_ROLL]) * 1000);
    DEBUG(AIRBORNE, 1, peakFilterOutput(&airborne.peakDeflection[FD_PITCH]) * 1000);
    DEBUG(AIRBORNE, 2, peakFilterOutput(&airborne.peakDeflection[FD_YAW]) * 1000);
    DEBUG(AIRBORNE, 3, airborne.responseDirection[FD_ROLL] ? (timeUs_t)(now - airborne.responseSince[FD_ROLL]) / 1000 : 0);
    DEBUG(AIRBORNE, 4, getCosTiltAngle() * 1000);
    DEBUG(AIRBORNE, 5, airborne.responseDirection[FD_PITCH] ? (timeUs_t)(now - airborne.responseSince[FD_PITCH]) / 1000 : 0);
    DEBUG(AIRBORNE, 6, (flightEvidence ? 1 : 0) | (!ARMING_FLAG(ARMED) ? 2 : 0));
    DEBUG(AIRBORNE, 7, airborne.state);
}

bool isAirborne(void)
{
    return airborne.state == AIRBORNE_STATE_AIRBORNE;
}

bool isHandsOn(void)
{
    return peakFilterOutput(&airborne.peakDeflection[FD_ROLL]) > airborne.handsOnThreshold[FD_ROLL]
        || peakFilterOutput(&airborne.peakDeflection[FD_PITCH]) > airborne.handsOnThreshold[FD_PITCH]
        || peakFilterOutput(&airborne.peakDeflection[FD_YAW]) > airborne.handsOnThreshold[FD_YAW];
}

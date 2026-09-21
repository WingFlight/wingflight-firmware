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

#include "config/config.h"
#include "config/feature.h"

#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/motors.h"
#include "flight/position.h"

#include "fc/runtime_config.h"
#include "fc/rc.h"

#include "sensors/sensors.h"

#include "airborne.h"

#define FILTER_CUTOFF                       5.0f

#define PEAK_UP_CUTOFF                     20.0f
#define PEAK_DN_CUTOFF                      0.5f

#define LIFTOFF_COS_ANGLE_THRESHOLD        0.80f
#define LANDING_COS_ANGLE_THRESHOLD        0.90f

// Motor output (getMotorOutput() units, per mille of full output) that counts as flying. Lower
// on the way down, so an output hovering at the limit does not flip the state back and forth.
#define LIFTOFF_MOTOR_OUTPUT                100
#define LANDING_MOTOR_OUTPUT                 50

// Height above the point where the aircraft was armed (centimeters) that counts as flying, lower
// on the way down for the same reason as the motor limits above.
#define LIFTOFF_ALTITUDE_CM                 200
#define LANDING_ALTITUDE_CM                 100

typedef enum {
    AIRBORNE_STATE_INIT = 0,
    AIRBORNE_STATE_LANDED,
    AIRBORNE_STATE_AIRBORNE,
} airborneState_e;

typedef struct
{
    airborneState_e state;

    float liftoffThreshold[XYZ_AXIS_COUNT];
    float landingThreshold[XYZ_AXIS_COUNT];

    pt1Filter_t filter[XYZ_AXIS_COUNT];
    peakFilter_t peakDeflection[XYZ_AXIS_COUNT];

} airborneData_t;

static FAST_DATA_ZERO_INIT airborneData_t airborne;


INIT_CODE void airborneInit(void)
{
    airborne.state = AIRBORNE_STATE_LANDED;

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        pt1FilterInit(&airborne.filter[axis], FILTER_CUTOFF, pidGetPidFrequency());
        peakFilterInit(&airborne.peakDeflection[axis], PEAK_UP_CUTOFF, PEAK_DN_CUTOFF, pidGetPidFrequency());
        airborne.liftoffThreshold[axis] = rcControlsConfig()->rc_threshold[axis] / 1000.0f;
        airborne.landingThreshold[axis] = rcControlsConfig()->rc_threshold[axis] / 1500.0f;
    }
}

static bool isOverThreshold(const float *threshold)
{
    return (
        peakFilterOutput(&airborne.peakDeflection[FD_ROLL]) > threshold[FD_ROLL] ||
        peakFilterOutput(&airborne.peakDeflection[FD_PITCH]) > threshold[FD_PITCH] ||
        peakFilterOutput(&airborne.peakDeflection[FD_YAW]) > threshold[FD_YAW]
    );
}

// The strongest motor output, whichever direction it is turning. 0 with no motors.
static int motorOutputPeak(void)
{
    int peak = 0;

    for (int i = 0; i < getMotorCount(); i++) {
        const int output = getMotorOutput(i);
        peak = MAX(peak, output < 0 ? -output : output);
    }

    return peak;
}

// Whether the motors say the aircraft is flying. Stick and tilt alone can not tell a wing
// cruising level with its sticks released from one sitting on the ground, and that is the
// common case for a hold mode, so a running motor keeps the state airborne.
//
// A model with no motor has nothing to go on: treat it as airborne whenever it is armed. Without
// a propeller the ground authority reduction buys little, and using the throttle channel instead
// would leave a glider permanently "landed".
static bool motorSaysAirborne(int threshold)
{
    return getMotorCount() == 0 || motorOutputPeak() > threshold;
}

// Whether the barometer says the aircraft is well above where it was armed. The altitude estimate
// is already relative to the arm point, and it is 0 when there is no usable source. This is
// evidence for flight only: an aircraft that is not high, or has no barometer, says nothing
// (a slope launch that flies below its arm point, for instance).
static bool altitudeSaysAirborne(int thresholdCm)
{
#ifdef USE_BARO
    return sensors(SENSOR_BARO) && getEstimatedAltitudeCm() > thresholdCm;
#else
    UNUSED(thresholdCm);
    return false;
#endif
}

static bool liftoff(void)
{
    return (
        ARMING_FLAG(ARMED) &&
        (
            isOverThreshold(airborne.liftoffThreshold) ||
            getCosTiltAngle() < LIFTOFF_COS_ANGLE_THRESHOLD ||
            FLIGHT_MODE(GPS_RESCUE_MODE | FAILSAFE_MODE) ||
            motorSaysAirborne(LIFTOFF_MOTOR_OUTPUT) ||
            altitudeSaysAirborne(LIFTOFF_ALTITUDE_CM)
        )
    );
}

static bool touchdown(void)
{
    return !(
        ARMING_FLAG(ARMED) &&
        (
            isOverThreshold(airborne.landingThreshold) ||
            getCosTiltAngle() < LANDING_COS_ANGLE_THRESHOLD ||
            FLIGHT_MODE(GPS_RESCUE_MODE | FAILSAFE_MODE) ||
            motorSaysAirborne(LANDING_MOTOR_OUTPUT) ||
            altitudeSaysAirborne(LANDING_ALTITUDE_CM)
        )
    );
}

static void updateStickDeflection(const float rc[XYZ_AXIS_COUNT])
{
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        float stick = pt1FilterApply(&airborne.filter[axis], rc[axis]);
        peakFilterApply(&airborne.peakDeflection[axis], fabsf(stick));
    }
}

void airborneUpdate(const float rc[XYZ_AXIS_COUNT])
{
    updateStickDeflection(rc);

    switch (airborne.state) {
        case AIRBORNE_STATE_INIT:
            break;
        case AIRBORNE_STATE_LANDED:
            if (liftoff()) {
                airborne.state = AIRBORNE_STATE_AIRBORNE;
            }
            break;
        case AIRBORNE_STATE_AIRBORNE:
            if (touchdown()) {
                airborne.state = AIRBORNE_STATE_LANDED;
            }
            break;
    }

    DEBUG(AIRBORNE, 0, peakFilterOutput(&airborne.peakDeflection[FD_ROLL]) * 1000);
    DEBUG(AIRBORNE, 1, peakFilterOutput(&airborne.peakDeflection[FD_PITCH]) * 1000);
    DEBUG(AIRBORNE, 2, peakFilterOutput(&airborne.peakDeflection[FD_YAW]) * 1000);
    DEBUG(AIRBORNE, 3, motorOutputPeak());
    DEBUG(AIRBORNE, 4, getCosTiltAngle() * 1000);
    DEBUG(AIRBORNE, 5, getEstimatedAltitudeCm());
    DEBUG(AIRBORNE, 6, (liftoff() ? 1 : 0) | (touchdown() ? 2 : 0));
    DEBUG(AIRBORNE, 7, airborne.state);
}

bool isAirborne(void)
{
    return (airborne.state == AIRBORNE_STATE_AIRBORNE);
}

bool isHandsOn(void)
{
    return isOverThreshold(airborne.liftoffThreshold);
}
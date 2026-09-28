/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "config/config.h"

#ifdef USE_ACC

#include "common/axis.h"
#include "common/filter.h"

#include "fc/rc_rates.h"
#include "fc/core.h"
#include "fc/rc.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/imu.h"
#include "flight/gps_rescue.h"
#include "flight/gps_nav.h"

#include "sensors/acceleration.h"
#include "sensors/gyro.h"

#include "leveling.h"

typedef struct {
    float Gain;
    float AngleLimit[2];
} level_t;

static FAST_DATA_ZERO_INIT level_t level;


INIT_CODE void levelingInit(const pidProfile_t *pidProfile)
{
    level.Gain = pidProfile->angle.level_strength / 10.0f;
    const attitudeLimits_t *limits = attitudeLimits(getCurrentPidProfileIndex());
    level.AngleLimit[FD_ROLL] = attitudeLimitDegrees(limits->angle_roll, pidProfile->angle.level_limit, FD_ROLL);
    level.AngleLimit[FD_PITCH] = attitudeLimitDegrees(limits->angle_pitch, pidProfile->angle.level_limit, FD_PITCH);
}

int get_ADJUSTMENT_ANGLE_LEVEL_GAIN(void)
{
    return currentPidProfile->angle.level_strength;
}

void set_ADJUSTMENT_ANGLE_LEVEL_GAIN(int value)
{
    currentPidProfile->angle.level_strength = value;
    level.Gain = value / 10.0f;
}

// calculate the stick deflection while applying level mode expo
static inline float getLevelModeDeflection(uint8_t axis)
{
    float deflection = getDeflection(axis);

    if (axis < FD_YAW) {
        const float expof = currentControlRateProfile->levelExpo[axis] / 100.0f;
        deflection = POWER3(deflection) * expof + deflection * (1 - expof);
    }

    return deflection;
}

static float calcLevelErrorAngle(int axis)
{
    const rollAndPitchTrims_t *angleTrim = &accelerometerConfig()->accelerometerTrims;
    const float angleLimit = level.AngleLimit[axis];
    float angle = angleLimit * getLevelModeDeflection(axis);

#ifdef USE_GPS_RESCUE
    angle += gpsRescueAngle[axis] / 100.0f; // ANGLE IS IN CENTIDEGREES
#endif
#ifdef USE_GPS_NAV
    angle += navAngle[axis] / 100.0f; // ANGLE IS IN CENTIDEGREES
#endif
    angle = constrainf(angle, -angleLimit, angleLimit);

    float currentAngle = ((attitude.raw[axis] - angleTrim->raw[axis]) / 10.0f);

    float error = angle - currentAngle;

    return error;
}

float angleModeApply(int axis, float pidSetpoint)
{
    if (axis == FD_ROLL || axis == FD_PITCH)
    {
        float errorAngle = calcLevelErrorAngle(axis);

        if (!isAirborne())
            errorAngle *= 0.25f;

        pidSetpoint = errorAngle * level.Gain;
    }
#ifdef USE_GPS_NAV
    else if (axis == FD_YAW && FLIGHT_MODE(LOITER_MODE | RTH_MODE)) {
        // Nav banks the aircraft but leaves yaw in rate mode; add the coordinated-turn yaw rate
        // so the yaw PID stops holding rudder against every nav turn. Pilot yaw still adds on top.
        pidSetpoint += navTurnCoordinationYawRate();
    }
#endif

    return pidSetpoint;
}

#endif // USE_ACC

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

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "platform.h"

#ifdef USE_GPS_NAV

#include "common/axis.h"
#include "common/maths.h"

#include "pg/gps.h"
#include "pg/gps_nav.h"

#include "io/gps.h"

#include "flight/position.h"

#include "flight/gps_nav.h"

typedef enum {
    NAV_PHASE_APPROACH, // flying toward the target center, outside the loiter radius
    NAV_PHASE_ORBIT,    // circling the target within the loiter radius
} navPhase_e;

typedef struct {
    bool active;
    navPhase_e phase;
    int32_t targetLat;
    int32_t targetLon;
    int32_t targetAltitudeCm;
} navState_t;

static navState_t nav;

int32_t navAngle[ANGLE_INDEX_COUNT] = { 0, 0 };

static bool navIsHealthy(void)
{
    return gpsIsHealthy() && gpsSol.numSat >= gpsNavConfig()->minSats;
}

static void navBegin(int32_t lat, int32_t lon, int32_t altitudeCm)
{
    nav.targetLat = lat;
    nav.targetLon = lon;
    nav.targetAltitudeCm = altitudeCm;
    nav.phase = NAV_PHASE_APPROACH;
    nav.active = navIsHealthy();
    navAngle[AI_ROLL] = 0;
    navAngle[AI_PITCH] = 0;
}

void navLoiterStart(void)
{
    navBegin(gpsSol.llh.lat, gpsSol.llh.lon, getEstimatedAltitudeCm());
}

void navRthStart(void)
{
    navBegin(GPS_home[GPS_LATITUDE], GPS_home[GPS_LONGITUDE], gpsNavConfig()->rthAltitudeM * 100);
}

void navStop(void)
{
    nav.active = false;
    navAngle[AI_ROLL] = 0;
    navAngle[AI_PITCH] = 0;
}

// wrap a decidegrees bearing difference into (-1800, 1800]
static int32_t wrapBearingErrorDdeg(int32_t errorDdeg)
{
    while (errorDdeg > 1800) {
        errorDdeg -= 3600;
    }
    while (errorDdeg <= -1800) {
        errorDdeg += 3600;
    }
    return errorDdeg;
}

void updateGpsNav(void)
{
    if (!nav.active) {
        return;
    }

    if (!navIsHealthy()) {
        navStop();
        return;
    }

    uint32_t distCm;
    int32_t bearingCdeg; // centidegrees
    GPS_distance_cm_bearing(&gpsSol.llh.lat, &gpsSol.llh.lon, &nav.targetLat, &nav.targetLon, &distCm, &bearingCdeg);

    const int32_t bearingToTargetDdeg = bearingCdeg / 10; // decidegrees
    const uint32_t distM = distCm / 100;
    const uint16_t radiusM = gpsNavConfig()->loiterRadiusM;

    int32_t desiredTrackDdeg;
    if (distM > radiusM) {
        nav.phase = NAV_PHASE_APPROACH;
        desiredTrackDdeg = bearingToTargetDdeg;
    } else {
        nav.phase = NAV_PHASE_ORBIT;
        const int32_t tangentOffsetDdeg = (gpsNavConfig()->loiterDirection == NAV_LOITER_CW) ? 900 : -900;
        desiredTrackDdeg = bearingToTargetDdeg + tangentOffsetDdeg;
    }

    const int32_t trackErrorDdeg = wrapBearingErrorDdeg(desiredTrackDdeg - (int32_t)gpsSol.groundCourse);

    const float bearingKp = gpsNavConfig()->bearingKp / 100.0f;
    const float maxBankDdeg = gpsNavConfig()->maxBankAngleDeg * 10.0f;
    const float bankDdeg = constrainf(bearingKp * trackErrorDdeg, -maxBankDdeg, maxBankDdeg);
    navAngle[AI_ROLL] = lrintf(bankDdeg * 10.0f); // decidegrees -> centidegrees

    const int32_t altitudeErrorM = (nav.targetAltitudeCm - getEstimatedAltitudeCm()) / 100;
    const float altitudeKp = gpsNavConfig()->altitudeKp / 100.0f;
    const float maxPitchDdeg = gpsNavConfig()->maxPitchAngleDeg * 10.0f;
    const float pitchDdeg = constrainf(altitudeKp * altitudeErrorM, -maxPitchDdeg, maxPitchDdeg);
    navAngle[AI_PITCH] = lrintf(pitchDdeg * 10.0f); // decidegrees -> centidegrees
}

#endif // USE_GPS_NAV

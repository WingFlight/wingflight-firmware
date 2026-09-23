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

#include "fc/runtime_config.h"

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
    // gpsIsHealthy() only means GPS frames are being received -- it says nothing about whether
    // the last one was actually a fix. UBLOX PVT in particular sets gpsSol.numSat from
    // _buffer.pvt.numSV unconditionally, independent of fixType/NAV_STATUS_FIX_VALID (see
    // gps.c's UBLOX_parse_gps()), so a receiver can report a healthy satellite count with no
    // valid fix at all. Without this, nav would keep commanding bank/pitch toward
    // gpsSol.llh.lat/lon even while that position is stale or invalid.
    return gpsIsHealthy() && STATE(GPS_FIX) && gpsSol.numSat >= gpsNavConfig()->minSats;
}

// Whether a return-to-home is even meaningful right now: needs a healthy GPS fix (same bar
// as any other nav start) and an actual recorded home position. Without the latter,
// navRthStart() would fly toward GPS_home = {0,0} -- "null island" -- since navBegin() itself
// only gates on navIsHealthy(), not on STATE(GPS_FIX_HOME). Exported so callers that want to
// start an RTH (or decide whether one is possible before falling back to something else, e.g.
// failsafe.c) don't have to duplicate either check.
bool navCanRTH(void)
{
    return navIsHealthy() && STATE(GPS_FIX_HOME);
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
        // Zero the commanded bank/pitch (falls back to plain Angle-mode leveling) but leave
        // nav.active/target set, rather than navStop()-ing outright: core.c's RTH_MODE/LOITER_MODE
        // switch-latch (wasRthActive/wasLoiterActive) only calls navRthStart()/navLoiterStart()
        // again on the switch's off->on edge, so a hard stop here would leave nav permanently
        // disengaged -- commanding nothing -- until the pilot cycles the switch, even after GPS
        // recovers. This resumes toward the original target the moment health returns.
        navAngle[AI_ROLL] = 0;
        navAngle[AI_PITCH] = 0;
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
        // bearingToTargetDdeg is the bearing FROM the aircraft TO the target. Orbiting clockwise
        // seen from above keeps the target on the aircraft's right, so the track is 90 degrees to
        // the LEFT of that bearing (an aircraft south of the target, bearing 0, flies west); anti-
        // clockwise is 90 degrees to the right.
        const int32_t tangentOffsetDdeg = (gpsNavConfig()->loiterDirection == NAV_LOITER_CW) ? -900 : 900;
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
    // altitudeErrorM is positive when below target (need to climb). Pitch in this codebase's
    // convention is positive NOSE-DOWN (bench-confirmed in autohover.c: +900 drives the elevator
    // toward nose-down, -900 is the physically-vertical nose-up target -- same convention
    // attitude.raw[]/navAngle[] use throughout, see leveling.c's calcLevelErrorAngle()), so
    // climbing needs a NEGATIVE pitch target. Negate here, rather than folding the sign into
    // altitudeKp, so a positive altitudeKp in the config still reads as "more correction".
    const float pitchDdeg = constrainf(-altitudeKp * altitudeErrorM, -maxPitchDdeg, maxPitchDdeg);
    navAngle[AI_PITCH] = lrintf(pitchDdeg * 10.0f); // decidegrees -> centidegrees
}

#endif // USE_GPS_NAV

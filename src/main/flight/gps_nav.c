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

#include "drivers/time.h"

#include "fc/runtime_config.h"

#include "pg/gps.h"
#include "pg/gps_nav.h"

#include "io/gps.h"

#include "flight/imu.h"
#include "flight/position.h"

#include "flight/gps_nav.h"

// Orbit guidance. The desired ground track is a vector field around the target: the tangent of
// the loiter circle when on it, turning progressively inward (toward "head straight at the
// target") the further outside the circle the aircraft is, and outward when inside. That
// converges smoothly onto the circle from anywhere. The old two-phase scheme (head at the target
// until inside the radius, then fly the bare tangent) had no radial correction at all: an aircraft
// that couldn't turn as tight as the radius at max bank stayed in the approach phase forever,
// circling at full bank on whatever radius its turn performance allowed.
//
// NAV_ORBIT_CONVERGENCE sets how sharply the field turns inward: the track offset from the
// tangent is atan(K * (distance - radius) / radius), so K=2 gives 63 deg at twice the radius and
// 27 deg at 1.25x the radius.
#define NAV_ORBIT_CONVERGENCE       2.0f

// Bank command slew limit. bearingKp saturates the bank command on any sizable track error, so
// without this a track error crossing zero flips the bank from limit to limit in one GPS update
// (logged at over 400 deg/s of roll).
#define NAV_BANK_SLEW_DDEG_PER_S    450.0f  // 45 deg/s

// Floor on the speed used by the turn feedforwards, so a slow GPS ground speed (a strong
// headwind, or a zero reading) can't demand an absurd bank or yaw rate.
#define NAV_MIN_TURN_SPEED_CMS      800.0f  // 8 m/s

// Cap on the coordinated-turn yaw rate fed to the yaw axis.
#define NAV_MAX_TURN_YAW_RATE_DPS   45.0f

#define NAV_GRAVITY_CMSS            980.665f

typedef struct {
    bool active;
    int32_t targetLat;
    int32_t targetLon;
    int32_t targetAltitudeCm;
    float bankDdeg;             // slew-limited bank command
    timeMs_t lastUpdateMs;
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

// Loiter's counterpart to navCanRTH(): it only needs a healthy fix, since it holds around the
// current position rather than a recorded home.
bool navCanLoiter(void)
{
    return navIsHealthy();
}

static void navBegin(int32_t lat, int32_t lon, int32_t altitudeCm)
{
    nav.targetLat = lat;
    nav.targetLon = lon;
    nav.targetAltitudeCm = altitudeCm;
    nav.active = navIsHealthy();
    nav.bankDdeg = 0;
    nav.lastUpdateMs = millis();
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
    nav.bankDdeg = 0;
    navAngle[AI_ROLL] = 0;
    navAngle[AI_PITCH] = 0;
}

// Throttle for every GPS-guided flight: BOXLOITER/BOXRTH (mixer.c) and the failsafe GPS rescue
// phase (failsafe.c). One setting, so a switch RTH and a failsafe RTH fly home the same way.
float navGetThrottle(void)
{
    return constrainf(gpsNavConfig()->throttle / 100.0f, 0.0f, 1.0f);
}

// Body yaw rate of a coordinated turn at the current bank, in deg/s, in the yaw gyro's sign
// convention -- a right turn reads NEGATIVE on the yaw gyro (flight logs of a steady right-hand
// loiter show about -9 deg/s). Without this the yaw axis stays in plain rate mode with a zero
// setpoint during nav, so the yaw PID treats the turn's own yaw rate as a disturbance and holds
// rudder against every turn (about 10% of travel in those same logs), skidding the aircraft round
// a wider, draggier turn.
//
// For a coordinated turn at bank phi, pitch theta and speed V, the earth-frame turn rate is
// g*tan(phi)/V; its body-yaw component is that times cos(phi)*cos(theta), which is
// g*sin(phi)*cos(theta)/V. GPS ground speed stands in for airspeed.
float navTurnCoordinationYawRate(void)
{
    const float gain = gpsNavConfig()->turnCoordination / 100.0f;
    if (!nav.active || gain <= 0 || !navIsHealthy()) {
        return 0;
    }

    const float speedCms = fmaxf(gpsSol.groundSpeed, NAV_MIN_TURN_SPEED_CMS);
    // attitude.values are int16 decidegrees -- divide as float; DECIDEGREES_TO_RADIANS() would
    // integer-divide by 10 first.
    const float rollRad = DEGREES_TO_RADIANS(attitude.values.roll / 10.0f);
    const float pitchRad = DEGREES_TO_RADIANS(attitude.values.pitch / 10.0f);
    const float yawRateDps = RADIANS_TO_DEGREES(NAV_GRAVITY_CMSS * sin_approx(rollRad) * cos_approx(pitchRad) / speedCms);

    return -gain * constrainf(yawRateDps, -NAV_MAX_TURN_YAW_RATE_DPS, NAV_MAX_TURN_YAW_RATE_DPS);
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

    const timeMs_t nowMs = millis();
    const float dtS = (nowMs - nav.lastUpdateMs) / 1000.0f;
    nav.lastUpdateMs = nowMs;

    if (!navIsHealthy()) {
        // Zero the commanded bank/pitch (falls back to plain Angle-mode leveling) but leave
        // nav.active/target set, rather than navStop()-ing outright: core.c's RTH_MODE/LOITER_MODE
        // switch-latch (wasRthActive/wasLoiterActive) only calls navRthStart()/navLoiterStart()
        // again on the switch's off->on edge, so a hard stop here would leave nav permanently
        // disengaged -- commanding nothing -- until the pilot cycles the switch, even after GPS
        // recovers. This resumes toward the original target the moment health returns.
        nav.bankDdeg = 0;
        navAngle[AI_ROLL] = 0;
        navAngle[AI_PITCH] = 0;
        return;
    }

    uint32_t distCm;
    int32_t bearingCdeg; // centidegrees
    GPS_distance_cm_bearing(&gpsSol.llh.lat, &gpsSol.llh.lon, &nav.targetLat, &nav.targetLon, &distCm, &bearingCdeg);

    const int32_t bearingToTargetDdeg = bearingCdeg / 10; // decidegrees
    const float radiusCm = MAX(gpsNavConfig()->loiterRadiusM, 1) * 100.0f;

    // +1 orbits clockwise seen from above (target kept on the right, banking right), -1 anticlockwise.
    const float orbitSign = (gpsNavConfig()->loiterDirection == NAV_LOITER_CW) ? 1.0f : -1.0f;

    // Offset of the desired track from the orbit tangent, toward the target: 0 on the circle,
    // approaching +90 deg (straight at the target) far outside, negative (away from it) inside.
    const float convergeRad = atan2_approx(NAV_ORBIT_CONVERGENCE * ((float)distCm - radiusCm), radiusCm);
    const float convergeDdeg = RADIANS_TO_DEGREES(convergeRad) * 10.0f;

    // bearingToTargetDdeg is the bearing FROM the aircraft TO the target. The clockwise tangent
    // keeps the target on the aircraft's right, so it is 90 degrees to the LEFT of that bearing
    // (an aircraft south of the target, bearing 0, flies west); anticlockwise is 90 degrees right.
    const int32_t desiredTrackDdeg = bearingToTargetDdeg - lrintf(orbitSign * (900.0f - convergeDdeg));

    const int32_t trackErrorDdeg = wrapBearingErrorDdeg(desiredTrackDdeg - (int32_t)gpsSol.groundCourse);

    // Feedforward: the bank a coordinated turn needs to follow the circle's curvature at the
    // current speed, atan(V^2 / (g * R)), faded out by cos(offset) as the desired track swings
    // away from the tangent. Outside the circle the field's curves widen with distance, so use
    // the larger of distance and radius. The P term then only corrects the residual error rather
    // than supplying the whole turn, so it stays off the bank limit on a flyable radius.
    const float speedCms = fmaxf(gpsSol.groundSpeed, NAV_MIN_TURN_SPEED_CMS);
    const float curveRadiusCm = fmaxf((float)distCm, radiusCm);
    const float feedforwardDdeg = orbitSign * cos_approx(convergeRad)
        * RADIANS_TO_DEGREES(atan2_approx(speedCms * speedCms, NAV_GRAVITY_CMSS * curveRadiusCm)) * 10.0f;

    const float bearingKp = gpsNavConfig()->bearingKp / 100.0f;
    const float maxBankDdeg = gpsNavConfig()->maxBankAngleDeg * 10.0f;
    const float bankTargetDdeg = constrainf(feedforwardDdeg + bearingKp * trackErrorDdeg, -maxBankDdeg, maxBankDdeg);

    const float maxStepDdeg = NAV_BANK_SLEW_DDEG_PER_S * fmaxf(dtS, 0.0f);
    nav.bankDdeg += constrainf(bankTargetDdeg - nav.bankDdeg, -maxStepDdeg, maxStepDdeg);
    navAngle[AI_ROLL] = lrintf(nav.bankDdeg * 10.0f); // decidegrees -> centidegrees

    // Altitude: PD on the altitude error, damped by the climb rate. Both gains are centidegrees
    // (of pitch per meter, and per m/s), so /100 gives degrees. This used to treat kp*meters as
    // DECIDEGREES, making the documented 1.0 deg/m default an effective 0.1 deg/m: 25 m below
    // the RTH altitude commanded just 2.5 deg of nose-up, and the aircraft sank through every
    // RTH and loiter.
    const float altitudeErrorM = (nav.targetAltitudeCm - getEstimatedAltitudeCm()) / 100.0f;
    const float climbRateMs = getEstimatedVarioCms() / 100.0f;
    const float altitudeKp = gpsNavConfig()->altitudeKp / 100.0f;
    const float altitudeKd = gpsNavConfig()->altitudeKd / 100.0f;
    const float maxPitchDeg = gpsNavConfig()->maxPitchAngleDeg;
    // altitudeErrorM is positive when below target (need to climb). Pitch in this codebase's
    // convention is positive NOSE-DOWN (bench-confirmed in autohover.c: +900 drives the elevator
    // toward nose-down, -900 is the physically-vertical nose-up target -- same convention
    // attitude.raw[]/navAngle[] use throughout, see leveling.c's calcLevelErrorAngle()), so
    // climbing needs a NEGATIVE pitch target. Negate here, rather than folding the sign into
    // altitudeKp, so a positive altitudeKp in the config still reads as "more correction". A
    // positive climb rate backs the nose-up command off, so the climb arrives without overshoot.
    const float pitchDeg = constrainf(-(altitudeKp * altitudeErrorM - altitudeKd * climbRateMs), -maxPitchDeg, maxPitchDeg);
    navAngle[AI_PITCH] = lrintf(pitchDeg * 100.0f); // degrees -> centidegrees
}

#endif // USE_GPS_NAV

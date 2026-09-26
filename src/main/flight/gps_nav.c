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
#include "common/utils.h"

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

// Dead reckoning. Every GPS task tick with a good fix (navSampleGps()) records the position,
// ground speed and course, plus the IMU heading at that moment. When the fix drops out -- a
// satellite or two lost in a bank, a single 2D frame, a short serial gap -- position is carried
// forward from the last good sample at the last ground speed, along a course turned by however
// far the IMU heading has turned since, so the estimate follows the loiter circle rather than
// flying off along its tangent. It stays usable for NAV_DEAD_RECKONING_MS; only after that does
// nav report itself unhealthy. Without this, every blip dropped LOITER/RTH guidance (and the
// radio called "unavailable") for as long as the blip lasted.
//
// Ground speed includes the wind, and turning it with the heading isn't exact in a crosswind,
// so the window is kept short: at 20 m/s on a 100 m loiter circle the error over 5 s is a few
// metres, well inside what the orbit guidance absorbs.
#define NAV_DEAD_RECKONING_MS       5000

// While the estimate is fresh, a fix with one satellite fewer than nav_min_sats is still
// accepted, so a count hovering on the threshold can't flip nav in and out every GPS frame.
// Never below 4, the minimum for a 3D fix.
#define NAV_MIN_SATS_HYSTERESIS     1
#define NAV_MIN_SATS_FLOOR          4

// 1e-7 degrees of latitude in cm (and of longitude, at the equator)
#define NAV_CM_PER_LATLON_UNIT      1.113195f

typedef struct {
    bool valid;                 // a good sample has been taken at some point
    int32_t lat;                // the last good sample
    int32_t lon;
    float speedCms;
    int32_t courseDdeg;
    int32_t headingDdeg;        // IMU heading when the sample was taken
    timeMs_t sampleMs;
} navEstimate_t;

typedef struct {
    int32_t lat;
    int32_t lon;
    int32_t courseDdeg;
} navPosition_t;

typedef enum {
    NAV_TARGET_NONE = 0,
    NAV_TARGET_LOITER,          // hold around the position at engage
    NAV_TARGET_HOME,            // fly back to GPS_home
} navTargetType_e;

typedef struct {
    bool active;
    navTargetType_e targetType;
    bool targetSet;             // false until there was a usable position (or home) to aim at
    int32_t targetLat;
    int32_t targetLon;
    int32_t targetAltitudeCm;
    bool targetAltitudeSet;     // false until there was a real altitude estimate to hold (loiter)
    float bankDdeg;            // slew-limited bank command
    timeMs_t lastUpdateMs;
} navState_t;

static navState_t nav;
static navEstimate_t est;

int32_t navAngle[ANGLE_INDEX_COUNT] = { 0, 0 };

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

static bool navEstimateFresh(timeMs_t nowMs)
{
    return est.valid && cmp32(nowMs, est.sampleMs) <= NAV_DEAD_RECKONING_MS;
}

// Whether the current GPS solution is good enough to sample. gpsIsHealthy() only means GPS
// frames are being received -- it says nothing about whether the last one was actually a fix.
// UBLOX PVT in particular sets gpsSol.numSat from _buffer.pvt.numSV unconditionally, independent
// of fixType/NAV_STATUS_FIX_VALID (see gps.c's UBLOX_parse_gps()), so a receiver can report a
// healthy satellite count with no valid fix at all.
static bool navGpsSampleGood(timeMs_t nowMs)
{
    int minSats = gpsNavConfig()->minSats;
    if (navEstimateFresh(nowMs)) {
        minSats = MAX(minSats - NAV_MIN_SATS_HYSTERESIS, NAV_MIN_SATS_FLOOR);
    }
    return gpsIsHealthy() && STATE(GPS_FIX) && gpsSol.numSat >= minSats;
}

static void navSampleGps(timeMs_t nowMs)
{
    if (!navGpsSampleGood(nowMs)) {
        return;
    }
    est.valid = true;
    est.lat = gpsSol.llh.lat;
    est.lon = gpsSol.llh.lon;
    est.speedCms = gpsSol.groundSpeed;
    est.courseDdeg = gpsSol.groundCourse;
    est.headingDdeg = attitude.values.yaw;
    est.sampleMs = nowMs;
}

static bool navIsHealthyAt(timeMs_t nowMs)
{
    return navGpsSampleGood(nowMs) || navEstimateFresh(nowMs);
}

static bool navIsHealthy(void)
{
    return navIsHealthyAt(millis());
}

// Position and course now: the last good sample, carried forward by dead reckoning when it's
// older than this tick. Only meaningful while navIsHealthyAt(nowMs).
static navPosition_t navEstimatePosition(timeMs_t nowMs)
{
    navPosition_t pos = { est.lat, est.lon, est.courseDdeg };

    const float ageS = cmp32(nowMs, est.sampleMs) / 1000.0f;
    if (ageS <= 0) {
        return pos;
    }

    // Turn the sampled ground course by the IMU heading change since, and move along the average
    // of the two courses -- the chord of a steady turn.
    const int32_t turnDdeg = wrapBearingErrorDdeg(attitude.values.yaw - est.headingDdeg);
    pos.courseDdeg = (est.courseDdeg + turnDdeg + 3600) % 3600;

    const float chordRad = DEGREES_TO_RADIANS((est.courseDdeg + turnDdeg / 2.0f) / 10.0f);
    const float distCm = est.speedCms * ageS;
    const float lonScale = fmaxf(cos_approx(DEGREES_TO_RADIANS(est.lat / 1e7f)), 0.01f);

    pos.lat = est.lat + lrintf(distCm * cos_approx(chordRad) / NAV_CM_PER_LATLON_UNIT);
    pos.lon = est.lon + lrintf(distCm * sin_approx(chordRad) / (NAV_CM_PER_LATLON_UNIT * lonScale));
    return pos;
}

// The position nav is steering from, in 1e-7 degrees: the live GPS fix, or the dead-reckoned
// estimate through a dropout. False when there is no usable position.
bool navGetEstimatedPosition(int32_t *lat, int32_t *lon)
{
    const timeMs_t nowMs = millis();
    if (!navIsHealthyAt(nowMs)) {
        return false;
    }
    const navPosition_t pos = navEstimatePosition(nowMs);
    *lat = pos.lat;
    *lon = pos.lon;
    return true;
}

// Whether a return-to-home is even meaningful right now: needs a usable position (same bar as
// any other nav) and an actual recorded home position. Without the latter, RTH would fly toward
// GPS_home = {0,0} -- "null island". Exported so callers that want to start an RTH (or decide
// whether one is possible before falling back to something else, e.g. failsafe.c) don't have to
// duplicate either check.
bool navCanRTH(void)
{
    return navIsHealthy() && STATE(GPS_FIX_HOME);
}

// Loiter's counterpart to navCanRTH(): it only needs a usable position, since it holds around the
// current position rather than a recorded home.
bool navCanLoiter(void)
{
    return navIsHealthy();
}

// Aim at the loiter point or home once there is one. RTH needs a recorded home: without it,
// navigating would fly toward GPS_home = {0,0}.
static void navCaptureTarget(timeMs_t nowMs)
{
    if (nav.targetType == NAV_TARGET_HOME) {
        if (STATE(GPS_FIX_HOME)) {
            nav.targetLat = GPS_home[GPS_LATITUDE];
            nav.targetLon = GPS_home[GPS_LONGITUDE];
            nav.targetSet = true;
        }
    } else if (nav.targetType == NAV_TARGET_LOITER) {
        navSampleGps(nowMs);
        if (navIsHealthyAt(nowMs)) {
            const navPosition_t pos = navEstimatePosition(nowMs);
            nav.targetLat = pos.lat;
            nav.targetLon = pos.lon;
            nav.targetSet = true;
        }
    }
}

// Engaging always activates nav, even while the position isn't usable; the target is captured on
// the first tick that it is. This used to set nav.active from the health at the instant the
// switch was flipped, and core.c only calls navLoiterStart()/navRthStart() on the switch's
// off->on edge, so a dropout at that instant left nav disengaged -- flying level at nav_throttle,
// never turning -- until the pilot cycled the switch.
static void navBegin(navTargetType_e targetType)
{
    const timeMs_t nowMs = millis();

    nav.active = true;
    nav.targetType = targetType;
    nav.targetSet = false;
    // Loiter holds the altitude at engage even if the position is captured a moment later -- or,
    // with no altitude estimate yet, the first real one.
    if (targetType == NAV_TARGET_HOME) {
        nav.targetAltitudeCm = gpsNavConfig()->rthAltitudeM * 100;
        nav.targetAltitudeSet = true;
    } else {
        nav.targetAltitudeSet = hasEstimatedAltitude();
        nav.targetAltitudeCm = getEstimatedAltitudeCm();
    }
    nav.bankDdeg = 0;
    nav.lastUpdateMs = nowMs;
    navAngle[AI_ROLL] = 0;
    navAngle[AI_PITCH] = 0;

    navCaptureTarget(nowMs);
}

void navLoiterStart(void)
{
    navBegin(NAV_TARGET_LOITER);
}

void navRthStart(void)
{
    navBegin(NAV_TARGET_HOME);
}

void navStop(void)
{
    nav.active = false;
    nav.targetType = NAV_TARGET_NONE;
    nav.targetSet = false;
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
    if (!nav.active || gain <= 0) {
        return 0;
    }

    const timeMs_t nowMs = millis();
    if (!navIsHealthyAt(nowMs)) {
        return 0;
    }

    // While dead reckoning, the live ground speed is stale (or zero); use the last good one.
    const float groundSpeedCms = navGpsSampleGood(nowMs) ? gpsSol.groundSpeed : est.speedCms;
    const float speedCms = fmaxf(groundSpeedCms, NAV_MIN_TURN_SPEED_CMS);
    // attitude.values are int16 decidegrees -- divide as float; DECIDEGREES_TO_RADIANS() would
    // integer-divide by 10 first.
    const float rollRad = DEGREES_TO_RADIANS(attitude.values.roll / 10.0f);
    const float pitchRad = DEGREES_TO_RADIANS(attitude.values.pitch / 10.0f);
    const float yawRateDps = RADIANS_TO_DEGREES(NAV_GRAVITY_CMSS * sin_approx(rollRad) * cos_approx(pitchRad) / speedCms);

    return -gain * constrainf(yawRateDps, -NAV_MAX_TURN_YAW_RATE_DPS, NAV_MAX_TURN_YAW_RATE_DPS);
}

// Called every GPS task tick, whether or not nav is engaged, so the estimate is current by the
// time LOITER/RTH is switched on.
void updateGpsNav(void)
{
    const timeMs_t nowMs = millis();

    navSampleGps(nowMs);

    if (!nav.active) {
        return;
    }

    const float dtS = (nowMs - nav.lastUpdateMs) / 1000.0f;
    nav.lastUpdateMs = nowMs;
    const float maxStepDdeg = NAV_BANK_SLEW_DDEG_PER_S * fmaxf(dtS, 0.0f);

    if (!nav.targetSet) {
        navCaptureTarget(nowMs);
    }

    if (!nav.targetSet || !navIsHealthyAt(nowMs)) {
        // Ease the bank back to level (plain Angle-mode leveling) and command no pitch, but leave
        // nav.active/target set rather than navStop()-ing: core.c's RTH_MODE/LOITER_MODE latch only
        // restarts nav on the switch's off->on edge, so this resumes toward the original target
        // the moment the position is usable again.
        nav.bankDdeg += constrainf(-nav.bankDdeg, -maxStepDdeg, maxStepDdeg);
        navAngle[AI_ROLL] = lrintf(nav.bankDdeg * 10.0f);
        navAngle[AI_PITCH] = 0;
        return;
    }

    navPosition_t pos = navEstimatePosition(nowMs);

    uint32_t distCm;
    int32_t bearingCdeg; // centidegrees
    GPS_distance_cm_bearing(&pos.lat, &pos.lon, &nav.targetLat, &nav.targetLon, &distCm, &bearingCdeg);

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

    const int32_t trackErrorDdeg = wrapBearingErrorDdeg(desiredTrackDdeg - pos.courseDdeg);

    // Feedforward: the bank a coordinated turn needs to follow the circle's curvature at the
    // current speed, atan(V^2 / (g * R)), faded out by cos(offset) as the desired track swings
    // away from the tangent. Outside the circle the field's curves widen with distance, so use
    // the larger of distance and radius. The P term then only corrects the residual error rather
    // than supplying the whole turn, so it stays off the bank limit on a flyable radius.
    const float speedCms = fmaxf(est.speedCms, NAV_MIN_TURN_SPEED_CMS);
    const float curveRadiusCm = fmaxf((float)distCm, radiusCm);
    const float feedforwardDdeg = orbitSign * cos_approx(convergeRad)
        * RADIANS_TO_DEGREES(atan2_approx(speedCms * speedCms, NAV_GRAVITY_CMSS * curveRadiusCm)) * 10.0f;

    const float bearingKp = gpsNavConfig()->bearingKp / 100.0f;
    const float maxBankDdeg = gpsNavConfig()->maxBankAngleDeg * 10.0f;
    const float bankTargetDdeg = constrainf(feedforwardDdeg + bearingKp * trackErrorDdeg, -maxBankDdeg, maxBankDdeg);

    nav.bankDdeg += constrainf(bankTargetDdeg - nav.bankDdeg, -maxStepDdeg, maxStepDdeg);
    navAngle[AI_ROLL] = lrintf(nav.bankDdeg * 10.0f); // decidegrees -> centidegrees

    // No real altitude (no baro, and GPS altitude not passing position_gps_min_sats): the
    // estimate reads 0, which would look like far below any target and pitch full nose-up --
    // on a GPS-only board, for the whole of a dead-reckoned dropout. Hold level pitch instead.
    if (!hasEstimatedAltitude()) {
        navAngle[AI_PITCH] = 0;
        return;
    }
    if (!nav.targetAltitudeSet) {
        nav.targetAltitudeCm = getEstimatedAltitudeCm();
        nav.targetAltitudeSet = true;
    }

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

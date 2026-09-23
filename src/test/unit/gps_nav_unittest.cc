/*
 * This file is part of Wingflight.
 *
 * Wingflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Wingflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 */

// Loiter orbit direction. The aircraft is put on each side of the target, inside the loiter
// radius, and flown along the tangent that a clockwise (or anticlockwise) orbit needs. Flying
// the right tangent needs no correction; the opposite tangent needs a full correction.

#include <cmath>

#include "gtest/gtest.h"

extern "C" {
#include "common/axis.h"
#include "fc/runtime_config.h"
#include "flight/gps_nav.h"
#include "io/gps.h"
#include "pg/gps_nav.h"
#include "pg/pg.h"
}

extern "C" {
// Stubs for what gps_nav.c pulls in.
gpsSolutionData_t gpsSol;
int32_t GPS_home[2];
bool gpsIsHealthy(void) { return true; }
// STATE(GPS_FIX_HOME) (navCanRTH()) reads this global directly, same as the real firmware.
uint8_t stateFlags = 0;
// Settable by tests directly, same pattern as gpsSol.llh.lat/lon below.
int32_t stubAltitudeCm = 0;
int getEstimatedAltitudeCm(void) { return stubAltitudeCm; }

// Same contract as io/gps.c: the bearing FROM the current position TO the destination, in
// centidegrees clockwise from north, in [0, 36000). Positions are 1e-7 degrees. Flat-earth with
// no longitude scaling, which is exact on the equator where these tests are placed.
void GPS_distance_cm_bearing(int32_t *currentLat1, int32_t *currentLon1, int32_t *destinationLat2,
                             int32_t *destinationLon2, uint32_t *dist, int32_t *bearing)
{
    const double north = *destinationLat2 - *currentLat1;
    const double east = *destinationLon2 - *currentLon1;
    *dist = static_cast<uint32_t>(std::sqrt(north * north + east * east) * 1.1119);
    // M_PI isn't standard C++ (only guaranteed under platform-specific feature-test macros,
    // e.g. missing by default on MinGW's <cmath>), so use a literal instead.
    constexpr double kPi = 3.14159265358979323846;
    double degrees = std::atan2(east, north) * 180.0 / kPi;
    if (degrees < 0) {
        degrees += 360.0;
    }
    *bearing = static_cast<int32_t>(std::lround(degrees * 100.0)) % 36000;
}
}

namespace {

// 1e-7 degrees per meter at the equator: 1 degree of latitude is 111.19 km.
constexpr int32_t UNITS_PER_METER = 90;

constexpr int32_t NORTH = 0, EAST = 900, SOUTH = 1800, WEST = 2700;       // decidegrees

struct Side {
    const char *name;
    int32_t dLat;       // where the aircraft is relative to the target, in 1e-7 degrees
    int32_t dLon;
    int32_t clockwise;  // the ground course a clockwise orbit needs there, in decidegrees
};

// Seen from above with north up: clockwise means south of the target flies west, west flies
// north, north flies east, east flies south.
const Side SIDES[] = {
    { "south of target", -40 * UNITS_PER_METER, 0, WEST },
    { "west of target", 0, -40 * UNITS_PER_METER, NORTH },
    { "north of target", 40 * UNITS_PER_METER, 0, EAST },
    { "east of target", 0, 40 * UNITS_PER_METER, SOUTH },
};

int32_t opposite(int32_t decidegrees)
{
    return (decidegrees + 1800) % 3600;
}

class GpsNavLoiterTest : public ::testing::Test {
  protected:
    void SetUp() override
    {
        pgResetAll();
        gpsNavConfigMutable()->loiterRadiusM = 75;
        gpsNavConfigMutable()->minSats = 6;
        gpsNavConfigMutable()->maxBankAngleDeg = 30;
        gpsNavConfigMutable()->bearingKp = 100;
        gpsSol.numSat = 12;
        gpsSol.llh.lat = 0;
        gpsSol.llh.lon = 0;
        stateFlags = GPS_FIX; // navIsHealthy() requires a fix, not just numSat/link health
    }

    // Start loitering about the origin, then put the aircraft on `side` flying `course`.
    // Returns the roll command in centidegrees; positive is a turn to the right.
    int32_t rollAt(const Side &side, uint8_t direction, int32_t course)
    {
        gpsNavConfigMutable()->loiterDirection = direction;
        gpsSol.llh.lat = 0;
        gpsSol.llh.lon = 0;
        navLoiterStart();

        gpsSol.llh.lat = side.dLat;
        gpsSol.llh.lon = side.dLon;
        gpsSol.groundCourse = static_cast<uint16_t>(course);
        updateGpsNav();
        return navAngle[AI_ROLL];
    }
};

TEST_F(GpsNavLoiterTest, ClockwiseFliesTheClockwiseTangentWithoutCorrection)
{
    for (const Side &side : SIDES) {
        EXPECT_EQ(0, rollAt(side, NAV_LOITER_CW, side.clockwise)) << side.name;
    }
}

TEST_F(GpsNavLoiterTest, AnticlockwiseFliesTheAnticlockwiseTangentWithoutCorrection)
{
    for (const Side &side : SIDES) {
        EXPECT_EQ(0, rollAt(side, NAV_LOITER_CCW, opposite(side.clockwise))) << side.name;
    }
}

TEST_F(GpsNavLoiterTest, ClockwiseTurnsAnAircraftFlyingTheWrongWayAround)
{
    // Flying the anticlockwise tangent while a clockwise orbit is wanted is a 180 degree error,
    // which saturates the bank command at the limit.
    for (const Side &side : SIDES) {
        EXPECT_EQ(30 * 100, std::abs(rollAt(side, NAV_LOITER_CW, opposite(side.clockwise)))) << side.name;
    }
}

TEST_F(GpsNavLoiterTest, ClockwiseBanksRightWhenTheCourseIsLeftOfTheTangent)
{
    // South of the target a clockwise orbit flies west. Heading south-west is left of west, so
    // the aircraft has to turn right to reach it.
    EXPECT_GT(rollAt(SIDES[0], NAV_LOITER_CW, 2250), 0);
    // Heading north-west is right of west, so it has to turn left.
    EXPECT_LT(rollAt(SIDES[0], NAV_LOITER_CW, 3150), 0);
}

TEST_F(GpsNavLoiterTest, TheApproachIgnoresTheLoiterDirection)
{
    // Outside the radius the aircraft heads straight for the target whichever way it will orbit.
    const Side farSouth = { "150 m south", -150 * UNITS_PER_METER, 0, WEST };
    EXPECT_EQ(0, rollAt(farSouth, NAV_LOITER_CW, NORTH));
    EXPECT_EQ(0, rollAt(farSouth, NAV_LOITER_CCW, NORTH));
}

// Altitude hold sign (regression test for H-3, see Flight Dynamics tech reference). Pitch in
// this codebase's convention is positive NOSE-DOWN, same as attitude.raw[]/navAngle[] throughout
// (see gps_nav.c's own comment, cross-referenced against autohover.c's bench-confirmed +900 =
// nose-down / -900 = nose-up). Below target altitude must command a negative (nose-up, climb)
// pitch target; above target must command positive (nose-down, descend).
class GpsNavAltitudeTest : public ::testing::Test {
  protected:
    void SetUp() override
    {
        pgResetAll();
        gpsNavConfigMutable()->minSats = 6;
        gpsNavConfigMutable()->maxPitchAngleDeg = 15;
        gpsNavConfigMutable()->altitudeKp = 100; // 1.0 deg pitch per meter of altitude error
        gpsNavConfigMutable()->rthAltitudeM = 50;
        gpsSol.numSat = 12;
        gpsSol.llh.lat = 0;
        gpsSol.llh.lon = 0;
        GPS_home[GPS_LATITUDE] = 0;
        GPS_home[GPS_LONGITUDE] = 0;
        stubAltitudeCm = 0;
        stateFlags = GPS_FIX; // navIsHealthy() requires a fix, not just numSat/link health
    }

    // Starts an RTH toward GPS_home (0,0) with the configured rthAltitudeM as target, then
    // reports the current altitude as currentAltitudeM and returns the resulting pitch command
    // in centidegrees.
    int32_t pitchAt(int32_t currentAltitudeM)
    {
        navRthStart();
        stubAltitudeCm = currentAltitudeM * 100;
        updateGpsNav();
        return navAngle[AI_PITCH];
    }
};

TEST_F(GpsNavAltitudeTest, BelowTargetCommandsNoseUp)
{
    // 50 m target, aircraft at 30 m: 20 m below, needs to climb -> negative (nose-up) pitch.
    EXPECT_LT(pitchAt(30), 0);
}

TEST_F(GpsNavAltitudeTest, AboveTargetCommandsNoseDown)
{
    // 50 m target, aircraft at 70 m: 20 m above, needs to descend -> positive (nose-down) pitch.
    EXPECT_GT(pitchAt(70), 0);
}

TEST_F(GpsNavAltitudeTest, AtTargetCommandsLevelPitch)
{
    EXPECT_EQ(0, pitchAt(50));
}

TEST_F(GpsNavAltitudeTest, PitchIsClampedToMaxPitchAngleDeg)
{
    // Grossly below target: clamped to -maxPitchAngleDeg (15 deg = 1500 centidegrees).
    EXPECT_EQ(-1500, pitchAt(-1000));
    // Grossly above target: clamped to +maxPitchAngleDeg.
    EXPECT_EQ(1500, pitchAt(1000));
}

// navIsHealthy()/nav.active health handling. UBLOX PVT sets gpsSol.numSat from the frame's
// numSV field unconditionally, independent of whether that frame carried a valid 3D fix (see
// gps.c's UBLOX_parse_gps()), so a receiver can report a healthy satellite count with
// STATE(GPS_FIX) false. Regression tests for that gap, and for the switch-latch bug where a
// temporary health loss left nav permanently disengaged until the pilot cycled the mode switch.
class GpsNavHealthTest : public ::testing::Test {
  protected:
    void SetUp() override
    {
        pgResetAll();
        gpsNavConfigMutable()->loiterRadiusM = 75;
        gpsNavConfigMutable()->minSats = 6;
        gpsNavConfigMutable()->maxBankAngleDeg = 30;
        gpsNavConfigMutable()->bearingKp = 100;
        gpsSol.numSat = 12;
        gpsSol.llh.lat = 0;
        gpsSol.llh.lon = 0;
        stateFlags = GPS_FIX;
    }

    // Puts the aircraft 40 m south of the loiter target, inside the radius, flying the wrong
    // way around a clockwise orbit -- the same "grossly wrong" setup GpsNavLoiterTest uses, so a
    // healthy update always commands a full-scale (nonzero, saturated) roll correction here.
    void placeAircraftForNonzeroRoll()
    {
        gpsNavConfigMutable()->loiterDirection = NAV_LOITER_CW;
        gpsSol.llh.lat = -40 * 90; // UNITS_PER_METER inlined: 1e-7 deg/m at the equator
        gpsSol.llh.lon = 0;
        gpsSol.groundCourse = EAST; // opposite of the CW tangent (WEST) at this position
    }
};

TEST_F(GpsNavHealthTest, HealthySatCountWithNoFixCommandsNothing)
{
    navLoiterStart();
    placeAircraftForNonzeroRoll();

    stateFlags = 0; // numSat stays 12 (well above minSats): link/sat-count alone must not pass
    updateGpsNav();

    EXPECT_EQ(0, navAngle[AI_ROLL]);
}

TEST_F(GpsNavHealthTest, ResumesTowardOriginalTargetAfterFixIsReacquired)
{
    navLoiterStart();
    placeAircraftForNonzeroRoll();

    updateGpsNav();
    const int32_t healthyRoll = navAngle[AI_ROLL];
    ASSERT_NE(0, healthyRoll) << "test setup should command a nonzero correction when healthy";

    // Fix lost mid-session (switch/mode stays engaged -- core.c never calls navLoiterStart()
    // again while the pilot leaves the switch on).
    stateFlags = 0;
    updateGpsNav();
    EXPECT_EQ(0, navAngle[AI_ROLL]) << "should command nothing while unhealthy, not a stale value";

    // Fix reacquired, aircraft hasn't moved, switch was never touched (no navLoiterStart() call
    // here): nav must resume toward the original target on its own.
    stateFlags = GPS_FIX;
    updateGpsNav();
    EXPECT_EQ(healthyRoll, navAngle[AI_ROLL]);
}

} // namespace

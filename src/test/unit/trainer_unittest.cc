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

#include <string.h>

#include "gtest/gtest.h"

extern "C" {
#include "build/debug.h"
#include "flight/trainer.h"
#include "flight/leveling.h"
#include "pg/rates.h"
#include "flight/imu.h"
#include "pg/accel.h"
#include "sensors/gyro.h"
}

extern "C" {
static uint8_t profileIndex;
static float stick[3];
static controlRateConfig_t rates;
controlRateConfig_t *currentControlRateProfile = &rates;
uint16_t flightModeFlags;
uint8_t getCurrentPidProfileIndex(void) { return profileIndex; }
float getDeflection(int axis) { return stick[axis]; }
bool isUpsidedown(void) { return false; }
bool isAirborne(void) { return true; }

// Globals that trainer.c reads
attitudeEulerAngles_t attitude = EULER_INITIALIZE;
gyro_t gyro = {};

// Globals required by headers
pidProfile_t *currentPidProfile;
uint8_t debugMode;
uint8_t debugAxis;
int32_t debug[DEBUG_VALUE_COUNT];
uint32_t __timing[DEBUG_VALUE_COUNT];

}

class AcroTrainerTest : public ::testing::Test {
  public:
    void SetUp() override {
        pgResetAll();
        profileIndex = 0;
        memset(stick, 0, sizeof(stick));
        currentPidProfile = pidProfilesMutable(0);

        // Configure trainer: gain=75 (7.5x), angle_limit=20, lookahead=50ms
        pidProfile_t *profile = pidProfilesMutable(0);
        profile->trainer.gain = 75;
        profile->trainer.angle_limit = 20;
        profile->trainer.lookahead_ms = 50;

        acroTrainerInit(profile);
        acroTrainerSetState(true);

        memset(&attitude, 0, sizeof(attitude));
        memset(&gyro, 0, sizeof(gyro));
    }

    void setAngle(int axis, float degrees) {
        attitude.raw[axis] = (int16_t)(degrees * 10);
    }
};

TEST_F(AcroTrainerTest, WithinLimitsPassthrough) {
    setAngle(FD_ROLL, 15.0f);

    EXPECT_FLOAT_EQ(acroTrainerApply(FD_ROLL, 100.0f), 100.0f);
}

TEST_F(AcroTrainerTest, PositiveOvershootCorrection) {
    setAngle(FD_ROLL, 25.0f);

    float result = acroTrainerApply(FD_ROLL, 100.0f);

    EXPECT_LT(result, 0);
}

TEST_F(AcroTrainerTest, NegativeOvershootCorrection) {
    setAngle(FD_ROLL, -25.0f);

    float result = acroTrainerApply(FD_ROLL, -100.0f);

    EXPECT_GT(result, 0);
}

TEST_F(AcroTrainerTest, CorrectionMagnitude) {
    setAngle(FD_ROLL, 25.0f);

    // correction = (20 * 1 - 25) * 7.5 = -37.5
    float result = acroTrainerApply(FD_ROLL, 0.0f);

    EXPECT_FLOAT_EQ(result, -37.5f);
}

TEST_F(AcroTrainerTest, PilotHelpsPositiveOvershoot) {
    setAngle(FD_ROLL, 25.0f);

    // Pilot pushes -100 (helping), correction is -37.5
    // MIN(-100, -37.5) = -100 -> pilot input preserved
    float result = acroTrainerApply(FD_ROLL, -100.0f);
    
    EXPECT_FLOAT_EQ(result, -100.0f);
}

TEST_F(AcroTrainerTest, PilotFightsPositiveOvershoot) {
    setAngle(FD_ROLL, 25.0f);

    // Pilot pushes +100 (fighting), correction is -37.5
    // MIN(+100, -37.5) = -37.5 -> correction overrides
    float result = acroTrainerApply(FD_ROLL, 100.0f);

    EXPECT_FLOAT_EQ(result, -37.5f);
}

TEST_F(AcroTrainerTest, PilotHelpsNegativeOvershoot) {
    setAngle(FD_ROLL, -25.0f);

    // Pilot pushes +100 (helping), correction is +37.5
    // MAX(+100, +37.5) = +100 -> pilot input preserved
    float result = acroTrainerApply(FD_ROLL, 100.0f);

    EXPECT_FLOAT_EQ(result, 100.0f);
}

TEST_F(AcroTrainerTest, PilotFightsNegativeOvershoot) {
    setAngle(FD_ROLL, -25.0f);

    // Pilot pushes -100 (fighting), correction is +37.5
    // MAX(-100, +37.5) = +37.5 -> correction overrides
    float result = acroTrainerApply(FD_ROLL, -100.0f);

    EXPECT_FLOAT_EQ(result, 37.5f);
}

TEST_F(AcroTrainerTest, SetpointLimitClamp) {
    setAngle(FD_ROLL, 180.0f);

    // correction = (20 - 180) * 7.5 = -1200, clamped to -1000
    float result = acroTrainerApply(FD_ROLL, 0.0f);

    EXPECT_FLOAT_EQ(result, -1000.0f);
}

TEST_F(AcroTrainerTest, YawPassthrough) {
    setAngle(FD_YAW, 90.0f);

    EXPECT_FLOAT_EQ(acroTrainerApply(FD_YAW, 200.0f), 200.0f);
}

TEST_F(AcroTrainerTest, PitchAxisCorrection) {
    setAngle(FD_PITCH, 25.0f);

    float result = acroTrainerApply(FD_PITCH, 100.0f);

    EXPECT_FLOAT_EQ(result, -37.5f);
}

TEST_F(AcroTrainerTest, InactivePassthrough) {
    acroTrainerSetState(false);
    setAngle(FD_ROLL, 90.0f);

    EXPECT_FLOAT_EQ(acroTrainerApply(FD_ROLL, 200.0f), 200.0f);
}

TEST_F(AcroTrainerTest, ZeroStickOvershootCorrects) {
    setAngle(FD_ROLL, 25.0f);

    // With setPoint=0: MIN(0, -37.5) = -37.5 -> correction applied
    float result = acroTrainerApply(FD_ROLL, 0.0f);

    EXPECT_FLOAT_EQ(result, -37.5f);
}

TEST_F(AcroTrainerTest, ExactBoundaryNoCorrection) {
    setAngle(FD_ROLL, 20.0f);

    float result = acroTrainerApply(FD_ROLL, 100.0f);

    EXPECT_FLOAT_EQ(result, 100.0f);
}

TEST_F(AcroTrainerTest, LookaheadTriggers) {
    setAngle(FD_ROLL, 18.0f);
    gyro.gyroADCf[FD_ROLL] = 400.0f;

    float result = acroTrainerApply(FD_ROLL, 400.0f);

    // checkInterval = constrainf(400/500, 0, 1) * 0.05 = 0.04
    // projectedAngle = 400 * 0.04 + 18 = 34
    // correction = limitf((20 - 34) * 7.5, 1000) = -105
    EXPECT_FLOAT_EQ(result, -105.0f);
}

TEST_F(AcroTrainerTest, LookaheadIgnoresOpposingStick) {
    setAngle(FD_ROLL, 18.0f);
    gyro.gyroADCf[FD_ROLL] = 400.0f;

    // Stick is negative - projectedAngleSign != Sign(setPoint)
    float result = acroTrainerApply(FD_ROLL, -100.0f);

    EXPECT_FLOAT_EQ(result, -100.0f);
}

TEST_F(AcroTrainerTest, LookaheadZeroStickNotLimited) {
    // When stick is centered (setPoint=0), Sign(0) returns -1.
    // projectedAngleSign is +1 for positive projected angle.
    // Condition (+1 == -1) is false, so no limiting is applied.
    // This is correct: centered stick is not driving into the limit.
    setAngle(FD_ROLL, 18.0f);
    gyro.gyroADCf[FD_ROLL] = 400.0f;

    float result = acroTrainerApply(FD_ROLL, 0.0f);

    EXPECT_FLOAT_EQ(result, 0.0f);
}

TEST_F(AcroTrainerTest, ZeroLookaheadDisablesLookahead) {
    pidProfile_t *profile = pidProfilesMutable(0);
    profile->trainer.lookahead_ms = 0;
    acroTrainerInit(profile);
    acroTrainerSetState(true);

    // Within limits but high gyro rate -> would normally trigger lookahead
    setAngle(FD_ROLL, 18.0f);
    gyro.gyroADCf[FD_ROLL] = 400.0f;

    float result = acroTrainerApply(FD_ROLL, 400.0f);

    EXPECT_FLOAT_EQ(result, 400.0f);
}

TEST_F(AcroTrainerTest, IndependentRollAndPitchLimits)
{
    attitudeLimitsMutable(0)->trainer_roll = 60;
    attitudeLimitsMutable(0)->trainer_pitch = 25;
    acroTrainerInit(pidProfiles(0));
    setAngle(FD_ROLL, 40);
    setAngle(FD_PITCH, 40);
    EXPECT_FLOAT_EQ(acroTrainerApply(FD_ROLL, 100), 100);
    EXPECT_LT(acroTrainerApply(FD_PITCH, 100), 0);
    setAngle(FD_PITCH, -40);
    EXPECT_GT(acroTrainerApply(FD_PITCH, -100), 0);
}

TEST_F(AcroTrainerTest, AxisOverridesAreBoundedAndZeroPreservesLegacy)
{
    EXPECT_EQ(attitudeLimitDegrees(255, 20, FD_ROLL), 90);
    EXPECT_EQ(attitudeLimitDegrees(255, 20, FD_PITCH), 75);
    EXPECT_EQ(attitudeLimitDegrees(1, 20, FD_PITCH), 10);
    EXPECT_EQ(attitudeLimitDegrees(0, 80, FD_PITCH), 80);
    EXPECT_EQ(attitudeLimitDegrees(0, 20, FD_ROLL), 20);
}

TEST_F(AcroTrainerTest, AngleDemandUsesIndependentAxisLimits)
{
    pidProfile_t *profile = pidProfilesMutable(0);
    profile->angle.level_strength = 10;
    attitudeLimitsMutable(0)->angle_roll = 60;
    attitudeLimitsMutable(0)->angle_pitch = 25;
    levelingInit(profile);
    stick[FD_ROLL] = 1;
    stick[FD_PITCH] = 1;
    EXPECT_FLOAT_EQ(angleModeApply(FD_ROLL, 0), 60);
    EXPECT_FLOAT_EQ(angleModeApply(FD_PITCH, 0), 25);
    stick[FD_PITCH] = -1;
    EXPECT_FLOAT_EQ(angleModeApply(FD_PITCH, 0), -25);
    stick[FD_ROLL] = 0;
    setAngle(FD_ROLL, 30);
    EXPECT_FLOAT_EQ(angleModeApply(FD_ROLL, 0), -30);
}

TEST_F(AcroTrainerTest, AxisLimitsFollowSelectedProfileAndKeepLegacyDefaults)
{
    attitudeLimitsMutable(0)->trainer_roll = 60;
    attitudeLimitsMutable(1)->trainer_roll = 30;
    profileIndex = 1;
    acroTrainerInit(pidProfiles(1));
    setAngle(FD_ROLL, 40);
    EXPECT_LT(acroTrainerApply(FD_ROLL, 100), 0);
    profileIndex = 0;
    acroTrainerInit(pidProfiles(0));
    EXPECT_FLOAT_EQ(acroTrainerApply(FD_ROLL, 100), 100);
    EXPECT_EQ(attitudeLimits(0)->trainer_pitch, 0);
    EXPECT_EQ(pidProfiles(0)->trainer.angle_limit, 20);
}

TEST_F(AcroTrainerTest, LimiterStateTracksEachAxisIndependently)
{
    setAngle(FD_ROLL, 25);
    setAngle(FD_PITCH, 10);
    acroTrainerApply(FD_ROLL, 100);
    acroTrainerApply(FD_PITCH, 100);
    EXPECT_TRUE(acroTrainerIsLimiting(FD_ROLL));
    EXPECT_FALSE(acroTrainerIsLimiting(FD_PITCH));
    EXPECT_FALSE(acroTrainerIsLimiting(FD_YAW));
    setAngle(FD_ROLL, 10);
    EXPECT_FLOAT_EQ(acroTrainerApply(FD_ROLL, 100), 100);
    EXPECT_FALSE(acroTrainerIsLimiting(FD_ROLL));
}

TEST_F(AcroTrainerTest, HelpingPilotInputDoesNotRetainLimiterState)
{
    for (int sign : {-1, 1}) {
        setAngle(FD_ROLL, sign * 25);
        acroTrainerApply(FD_ROLL, sign * 100);
        EXPECT_TRUE(acroTrainerIsLimiting(FD_ROLL));
        EXPECT_FLOAT_EQ(acroTrainerApply(FD_ROLL, -sign * 100), -sign * 100);
        EXPECT_FALSE(acroTrainerIsLimiting(FD_ROLL));
    }
}

TEST_F(AcroTrainerTest, LookaheadReportsActualIntervention)
{
    for (int sign : {-1, 1}) {
        setAngle(FD_ROLL, sign * 18);
        gyro.gyroADCf[FD_ROLL] = sign * 400;
        acroTrainerApply(FD_ROLL, sign * 400);
        EXPECT_TRUE(acroTrainerIsLimiting(FD_ROLL));
        acroTrainerApply(FD_ROLL, -sign * 400);
        EXPECT_FALSE(acroTrainerIsLimiting(FD_ROLL));
    }
}

TEST_F(AcroTrainerTest, ModeExitClearsLimiterStateBeforeReentry)
{
    setAngle(FD_ROLL, 25);
    acroTrainerApply(FD_ROLL, 100);
    EXPECT_TRUE(acroTrainerIsLimiting(FD_ROLL));
    acroTrainerSetState(false);
    EXPECT_FALSE(acroTrainerIsLimiting(FD_ROLL));
    acroTrainerSetState(true);
    EXPECT_FALSE(acroTrainerIsLimiting(FD_ROLL));
}

TEST_F(AcroTrainerTest, ProfileReloadClearsLimiterState)
{
    setAngle(FD_PITCH, -25);
    acroTrainerApply(FD_PITCH, -100);
    EXPECT_TRUE(acroTrainerIsLimiting(FD_PITCH));
    acroTrainerInit(pidProfiles(0));
    EXPECT_FALSE(acroTrainerIsLimiting(FD_PITCH));
}

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

// Vertical accelerometer/altitude fusion (flight/alt_fusion.c). Each test flies a known vertical
// trajectory and feeds the filter what the sensors would report.

#include <cmath>

#include "gtest/gtest.h"

extern "C" {
#include "flight/alt_fusion.h"
}

namespace {

constexpr float DT = 1.0f / 1000.0f;   // 1 kHz, a typical PID loop rate
constexpr float TAU = 2.0f;

class AltFusionTest : public ::testing::Test {
  protected:
    altFusion_t f;

    void SetUp() override
    {
        altFusionInit(&f);
    }

    // Run for `seconds` with the true vertical acceleration `trueAcc`, starting from the true
    // altitude/velocity in trueAlt/trueVel (updated in place). The accelerometer reads trueAcc +
    // accError; the measurement (when present) reads the true altitude.
    void fly(float seconds, float trueAcc, float &trueAlt, float &trueVel, bool haveMeas = true,
             float accError = 0, bool haveAcc = true)
    {
        const int steps = static_cast<int>(std::lround(seconds / DT));
        for (int i = 0; i < steps; i++) {
            trueAlt += trueVel * DT + 0.5f * trueAcc * DT * DT;
            trueVel += trueAcc * DT;
            altFusionUpdate(&f, DT, haveAcc, trueAcc + accError, haveMeas, trueAlt, TAU);
        }
    }
};

TEST_F(AltFusionTest, InvalidUntilTheFirstMeasurement)
{
    EXPECT_FALSE(f.valid);
    altFusionUpdate(&f, DT, true, 0, false, 0, TAU);
    EXPECT_FALSE(f.valid);
}

TEST_F(AltFusionTest, StartsAtTheFirstMeasurement)
{
    // No slow convergence from 0 up to the altitude of the first measurement.
    altFusionUpdate(&f, DT, true, 0, true, 120.0f, TAU);
    EXPECT_TRUE(f.valid);
    EXPECT_NEAR(120.0f, f.altitude, 0.01f);
    EXPECT_NEAR(0.0f, f.vario, 0.01f);
}

TEST_F(AltFusionTest, HoldsStillAtRest)
{
    float alt = 50, vel = 0;
    fly(30, 0, alt, vel);
    EXPECT_NEAR(50.0f, f.altitude, 0.01f);
    EXPECT_NEAR(0.0f, f.vario, 0.01f);
}

TEST_F(AltFusionTest, VarioFollowsAClimbWithoutLag)
{
    // Pull up into a 5 m/s climb over half a second. With the accelerometer carrying the short
    // term, vario tracks the true climb rate the whole way -- the old baro-derivative vario lagged
    // by its filter.
    float alt = 0, vel = 0;
    fly(1, 0, alt, vel);
    fly(0.5f, 10.0f, alt, vel);
    EXPECT_NEAR(5.0f, vel, 0.01f);
    EXPECT_NEAR(vel, f.vario, 0.05f);
    EXPECT_NEAR(alt, f.altitude, 0.05f);

    fly(10, 0, alt, vel);
    EXPECT_NEAR(5.0f, f.vario, 0.05f);
    EXPECT_NEAR(alt, f.altitude, 0.05f);
}

TEST_F(AltFusionTest, LearnsAConstantAccelerometerError)
{
    // The accelerometer reads 0.5 m/s^2 high (calibration, or IMU attitude error in a long
    // turn). The measurement says nothing is moving; over a few time constants the filter learns
    // the error as bias and settles back on the truth instead of drifting.
    float alt = 100, vel = 0;
    fly(60, 0, alt, vel, true, 0.5f);
    EXPECT_NEAR(0.5f, f.accBias, 0.01f);
    EXPECT_NEAR(100.0f, f.altitude, 0.05f);
    EXPECT_NEAR(0.0f, f.vario, 0.05f);
}

TEST_F(AltFusionTest, AnAccelerometerErrorIsBoundedWhileItIsLearned)
{
    // Before the bias is learned the error shows as a transient. For a 0.5 m/s^2 step it must
    // stay small -- the measurement holds it within a fraction of a metre.
    float alt = 100, vel = 0;
    fly(1, 0, alt, vel);
    float worst = 0;
    for (int i = 0; i < 300; i++) {
        fly(0.1f, 0, alt, vel, true, 0.5f);
        worst = std::fmax(worst, std::fabs(f.altitude - alt));
    }
    EXPECT_LT(worst, 1.5f);
}

TEST_F(AltFusionTest, CoastsOnTheAccelerometerThroughAShortDropout)
{
    // GPS-only board, GPS altitude lost for 3 s in a steady 2 m/s climb: the estimate carries
    // on at the climb rate and stays valid.
    float alt = 0, vel = 0;
    fly(1, 0, alt, vel);
    fly(1, 2.0f, alt, vel);
    fly(10, 0, alt, vel);
    fly(3, 0, alt, vel, false);
    EXPECT_TRUE(f.valid);
    EXPECT_NEAR(alt, f.altitude, 0.5f);
    EXPECT_NEAR(2.0f, f.vario, 0.1f);
}

TEST_F(AltFusionTest, ExpiresAfterTheCoastWindowAndHoldsTheLastAltitude)
{
    float alt = 80, vel = 0;
    fly(5, 0, alt, vel);
    fly(ALT_FUSION_COAST_S + 0.5f, 0, alt, vel, false);
    EXPECT_FALSE(f.valid);
    EXPECT_NEAR(80.0f, f.altitude, 0.5f) << "telemetry keeps the last altitude, not 0";
    EXPECT_EQ(0.0f, f.vario);
}

TEST_F(AltFusionTest, RestartsFromTheMeasurementAfterExpiring)
{
    float alt = 80, vel = 0;
    fly(5, 0, alt, vel);
    fly(ALT_FUSION_COAST_S + 0.5f, 0, alt, vel, false);
    ASSERT_FALSE(f.valid);

    // The measurement comes back at a quite different altitude: take it straight away.
    altFusionUpdate(&f, DT, true, 0, true, 140.0f, TAU);
    EXPECT_TRUE(f.valid);
    EXPECT_NEAR(140.0f, f.altitude, 0.01f);
}

TEST_F(AltFusionTest, WithoutAnAccelerometerItTracksTheMeasurement)
{
    float alt = 0, vel = 3.0f;
    fly(20, 0, alt, vel, true, 0, false);
    EXPECT_NEAR(alt, f.altitude, 0.1f);
    EXPECT_NEAR(3.0f, f.vario, 0.05f);

    // ...and with nothing to coast on, is invalid the moment the measurement goes.
    altFusionUpdate(&f, DT, false, 0, false, 0, TAU);
    EXPECT_FALSE(f.valid);
}

TEST_F(AltFusionTest, AnImplausibleAccelerometerSpikeIsClipped)
{
    // 0.1 s of 20 g garbage on the vertical axis (a snap, or a lost attitude estimate): clipped
    // at about 3 g, so the estimate moves by well under the 20 m/s it would otherwise gain.
    float alt = 50, vel = 0;
    fly(5, 0, alt, vel);
    for (int i = 0; i < 100; i++) {
        altFusionUpdate(&f, DT, true, 200.0f, true, 50.0f, TAU);
    }
    EXPECT_LT(f.vario, 3.5f);
    EXPECT_LT(f.altitude, 50.2f);
}

} // namespace

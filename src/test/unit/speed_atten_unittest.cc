/*
 * This file is part of Wingflight.
 *
 * Wingflight is free software. You can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Wingflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software. If not, see <https://www.gnu.org/licenses/>.
 */

#include "gtest/gtest.h"

extern "C" {
#include "fc/runtime_config.h"
#include "flight/speed_atten.h"
#include "io/gps.h"

uint8_t stateFlags;
gpsSolutionData_t gpsSol;
static bool gpsSensor;
bool sensors(uint32_t) { return gpsSensor; }

// Stand-in curve: 100% at rest falling linearly to 50% at speed_max and beyond
float pidGetGainCurveScaleAt(uint8_t curveIndex, float position)
{
    if (!curveIndex)
        return 1.0f;
    if (position > 1.0f)
        position = 1.0f;
    return 1.0f - 0.5f * position;
}
}

static const float RATE = 1000.0f;
static const float DT = 1.0f / RATE;

class SpeedAttenTest : public ::testing::Test {
protected:
    fwSpaConfig_t config;

    void SetUp() override
    {
        config.gain = 100;
        config.curve = 1;
        config.speed_max = 200;
        gpsSensor = true;
        stateFlags = 0;
        gpsSol = {};
        speedAttenReset();
        speedAttenInit(&config, RATE);
    }

    void run(bool fix, float speedKmh, float seconds)
    {
        const int steps = (int)(seconds * RATE + 0.5f);
        for (int i = 0; i < steps; i++)
            speedAttenUpdateFrom(fix, speedKmh, DT);
    }
};

TEST_F(SpeedAttenTest, NoCurveIsExactlyUnity)
{
    config.curve = 0;
    speedAttenInit(&config, RATE);
    run(true, 200, 5);
    EXPECT_FLOAT_EQ(1.0f, speedAttenGetScale());
    EXPECT_FALSE(speedAttenIsEnabled());
}

TEST_F(SpeedAttenTest, OutOfRangeCurveIsOff)
{
    config.curve = GAIN_CURVE_COUNT + 1;
    speedAttenInit(&config, RATE);
    run(true, 200, 5);
    EXPECT_FLOAT_EQ(1.0f, speedAttenGetScale());
}

TEST_F(SpeedAttenTest, EasesOntoCurveThenTracks)
{
    // Target at speed_max is 0.5; easing at 0.2/s takes 2.5 s
    run(true, 200, 1);
    EXPECT_NEAR(0.8f, speedAttenGetScale(), 0.01f);
    run(true, 200, 2);
    EXPECT_NEAR(0.5f, speedAttenGetScale(), 0.001f);

    // Tracking now: a speed change shows through the 1 Hz filter, not the slew limit
    run(true, 100, 1);
    EXPECT_LT(speedAttenGetScale(), 0.76f);
    EXPECT_GT(speedAttenGetScale(), 0.73f);
}

TEST_F(SpeedAttenTest, FirstReadingSeedsFilter)
{
    run(true, 200, 0.001f);
    EXPECT_NEAR(200.0f, speedAttenGetSpeed(), 0.01f);
}

TEST_F(SpeedAttenTest, GainScalesCurve)
{
    config.gain = 80;
    speedAttenInit(&config, RATE);
    run(true, 0, 5);
    EXPECT_NEAR(0.8f, speedAttenGetScale(), 0.001f);
}

TEST_F(SpeedAttenTest, FixLossHoldsThenEasesBack)
{
    run(true, 200, 5);
    ASSERT_NEAR(0.5f, speedAttenGetScale(), 0.001f);

    // Held for the hold time
    run(false, 0, FW_SPA_HOLD_TIME_S - 0.1f);
    EXPECT_NEAR(0.5f, speedAttenGetScale(), 0.001f);

    // Then eases at the slew rate, not a step
    run(false, 0, 1.1f);
    EXPECT_NEAR(0.7f, speedAttenGetScale(), 0.01f);

    run(false, 0, 5);
    EXPECT_FLOAT_EQ(1.0f, speedAttenGetScale());
}

TEST_F(SpeedAttenTest, BriefDropoutKeepsScale)
{
    run(true, 200, 5);
    run(false, 0, 1);
    run(true, 200, 0.1f);
    EXPECT_NEAR(0.5f, speedAttenGetScale(), 0.001f);
}

TEST_F(SpeedAttenTest, UpdateReadsGps)
{
    // No fix: stays at unity
    gpsSol.groundSpeed = 5556; // 200 km/h in cm/s
    speedAttenUpdate(DT);
    EXPECT_FLOAT_EQ(1.0f, speedAttenGetScale());

    // Fix: uses the larger of 3D and ground speed
    stateFlags = GPS_FIX;
    gpsSol.speed3d = 2778;
    speedAttenUpdate(DT);
    EXPECT_NEAR(200.0f, speedAttenGetSpeed(), 0.1f);

    // GPS sensor not present counts as no fix
    gpsSensor = false;
    for (int i = 0; i < (int)(10 * RATE); i++)
        speedAttenUpdate(DT);
    EXPECT_FLOAT_EQ(1.0f, speedAttenGetScale());
}

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

#include <math.h>

#include "gtest/gtest.h"

extern "C" {
#include "flight/auto_gain.h"
}

static const float RATE = 1000.0f;
static const float DT = 1.0f / RATE;
static const float PI_F = 3.14159265f;

class AutoGainTest : public ::testing::Test {
protected:
    autoGainConfig_t config;
    float t;

    void SetUp() override
    {
        config.enable[PID_ROLL] = 1;
        config.enable[PID_PITCH] = 0;
        config.enable[PID_YAW] = 1;
        config.min = 40;
        config.threshold = 20;      // 2 % of travel
        config.recovery = 5;        // 5 %/s
        t = 0;
        autoGainReset();
        autoGainSetSwitch(false);
        autoGainInit(&config, RATE);
        quiet(1.0f);
    }

    // Feedback is a sine of this amplitude (surface travel) and frequency on every axis.
    // With loop set, the amplitude follows the roll scale, as it does in a real loop.
    void run(float seconds, float amplitude, float hz, float stickAmplitude = 0, bool active = true, bool loop = false)
    {
        const int steps = (int)(seconds * RATE + 0.5f);
        for (int i = 0; i < steps; i++) {
            const float s = sinf(2 * PI_F * hz * t);
            for (int axis = 0; axis < PID_AXIS_COUNT; axis++) {
                const float a = loop ? amplitude * autoGainGetScale(PID_ROLL) : amplitude;
                autoGainUpdate(axis, 0.1f + a * s, stickAmplitude * s, active, DT);
            }
            t += DT;
        }
    }

    void quiet(float seconds)
    {
        run(seconds, 0, 1);
    }
};

TEST_F(AutoGainTest, QuietFlightKeepsFullGain)
{
    quiet(10.0f);
    EXPECT_FLOAT_EQ(1.0f, autoGainGetScale(PID_ROLL));
    EXPECT_FLOAT_EQ(1.0f, autoGainGetScale(PID_YAW));
}

TEST_F(AutoGainTest, OscillationCutsGainToTheFloor)
{
    run(0.25f, 0.05f, 6);
    // Under two cycles: not yet
    EXPECT_FLOAT_EQ(1.0f, autoGainGetScale(PID_ROLL));

    run(0.5f, 0.05f, 6);
    EXPECT_LT(autoGainGetScale(PID_ROLL), 0.8f);

    run(3.0f, 0.05f, 6);
    EXPECT_FLOAT_EQ(0.4f, autoGainGetScale(PID_ROLL));
    EXPECT_FLOAT_EQ(0.4f, autoGainGetScale(PID_YAW));
}

TEST_F(AutoGainTest, DisabledAxisIsExactlyUnity)
{
    run(3.0f, 0.05f, 6);
    EXPECT_FALSE(autoGainIsEnabled(PID_PITCH));
    EXPECT_TRUE(autoGainIsEnabled(PID_ROLL));
    EXPECT_FLOAT_EQ(1.0f, autoGainGetScale(PID_PITCH));
}

TEST_F(AutoGainTest, ModeSwitchTurnsEveryAxisOn)
{
    autoGainSetSwitch(true);
    EXPECT_TRUE(autoGainIsEnabled(PID_PITCH));
    run(3.0f, 0.05f, 6);
    EXPECT_FLOAT_EQ(0.4f, autoGainGetScale(PID_PITCH));

    // Off again: the axis that is off in the profile goes straight back to full gain
    autoGainSetSwitch(false);
    run(0.01f, 0.05f, 6);
    EXPECT_FLOAT_EQ(1.0f, autoGainGetScale(PID_PITCH));
    EXPECT_FLOAT_EQ(0.4f, autoGainGetScale(PID_ROLL));
}

TEST_F(AutoGainTest, SmallOscillationIsIgnored)
{
    run(5.0f, 0.015f, 6);
    EXPECT_FLOAT_EQ(1.0f, autoGainGetScale(PID_ROLL));
}

TEST_F(AutoGainTest, SlowSwingIsNotOscillation)
{
    run(5.0f, 0.3f, 1);
    EXPECT_FLOAT_EQ(1.0f, autoGainGetScale(PID_ROLL));
}

TEST_F(AutoGainTest, FastBuzzIsNotOscillation)
{
    // 40 Hz: 12.5 ms per half cycle, under AUTO_GAIN_HALF_PERIOD_MIN
    run(3.0f, 0.05f, 40);
    EXPECT_FLOAT_EQ(1.0f, autoGainGetScale(PID_ROLL));
}

TEST_F(AutoGainTest, OneGustAndReboundIsNotOscillation)
{
    for (int i = 0; i < 5; i++) {
        // One full cycle is two swings at most
        run(1.0f / 6, 0.2f, 6);
        quiet(1.0f);
    }
    EXPECT_FLOAT_EQ(1.0f, autoGainGetScale(PID_ROLL));
}

TEST_F(AutoGainTest, StickWagIsNotOscillation)
{
    run(3.0f, 0.2f, 4, 0.5f);
    EXPECT_FLOAT_EQ(1.0f, autoGainGetScale(PID_ROLL));

    // The same feedback with the stick still is cut
    quiet(1.0f);
    run(3.0f, 0.2f, 4);
    EXPECT_FLOAT_EQ(0.4f, autoGainGetScale(PID_ROLL));
}

TEST_F(AutoGainTest, HoldsThenRecovers)
{
    run(3.0f, 0.05f, 6);
    ASSERT_FLOAT_EQ(0.4f, autoGainGetScale(PID_ROLL));

    quiet(AUTO_GAIN_HOLD_TIME_S - 0.3f);
    EXPECT_FLOAT_EQ(0.4f, autoGainGetScale(PID_ROLL));

    // 5 %/s
    quiet(0.3f + 4.0f);
    EXPECT_NEAR(0.6f, autoGainGetScale(PID_ROLL), 0.02f);

    quiet(10.0f);
    EXPECT_FLOAT_EQ(1.0f, autoGainGetScale(PID_ROLL));
}

TEST_F(AutoGainTest, StopsCuttingOnceTheFeedbackIsBelowTheThreshold)
{
    // 3 % of travel at full gain against a 2 % threshold: a cut to about 2/3 ends it
    run(5.0f, 0.03f, 6, 0, true, true);
    EXPECT_LT(autoGainGetScale(PID_ROLL), 0.75f);
    EXPECT_GT(autoGainGetScale(PID_ROLL), 0.5f);
}

TEST_F(AutoGainTest, InactiveIsUnityAndCountsNothing)
{
    run(3.0f, 0.05f, 6);
    ASSERT_FLOAT_EQ(0.4f, autoGainGetScale(PID_ROLL));

    run(0.01f, 0.05f, 6, 0, false);
    EXPECT_FLOAT_EQ(1.0f, autoGainGetScale(PID_ROLL));

    run(3.0f, 0.05f, 6, 0, false);
    EXPECT_FLOAT_EQ(1.0f, autoGainGetScale(PID_ROLL));
}

TEST_F(AutoGainTest, ProfileChangeKeepsTheScaleAboveTheNewFloor)
{
    run(3.0f, 0.05f, 6);
    ASSERT_FLOAT_EQ(0.4f, autoGainGetScale(PID_ROLL));

    config.min = 70;
    config.enable[PID_YAW] = 0;
    autoGainInit(&config, RATE);
    EXPECT_FLOAT_EQ(0.7f, autoGainGetScale(PID_ROLL));
    EXPECT_FLOAT_EQ(1.0f, autoGainGetScale(PID_YAW));
}

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
#include "flight/trim_monitor.h"
}

static const float DT = 1.0f / TRIM_MONITOR_SAMPLE_HZ;

class TrimMonitorTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        trimMonitorReset();
    }

    static trimMonitorSample_t level(float roll, float pitch, float yaw)
    {
        trimMonitorSample_t s = {};
        s.valid = true;
        s.iterm[FD_ROLL] = roll;
        s.iterm[FD_PITCH] = pitch;
        s.iterm[FD_YAW] = yaw;
        return s;
    }

    static void fly(const trimMonitorSample_t &s, float seconds)
    {
        for (int i = 0; i < (int)(seconds * TRIM_MONITOR_SAMPLE_HZ); i++) {
            trimMonitorProcessSample(&s, DT);
        }
    }
};

TEST_F(TrimMonitorTest, InTrimByDefault)
{
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        EXPECT_EQ(TRIM_MONITOR_IN_TRIM, trimMonitorGetState(axis));
    }
}

TEST_F(TrimMonitorTest, SmallLoadIsInTrim)
{
    fly(level(0.03f, -0.03f, 0.03f), 20);

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        EXPECT_EQ(TRIM_MONITOR_IN_TRIM, trimMonitorGetState(axis));
    }
}

TEST_F(TrimMonitorTest, Directions)
{
    // Positive I: roll right, pitch nose down (stick forward), yaw left (yaw is negated).
    fly(level(0.1f, 0.1f, 0.1f), 10);
    EXPECT_EQ(TRIM_MONITOR_RIGHT_UP, trimMonitorGetState(FD_ROLL));
    EXPECT_EQ(TRIM_MONITOR_LEFT_DOWN, trimMonitorGetState(FD_PITCH));
    EXPECT_EQ(TRIM_MONITOR_LEFT_DOWN, trimMonitorGetState(FD_YAW));

    trimMonitorReset();
    fly(level(-0.1f, -0.1f, -0.1f), 10);
    EXPECT_EQ(TRIM_MONITOR_LEFT_DOWN, trimMonitorGetState(FD_ROLL));
    EXPECT_EQ(TRIM_MONITOR_RIGHT_UP, trimMonitorGetState(FD_PITCH));
    EXPECT_EQ(TRIM_MONITOR_RIGHT_UP, trimMonitorGetState(FD_YAW));
}

TEST_F(TrimMonitorTest, WaitsToSettle)
{
    fly(level(0.0f, -0.5f, 0.0f), TRIM_MONITOR_SETTLE - 0.5f);
    EXPECT_EQ(TRIM_MONITOR_IN_TRIM, trimMonitorGetState(FD_PITCH));

    fly(level(0.0f, -0.5f, 0.0f), 1.0f);
    EXPECT_EQ(TRIM_MONITOR_RIGHT_UP, trimMonitorGetState(FD_PITCH));
}

TEST_F(TrimMonitorTest, Hysteresis)
{
    fly(level(0.0f, -0.1f, 0.0f), 10);
    EXPECT_EQ(TRIM_MONITOR_RIGHT_UP, trimMonitorGetState(FD_PITCH));

    // Between OFF and ON: still reported.
    fly(level(0.0f, -0.03f, 0.0f), 20);
    EXPECT_EQ(TRIM_MONITOR_RIGHT_UP, trimMonitorGetState(FD_PITCH));

    // Trimmed out: clears.
    fly(level(0.0f, 0.0f, 0.0f), 20);
    EXPECT_EQ(TRIM_MONITOR_IN_TRIM, trimMonitorGetState(FD_PITCH));
}

TEST_F(TrimMonitorTest, IgnoresManoeuvres)
{
    // Large I only while the stick is in, the axis is rotating, saturated, or outside
    // qualifying flight: never reported.
    trimMonitorSample_t s = level(0.5f, 0.5f, 0.5f);

    s.deflection[FD_ROLL] = 0.3f;
    s.gyro[FD_PITCH] = 120.0f;
    s.saturated[FD_YAW] = true;
    fly(s, 20);
    EXPECT_EQ(TRIM_MONITOR_IN_TRIM, trimMonitorGetState(FD_ROLL));
    EXPECT_EQ(TRIM_MONITOR_IN_TRIM, trimMonitorGetState(FD_PITCH));
    EXPECT_EQ(TRIM_MONITOR_IN_TRIM, trimMonitorGetState(FD_YAW));

    s = level(0.5f, 0.5f, 0.5f);
    s.valid = false;
    fly(s, 20);
    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        EXPECT_EQ(TRIM_MONITOR_IN_TRIM, trimMonitorGetState(axis));
    }
}

TEST_F(TrimMonitorTest, RestartKeepsReportUntilSettled)
{
    fly(level(0.0f, -0.1f, 0.0f), 10);
    EXPECT_EQ(TRIM_MONITOR_RIGHT_UP, trimMonitorGetState(FD_PITCH));

    // Next flight, trimmed out on the ground: the old report holds until it has settled.
    trimMonitorRestart();
    EXPECT_EQ(TRIM_MONITOR_RIGHT_UP, trimMonitorGetState(FD_PITCH));
    fly(level(0.0f, 0.0f, 0.0f), TRIM_MONITOR_SETTLE - 0.5f);
    EXPECT_EQ(TRIM_MONITOR_RIGHT_UP, trimMonitorGetState(FD_PITCH));
    fly(level(0.0f, 0.0f, 0.0f), 1.0f);
    EXPECT_EQ(TRIM_MONITOR_IN_TRIM, trimMonitorGetState(FD_PITCH));
}

TEST_F(TrimMonitorTest, HeldOutsideQualifyingFlight)
{
    fly(level(0.1f, 0.0f, 0.0f), 10);
    EXPECT_EQ(TRIM_MONITOR_RIGHT_UP, trimMonitorGetState(FD_ROLL));

    // A loop or a roll doesn't clear the report.
    trimMonitorSample_t s = level(-0.5f, 0.0f, 0.0f);
    s.gyro[FD_ROLL] = 300.0f;
    fly(s, 5);
    EXPECT_EQ(TRIM_MONITOR_RIGHT_UP, trimMonitorGetState(FD_ROLL));
}

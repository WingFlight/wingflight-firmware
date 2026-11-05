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
#include "flight/reverse_flight.h"
}

static const float DT = 0.001f;         // 1 kHz PID loop
static const float HALF_RANGE = 459;    // 0.9 * rc_deflection 510

class ReverseFlightTest : public ::testing::Test {
protected:
    reverseFlightConfig_t config;

    void SetUp() override
    {
        config.throttle_mode = THROTTLE_MODE_BIDIRECTIONAL;
        config.throttle_deadband = 40;
        config.throttle_reverse_hysteresis = 50;
        config.throttle_reverse_limit = 100;
        config.reverse_motor_time = 80;
        config.reverse_time_constant = 400;
        config.reverse_surface_gain = 100;
        config.reverse_fade = 15;
        config.reverse_motor_dwell = 0;
        config.reverse_coupled_axes = 0;
        reverseFlightInit(&config);
    }

    // Holds the stick for the given time, armed and airborne
    void fly(float stickUs, float seconds, bool airborne = true)
    {
        const int steps = lrintf(seconds / DT);
        for (int i = 0; i < steps; i++) {
            const float thrust = reverseFlightMapStick(stickUs, HALF_RANGE);
            reverseFlightUpdate(thrust, true, airborne, DT);
        }
    }
};

TEST_F(ReverseFlightTest, NormalModeIsTransparent)
{
    config.throttle_mode = THROTTLE_MODE_NORMAL;
    reverseFlightInit(&config);

    reverseFlightUpdate(0.6f, true, true, DT);

    EXPECT_FALSE(reverseFlightIsEnabled());
    EXPECT_FLOAT_EQ(reverseFlightGetThrottle(), 0.6f);
    EXPECT_FLOAT_EQ(reverseFlightGetSurfaceFactor(), 1.0f);
    EXPECT_FLOAT_EQ(reverseFlightGetVectorFactor(), 1.0f);
    EXPECT_FALSE(reverseFlightJustReversed());
}

TEST_F(ReverseFlightTest, DeadbandStopsMotor)
{
    EXPECT_TRUE(reverseFlightStickInDeadband(0));
    EXPECT_TRUE(reverseFlightStickInDeadband(40));
    EXPECT_TRUE(reverseFlightStickInDeadband(-40));
    EXPECT_FALSE(reverseFlightStickInDeadband(41));

    EXPECT_FLOAT_EQ(reverseFlightMapStick(0, HALF_RANGE), 0.0f);
    EXPECT_FLOAT_EQ(reverseFlightMapStick(39, HALF_RANGE), 0.0f);
}

TEST_F(ReverseFlightTest, ThrustStartsFromZeroAtDeadbandEdge)
{
    EXPECT_NEAR(reverseFlightMapStick(41, HALF_RANGE), 1.0f / (HALF_RANGE - 40), 1e-5f);
    EXPECT_FLOAT_EQ(reverseFlightMapStick(HALF_RANGE, HALF_RANGE), 1.0f);
    EXPECT_FLOAT_EQ(reverseFlightMapStick(600, HALF_RANGE), 1.0f);
}

TEST_F(ReverseFlightTest, HysteresisHoldsDirection)
{
    // Latched forward: just past the reverse deadband edge is still zero thrust
    EXPECT_GT(reverseFlightMapStick(200, HALF_RANGE), 0.0f);
    EXPECT_FLOAT_EQ(reverseFlightMapStick(-50, HALF_RANGE), 0.0f);
    EXPECT_FLOAT_EQ(reverseFlightMapStick(-60, HALF_RANGE), 0.0f);

    // Past the flip point (40 + 50% = 60 us) it reverses
    EXPECT_LT(reverseFlightMapStick(-61, HALF_RANGE), 0.0f);

    // Now latched reverse: forward side needs +60 too
    EXPECT_FLOAT_EQ(reverseFlightMapStick(50, HALF_RANGE), 0.0f);
    EXPECT_LT(reverseFlightMapStick(-50, HALF_RANGE), 0.0f);
    EXPECT_GT(reverseFlightMapStick(61, HALF_RANGE), 0.0f);
}

TEST_F(ReverseFlightTest, ReverseLimit)
{
    config.throttle_reverse_limit = 50;
    reverseFlightInit(&config);

    EXPECT_FLOAT_EQ(reverseFlightMapStick(-HALF_RANGE, HALF_RANGE), -0.5f);
    EXPECT_FLOAT_EQ(reverseFlightMapStick(HALF_RANGE, HALF_RANGE), 1.0f);
}

TEST_F(ReverseFlightTest, DisarmedHasFullAuthority)
{
    reverseFlightUpdate(-0.5f, false, false, DT);

    EXPECT_EQ(reverseFlightGetState(), REVERSE_STATE_DISARMED);
    EXPECT_FLOAT_EQ(reverseFlightGetSurfaceFactor(), 1.0f);
    EXPECT_FLOAT_EQ(reverseFlightGetVectorFactor(), 1.0f);
    EXPECT_FLOAT_EQ(reverseFlightGetThrottle(), 0.0f);
}

TEST_F(ReverseFlightTest, GroundKeepsLatchedDirection)
{
    fly(0, 0.1f, false);
    EXPECT_EQ(reverseFlightGetState(), REVERSE_STATE_GROUND);
    EXPECT_FLOAT_EQ(reverseFlightGetSurfaceFactor(), 1.0f);
    EXPECT_FLOAT_EQ(reverseFlightGetVectorFactor(), 1.0f);
}

TEST_F(ReverseFlightTest, ForwardFlightSettlesAtFullAuthority)
{
    fly(300, 2.0f);
    EXPECT_EQ(reverseFlightGetState(), REVERSE_STATE_FORWARD);
    EXPECT_FLOAT_EQ(reverseFlightGetSurfaceFactor(), 1.0f);
    EXPECT_FLOAT_EQ(reverseFlightGetVectorFactor(), 1.0f);
    EXPECT_EQ(reverseFlightGetDirection(), 1);
}

// The core of the concept: from forward flight, snap the stick to reverse.
TEST_F(ReverseFlightTest, ReversalHandsOverFromSurfacesToGimbal)
{
    fly(362, 2.0f);     // ~70% forward, settled

    // Snap to ~70% reverse
    float thrust = reverseFlightMapStick(-362, HALF_RANGE);
    reverseFlightUpdate(thrust, true, true, DT);
    EXPECT_TRUE(reverseFlightJustReversed());
    EXPECT_EQ(reverseFlightGetDirection(), -1);
    EXPECT_LT(reverseFlightGetThrottle(), 0.0f);

    // Still moving forward: surfaces off, gimbal still pushing the old way (prop still turning forward)
    EXPECT_EQ(reverseFlightGetState(), REVERSE_STATE_HANDOVER);
    EXPECT_FLOAT_EQ(reverseFlightGetSurfaceFactor(), 0.0f);
    EXPECT_GT(reverseFlightGetVectorFactor(), 0.0f);

    // One cycle only
    reverseFlightUpdate(thrust, true, true, DT);
    EXPECT_FALSE(reverseFlightJustReversed());

    // The gimbal flips once the estimated thrust crosses zero: tau * ln 2 = 55 ms
    float gimbalFlip = -1, surfacesBack = -1;
    for (float t = 2 * DT; t < 1.5f; t += DT) {
        thrust = reverseFlightMapStick(-362, HALF_RANGE);
        reverseFlightUpdate(thrust, true, true, DT);
        if (gimbalFlip < 0 && reverseFlightGetVectorFactor() < 0)
            gimbalFlip = t;
        if (surfacesBack < 0 && reverseFlightGetSurfaceFactor() < 0)
            surfacesBack = t;
        // Never a wrong-sign surface during the handover
        EXPECT_LE(reverseFlightGetSurfaceFactor(), 0.0f);
    }
    EXPECT_NEAR(gimbalFlip, 0.080f * logf(2), 0.005f);

    // The surfaces come back reversed once the speed estimate crosses zero: tau * ln 2 = 277 ms
    EXPECT_NEAR(surfacesBack, 0.400f * logf(2), 0.01f);

    // Settled in reverse
    EXPECT_EQ(reverseFlightGetState(), REVERSE_STATE_REVERSE);
    EXPECT_FLOAT_EQ(reverseFlightGetSurfaceFactor(), -1.0f);
    EXPECT_FLOAT_EQ(reverseFlightGetVectorFactor(), -1.0f);
}

TEST_F(ReverseFlightTest, ReverseSurfaceGain)
{
    config.reverse_surface_gain = 150;
    reverseFlightInit(&config);

    fly(-300, 2.0f);
    EXPECT_FLOAT_EQ(reverseFlightGetSurfaceFactor(), -1.5f);
    EXPECT_FLOAT_EQ(reverseFlightGetVectorFactor(), -1.0f);
}

TEST_F(ReverseFlightTest, GimbalFadesOutWithMotorStopped)
{
    fly(300, 2.0f);
    fly(0, 1.0f);       // back to the deadband

    EXPECT_FLOAT_EQ(reverseFlightGetThrottle(), 0.0f);
    EXPECT_EQ(reverseFlightGetState(), REVERSE_STATE_MOTOR_STOPPED);
    EXPECT_NEAR(reverseFlightGetVectorFactor(), 0.0f, 0.01f);
    // Still drifting forward: surfaces keep their sign
    EXPECT_GE(reverseFlightGetSurfaceFactor(), 0.0f);
}

TEST_F(ReverseFlightTest, DwellHoldsMotorAtZeroBeforeReversing)
{
    config.reverse_motor_dwell = 50;
    reverseFlightInit(&config);

    fly(300, 1.0f);

    // Reverse straight away: the motor stays stopped for 50 ms first
    fly(-300, 0.040f);
    EXPECT_FLOAT_EQ(reverseFlightGetThrottle(), 0.0f);
    EXPECT_EQ(reverseFlightGetDirection(), 1);

    fly(-300, 0.020f);
    EXPECT_LT(reverseFlightGetThrottle(), 0.0f);
    EXPECT_EQ(reverseFlightGetDirection(), -1);
}

TEST_F(ReverseFlightTest, NoDwellAfterRestingInDeadband)
{
    config.reverse_motor_dwell = 50;
    reverseFlightInit(&config);

    fly(300, 1.0f);
    fly(0, 0.1f);       // rested longer than the dwell

    fly(-300, DT);
    EXPECT_LT(reverseFlightGetThrottle(), 0.0f);
}

TEST_F(ReverseFlightTest, FirstReversalAfterArmingHasNoDwell)
{
    config.reverse_motor_dwell = 50;
    reverseFlightInit(&config);

    fly(-300, DT, false);
    EXPECT_LT(reverseFlightGetThrottle(), 0.0f);
}

TEST_F(ReverseFlightTest, DisarmResetsEstimates)
{
    fly(-300, 2.0f);
    reverseFlightUpdate(0, false, false, DT);

    EXPECT_EQ(reverseFlightGetDirection(), 1);
    EXPECT_FLOAT_EQ(reverseFlightGetSpeedEstimate(), 0.0f);
    EXPECT_FLOAT_EQ(reverseFlightGetThrustEstimate(), 0.0f);
}

TEST_F(ReverseFlightTest, SurfacesForwardOnlyInForwardFlight)
{
    fly(300, 2.0f);
    EXPECT_TRUE(reverseFlightSurfacesForward());

    fly(-300, 0.1f);
    EXPECT_FALSE(reverseFlightSurfacesForward());

    fly(-300, 2.0f);
    EXPECT_FALSE(reverseFlightSurfacesForward());
}

// Rudder linked to the gimbal's yaw tilt on one servo
TEST_F(ReverseFlightTest, CoupledAxisFollowsGimbalThenSurface)
{
    config.reverse_coupled_axes = 1 << 2;   // yaw
    reverseFlightInit(&config);

    fly(362, 2.0f);
    EXPECT_FLOAT_EQ(reverseFlightGetAxisFactor(2), 1.0f);
    EXPECT_FLOAT_EQ(reverseFlightGetAxisFactor(1), 1.0f);

    // Stopping: surfaces are off, but the coupled axis follows the gimbal
    fly(-362, 0.15f);
    EXPECT_EQ(reverseFlightGetState(), REVERSE_STATE_HANDOVER);
    EXPECT_FLOAT_EQ(reverseFlightGetAxisFactor(1), 0.0f);
    EXPECT_FLOAT_EQ(reverseFlightGetAxisFactor(2), reverseFlightGetVectorFactor());
    EXPECT_FLOAT_EQ(reverseFlightGetAxisFactor(2), -1.0f);

    // Reverse flight: -1 like the others
    fly(-362, 2.0f);
    EXPECT_FLOAT_EQ(reverseFlightGetAxisFactor(2), -1.0f);

    // Motor stopped, drifting backwards: the surface sets the sign
    fly(0, 1.0f);
    EXPECT_NEAR(reverseFlightGetVectorFactor(), 0.0f, 0.01f);
    EXPECT_NEAR(reverseFlightGetAxisFactor(2), reverseFlightGetSurfaceFactor(), 0.01f);
    EXPECT_LT(reverseFlightGetAxisFactor(2), 0.0f);
}

TEST_F(ReverseFlightTest, AxisFactorIsOneWhenOff)
{
    config.throttle_mode = THROTTLE_MODE_NORMAL;
    config.reverse_coupled_axes = 7;
    reverseFlightInit(&config);
    reverseFlightUpdate(-0.5f, true, true, DT);
    EXPECT_FLOAT_EQ(reverseFlightGetAxisFactor(0), 1.0f);
    EXPECT_FLOAT_EQ(reverseFlightGetAxisFactor(2), 1.0f);
}

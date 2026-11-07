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
#include "platform.h"
#include "common/axis.h"
#include "common/maths.h"
#include "flight/hold_engine.h"
#include "flight/imu.h"
#include "flight/pid.h"

// Stubs: the aircraft's attitude and body rates are set by the test
static quaternion attitudeQ = { .w = 1, .x = 0, .y = 0, .z = 0 };
static pidAxisData_t axisData[PID_AXIS_COUNT];
static float stick[3];

void getQuaternion(quaternion *q) { *q = attitudeQ; }
const pidAxisData_t *pidGetAxisData(void) { return axisData; }
float pidGetDT(void) { return 0.001f; }
float getDeflection(int axis) { return stick[axis]; }
bool isAirborne(void) { return true; }

void imuQuaternionMultiplication(quaternion *q1, quaternion *q2, quaternion *result)
{
    const float w = q1->w * q2->w - q1->x * q2->x - q1->y * q2->y - q1->z * q2->z;
    const float x = q1->w * q2->x + q1->x * q2->w + q1->y * q2->z - q1->z * q2->y;
    const float y = q1->w * q2->y - q1->x * q2->z + q1->y * q2->w + q1->z * q2->x;
    const float z = q1->w * q2->z + q1->x * q2->y - q1->y * q2->x + q1->z * q2->w;
    result->w = w; result->x = x; result->y = y; result->z = z;
}
}

static const float DT = 0.001f;

class HoldEngineTest : public ::testing::Test {
protected:
    quatHold_t hold = {};
    float rollDeg = 0;

    void SetUp() override
    {
        // atthold_gain 40, deadband 5 %, max_rate 300 (the Nexus 4D setup)
        quatHoldInit(&hold, 4.0f, 0.05f, 300);
        rollDeg = 0;
        setRoll(0, 0);
        for (int i = 0; i < 3; i++) {
            stick[i] = 0;
        }
        quatHoldSetState(&hold, true);
    }

    void setRoll(float deg, float rateDegS)
    {
        rollDeg = deg;
        const float half = DEGREES_TO_RADIANS(deg) / 2;
        attitudeQ = { .w = cosf(half), .x = sinf(half), .y = 0, .z = 0 };
        axisData[FD_ROLL].gyroRate = rateDegS;
        axisData[FD_PITCH].gyroRate = 0;
        axisData[FD_YAW].gyroRate = 0;
    }

    // One PID loop: returns the roll setpoint (pilot asking for zero rate)
    float step()
    {
        const float roll = quatHoldApply(&hold, FD_ROLL, 0);
        quatHoldApply(&hold, FD_PITCH, 0);
        quatHoldApply(&hold, FD_YAW, 0);
        return roll;
    }
};

// The torque roll from the reverse-thrust hang test: the aircraft keeps turning and the ailerons
// can't stop it. The hold used to stay on the old target, and each time the error passed 180
// degrees the correction flipped from -max to +max in one loop: a full-throw aileron snap per turn.
TEST_F(HoldEngineTest, ContinuousRollNeverSnaps)
{
    const float rate = 150;    // deg/s, as logged
    float last = step();
    float maxStep = 0;
    bool lost = false;

    for (int i = 1; i < 10000; i++) {      // 10 s, about four full turns
        setRoll(fmodf(i * DT * rate + 180, 360) - 180, rate);
        const float sp = step();
        maxStep = fmaxf(maxStep, fabsf(sp - last));
        last = sp;
        lost |= hold.Lost[FD_ROLL];

        // Never pushes the turn along
        EXPECT_LE(sp, 0.5f) << "at " << i * DT << " s";
    }

    EXPECT_TRUE(lost);
    // The correction ramps (300 deg/s over 0.3 s is 1 deg/s per loop), it never jumps
    EXPECT_LT(maxStep, 5.0f);
    // Gave up once, and doesn't re-freeze mid-turn and lose it again
    EXPECT_EQ(1, hold.RecaptureCount);
    EXPECT_FALSE(quatHoldIsHolding(&hold, FD_ROLL));
    EXPECT_FLOAT_EQ(0, last);
}

TEST_F(HoldEngineTest, LostAxisHoldsAgainOnceTheRotationStops)
{
    for (int i = 1; i < 2000; i++) {
        setRoll(i * DT * 150, 150);        // 0 .. 300 degrees
        step();
    }
    ASSERT_TRUE(hold.Lost[FD_ROLL]);

    // Stops at -60 degrees (300) and stays there: holds that attitude, no snap back
    const float stopped = rollDeg;
    for (int i = 0; i < 1000; i++) {
        setRoll(stopped, 0);
        step();
    }
    EXPECT_TRUE(quatHoldIsHolding(&hold, FD_ROLL));
    EXPECT_FALSE(hold.Lost[FD_ROLL]);
    EXPECT_NEAR(0, step(), 1.0f);

    // And corrects a disturbance from there as before
    setRoll(stopped + 10, 0);
    EXPECT_NEAR(-40, step(), 1.0f);
}

TEST_F(HoldEngineTest, LargeErrorBelowTheLimitIsStillCorrected)
{
    step();
    setRoll(80, 0);                        // knocked 80 degrees off, not turning
    const float sp = step();
    EXPECT_FALSE(hold.Lost[FD_ROLL]);
    EXPECT_TRUE(quatHoldIsHolding(&hold, FD_ROLL));
    // Full correction back: the error term is 2 sin(40 deg) rad = 73.7 deg, times gain 4
    EXPECT_NEAR(-4 * RADIANS_TO_DEGREES(2 * sinf(DEGREES_TO_RADIANS(40))), sp, 1.0f);
}

TEST_F(HoldEngineTest, StickClearsTheLostState)
{
    for (int i = 1; i < 1000; i++) {
        setRoll(i * DT * 150, 150);
        step();
    }
    ASSERT_TRUE(hold.Lost[FD_ROLL]);

    stick[FD_ROLL] = 0.5f;
    step();
    EXPECT_FALSE(hold.Lost[FD_ROLL]);
}

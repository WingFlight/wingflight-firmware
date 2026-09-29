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

#include <limits.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "gtest/gtest.h"
#include "gmock/gmock.h"

float throttle;

// Dummies
extern "C" {
#include "pg/pid.h"
#include "flight/pid.h"
#include "sensors/gyro.h"
#include "io/gps.h"

gpsSolutionData_t gpsSol;
bool gyroOverflowDetected(void) { return false; }
void beeperConfirmationBeeps(uint8_t) {}
float getThrottle(void) { return throttle; }
void governorInitProfile(const pidProfile_t *) {}
void levelingInit(const pidProfile_t *) {}
void INIT_CODE rescueInitProfile(const pidProfile_t *) {}
bool isSpooledUp(void) { return true; }
void setpointInitProfile(void) {}
bool isAirborne(void) { return true; }
float getMotor1Speedf(void) { return 0; }
float rescueApply(uint8_t, float setpoint) { return setpoint; }
float angleModeApply(int, float pidSetpoint) { return pidSetpoint; }
void angleModeReset(void) {}
float getSpoolUpRatio(void) { return 1.0f; }
float mixerGetInput(uint8_t) { return 0.0; }
bool mixerSaturated(uint8_t) { return false; }
void attHoldInit(const pidProfile_t *) {}
float attHoldApply(int, float pidSetpoint) { return pidSetpoint; }
float attHoldIDecayScale(int) { return 1.0f; }
float getRcDeflection(int) { return 0; }
uint8_t getCurrentPidProfileIndex(void) { return 0; }
void changePidProfile(uint8_t) {}
} // extern "C"

// Mocks
class MockInterface {
  public:
    MOCK_METHOD(float, getDeflection, (int axis), ());
} *g_mock = nullptr;

extern "C" {
// Fixed 1000Hz
gyro_t gyro = {.targetLooptime = 1000000 / 1000};
pidProfile_t *mockPidProfile = pidProfilesMutable(0);
pidProfile_t *currentPidProfile = mockPidProfile;

float getSetpoint(int axis) { return g_mock->getDeflection(axis) * 360; }
float getDeflection(int axis) { return g_mock->getDeflection(axis); }
}

TEST(Empty, BuildTest) {}

// 4xN matrix for PID input/output
using PIDIO = std::array<std::vector<float>, 4>;

using ::testing::StrictMock;

class PIDTestBase : public ::testing::Test {
  public:
    void SetUp() override {
        g_mock = &mock;
        pgResetAll();
        pidInit(mockPidProfile);
    }
    void TearDown() override { g_mock = nullptr; }
    PIDIO getResponse(PIDIO &input)
    {
        PIDIO output;

        uint32_t time = 0;
        for (size_t i = 0; i < input[0].size(); i++) {
            time += gyro.targetLooptime;
            for (int axis = 0; axis < 4; axis++) {
                EXPECT_CALL(mock, getDeflection(axis))
                    .WillRepeatedly(testing::Return(input[axis][i]));
            }
            pidController(mockPidProfile, time);
            for (int axis = 0; axis < 4; axis++) {
                output[axis].push_back(pidGetOutput(axis));
            }
        }

        return output;
    }
    StrictMock<MockInterface> mock;
};

TEST_F(PIDTestBase, Mode0)
// Mode 0 sanity check
{
    mockPidProfile->pid_mode = 0;
    mockPidProfile->pid[0].F = 1;
    pidLoadProfile(mockPidProfile);

    PIDIO input;
    for (int i = 0; i < 100; i++) {
        input[0].push_back(0);
        input[1].push_back(0);
        input[2].push_back(0);
        input[3].push_back(0);
    }
    for (int i = 0; i < 100; i++) {
        input[0].push_back(1);
        input[1].push_back(1);
        input[2].push_back(1);
        input[3].push_back(1);
    }

    PIDIO output = getResponse(input);
    for (int i = 0; i < 100; i++) {
        EXPECT_EQ(output[0][i], 0);
    }
    for (int i = 100; i < 200; i++) {
        EXPECT_NE(output[0][i], 0);
    }
}

class PIDFBTest : public PIDTestBase {
  public:
};

TEST_F(PIDFBTest, B)
// Basic B test
{
    mockPidProfile->pid_mode = 3;
    mockPidProfile->pid[0].P = 0;
    mockPidProfile->pid[0].I = 0;
    mockPidProfile->pid[0].D = 0;
    mockPidProfile->pid[0].F = 0;
    mockPidProfile->pid[0].B = 100;
    mockPidProfile->bterm_cutoff[0] = 30;
    pidLoadProfile(mockPidProfile);

    PIDIO input;
    for (int i = 0; i < 500; i++) {
        input[0].push_back(0);
        input[1].push_back(0);
        input[2].push_back(0);
        input[3].push_back(0);
    }
    for (int i = 0; i < 500; i++) {
        input[0].push_back(1);
        input[1].push_back(1);
        input[2].push_back(1);
        input[3].push_back(1);
    }

    PIDIO output = getResponse(input);
    // This test is a NOP so far.
}

class PIDAttenuationTest : public PIDTestBase {
  public:
    // Steady roll output with only one term active, after a small stick step
    float rollOutputAt(uint8_t tpaGain, int sample)
    {
        mockPidProfile->fw_tpa_gain = tpaGain;
        pidInit(mockPidProfile);

        PIDIO input;
        for (int i = 0; i < 50; i++) {
            for (int axis = 0; axis < 4; axis++) {
                input[axis].push_back(0);
            }
        }
        for (int i = 0; i < 200; i++) {
            for (int axis = 0; axis < 4; axis++) {
                input[axis].push_back(0.01f);
            }
        }
        return getResponse(input)[0][sample];
    }

    void onlyRoll(uint16_t F, uint16_t B)
    {
        mockPidProfile->pid_mode = 1;
        for (int axis = 0; axis < 3; axis++) {
            mockPidProfile->pid[axis].P = 0;
            mockPidProfile->pid[axis].I = 0;
            mockPidProfile->pid[axis].D = 0;
            mockPidProfile->pid[axis].F = 0;
            mockPidProfile->pid[axis].B = 0;
        }
        mockPidProfile->pid[0].F = F;
        mockPidProfile->pid[0].B = B;
    }
};

TEST_F(PIDAttenuationTest, ThrottleAttenuationScalesF)
{
    onlyRoll(100, 0);
    const float full = rollOutputAt(100, 240);
    const float half = rollOutputAt(50, 240);
    EXPECT_GT(full, 0);
    EXPECT_NEAR(half, full * 0.5f, fabsf(full) * 1e-4f);
}

TEST_F(PIDAttenuationTest, ThrottleAttenuationScalesB)
{
    onlyRoll(0, 100);
    const float full = rollOutputAt(100, 52);
    const float half = rollOutputAt(50, 52);
    EXPECT_GT(full, 0);
    EXPECT_NEAR(half, full * 0.5f, fabsf(full) * 1e-4f);
}

TEST_F(PIDAttenuationTest, AttenuationNeverDropsBelowFloor)
{
    // 10 % is below anything the CLI allows, standing in for a curve point near zero
    onlyRoll(100, 0);
    const float full = rollOutputAt(100, 240);
    const float floored = rollOutputAt(10, 240);
    EXPECT_GT(full, 0);
    EXPECT_NEAR(floored, full * PID_ATTENUATION_MIN, fabsf(full) * 1e-4f);
}

TEST_F(PIDAttenuationTest, ManualFeedforwardIgnoresAttenuation)
{
    onlyRoll(100, 0);
    mockPidProfile->fw_tpa_gain = 100;
    pidLoadProfile(mockPidProfile);
    const float full = pidGetFeedforward(0, 100);
    mockPidProfile->fw_tpa_gain = 25;
    pidLoadProfile(mockPidProfile);
    EXPECT_GT(full, 0);
    EXPECT_FLOAT_EQ(pidGetFeedforward(0, 100), full);
}

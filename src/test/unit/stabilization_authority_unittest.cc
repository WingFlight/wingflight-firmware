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

#include <cmath>
#include <cstring>
#include "gtest/gtest.h"

extern "C" {
#include "build/debug.h"
#include "fc/runtime_config.h"
#include "fc/rc_rates.h"
#include "flight/autohover.h"
#include "flight/hold_engine.h"
#include "flight/leveling.h"
#include "flight/pid.h"
#include "drivers/time.h"
#include "pg/accel.h"

uint8_t armingFlags;
uint16_t flightModeFlags;
attitudeEulerAngles_t attitude = EULER_INITIALIZE;
pidProfile_t *currentPidProfile;
controlRateConfig_t *currentControlRateProfile;
uint8_t debugMode;
uint8_t debugAxis;
int32_t debug[DEBUG_VALUE_COUNT];
uint32_t __timing[DEBUG_VALUE_COUNT];
}

static float pilotThrottle;
static bool receivingSignal;
static bool throttleOff;
static timeMs_t nowMs;
static quaternion orientation;
static pidAxisData_t pidAxes[3];

extern "C" {
float getThrottle(void) { return pilotThrottle; }
bool isThrottleOff(void) { return throttleOff; }
bool rxIsReceivingSignal(void) { return receivingSignal; }
float getDeflection(int) { return 0.0f; }
float pidGetDT(void) { return 0.01f; }
const pidAxisData_t *pidGetAxisData(void) { return pidAxes; }
timeMs_t millis(void) { return nowMs; }
bool isUpsidedown(void) { return false; }
// Deliberately always landed: no attitude controller may depend on this estimate.
bool isAirborne(void) { return false; }
void getQuaternion(quaternion *q) { *q = orientation; }
void imuQuaternionMultiplication(quaternion *a, quaternion *b, quaternion *out)
{
    *out = {a->w*b->w - a->x*b->x - a->y*b->y - a->z*b->z,
            a->w*b->x + a->x*b->w + a->y*b->z - a->z*b->y,
            a->w*b->y - a->x*b->z + a->y*b->w + a->z*b->x,
            a->w*b->z + a->x*b->y - a->y*b->x + a->z*b->w};
}
void imuEulerToQuaternion(int16_t roll, int16_t pitch, int16_t yaw, quaternion *out)
{
    const float r = roll * M_PI / 3600.0f;
    const float p = pitch * M_PI / 3600.0f;
    const float y = yaw * M_PI / 3600.0f;
    *out = {cosf(r)*cosf(p)*cosf(y) + sinf(r)*sinf(p)*sinf(y),
            sinf(r)*cosf(p)*cosf(y) - cosf(r)*sinf(p)*sinf(y),
            cosf(r)*sinf(p)*cosf(y) + sinf(r)*cosf(p)*sinf(y),
            cosf(r)*cosf(p)*sinf(y) - sinf(r)*sinf(p)*cosf(y)};
}
}

class StabilizationAuthorityTest : public ::testing::Test {
protected:
    pidProfile_t profile;
    controlRateConfig_t rates;

    void SetUp() override
    {
        memset(&profile, 0, sizeof(profile));
        memset(&rates, 0, sizeof(rates));
        memset(&attitude, 0, sizeof(attitude));
        memset(pidAxes, 0, sizeof(pidAxes));
        currentPidProfile = &profile;
        currentControlRateProfile = &rates;
        armingFlags = ARMED;
        flightModeFlags = 0;
        pilotThrottle = 0.6f;
        receivingSignal = true;
        throttleOff = false;
        nowMs = 100;
        orientation = {1, 0, 0, 0};
        profile.autohover.gain = 10;
        profile.autohover.max_rate = 30;
        profile.autohover.throttle_assist_gain = 100;
        profile.autohover.throttle_assist_max = 15;
        profile.autohover.throttle_assist_trigger_ms = 100;
        autoHoverSetState(false);
        autoHoverInit(&profile);
        autoHoverSetState(true);
    }

    float tick(int count = 1)
    {
        float boost = 0;
        for (int i = 0; i < count; i++) {
            autoHoverApply(FD_ROLL, 0);
            autoHoverApply(FD_PITCH, 0);
            autoHoverApply(FD_YAW, 0);
            boost = autoHoverThrottleBoost();
            nowMs += 10;
        }
        return boost;
    }
};

TEST_F(StabilizationAuthorityTest, StrictPilotThrottleThreshold)
{
    for (float throttle : {0.0f, 0.39f, 0.40f}) {
        pilotThrottle = throttle;
        EXPECT_FLOAT_EQ(tick(50), 0);
    }
    pilotThrottle = 0.401f;
    EXPECT_FLOAT_EQ(tick(10), 0); // A fresh saturation delay, even after waiting below threshold.
    EXPECT_GT(tick(), 0);
    EXPECT_NEAR(tick(50), 0.15f, 1e-6f);
}

TEST_F(StabilizationAuthorityTest, ThrottleDropClearsBoostAndRestartsDelay)
{
    EXPECT_GT(tick(50), 0);
    pilotThrottle = 0.40f;
    // Getter must clear immediately, without waiting for another attitude calculation.
    EXPECT_FLOAT_EQ(autoHoverThrottleBoost(), 0);
    pilotThrottle = 0.41f;
    EXPECT_FLOAT_EQ(tick(10), 0);
    EXPECT_NEAR(tick(), 0.01f, 1e-6f);
}

TEST_F(StabilizationAuthorityTest, CalculationAlsoClearsBoostWhenIneligible)
{
    EXPECT_GT(tick(50), 0);
    pilotThrottle = 0.4f;
    autoHoverApply(FD_ROLL, 0);
    pilotThrottle = 0.6f;
    EXPECT_FLOAT_EQ(autoHoverThrottleBoost(), 0);
    EXPECT_FLOAT_EQ(tick(10), 0);
}

TEST_F(StabilizationAuthorityTest, DisarmRxLossOffThrottleAndModeExitResetAssist)
{
    for (int condition = 0; condition < 4; condition++) {
        EXPECT_GT(tick(50), 0);
        if (condition == 0) armingFlags = 0;
        if (condition == 1) receivingSignal = false;
        if (condition == 2) throttleOff = true;
        if (condition == 3) autoHoverSetState(false);
        EXPECT_FLOAT_EQ(autoHoverThrottleBoost(), 0);
        EXPECT_FLOAT_EQ(tick(20), 0);
        armingFlags = ARMED;
        receivingSignal = true;
        throttleOff = false;
        autoHoverSetState(true);
        EXPECT_FLOAT_EQ(tick(10), 0);
        EXPECT_GT(tick(), 0);
    }
}

TEST_F(StabilizationAuthorityTest, DisabledAssistAndZeroMaxRateNeverBoost)
{
    profile.autohover.throttle_assist_gain = 0;
    autoHoverInit(&profile);
    EXPECT_FLOAT_EQ(tick(50), 0);
    profile.autohover.throttle_assist_gain = 100;
    profile.autohover.max_rate = 0;
    autoHoverInit(&profile);
    EXPECT_FLOAT_EQ(tick(50), 0);
}

TEST_F(StabilizationAuthorityTest, PitchMustRemainSaturatedForEntireDelay)
{
    tick(5);
    imuEulerToQuaternion(0, -900, 0, &orientation); // At target: clears saturation timer.
    EXPECT_FLOAT_EQ(tick(20), 0);
    orientation = {1, 0, 0, 0};
    EXPECT_FLOAT_EQ(tick(10), 0);
    EXPECT_GT(tick(), 0);
}

TEST_F(StabilizationAuthorityTest, AutoHoverHasFullArmedAuthorityAtIdle)
{
    profile.autohover.max_rate = 200;
    autoHoverInit(&profile);
    pilotThrottle = 0;
    armingFlags = 0;
    tick();
    const float bench = autoHoverApply(FD_PITCH, 0);
    armingFlags = ARMED;
    tick();
    const float armed = autoHoverApply(FD_PITCH, 0);
    EXPECT_GT(fabsf(armed), 70);
    EXPECT_NEAR(armed, bench * 4, 1e-4f);
    EXPECT_FLOAT_EQ(autoHoverThrottleBoost(), 0);
}

TEST_F(StabilizationAuthorityTest, AngleAndHorizonHaveFullArmedAuthorityAtIdle)
{
    profile.angle.level_strength = 10;
    profile.angle.level_limit = 45;
    profile.horizon.level_strength = 10;
    profile.horizon.transition = 100;
    levelingInit(&profile);
    pilotThrottle = 0;
    attitude.values.roll = 100;
    attitude.values.pitch = 100;
    for (int axis : {FD_ROLL, FD_PITCH}) {
        armingFlags = 0;
        const float benchAngle = angleModeApply(axis, 0);
        const float benchHorizon = horizonModeApply(axis, 0);
        armingFlags = ARMED;
        EXPECT_FLOAT_EQ(angleModeApply(axis, 0), -10);
        EXPECT_FLOAT_EQ(angleModeApply(axis, 0), benchAngle * 4);
        EXPECT_LT(horizonModeApply(axis, 0), 0);
        EXPECT_FLOAT_EQ(horizonModeApply(axis, 0), benchHorizon * 4);
    }
}

TEST_F(StabilizationAuthorityTest, SharedHoldEngineHasFullArmedAuthorityOnAllAxes)
{
    quatHold_t hold = {};
    quatHoldInit(&hold, 1, 0.05f, 200);
    quatHoldSetState(&hold, true);
    imuEulerToQuaternion(20, 20, 20, &orientation);
    pilotThrottle = 0;
    armingFlags = 0;
    float bench[3];
    for (int axis = 0; axis < 3; axis++) bench[axis] = quatHoldApply(&hold, axis, 0);
    armingFlags = ARMED;
    for (int axis = 0; axis < 3; axis++) {
        const float armed = quatHoldApply(&hold, axis, 0);
        EXPECT_GT(fabsf(armed), 1);
        EXPECT_NEAR(armed, bench[axis] * 4, 1e-4f);
    }
}

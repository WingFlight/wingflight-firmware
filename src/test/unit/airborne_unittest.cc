#include <cstring>
#include "gtest/gtest.h"

extern "C" {
#include "build/debug.h"
#include "drivers/time.h"
#include "fc/runtime_config.h"
#include "flight/airborne.h"
#include "pg/rx.h"
#include "sensors/gyro.h"

rcControlsConfig_t rcControlsConfig_System;
uint8_t armingFlags;
uint16_t flightModeFlags;
gyro_t gyro;
uint8_t debugMode;
uint8_t debugAxis;
int32_t debug[DEBUG_VALUE_COUNT];
uint32_t __timing[DEBUG_VALUE_COUNT];
static timeUs_t now;
static bool receiving;
static float tilt;
timeUs_t micros(void) { return now; }
float pidGetPidFrequency(void) { return 1000; }
float getCosTiltAngle(void) { return tilt; }
bool rxIsReceivingSignal(void) { return receiving; }
}

class AirborneTest : public ::testing::Test {
protected:
    float rc[3];
    void SetUp() override
    {
        memset(rc, 0, sizeof(rc));
        memset(&gyro, 0, sizeof(gyro));
        for (int i = 0; i < 3; i++) rcControlsConfigMutable()->rc_threshold[i] = 25;
        armingFlags = ARMED;
        flightModeFlags = 0;
        receiving = true;
        now = 0;
        tilt = 1;
        airborneInit();
    }
    void tick(int ms)
    {
        for (int i = 0; i < ms; i++) {
            airborneUpdate(rc);
            now += 1000;
        }
    }
    void response(int axis = FD_ROLL, float sign = 1)
    {
        rc[axis] = sign * 0.2f;
        gyro.gyroADCf[axis] = sign * 30;
    }
};

TEST_F(AirborneTest, ArmedStationaryAndStaticTiltStayGrounded)
{
    tick(1000);
    EXPECT_FALSE(isAirborne());
    tilt = -1;
    rc[FD_ROLL] = 1;
    rc[FD_PITCH] = 1;
    tick(1000);
    EXPECT_FALSE(isAirborne());
}

TEST_F(AirborneTest, RotationWithoutPilotInputDoesNotQualify)
{
    gyro.gyroADCf[FD_ROLL] = 50;
    gyro.gyroADCf[FD_PITCH] = 50;
    tick(1000);
    EXPECT_FALSE(isAirborne());
}

TEST_F(AirborneTest, BothRollAndPitchDirectionsQualify)
{
    for (int axis : {FD_ROLL, FD_PITCH}) {
        for (float sign : {-1.0f, 1.0f}) {
            SetUp();
            response(axis, sign);
            tick(251);
            EXPECT_TRUE(isAirborne());
        }
    }
}

TEST_F(AirborneTest, RequiresFullContinuousWindowIncludingTimeZero)
{
    response();
    tick(250);
    EXPECT_FALSE(isAirborne());
    tick(1);
    EXPECT_TRUE(isAirborne());
}

TEST_F(AirborneTest, SteeringAndOppositeResponseDoNotQualify)
{
    response(FD_YAW);
    response(FD_ROLL);
    gyro.gyroADCf[FD_ROLL] = -30;
    tick(1000);
    EXPECT_FALSE(isAirborne());
}

TEST_F(AirborneTest, SmallInputsAndSmallResponsesDoNotQualify)
{
    response();
    rc[FD_ROLL] = 0.099f;
    tick(1000);
    EXPECT_FALSE(isAirborne());
    rc[FD_ROLL] = 0.1f;
    gyro.gyroADCf[FD_ROLL] = 14.99f;
    tick(1000);
    EXPECT_FALSE(isAirborne());
    gyro.gyroADCf[FD_ROLL] = 15;
    tick(251);
    EXPECT_TRUE(isAirborne());
}

TEST_F(AirborneTest, InterruptedResponseRequiresFreshWindow)
{
    response();
    tick(200);
    gyro.gyroADCf[FD_ROLL] = 0;
    tick(1);
    response();
    tick(250);
    EXPECT_FALSE(isAirborne());
    tick(1);
    EXPECT_TRUE(isAirborne());
}

TEST_F(AirborneTest, ReleasedStickCancelsPendingEvidence)
{
    response();
    tick(200);
    rc[FD_ROLL] = 0;
    tick(1000);
    EXPECT_FALSE(isAirborne());
}

TEST_F(AirborneTest, DirectionAndAxisCannotShareConfirmationTime)
{
    response();
    tick(200);
    response(FD_ROLL, -1);
    tick(200);
    EXPECT_FALSE(isAirborne());
    rc[FD_ROLL] = 0;
    response(FD_PITCH);
    tick(200);
    EXPECT_FALSE(isAirborne());
    tick(51);
    EXPECT_TRUE(isAirborne());
}

TEST_F(AirborneTest, DisarmedResponseCannotPrimeTakeoff)
{
    armingFlags = 0;
    response();
    tick(1000);
    EXPECT_FALSE(isAirborne());
    armingFlags = ARMED;
    tick(250);
    EXPECT_FALSE(isAirborne());
    tick(1);
    EXPECT_TRUE(isAirborne());
}

TEST_F(AirborneTest, FlightRemainsLatchedThroughQuietFlightAndReceiverLoss)
{
    response();
    tick(251);
    memset(rc, 0, sizeof(rc));
    memset(&gyro, 0, sizeof(gyro));
    receiving = false;
    tick(10000);
    EXPECT_TRUE(isAirborne());
    armingFlags = 0;
    tick(1);
    EXPECT_FALSE(isAirborne());
    armingFlags = ARMED;
    tick(1000);
    EXPECT_FALSE(isAirborne());
}

TEST_F(AirborneTest, ReceiverLossCancelsPendingEvidence)
{
    response();
    tick(200);
    receiving = false;
    tick(1000);
    EXPECT_FALSE(isAirborne());
    receiving = true;
    tick(250);
    EXPECT_FALSE(isAirborne());
    tick(1);
    EXPECT_TRUE(isAirborne());
}

TEST_F(AirborneTest, RecoveryOverridesRequireArming)
{
    for (uint16_t mode : {GPS_RESCUE_MODE, FAILSAFE_MODE}) {
        SetUp();
        flightModeFlags = mode;
        armingFlags = 0;
        tick(1);
        EXPECT_FALSE(isAirborne());
        armingFlags = ARMED;
        tick(1);
        EXPECT_TRUE(isAirborne());
        flightModeFlags = 0;
        tick(1000);
        EXPECT_TRUE(isAirborne());
    }
}

TEST_F(AirborneTest, TimerWrapDoesNotBreakConfirmation)
{
    now = UINT32_MAX - 100000;
    response();
    tick(250);
    EXPECT_FALSE(isAirborne());
    tick(1);
    EXPECT_TRUE(isAirborne());
}

TEST_F(AirborneTest, HandsOnStillUsesYawAndDecayIndependently)
{
    armingFlags = 0;
    rc[FD_YAW] = 1;
    tick(1000);
    EXPECT_TRUE(isHandsOn());
    EXPECT_FALSE(isAirborne());
    rc[FD_YAW] = 0;
    tick(5000);
    EXPECT_FALSE(isHandsOn());
}

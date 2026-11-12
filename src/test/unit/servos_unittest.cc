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

// The servo output stage: one logical servo list, sent on the PWM outputs the
// board has and on the channels of a bus output (SBUS, F.Bus).

#include <cstring>
#include "gtest/gtest.h"

extern "C" {
#include "platform.h"

#include "drivers/io.h"
#include "drivers/pwm_output.h"
#include "drivers/time.h"
#include "drivers/timer.h"
#include "fc/runtime_config.h"
#include "flight/mixer.h"
#include "flight/servos.h"
#include "pg/bus_servo.h"
#include "pg/mixer.h"
#include "pg/servo_curve.h"
#include "pg/servo_trim.h"
#include "pg/servos.h"

servoParam_t servoParams_SystemArray[MAX_SUPPORTED_SERVOS];
servoConfig_t servoConfig_System;
servoCurve_t servoCurves_SystemArray[MAX_SUPPORTED_SERVOS];
servoTrim_t servoTrims_SystemArray[MAX_SUPPORTED_SERVOS];
mixerRule_t mixerRules_SystemArray[MIXER_RULE_COUNT];
mixerInput_t mixerInputs_SystemArray[MIXER_INPUT_COUNT];

uint8_t armingFlags;

static timerHardware_t fakeTimers[MAX_SUPPORTED_PWM_SERVOS];
static timCCR_t ccr[MAX_SUPPORTED_PWM_SERVOS];
static int ccrNext;
static timeMs_t nowMs;
static uint8_t busServoCount;
static float mixerOutput[MIXER_OUTPUT_COUNT];
static bool mixerOutputSaturated[MIXER_OUTPUT_COUNT];

const timerHardware_t *timerAllocate(ioTag_t ioTag, resourceOwner_e, uint8_t) { return &fakeTimers[ioTag - 1]; }
// A 1MHz timer clock gives a 1MHz timebase, so the compare value is the pulse in us
uint32_t timerClock(TIM_TypeDef *) { return 1000000; }
IO_t IOGetByTag(ioTag_t) { return NULL; }
void IOInit(IO_t, resourceOwner_e, uint8_t) {}
void IOConfigGPIO(IO_t, ioConfig_t) {}
void IOConfigGPIOAF(IO_t, ioConfig_t, uint8_t) {}
void pwmOutConfig(timerChannel_t *channel, const timerHardware_t *, uint32_t, uint16_t, uint16_t, uint8_t) { channel->ccr = &ccr[ccrNext++]; }
void delay(timeMs_t) {}
timeMs_t millis(void) { return nowMs; }
float pidGetDT(void) { return 0.001f; }
uint8_t getBusServoCount(void) { return busServoCount; }
float mixerGetOutput(uint8_t index) { return mixerOutput[index]; }
void mixerSaturateOutput(uint8_t index) { mixerOutputSaturated[index] = true; }
}

class ServosTest : public ::testing::Test {
protected:
    // A board with this many PWM servo outputs, and a bus output sending this many servos
    void start(int pwmOutputs, int busServos)
    {
        memset(&servoConfig_System, 0, sizeof(servoConfig_System));
        memset(servoCurves_SystemArray, 0, sizeof(servoCurves_SystemArray));
        memset(servoTrims_SystemArray, 0, sizeof(servoTrims_SystemArray));
        memset(mixerRules_SystemArray, 0, sizeof(mixerRules_SystemArray));
        memset(mixerInputs_SystemArray, 0, sizeof(mixerInputs_SystemArray));
        memset(mixerOutput, 0, sizeof(mixerOutput));
        memset(mixerOutputSaturated, 0, sizeof(mixerOutputSaturated));
        memset(ccr, 0, sizeof(ccr));
        ccrNext = 0;
        nowMs = 0;
        armingFlags = 0;

        for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
            servoParam_t *servo = servoParamsMutable(i);
            servo->mid = 1500;
            servo->min = -700;
            servo->max = 700;
            servo->rneg = 500;
            servo->rpos = 500;
            servo->rate = 50;
            servo->speed = 0;
            servo->flags = 0;
        }
        for (int i = 0; i < pwmOutputs; i++) {
            servoConfigMutable()->ioTags[i] = i + 1;
            fakeTimers[i].tim = (TIM_TypeDef *)&fakeTimers[i];
        }
        busServoCount = busServos;

        servoShutdown();
        servoInit();
    }

    void setMixer(int servo, float value) { mixerOutput[MIXER_SERVO_OFFSET + servo] = value; }
    bool saturated(int servo) { return mixerOutputSaturated[MIXER_SERVO_OFFSET + servo]; }
};

TEST_F(ServosTest, ServoCountIsTwentyFour)
{
    EXPECT_EQ(24, MAX_SUPPORTED_SERVOS);
    // Mixer output numbers are stored in rules and backups
    EXPECT_EQ(1, MIXER_SERVO_OFFSET);
    EXPECT_EQ(27, MIXER_MOTOR_OFFSET);
}

TEST_F(ServosTest, PwmOutputsOnly)
{
    start(4, 0);
    EXPECT_EQ(4, getServoCount());
    EXPECT_EQ(4, getPwmServoCount());

    setMixer(0, 0.5f);
    setMixer(3, -1.0f);
    setMixer(4, 1.0f);
    servoUpdate();

    EXPECT_EQ(1750, getServoOutput(0));
    EXPECT_EQ(1750u, ccr[0]);
    EXPECT_EQ(1000, getServoOutput(3));
    EXPECT_EQ(1000u, ccr[3]);
    // S5 has no output anywhere: left at center, and not part of trimming
    EXPECT_EQ(1500, getServoOutput(4));
    EXPECT_TRUE(isServoOutputInUse(3));
    EXPECT_FALSE(isServoOutputInUse(4));
}

TEST_F(ServosTest, BusOutputAddsServosPastThePwmOutputs)
{
    start(4, 16);
    EXPECT_EQ(16, getServoCount());
    EXPECT_EQ(4, getPwmServoCount());

    setMixer(1, 0.5f);
    setMixer(9, -0.5f);
    setMixer(16, 1.0f);
    servoUpdate();

    // S2 is on PWM output 2 and bus channel 2: one value for both
    EXPECT_EQ(1750u, ccr[1]);
    EXPECT_FLOAT_EQ(1750.0f, getServoOutputFloat(1));
    // S10 is bus channel 10 only
    EXPECT_FLOAT_EQ(1250.0f, getServoOutputFloat(9));
    EXPECT_TRUE(isServoOutputInUse(9));
    // S17 is past what the bus sends
    EXPECT_FLOAT_EQ(1500.0f, getServoOutputFloat(16));
    EXPECT_FALSE(isServoOutputInUse(16));
}

TEST_F(ServosTest, EveryServoHasTheSameSignalRange)
{
    start(4, 8);
    // S1 has a PWM output, S6 is only on the bus
    for (int i : {0, 5}) {
        servoParamsMutable(i)->rpos = 1000;
        servoParamsMutable(i)->rneg = 1000;
        setMixer(i, 1.0f);
    }
    servoUpdate();

    for (int i : {0, 5}) {
        EXPECT_EQ(SERVO_SIGNAL_MAX, getServoOutput(i));
        EXPECT_EQ(659, servoTravelMax(i));
        EXPECT_EQ(-620, servoTravelMin(i));
        EXPECT_TRUE(saturated(i));
    }
    EXPECT_EQ(2159u, ccr[0]);

    // Inside the range the travel limits are the ones set
    for (int i : {0, 5}) {
        servoParamsMutable(i)->max = 400;
        setMixer(i, 1.0f);
    }
    servoUpdate();
    EXPECT_EQ(1900, getServoOutput(0));
    EXPECT_EQ(1900, getServoOutput(5));
}

TEST_F(ServosTest, ReverseTrimAndSpeedApplyToBusOnlyServos)
{
    start(2, 12);
    servoParamsMutable(7)->flags = SERVO_FLAG_REVERSED;
    setServoSavedTrim(7, 40);
    setMixer(7, 0.5f);
    servoUpdate();
    EXPECT_FLOAT_EQ(1500.0f - 250.0f + 40.0f, getServoOutputFloat(7));

    // 1200 * dt / speed per loop: speed 600 at 1kHz is 0.002 of full travel
    servoParamsMutable(8)->speed = 600;
    setMixer(8, 1.0f);
    servoUpdate();
    EXPECT_NEAR(1501.0f, getServoOutputFloat(8), 0.01f);
    servoUpdate();
    EXPECT_NEAR(1502.0f, getServoOutputFloat(8), 0.01f);
}

TEST_F(ServosTest, OverrideAndProbeWorkOnAnyServo)
{
    start(2, 8);
    setMixer(6, 1.0f);
    setServoOverride(6, -500);
    servoUpdate();
    EXPECT_EQ(1250, getServoOutput(6));

    armingFlags = ARMED;
    servoUpdate();
    EXPECT_EQ(2000, getServoOutput(6));
    EXPECT_FALSE(setServoProbe(6, 100, 1000));

    armingFlags = 0;
    setServoOverride(6, SERVO_OVERRIDE_OFF);
    EXPECT_TRUE(setServoProbe(6, 900, 1000));
    servoUpdate();
    // The probe ignores the travel limits but not the signal range
    EXPECT_EQ(2159, getServoOutput(6));
    EXPECT_TRUE(setServoProbe(6, 0, 0));
    EXPECT_FALSE(setServoProbe(MAX_SUPPORTED_SERVOS, 0, 1000));
}

TEST_F(ServosTest, BusChannelValueCoversTheWholeField)
{
    // The usual SBUS / F.Bus scale
    EXPECT_EQ(992, busServoValue(1500));
    EXPECT_EQ(192, busServoValue(1000));
    EXPECT_EQ(1792, busServoValue(2000));
    // The ends of the 11-bit field
    EXPECT_EQ(0, busServoValue(BUS_SERVO_MIN_SIGNAL));
    EXPECT_EQ(2046, busServoValue(BUS_SERVO_MAX_SIGNAL));
    EXPECT_EQ(0, busServoValue(50));
    EXPECT_EQ(2047, busServoValue(2250));
}

TEST_F(ServosTest, CenterIsAStandardPulseOnEveryServo)
{
    start(4, 16);
    for (int i : {0, 12}) {
        servoParamsMutable(i)->mid = 760;
        servoParamsMutable(i + 1)->mid = 2200;
    }
    servoParamsMutable(2)->rate = 560;
    servoParamsMutable(3)->rate = 10;
    validateAndFixServoConfig();
    for (int i : {0, 12}) {
        EXPECT_EQ(1000, servoParams(i)->mid);
        EXPECT_EQ(2000, servoParams(i + 1)->mid);
    }
    EXPECT_EQ(333, servoParams(2)->rate);
    EXPECT_EQ(25, servoParams(3)->rate);
}

TEST_F(ServosTest, ServoCountFollowsTheBusChannelCount)
{
    start(4, 8);
    EXPECT_EQ(8, getServoCount());

    busServoCount = 24;
    for (int i = 0; i < 300; i++) {
        servoUpdate();
    }
    EXPECT_EQ(24, getServoCount());

    setMixer(23, 0.2f);
    servoUpdate();
    EXPECT_EQ(1600, getServoOutput(23));

    busServoCount = 0;
    for (int i = 0; i < 300; i++) {
        servoUpdate();
    }
    EXPECT_EQ(4, getServoCount());
}

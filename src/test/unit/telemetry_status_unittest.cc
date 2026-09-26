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

#include <cstring>

#include "gtest/gtest.h"

extern "C" {
#include "platform.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"
#include "flight/autotrim.h"
#include "flight/failsafe.h"
#include "sensors/battery.h"
#include "sensors/sensors.h"
#include "telemetry/status.h"

uint8_t armingFlags;
uint8_t stateFlags;
uint16_t flightModeFlags;

// Everything status.c reads, reset per test.
static struct {
    bool airborne, motorsRunning, rxLink, backupEnabled, backupActive;
    failsafePhase_e failsafe;
    bool gpsHealthy, canRth, canLoiter, rthSwitch, loiterSwitch;
    uint32_t sensorMask;
    batteryState_e battery;
    bool saturationLatch;
    timeMs_t now;
    bool gyroOverflow, accCalibrated;
    bool servoOverride, mixerOverride, motorOverride;
    bool attHold, tvHold, autoHover, trainer;
    autoTrimState_e autoTrim;
    bool blackboxLogging, blackboxFull;
    bool logic[16];
    uint8_t pidProfile, ratesProfile, batteryProfile, tvProfile;
    bool dirty, saving, reboot, beeper, rpm;
} fc;

bool isAirborne(void) { return fc.airborne; }
bool areMotorsRunning(void) { return fc.motorsRunning; }
uint8_t getMotorCount(void) { return 2; }
bool hasMotorOverride(uint8_t motor) { return fc.motorOverride && motor == 1; }
bool isRpmSourceActive(void) { return fc.rpm; }
bool rxIsReceivingSignal(void) { return fc.rxLink; }
bool rxInputBackupIsEnabled(void) { return fc.backupEnabled; }
bool rxInputBackupIsActive(void) { return fc.backupActive; }
failsafePhase_e failsafePhase(void) { return fc.failsafe; }
bool gpsIsHealthy(void) { return fc.gpsHealthy; }
bool navCanRTH(void) { return fc.canRth; }
bool navCanLoiter(void) { return fc.canLoiter; }
bool IS_RC_MODE_ACTIVE(boxId_e box)
{
    return (box == BOXRTH && fc.rthSwitch) || (box == BOXLOITER && fc.loiterSwitch);
}
bool sensors(uint32_t mask) { return fc.sensorMask & mask; }
batteryState_e getBatteryState(void) { return fc.battery; }
bool mixerTakeStabilizedSaturation(void)
{
    const bool latched = fc.saturationLatch;
    fc.saturationLatch = false;
    return latched;
}
timeMs_t millis(void) { return fc.now; }
bool gyroOverflowDetected(void) { return fc.gyroOverflow; }
bool accHasBeenCalibrated(void) { return fc.accCalibrated; }
bool isServoOverrideActive(void) { return fc.servoOverride; }
bool isMixerOverrideActive(void) { return fc.mixerOverride; }
bool attHoldIsHolding(int axis) { return fc.attHold && axis == 0; }
bool tvHoldIsHolding(int axis) { return fc.tvHold && axis == 1; }
bool autoHoverIsHolding(int axis) { return fc.autoHover && axis == 2; }
bool acroTrainerIsLimiting(int axis) { return fc.trainer && axis == 1; }
autoTrimState_e autoTrimGetState(void) { return fc.autoTrim; }
bool blackboxIsLogging(void) { return fc.blackboxLogging; }
bool isBlackboxDeviceFull(void) { return fc.blackboxFull; }
bool logicConditionGetValue(int index) { return index >= 0 && index < 16 && fc.logic[index]; }
uint8_t getCurrentPidProfileIndex(void) { return fc.pidProfile; }
uint8_t getCurrentControlRateProfileIndex(void) { return fc.ratesProfile; }
uint8_t getCurrentBatteryProfileIndex(void) { return fc.batteryProfile; }
uint8_t getCurrentTvProfileIndex(void) { return fc.tvProfile; }
bool isConfigDirty(void) { return fc.dirty; }
bool isEepromWriteInProgress(void) { return fc.saving; }
bool getRebootRequired(void) { return fc.reboot; }
bool isBeeperOn(void) { return fc.beeper; }
}

class TelemetryStatusTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        // The saturation hold is static state in status.c: move the clock well past any hold
        // left by the previous test and let it expire before starting.
        static timeMs_t clock = 1000000;
        clock += 10000;

        memset(&fc, 0, sizeof(fc));
        armingFlags = 0;
        stateFlags = 0;
        fc.accCalibrated = true;
        fc.now = clock;
        telemetrySystemStatus();
    }

    static uint32_t statusField(int shift, uint32_t mask)
    {
        return (telemetrySystemStatus() >> shift) & mask;
    }
};

// Pins every bit position: the radio Lua decodes by position, so a move here must be matched
// in wingflight-lua-ethos-suite's lib/system_status.lua and wingflight-lua-edgetx.
TEST_F(TelemetryStatusTest, StatusLayoutIsFixed)
{
    armingFlags = ARMED;
    stateFlags = GPS_FIX | GPS_FIX_HOME;
    fc.airborne = fc.motorsRunning = true;
    fc.backupEnabled = fc.backupActive = true;        // main link down: backup in control
    fc.failsafe = FAILSAFE_GPS_RESCUE;                // 6
    fc.gpsHealthy = true;
    fc.rthSwitch = true;                              // no ACC -> RTH blocked
    fc.battery = BATTERY_CRITICAL;                    // 2
    fc.saturationLatch = true;
    fc.gyroOverflow = true;
    fc.servoOverride = true;
    fc.trainer = true;
    fc.autoTrim = AUTOTRIM_SAVE_PENDING;              // 2
    fc.blackboxLogging = true;
    fc.logic[0] = fc.logic[3] = true;
    fc.logic[4] = true;                               // beyond the four reported

    const uint32_t expected =
        (1u << 0) | (1u << 1) | (1u << 2) |           // armed, airborne, motors
        (1u << 4) | (1u << 5) |                       // backup up, backup in control
        (6u << 6) |                                   // failsafe phase
        (2u << 9) | (1u << 11) |                      // fix + home, GPS healthy
        (2u << 12) |                                  // RTH blocked
        (2u << 14) |                                  // battery critical
        (1u << 17) | (1u << 18) |                     // saturated, gyro overflow
        (1u << 20) | (1u << 21) |                     // override, assist
        (2u << 22) |                                  // autotrim save pending
        (1u << 24) |                                  // blackbox logging
        (1u << 25) | (1u << 28);                      // logic conditions 1 and 4

    EXPECT_EQ(expected, telemetrySystemStatus());
}

TEST_F(TelemetryStatusTest, IdleModelReportsNothing)
{
    EXPECT_EQ(0u, telemetrySystemStatus());
}

TEST_F(TelemetryStatusTest, BackupRxOnlyInControlWhenMainLinkIsDown)
{
    fc.rxLink = true;
    fc.backupEnabled = fc.backupActive = true;
    EXPECT_EQ(TELEM_STATUS_RX_LINK_UP | TELEM_STATUS_RX_BACKUP_LINK_UP, telemetrySystemStatus());

    fc.backupEnabled = false;
    EXPECT_EQ(TELEM_STATUS_RX_LINK_UP, telemetrySystemStatus());
}

TEST_F(TelemetryStatusTest, GpsFixField)
{
    EXPECT_EQ(TELEM_GPS_FIX_NONE, statusField(TELEM_STATUS_GPS_FIX_SHIFT, TELEM_STATUS_GPS_FIX_MASK));
    stateFlags = GPS_FIX;
    EXPECT_EQ(TELEM_GPS_FIX_OK, statusField(TELEM_STATUS_GPS_FIX_SHIFT, TELEM_STATUS_GPS_FIX_MASK));
    stateFlags = GPS_FIX | GPS_FIX_HOME;
    EXPECT_EQ(TELEM_GPS_FIX_HOME, statusField(TELEM_STATUS_GPS_FIX_SHIFT, TELEM_STATUS_GPS_FIX_MASK));
}

TEST_F(TelemetryStatusTest, NavBlockedFollowsEngageRules)
{
    auto blocked = [] { return statusField(TELEM_STATUS_NAV_BLOCKED_SHIFT, TELEM_STATUS_NAV_BLOCKED_MASK); };

    fc.canLoiter = fc.canRth = true;
    fc.sensorMask = SENSOR_ACC;

    fc.loiterSwitch = true;
    EXPECT_EQ(TELEM_NAV_BLOCKED_LOITER, blocked());   // disarmed
    armingFlags = ARMED;
    EXPECT_EQ(TELEM_NAV_BLOCKED_NONE, blocked());

    fc.rthSwitch = true;                              // RTH wins over LOITER
    fc.canRth = false;                                // e.g. no home
    EXPECT_EQ(TELEM_NAV_BLOCKED_RTH, blocked());
    fc.canRth = true;
    EXPECT_EQ(TELEM_NAV_BLOCKED_NONE, blocked());

    fc.sensorMask = 0;                                // no ACC
    EXPECT_EQ(TELEM_NAV_BLOCKED_RTH, blocked());

    fc.rthSwitch = fc.loiterSwitch = false;
    EXPECT_EQ(TELEM_NAV_BLOCKED_NONE, blocked());
}

TEST_F(TelemetryStatusTest, ControlSaturationIsHeldBriefly)
{
    fc.saturationLatch = true;
    EXPECT_TRUE(telemetrySystemStatus() & TELEM_STATUS_CONTROL_SATURATED);

    fc.now += 499;
    EXPECT_TRUE(telemetrySystemStatus() & TELEM_STATUS_CONTROL_SATURATED);

    fc.now += 1;
    EXPECT_FALSE(telemetrySystemStatus() & TELEM_STATUS_CONTROL_SATURATED);

    // Once expired it stays clear, even after millis() wraps past the old timestamp.
    fc.now += 0x80000000u;
    EXPECT_FALSE(telemetrySystemStatus() & TELEM_STATUS_CONTROL_SATURATED);
}

TEST_F(TelemetryStatusTest, AccNotCalibratedOnlyWithAcc)
{
    fc.accCalibrated = false;
    EXPECT_FALSE(telemetrySystemStatus() & TELEM_STATUS_ACC_NOT_CALIBRATED);
    fc.sensorMask = SENSOR_ACC;
    EXPECT_TRUE(telemetrySystemStatus() & TELEM_STATUS_ACC_NOT_CALIBRATED);
}

TEST_F(TelemetryStatusTest, AnyOverrideOrAssistSetsItsBit)
{
    fc.motorOverride = true;
    EXPECT_EQ(TELEM_STATUS_OVERRIDE_ACTIVE, telemetrySystemStatus());
    fc.motorOverride = false;
    fc.mixerOverride = true;
    EXPECT_EQ(TELEM_STATUS_OVERRIDE_ACTIVE, telemetrySystemStatus());
    fc.mixerOverride = false;

    fc.attHold = true;
    EXPECT_EQ(TELEM_STATUS_ASSIST_HOLDING, telemetrySystemStatus());
    fc.attHold = false;
    fc.tvHold = true;
    EXPECT_EQ(TELEM_STATUS_ASSIST_HOLDING, telemetrySystemStatus());
    fc.tvHold = false;
    fc.autoHover = true;
    EXPECT_EQ(TELEM_STATUS_ASSIST_HOLDING, telemetrySystemStatus());
}

TEST_F(TelemetryStatusTest, ConfigLayoutIsFixed)
{
    fc.pidProfile = 5;          // reported 1-based
    fc.ratesProfile = 0;
    fc.batteryProfile = 2;
    fc.tvProfile = 3;
    fc.dirty = fc.saving = fc.reboot = fc.beeper = true;
    fc.sensorMask = SENSOR_ACC | SENSOR_BARO | SENSOR_MAG | SENSOR_GPS;
    fc.backupEnabled = true;
    fc.blackboxFull = true;
    fc.rpm = true;

    const uint32_t expected =
        (6u << 0) | (1u << 3) | (3u << 6) | (4u << 9) |
        (1u << 12) | (1u << 13) | (1u << 14) | (1u << 15) |
        (1u << 16) | (1u << 17) | (1u << 18) | (1u << 19) |
        (1u << 20) | (1u << 21) | (1u << 22);

    EXPECT_EQ(expected, telemetrySystemConfig());
}

TEST_F(TelemetryStatusTest, SignBitIsNeverSet)
{
    for (bool *flag : {&fc.airborne, &fc.motorsRunning, &fc.rxLink, &fc.backupEnabled, &fc.backupActive,
            &fc.gpsHealthy, &fc.canRth, &fc.canLoiter, &fc.rthSwitch, &fc.loiterSwitch, &fc.saturationLatch,
            &fc.gyroOverflow, &fc.servoOverride, &fc.mixerOverride, &fc.motorOverride, &fc.attHold,
            &fc.tvHold, &fc.autoHover, &fc.trainer, &fc.blackboxLogging, &fc.blackboxFull,
            &fc.dirty, &fc.saving, &fc.reboot, &fc.beeper, &fc.rpm}) {
        *flag = true;
    }
    for (bool &logic : fc.logic) {
        logic = true;
    }
    fc.sensorMask = UINT32_MAX;
    fc.failsafe = FAILSAFE_GPS_RESCUE;
    fc.battery = BATTERY_INIT;
    fc.autoTrim = AUTOTRIM_SAVE_PENDING;
    fc.pidProfile = fc.ratesProfile = fc.batteryProfile = fc.tvProfile = 5;
    armingFlags = 0xFF;
    stateFlags = 0xFF;

    EXPECT_FALSE(telemetrySystemStatus() & 0x80000000u);
    EXPECT_FALSE(telemetrySystemConfig() & 0x80000000u);
}

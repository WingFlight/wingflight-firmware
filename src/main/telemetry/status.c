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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_TELEMETRY

#include "blackbox/blackbox.h"
#include "blackbox/blackbox_io.h"

#include "common/utils.h"

#include "config/config.h"

#include "drivers/rx_input_backup.h"
#include "drivers/time.h"

#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/airborne.h"
#include "flight/atthold.h"
#include "flight/autohover.h"
#include "flight/autotrim.h"
#include "flight/failsafe.h"
#include "flight/gps_nav.h"
#include "flight/logic_condition.h"
#include "flight/mixer.h"
#include "flight/motors.h"
#include "flight/servos.h"
#include "flight/trainer.h"
#include "flight/tv_hold.h"

#include "io/beeper.h"
#include "io/gps.h"

#include "rx/rx.h"

#include "sensors/acceleration.h"
#include "sensors/battery.h"
#include "sensors/gyro.h"
#include "sensors/sensors.h"

#include "telemetry/status.h"

// How long TELEM_STATUS_CONTROL_SATURATED stays set after the mixer last saturated, so a brief
// hit survives until the next telemetry frame and a radio callout can't chatter.
#define CONTROL_SATURATED_HOLD_MS   500

static uint32_t field(uint32_t value, uint32_t mask, int shift)
{
    return (value & mask) << shift;
}

static telemetryGpsFix_e gpsFix(void)
{
    return STATE(GPS_FIX_HOME) ? TELEM_GPS_FIX_HOME :
        (STATE(GPS_FIX) ? TELEM_GPS_FIX_OK : TELEM_GPS_FIX_NONE);
}

// Mirrors fc/core.c's LOITER/RTH engage rules: RTH wins over LOITER when both switches are on,
// and neither engages while disarmed or without an accelerometer.
static telemetryNavBlocked_e navBlocked(void)
{
#ifdef USE_GPS_NAV
    const bool canFly = ARMING_FLAG(ARMED) && sensors(SENSOR_ACC);

    if (IS_RC_MODE_ACTIVE(BOXRTH)) {
        return (canFly && navCanRTH()) ? TELEM_NAV_BLOCKED_NONE : TELEM_NAV_BLOCKED_RTH;
    }
    if (IS_RC_MODE_ACTIVE(BOXLOITER)) {
        return (canFly && navCanLoiter()) ? TELEM_NAV_BLOCKED_NONE : TELEM_NAV_BLOCKED_LOITER;
    }
#endif
    return TELEM_NAV_BLOCKED_NONE;
}

static bool controlSaturated(void)
{
    static timeMs_t saturatedAt;
    static bool seen;

    const timeMs_t now = millis();

    if (mixerTakeStabilizedSaturation()) {
        saturatedAt = now;
        seen = true;
    }

    if (seen && cmp32(now, saturatedAt) >= CONTROL_SATURATED_HOLD_MS) {
        seen = false;
    }

    return seen;
}

static bool overrideActive(void)
{
    if (isServoOverrideActive() || isMixerOverrideActive()) {
        return true;
    }

    for (int i = 0; i < getMotorCount(); i++) {
        if (hasMotorOverride(i)) {
            return true;
        }
    }

    return false;
}

static bool assistHolding(void)
{
    for (int axis = 0; axis < 3; axis++) {
#ifdef USE_ACC
        if (attHoldIsHolding(axis) || tvHoldIsHolding(axis) || autoHoverIsHolding(axis)) {
            return true;
        }
#endif
#ifdef USE_ACRO_TRAINER
        if (acroTrainerIsLimiting(axis)) {
            return true;
        }
#endif
    }

    return false;
}

uint32_t telemetrySystemStatus(void)
{
    uint32_t status = 0;

    if (ARMING_FLAG(ARMED))
        status |= TELEM_STATUS_ARMED;
    if (isAirborne())
        status |= TELEM_STATUS_AIRBORNE;
    if (areMotorsRunning())
        status |= TELEM_STATUS_MOTORS_RUNNING;

    const bool mainLinkUp = rxIsReceivingSignal();
    if (mainLinkUp)
        status |= TELEM_STATUS_RX_LINK_UP;
#ifdef USE_RX_INPUT_BACKUP
    if (rxInputBackupIsEnabled() && rxInputBackupIsActive()) {
        status |= TELEM_STATUS_RX_BACKUP_LINK_UP;
        if (!mainLinkUp)
            status |= TELEM_STATUS_RX_BACKUP_IN_CONTROL;
    }
#endif

    status |= field(failsafePhase(), TELEM_STATUS_FAILSAFE_PHASE_MASK, TELEM_STATUS_FAILSAFE_PHASE_SHIFT);

#ifdef USE_GPS
    status |= field(gpsFix(), TELEM_STATUS_GPS_FIX_MASK, TELEM_STATUS_GPS_FIX_SHIFT);
    if (gpsIsHealthy())
        status |= TELEM_STATUS_GPS_HEALTHY;
#endif
    status |= field(navBlocked(), TELEM_STATUS_NAV_BLOCKED_MASK, TELEM_STATUS_NAV_BLOCKED_SHIFT);

    status |= field(getBatteryState(), TELEM_STATUS_BATTERY_MASK, TELEM_STATUS_BATTERY_SHIFT);

    if (controlSaturated())
        status |= TELEM_STATUS_CONTROL_SATURATED;
    if (gyroOverflowDetected())
        status |= TELEM_STATUS_GYRO_OVERFLOW;
#ifdef USE_ACC
    if (sensors(SENSOR_ACC) && !accHasBeenCalibrated())
        status |= TELEM_STATUS_ACC_NOT_CALIBRATED;
#endif
    if (overrideActive())
        status |= TELEM_STATUS_OVERRIDE_ACTIVE;
    if (assistHolding())
        status |= TELEM_STATUS_ASSIST_HOLDING;

#ifdef USE_SERVOS
    status |= field(autoTrimGetState(), TELEM_STATUS_AUTOTRIM_MASK, TELEM_STATUS_AUTOTRIM_SHIFT);
#endif

#ifdef USE_BLACKBOX
    if (blackboxIsLogging())
        status |= TELEM_STATUS_BLACKBOX_LOGGING;
#endif

    for (int i = 0; i < TELEM_STATUS_LOGIC_COUNT; i++) {
        if (logicConditionGetValue(i))
            status |= 1u << (TELEM_STATUS_LOGIC_SHIFT + i);
    }

    return status;
}

uint32_t telemetrySystemConfig(void)
{
    uint32_t config = 0;

    config |= field(getCurrentPidProfileIndex() + 1, TELEM_CONFIG_PROFILE_MASK, TELEM_CONFIG_PID_PROFILE_SHIFT);
    config |= field(getCurrentControlRateProfileIndex() + 1, TELEM_CONFIG_PROFILE_MASK, TELEM_CONFIG_RATES_PROFILE_SHIFT);
    config |= field(getCurrentBatteryProfileIndex() + 1, TELEM_CONFIG_PROFILE_MASK, TELEM_CONFIG_BATTERY_PROFILE_SHIFT);
    config |= field(getCurrentTvProfileIndex() + 1, TELEM_CONFIG_PROFILE_MASK, TELEM_CONFIG_TV_PROFILE_SHIFT);

    if (isConfigDirty())
        config |= TELEM_CONFIG_DIRTY;
    if (isEepromWriteInProgress())
        config |= TELEM_CONFIG_SAVING;
    if (getRebootRequired())
        config |= TELEM_CONFIG_REBOOT_REQUIRED;
    if (isBeeperOn())
        config |= TELEM_CONFIG_BEEPER_ON;

    if (sensors(SENSOR_ACC))
        config |= TELEM_CONFIG_ACC_PRESENT;
    if (sensors(SENSOR_BARO))
        config |= TELEM_CONFIG_BARO_PRESENT;
    if (sensors(SENSOR_MAG))
        config |= TELEM_CONFIG_MAG_PRESENT;
    if (sensors(SENSOR_GPS))
        config |= TELEM_CONFIG_GPS_PRESENT;

#ifdef USE_RX_INPUT_BACKUP
    if (rxInputBackupIsEnabled())
        config |= TELEM_CONFIG_RX_BACKUP_CONFIGURED;
#endif
#ifdef USE_BLACKBOX
    if (isBlackboxDeviceFull())
        config |= TELEM_CONFIG_BLACKBOX_FULL;
#endif
    if (isRpmSourceActive())
        config |= TELEM_CONFIG_RPM_SOURCE_ACTIVE;

    return config;
}

#endif /* USE_TELEMETRY */

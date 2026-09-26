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

#pragma once

#include <stdint.h>

/*
 * Packed status words for the TELEM_SYSTEM_STATUS and TELEM_SYSTEM_CONFIG telemetry sensors.
 *
 * The radio-side Lua (wingflight-lua-ethos-suite lib/system_status.lua, wingflight-lua-edgetx)
 * decode these by bit position, so any change to the layout must ship together with the matching
 * Lua update. Bit 31 of both words stays clear because SmartPort carries the value as a signed
 * 32-bit int.
 *
 * A field that is not compiled into a build (no GPS, no backup RX, ...) reads as 0.
 */

/** TELEM_SYSTEM_STATUS: live state that drives radio callouts **/

#define TELEM_STATUS_ARMED                  (1u << 0)   // armingFlags ARMED
#define TELEM_STATUS_AIRBORNE               (1u << 1)   // isAirborne()
#define TELEM_STATUS_MOTORS_RUNNING         (1u << 2)   // areMotorsRunning()
#define TELEM_STATUS_RX_LINK_UP             (1u << 3)   // main RX receiving
#define TELEM_STATUS_RX_BACKUP_LINK_UP      (1u << 4)   // backup RX has fresh frames
#define TELEM_STATUS_RX_BACKUP_IN_CONTROL   (1u << 5)   // main RX down, backup RX flying the model

#define TELEM_STATUS_FAILSAFE_PHASE_SHIFT   6           // failsafePhase_e, 0 = idle
#define TELEM_STATUS_FAILSAFE_PHASE_MASK    0x7u

#define TELEM_STATUS_GPS_FIX_SHIFT          9           // telemetryGpsFix_e
#define TELEM_STATUS_GPS_FIX_MASK           0x3u
#define TELEM_STATUS_GPS_HEALTHY            (1u << 11)  // GPS module is talking to the FC

#define TELEM_STATUS_NAV_BLOCKED_SHIFT      12          // telemetryNavBlocked_e
#define TELEM_STATUS_NAV_BLOCKED_MASK       0x3u

#define TELEM_STATUS_BATTERY_SHIFT          14          // batteryState_e
#define TELEM_STATUS_BATTERY_MASK           0x7u

#define TELEM_STATUS_CONTROL_SATURATED      (1u << 17)  // stabilized roll/pitch/yaw hit its mixer limit recently
#define TELEM_STATUS_GYRO_OVERFLOW          (1u << 18)  // gyroOverflowDetected()
#define TELEM_STATUS_ACC_NOT_CALIBRATED     (1u << 19)  // ACC present but never calibrated
#define TELEM_STATUS_OVERRIDE_ACTIVE        (1u << 20)  // Configurator servo/motor/mixer test override on
#define TELEM_STATUS_ASSIST_HOLDING         (1u << 21)  // ATTHOLD/TV hold/autohover holding, or acro trainer limiting

#define TELEM_STATUS_AUTOTRIM_SHIFT         22          // autoTrimState_e
#define TELEM_STATUS_AUTOTRIM_MASK          0x3u

#define TELEM_STATUS_BLACKBOX_LOGGING       (1u << 24)  // Blackbox is writing a log

#define TELEM_STATUS_LOGIC_SHIFT            25          // logic conditions 1-4, one bit each
#define TELEM_STATUS_LOGIC_COUNT            4

// Bits 29-30 spare, bit 31 reserved.

/** TELEM_SYSTEM_CONFIG: slow-changing configuration and hardware state **/

#define TELEM_CONFIG_PID_PROFILE_SHIFT      0           // 1-based profile number
#define TELEM_CONFIG_RATES_PROFILE_SHIFT    3
#define TELEM_CONFIG_BATTERY_PROFILE_SHIFT  6
#define TELEM_CONFIG_TV_PROFILE_SHIFT       9
#define TELEM_CONFIG_PROFILE_MASK           0x7u

#define TELEM_CONFIG_DIRTY                  (1u << 12)  // settings changed but not saved
#define TELEM_CONFIG_SAVING                 (1u << 13)  // EEPROM write in progress
#define TELEM_CONFIG_REBOOT_REQUIRED        (1u << 14)
#define TELEM_CONFIG_BEEPER_ON              (1u << 15)  // beeper sounding (e.g. lost model)
#define TELEM_CONFIG_ACC_PRESENT            (1u << 16)
#define TELEM_CONFIG_BARO_PRESENT           (1u << 17)
#define TELEM_CONFIG_MAG_PRESENT            (1u << 18)
#define TELEM_CONFIG_GPS_PRESENT            (1u << 19)
#define TELEM_CONFIG_RX_BACKUP_CONFIGURED   (1u << 20)
#define TELEM_CONFIG_BLACKBOX_FULL          (1u << 21)
#define TELEM_CONFIG_RPM_SOURCE_ACTIVE      (1u << 22)  // motor RPM telemetry is arriving

// Bits 23-30 spare, bit 31 reserved.

typedef enum {
    TELEM_GPS_FIX_NONE = 0,
    TELEM_GPS_FIX_OK,
    TELEM_GPS_FIX_HOME,             // fix, and home position captured
} telemetryGpsFix_e;

// LOITER/RTH switched on but the mode can't fly: disarmed, no ACC, no usable fix, or (RTH) no home.
typedef enum {
    TELEM_NAV_BLOCKED_NONE = 0,
    TELEM_NAV_BLOCKED_LOITER,
    TELEM_NAV_BLOCKED_RTH,
} telemetryNavBlocked_e;

uint32_t telemetrySystemStatus(void);
uint32_t telemetrySystemConfig(void);

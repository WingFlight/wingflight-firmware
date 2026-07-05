/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "common/utils.h"

// FIXME some of these are flight modes, some of these are general status indicators
typedef enum {
    ARMED                       = (1 << 0),
    WAS_EVER_ARMED              = (1 << 1),
    WAS_ARMED_WITH_PREARM       = (1 << 2)
} armingFlag_e;

extern uint8_t armingFlags;

#define DISABLE_ARMING_FLAG(mask) (armingFlags &= ~(mask))
#define ENABLE_ARMING_FLAG(mask) (armingFlags |= (mask))
#define ARMING_FLAG(mask) (armingFlags & (mask))

/*
 * Arming disable flags are listed in the order of criticalness.
 * (Beeper code can notify the most critical reason.)
 */
typedef enum {
    ARMING_DISABLED_NO_GYRO         = (1 << 0),
    ARMING_DISABLED_FAILSAFE        = (1 << 1),
    ARMING_DISABLED_RX_FAILSAFE     = (1 << 2),
    ARMING_DISABLED_BAD_RX_RECOVERY = (1 << 3),
    ARMING_DISABLED_BOXFAILSAFE     = (1 << 4),
    ARMING_DISABLED_GOVERNOR        = (1 << 5), // reserved (heli governor removed) -- never set
    ARMING_DISABLED_RPM_SIGNAL      = (1 << 6),
    ARMING_DISABLED_THROTTLE        = (1 << 7),
    ARMING_DISABLED_ANGLE           = (1 << 8),
    ARMING_DISABLED_BOOT_GRACE_TIME = (1 << 9),
    ARMING_DISABLED_NOPREARM        = (1 << 10),
    ARMING_DISABLED_LOAD            = (1 << 11),
    ARMING_DISABLED_CALIBRATING     = (1 << 12),
    ARMING_DISABLED_CLI             = (1 << 13),
    ARMING_DISABLED_CMS_MENU        = (1 << 14),
    ARMING_DISABLED_BST             = (1 << 15),
    ARMING_DISABLED_MSP             = (1 << 16),
    ARMING_DISABLED_PARALYZE        = (1 << 17),
    ARMING_DISABLED_GPS             = (1 << 18),
    ARMING_DISABLED_RESC            = (1 << 19),
    ARMING_DISABLED_RPMFILTER       = (1 << 20),
    ARMING_DISABLED_REBOOT_REQUIRED = (1 << 21),
    ARMING_DISABLED_DSHOT_BITBANG   = (1 << 22),
    ARMING_DISABLED_ACC_CALIBRATION = (1 << 23),
    ARMING_DISABLED_MOTOR_PROTOCOL  = (1 << 24),
    ARMING_DISABLED_OVERRIDE        = (1 << 25),
    ARMING_DISABLED_ARM_SWITCH      = (1 << 26), // Needs to be the last element, since it's always activated if one of the others is active when arming
} armingDisableFlags_e;

#define ARMING_DISABLE_FLAGS_COUNT (LOG2(ARMING_DISABLED_ARM_SWITCH) + 1)

extern const char *armingDisableFlagNames[ARMING_DISABLE_FLAGS_COUNT];

void setArmingDisabled(armingDisableFlags_e flag);
void unsetArmingDisabled(armingDisableFlags_e flag);
bool isArmingDisabled(void);
armingDisableFlags_e getArmingDisableFlags(void);

typedef enum {
    FAILSAFE_MODE_BIT    = 0,
    ANGLE_MODE_BIT       = 1,
    HORIZON_MODE_BIT     = 2,
    TRAINER_MODE_BIT     = 3,
    ALTHOLD_MODE_BIT     = 4,
    ATTHOLD_MODE_BIT     = 5,  // reserved (attitude hold removed) -- kept to avoid renumbering later bits
    GPS_RESCUE_MODE_BIT  = 6,
    PASSTHROUGH_MODE_BIT = 7,
    INFLIGHT_MODE_BIT    = 8,
    AUTOHOVER_MODE_BIT   = 9,
    MANUAL_MODE_BIT      = 10,
} flightModeBits_e;

typedef enum {
    FAILSAFE_MODE        = BIT(FAILSAFE_MODE_BIT),
    ANGLE_MODE           = BIT(ANGLE_MODE_BIT),
    HORIZON_MODE         = BIT(HORIZON_MODE_BIT),
    TRAINER_MODE         = BIT(TRAINER_MODE_BIT),
    ALTHOLD_MODE         = BIT(ALTHOLD_MODE_BIT),
    ATTHOLD_MODE         = BIT(ATTHOLD_MODE_BIT),  // reserved (attitude hold removed) -- kept to avoid renumbering later bits
    GPS_RESCUE_MODE      = BIT(GPS_RESCUE_MODE_BIT),
    PASSTHROUGH_MODE     = BIT(PASSTHROUGH_MODE_BIT),
    // Latched "are we actually flying" signal for telemetry/dashboard consumers, not a pilot-
    // selectable mode. Engaging IDLE UP is treated as a deliberate "starting the flight" signal
    // (robust even at idle throttle -- unlike a bare throttle threshold, which a plane legitimately
    // dips below in normal flight); a plain throttle threshold and, where a baro/GPS fix is
    // available, altitude gain above the armed baseline (for gentle/glider-style launches that
    // never cross the throttle threshold) are the fallbacks for setups that don't use idle up.
    // Once set, holds through momentary throttle/mode dips so mid-flight noise doesn't flicker it
    // back off -- it only clears on disarm, or earlier if idle up was used this flight and gets
    // switched off with throttle back at idle (read as "landed", since some pilots delay disarming
    // while taxiing back).
    INFLIGHT_MODE        = BIT(INFLIGHT_MODE_BIT),
    // AUTO HOVER: quaternion-based hold of a vertical (90 degree pitch) attitude and heading, for
    // 3D "prop hang" hover. See flight/autohover.c for why this can't reuse ANGLE_MODE/HORIZON_MODE's
    // Euler-angle leveling code (gimbal lock exactly at the target attitude).
    AUTOHOVER_MODE       = BIT(AUTOHOVER_MODE_BIT),
    // MANUAL: pilot flies the same rate/expo curve as stabilised (rate) flight, but the mixer
    // takes that shaped setpoint directly instead of the gyro-corrected PID output -- no
    // stabilisation at all, unlike PASSTHROUGH_MODE which also strips the rates/expo curve itself
    // and feeds raw radio deflection straight through.
    MANUAL_MODE          = BIT(MANUAL_MODE_BIT),
} flightModeFlags_e;

extern uint16_t flightModeFlags;

#define DISABLE_FLIGHT_MODE(mask) disableFlightMode(mask)
#define ENABLE_FLIGHT_MODE(mask) enableFlightMode(mask)
#define FLIGHT_MODE(mask) (flightModeFlags & (mask))

// macro to initialize map from boxId_e flightModeBits. Keep it in sync with flightModeFlags_e enum.
// [BOXARM], [BOXRESCUE] (heli rescue removed), and [BOXATTHOLD] (attitude hold removed) are left unpopulated
#define BOXID_TO_FLIGHT_MODE_MAP_INITIALIZER {           \
   [BOXANGLE]       = ANGLE_MODE_BIT,                    \
   [BOXHORIZON]     = HORIZON_MODE_BIT,                  \
   [BOXTRAINER]     = TRAINER_MODE_BIT,                  \
   [BOXALTHOLD]     = ALTHOLD_MODE_BIT,                  \
   [BOXGPSRESCUE]   = GPS_RESCUE_MODE_BIT,               \
   [BOXFAILSAFE]    = FAILSAFE_MODE_BIT,                 \
   [BOXPASSTHROUGH] = PASSTHROUGH_MODE_BIT,              \
}                                                        \
/**/

typedef enum {
    GPS_FIX_HOME   = (1 << 0),
    GPS_FIX        = (1 << 1),
    GPS_FIX_EVER   = (1 << 2),
} stateFlags_t;

#define DISABLE_STATE(mask) (stateFlags &= ~(mask))
#define ENABLE_STATE(mask) (stateFlags |= (mask))
#define STATE(mask) (stateFlags & (mask))

extern uint8_t stateFlags;

uint16_t enableFlightMode(flightModeFlags_e mask);
uint16_t disableFlightMode(flightModeFlags_e mask);

bool sensors(uint32_t mask);
void sensorsSet(uint32_t mask);
void sensorsClear(uint32_t mask);
uint32_t sensorsMask(void);

void mwDisarm(void);

/*
 * Wingflight platform identifier.
 * Define WINGFLIGHT_AIRPLANE in the target's target.h (or via make EXTRA_FLAGS=-DWINGFLIGHT_AIRPLANE)
 * to enable fixed-wing-specific code paths. All wing additions should be guarded with this flag
 * to keep the diff small and upstream-merge-friendly.
 */
// #define WINGFLIGHT_AIRPLANE

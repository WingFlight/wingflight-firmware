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

/*
 * Tables the CLI text formats need that nothing in the firmware keeps.
 *
 * `feature`, `serial`, `aux`, `map` and `mixer` lines are part of every backup, and the
 * configurator's CLI has to print and parse them. Most of what that takes is
 * already in the image -- baudRates, serialPortIdentifiers, the box table,
 * rcChannelLetters -- and the manifest generator reads those where they are.
 * These are the leftovers: featureNames and the mixer names lived in cli.c
 * and went with it, and the ranges the CLI checked are macros, which have no
 * symbol to read.
 *
 * Like settings.c and resources.c this is build-time metadata, compiled only
 * into the manifest build and never flashed.
 */

#include <stdint.h>

#include "platform.h"

#include "build/build_config.h"

#include "common/color.h"
#include "common/utils.h"

#include "fc/rc_adjustments.h"

#include "flight/mixer.h"
#include "flight/servos.h"

#include "io/beeper.h"
#include "io/ledstrip.h"
#include "io/serial.h"

#include "pg/bus_servo.h"
#include "pg/logic_condition.h"
#include "pg/mixer.h"

#include "rx/rx.h"

// Indexed by feature bit. An empty name marks a bit that is not a feature
// here; the CLI skipped those, and so does the configurator.
const char * const featureNames[] = {
    [ 0] = "RX_PPM",
    [ 1] = "",
    [ 2] = "",
    [ 3] = "RX_SERIAL",
    [ 4] = "",
    [ 5] = "",
    [ 6] = "SOFTSERIAL",
    [ 7] = "GPS",
    [ 8] = "",
    [ 9] = "RANGEFINDER",
    [10] = "TELEMETRY",
    [11] = "",
    [12] = "",
    [13] = "RX_PARALLEL_PWM",
    [14] = "RX_MSP",
    [15] = "RSSI_ADC",
    [16] = "LED_STRIP",
    [17] = "DASHBOARD",
    [18] = "",
    [19] = "",
    [20] = "",
    [21] = "",
    [22] = "",
    [23] = "",
    [24] = "THRUST_VECTOR",
    [25] = "RX_SPI",
    [26] = "",
    [27] = "ESC_SENSOR",
    [28] = "FREQ_SENSOR",
    [29] = "DYN_NOTCH",
    [30] = "RPM_FILTER",
    [31] = NULL
};

// `aux` rejected a channel index at or above this.
const uint8_t cliAuxChannelCount = MAX_AUX_CHANNEL_COUNT;

// `mixer input` / `mixer rule` name inputs, outputs and operations by these.
const char * const mixerInputNames[MIXER_INPUT_COUNT] = {
    [MIXER_IN_NONE]                  = "-",
    [MIXER_IN_STABILIZED_ROLL]       = "SR",
    [MIXER_IN_STABILIZED_PITCH]      = "SP",
    [MIXER_IN_STABILIZED_YAW]        = "SY",
    [MIXER_IN_STABILIZED_THROTTLE]   = "ST",
    [MIXER_IN_RC_COMMAND_ROLL]       = "CR",
    [MIXER_IN_RC_COMMAND_PITCH]      = "CP",
    [MIXER_IN_RC_COMMAND_YAW]        = "CY",
    [MIXER_IN_RC_COMMAND_THROTTLE]   = "CT",
    [MIXER_IN_RC_CHANNEL_ROLL]       = "RR",
    [MIXER_IN_RC_CHANNEL_PITCH]      = "RP",
    [MIXER_IN_RC_CHANNEL_YAW]        = "RY",
    [MIXER_IN_RC_CHANNEL_THROTTLE]   = "RT",
    [MIXER_IN_RC_CHANNEL_AUX1]       = "AUX1",
    [MIXER_IN_RC_CHANNEL_AUX2]       = "AUX2",
    [MIXER_IN_RC_CHANNEL_AUX3]       = "AUX3",
    [MIXER_IN_RC_CHANNEL_8]          = "CH8",
    [MIXER_IN_RC_CHANNEL_9]          = "CH9",
    [MIXER_IN_RC_CHANNEL_10]         = "CH10",
    [MIXER_IN_RC_CHANNEL_11]         = "CH11",
    [MIXER_IN_RC_CHANNEL_12]         = "CH12",
    [MIXER_IN_RC_CHANNEL_13]         = "CH13",
    [MIXER_IN_RC_CHANNEL_14]         = "CH14",
    [MIXER_IN_RC_CHANNEL_15]         = "CH15",
    [MIXER_IN_RC_CHANNEL_16]         = "CH16",
    [MIXER_IN_RC_CHANNEL_17]         = "CH17",
    [MIXER_IN_RC_CHANNEL_18]         = "CH18",
    [MIXER_IN_STABILIZED_TV_ROLL]    = "TR",
    [MIXER_IN_STABILIZED_TV_PITCH]   = "TP",
    [MIXER_IN_STABILIZED_TV_YAW]     = "TY",
};

// Servos S1..S26, then motors M1..M4.
const char * const mixerOutputNames[] = {
    "-", "S1", "S2", "S3", "S4", "S5", "S6", "S7", "S8", "S9", "S10", "S11", "S12", "S13", "S14", "S15", "S16", "S17", "S18",
    "S19", "S20", "S21", "S22", "S23", "S24", "S25", "S26", "M1", "M2", "M3", "M4"
};
STATIC_ASSERT(ARRAYLEN(mixerOutputNames) == MIXER_OUTPUT_COUNT, mixer_output_names_match_output_count);

const char * const mixerOpNames[MIXER_OP_COUNT] = {
    [MIXER_OP_NUL]     = "-",
    [MIXER_OP_SET]     = "set",
    [MIXER_OP_ADD]     = "add",
    [MIXER_OP_MUL]     = "mul",
};

// `status` names the MCU and the configuration state by these.
const char * const mcuTypeNames[] = {
    [MCU_TYPE_SIMULATOR]         = "SIMULATOR",
    [MCU_TYPE_F40X]              = "F40X",
    [MCU_TYPE_F411]              = "F411",
    [MCU_TYPE_F446]              = "F446",
    [MCU_TYPE_F722]              = "F722",
    [MCU_TYPE_F745]              = "F745",
    [MCU_TYPE_F746]              = "F746",
    [MCU_TYPE_F765]              = "F765",
    [MCU_TYPE_H750]              = "H750",
    [MCU_TYPE_H743_REV_UNKNOWN]  = "H743 (Rev Unknown)",
    [MCU_TYPE_H743_REV_Y]        = "H743 (Rev.Y)",
    [MCU_TYPE_H743_REV_X]        = "H743 (Rev.X)",
    [MCU_TYPE_H743_REV_V]        = "H743 (Rev.V)",
    [MCU_TYPE_H7A3]              = "H7A3",
    [MCU_TYPE_H723_725]          = "H723/H725",
    [MCU_TYPE_G474]              = "G474",
    [MCU_TYPE_H730]              = "H730",
};

const char * const configurationStateNames[] = { "UNCONFIGURED", "CUSTOM DEFAULTS", "CONFIGURED" };

/*
 * Ranges the CLI validated arguments against. They are macros, so they have
 * no symbol of their own; the generator reads this struct member by member
 * through DWARF, so the order here does not matter to it.
 */
typedef struct cliLimits_s {
    int32_t mixerInputMin;
    int32_t mixerInputMax;
    int32_t mixerRateMin;
    int32_t mixerRateMax;
    int32_t mixerWeightMin;
    int32_t mixerWeightMax;
    int32_t servoSpeedMin;
    int32_t servoSpeedMax;
    int32_t mixerCurveCount;
    int32_t logicConditionCount;
    int32_t mixerRuleRoleCount;
    // servo
    int32_t servoPulseMin;
    int32_t servoPulseMax;
    int32_t servoLimitMin;
    int32_t servoLimitMax;
    int32_t servoScaleMin;
    int32_t servoScaleMax;
    int32_t servoRateMin;
    int32_t servoRateMax;
    int32_t servoFlagsAll;
    int32_t busServoOffset;
    int32_t busServoFunctionMask;   // serial functions that make S9.. bus servos
    // rxfail
    int32_t rxfailPulseMin;
    int32_t rxfailPulseMax;
    int32_t rxfailRangeMax;
    int32_t controlChannelCount;
    // adjfunc
    int32_t adjustmentFunctionCount;
    // beeper / beacon
    int32_t beeperAll;                  // the BEEPER_ALL mode
    int32_t beeperAllowedModes;
    int32_t dshotBeaconAllowedModes;
    int32_t dshotBeacon;                // 1 if `beacon` exists (USE_DSHOT)
    // led / color / mode_color
    int32_t hsvHueMax;
    int32_t hsvSaturationMax;
    int32_t hsvValueMax;
    int32_t ledSpecial;                 // mode_color index of the special colors
    int32_t ledAuxChannel;              // mode_color index of the aux channel
    // action commands: what this build can do, so the client refuses rather than guesses
    int32_t flashBootLoader;            // `bl` defaults to the flash bootloader
    int32_t escSerial;                  // `escprog` exists
    int32_t gpsFunctionBit;             // `gpspassthrough`: bit index of FUNCTION_GPS
    int32_t escSensorFunctionBit;       // `serialpassthrough esc_sensor`
} cliLimits_t;

const cliLimits_t cliLimits = {
    .mixerInputMin       = MIXER_INPUT_MIN,
    .mixerInputMax       = MIXER_INPUT_MAX,
    .mixerRateMin        = MIXER_RATE_MIN,
    .mixerRateMax        = MIXER_RATE_MAX,
    .mixerWeightMin      = MIXER_WEIGHT_MIN,
    .mixerWeightMax      = MIXER_WEIGHT_MAX,
    .servoSpeedMin       = SERVO_SPEED_MIN,
    .servoSpeedMax       = SERVO_SPEED_MAX,
    .mixerCurveCount     = MIXER_CURVE_COUNT,
    .logicConditionCount = LOGIC_CONDITION_COUNT,
    .mixerRuleRoleCount  = MIXER_RULE_ROLE_COUNT,
    .servoPulseMin       = PWM_SERVO_PULSE_MIN,
    .servoPulseMax       = PWM_SERVO_PULSE_MAX,
    .servoLimitMin       = SERVO_LIMIT_MIN,
    .servoLimitMax       = SERVO_LIMIT_MAX,
    .servoScaleMin       = SERVO_SCALE_MIN,
    .servoScaleMax       = SERVO_SCALE_MAX,
    .servoRateMin        = SERVO_RATE_MIN,
    .servoRateMax        = SERVO_RATE_MAX,
    .servoFlagsAll       = SERVO_FLAGS_ALL,
    .busServoOffset      = BUS_SERVO_OFFSET,
    .busServoFunctionMask = FUNCTION_SBUS_OUT | FUNCTION_FBUS_MASTER,
    .rxfailPulseMin      = RXFAIL_PULSE_MIN,
    .rxfailPulseMax      = RXFAIL_PULSE_MAX,
    .rxfailRangeMax      = RXFAIL_RANGE_MAX,
    .controlChannelCount = CONTROL_CHANNEL_COUNT,
    .adjustmentFunctionCount = ADJUSTMENT_FUNCTION_COUNT,
    .beeperAll           = BEEPER_ALL,
    .beeperAllowedModes  = BEEPER_ALLOWED_MODES,
    .dshotBeaconAllowedModes = DSHOT_BEACON_ALLOWED_MODES,
#ifdef USE_DSHOT
    .dshotBeacon         = 1,
#else
    .dshotBeacon         = 0,
#endif
    .hsvHueMax           = HSV_HUE_MAX,
    .hsvSaturationMax    = HSV_SATURATION_MAX,
    .hsvValueMax         = HSV_VALUE_MAX,
    .ledSpecial          = LED_SPECIAL,
    .ledAuxChannel       = LED_AUX_CHANNEL,
#ifdef USE_FLASH_BOOT_LOADER
    .flashBootLoader     = 1,
#endif
#ifdef USE_ESCSERIAL
    .escSerial           = 1,
#endif
    .gpsFunctionBit      = __builtin_ctz(FUNCTION_GPS),
    .escSensorFunctionBit = __builtin_ctz(FUNCTION_ESC_SENSOR),
};

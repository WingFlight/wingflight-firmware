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

#include "platform.h"

#define DEBUG_VALUE_COUNT 8

extern uint8_t debugMode;
extern uint8_t debugAxis;

extern int32_t debug[DEBUG_VALUE_COUNT];

extern uint32_t __timing[DEBUG_VALUE_COUNT];

#define DEBUG_SET(mode, index, value)             do { if (debugMode == (mode)) { debug[(index)] = (value); } } while (0)
#define DEBUG_AXIS_SET(mode, axis, index, value)  do { if (debugAxis == (axis) && debugMode == (mode)) { debug[(index)] = (value); } } while (0)
#define DEBUG_COND_SET(mode, cond, index, value)  do { if ((cond) && debugMode == (mode)) { debug[(index)] = (value); } } while (0)

#define DEBUG(mode, index, value)                 DEBUG_SET(DEBUG_ ## mode, index, value)
#define DEBUG_AXIS(mode, axis, index, value)      DEBUG_AXIS_SET(DEBUG_ ## mode, axis, index, value)
#define DEBUG_COND(mode, cond, index, value)      DEBUG_COND_SET(DEBUG_ ## mode, cond, index, value)

#define DEBUG_TIME_START(mode, index)             do { if (debugMode == (DEBUG_ ## mode)) { __timing[(index)] = micros(); } } while (0)
#define DEBUG_TIME_END(mode, index)               do { if (debugMode == (DEBUG_ ## mode)) { debug[(index)] = micros() - __timing[(index)]; } } while (0)


typedef enum {
    DEBUG_NONE,
    DEBUG_CYCLETIME,
    DEBUG_BATTERY,
    DEBUG_GYRO_FILTERED,
    DEBUG_ACCELEROMETER,
    DEBUG_PIDLOOP,
    DEBUG_GYRO_SCALED,
    DEBUG_RC_COMMAND,
    DEBUG_UNUSED_8,     // was RC_SETPOINT -- slot kept so later modes keep their numbers
    DEBUG_ESC_SENSOR,
    DEBUG_SCHEDULER,
    DEBUG_STACK,
    DEBUG_ESC_SENSOR_DATA,
    DEBUG_ESC_SENSOR_FRAME,
    DEBUG_ALTITUDE,
    DEBUG_DYN_NOTCH,
    DEBUG_DYN_NOTCH_TIME,
    DEBUG_DYN_NOTCH_FREQ,
    DEBUG_RX_FRSKY_SPI,
    DEBUG_RX_SFHSS_SPI,
    DEBUG_GYRO_RAW,
    DEBUG_DUAL_GYRO_RAW,
    DEBUG_DUAL_GYRO_DIFF,
    DEBUG_SBUS,
    DEBUG_FPORT,
    DEBUG_UNUSED_25,    // was RANGEFINDER -- slot kept so later modes keep their numbers
    DEBUG_UNUSED_26,    // was RANGEFINDER_QUALITY -- slot kept so later modes keep their numbers
    DEBUG_UNUSED_27,    // was LIDAR_TF -- slot kept so later modes keep their numbers
    DEBUG_ADC_INTERNAL,
    DEBUG_UNUSED_29,    // was GOVERNOR -- slot kept so later modes keep their numbers
    DEBUG_UNUSED_30,    // was SDIO -- slot kept so later modes keep their numbers
    DEBUG_CURRENT_SENSOR,
    DEBUG_USB,
    DEBUG_UNUSED_33,    // was SMARTAUDIO -- slot kept so later modes keep their numbers
    DEBUG_RTH,
    DEBUG_ITERM_RELAX,
    DEBUG_ACRO_TRAINER,
    DEBUG_SETPOINT,
    DEBUG_RX_SIGNAL_LOSS,
    DEBUG_RC_RAW,
    DEBUG_RC_DATA,
    DEBUG_DYN_LPF,
    DEBUG_RX_SPEKTRUM_SPI,
    DEBUG_DSHOT_RPM_TELEMETRY,
    DEBUG_RPM_FILTER,
    DEBUG_RPM_SOURCE,
    DEBUG_UNUSED_46,    // was TTA -- slot kept so later modes keep their numbers
    DEBUG_AIRBORNE,
    DEBUG_DUAL_GYRO_SCALED,
    DEBUG_DSHOT_RPM_ERRORS,
    DEBUG_CRSF_LINK_STATISTICS_UPLINK,
    DEBUG_CRSF_LINK_STATISTICS_PWR,
    DEBUG_CRSF_LINK_STATISTICS_DOWN,
    DEBUG_BARO,
    DEBUG_GPS_RESCUE_THROTTLE_PID,
    DEBUG_FREQ_SENSOR,
    DEBUG_UNUSED_56,    // was FEEDFORWARD_LIMIT -- slot kept so later modes keep their numbers
    DEBUG_UNUSED_57,    // was FEEDFORWARD -- slot kept so later modes keep their numbers
    DEBUG_BLACKBOX_OUTPUT,
    DEBUG_GYRO_SAMPLE,
    DEBUG_RX_TIMING,
    DEBUG_UNUSED_61,    // was D_LPF -- slot kept so later modes keep their numbers
    DEBUG_UNUSED_62,    // was VTX_TRAMP -- slot kept so later modes keep their numbers
    DEBUG_GHST,
    DEBUG_SCHEDULER_DETERMINISM,
    DEBUG_TIMING_ACCURACY,
    DEBUG_RX_EXPRESSLRS_SPI,
    DEBUG_RX_EXPRESSLRS_PHASELOCK,
    DEBUG_RX_STATE_TIME,
    DEBUG_UNUSED_69,    // was PITCH_PRECOMP -- slot kept so later modes keep their numbers
    DEBUG_UNUSED_70,    // was YAW_PRECOMP -- slot kept so later modes keep their numbers
    DEBUG_UNUSED_71,    // was RESCUE -- slot kept so later modes keep their numbers
    DEBUG_UNUSED_72,    // was RESCUE_ALTHOLD -- slot kept so later modes keep their numbers
    DEBUG_UNUSED_73,    // was CROSS_COUPLING -- slot kept so later modes keep their numbers
    DEBUG_UNUSED_74,    // was ERROR_DECAY -- slot kept so later modes keep their numbers
    DEBUG_UNUSED_75,    // was HS_OFFSET -- slot kept so later modes keep their numbers
    DEBUG_UNUSED_76,    // was HS_BLEED -- slot kept so later modes keep their numbers
    DEBUG_UNUSED_77,    // was GOV_MOTOR -- slot kept so later modes keep their numbers
    DEBUG_GYRO_CALIBRATION,
    DEBUG_AUTOHOVER,
    DEBUG_ATTHOLD,
    DEBUG_TVHOLD,
    DEBUG_COUNT
} debugType_e;

extern const char * const debugModeNames[DEBUG_COUNT];

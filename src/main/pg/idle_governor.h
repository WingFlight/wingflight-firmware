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

#include "pg/pg.h"

typedef enum {
    IDLE_GOVERNOR_MODE_OFF = 0,        // feature inert, throttle passthrough
    IDLE_GOVERNOR_MODE_RPM,            // closed-loop: hold idle_governor_rpm using motor RPM feedback
    IDLE_GOVERNOR_MODE_THROTTLE,       // open-loop: hold a fixed idle_governor_throttle %, no RPM source needed
} idleGovernorMode_e;

typedef struct idleGovernorConfig_s {
    uint8_t  idle_governor_mode;      // idleGovernorMode_e
    uint16_t idle_governor_rpm;       // target idle RPM (RPM mode)
    uint16_t idle_governor_gain;      // P gain, scaled (x0.0001 -> throttle fraction per RPM error) (RPM mode)
    uint8_t  idle_governor_throttle;  // fixed idle throttle output, % (0-100) (THROTTLE mode)
    uint8_t  idle_governor_handover;  // throttle handover threshold, % (0-100)
    uint8_t  idle_governor_ceiling;   // max throttle % the governor may output (safety clamp)
} idleGovernorConfig_t;

PG_DECLARE(idleGovernorConfig_t, idleGovernorConfig);

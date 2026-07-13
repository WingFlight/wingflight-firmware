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
    GOVERNOR_MODE_OFF = 0,        // feature inert, throttle passthrough
    GOVERNOR_MODE_RPM,            // closed-loop: hold governor_rpm using motor RPM feedback, below handover only
    GOVERNOR_MODE_THROTTLE,       // open-loop: hold a fixed governor_throttle %, no RPM source needed, below handover only
    GOVERNOR_MODE_RPM_RANGE,      // closed-loop: hold a target RPM mapped across the *entire* throttle
                                   // stick (governor_rpm_min at 0% -> governor_rpm_max at 100%),
                                   // no handover split -- for holding engine speed through a whole flight
                                   // (e.g. dive speed control), not just idle
} governorMode_e;

typedef struct governorConfig_s {
    uint8_t  governor_mode;      // governorMode_e
    uint16_t governor_rpm;       // target idle RPM (RPM mode)
    uint16_t governor_gain;      // P gain, scaled (x0.000005 -> throttle fraction per RPM error) (RPM mode)
    uint16_t governor_i_gain;    // I gain, scaled (x0.000005 -> throttle fraction per RPM error per second); closes P droop to zero (RPM mode)
    uint8_t  governor_throttle;  // fixed idle throttle output, % (0-100) (THROTTLE mode)
    uint8_t  governor_handover;  // throttle handover threshold, % (0-100) (RPM/THROTTLE modes only)
    uint8_t  governor_ceiling;   // max throttle % the governor may output (safety clamp)
    uint16_t governor_rpm_min;   // target RPM at 0% throttle stick (RPM_RANGE mode)
    uint16_t governor_rpm_max;   // target RPM at 100% throttle stick (RPM_RANGE mode); 0 = not configured
} governorConfig_t;

PG_DECLARE(governorConfig_t, governorConfig);

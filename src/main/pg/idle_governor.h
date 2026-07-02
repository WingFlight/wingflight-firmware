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

typedef struct idleGovernorConfig_s {
    uint16_t idle_governor_rpm;       // target idle RPM; 0 = feature inert
    uint16_t idle_governor_gain;      // P gain, scaled (x0.0001 -> throttle fraction per RPM error)
    uint8_t  idle_governor_handover;  // throttle handover threshold, % (0-100)
    uint8_t  idle_governor_ceiling;   // max throttle % the governor may output (safety clamp)
} idleGovernorConfig_t;

PG_DECLARE(idleGovernorConfig_t, idleGovernorConfig);

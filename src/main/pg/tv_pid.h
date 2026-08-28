/*
 * This file is part of Rotorflight.
 *
 * Rotorflight is free software. You can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Rotorflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software. If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include "pg/pg.h"
#include "pg/pid.h"

// Config for the independent Thrust Vector PID loop (FEATURE_THRUST_VECTOR).
// A deliberately trimmed-down sibling of pidProfile_t: no pid_mode, master_gain/
// gain_curve, fw_tpa, leveling/trainer/autohover/atthold sub-modes, or cross-axis
// relax -- those are all main-loop flight-mode concerns that don't apply to a raw
// vectoring actuator loop. Single config (no multi-profile switching).
typedef struct {

    pidf_t   pid[PID_ITEM_COUNT];

    uint16_t master_gain[PID_AXIS_COUNT]; // Live per-axis P/I/D/F scale, percent (100 = unscaled) - in-flight tuning aid, doesn't alter the underlying gains

    uint8_t  iterm_decay_time;
    uint8_t  iterm_decay_limit;

    uint8_t  iterm_relax_type;
    uint8_t  iterm_relax_level[PID_AXIS_COUNT];
    uint8_t  iterm_relax_cutoff[PID_AXIS_COUNT];

    uint8_t  error_limit[PID_AXIS_COUNT];

    uint8_t  dterm_cutoff[PID_AXIS_COUNT];
    uint8_t  bterm_cutoff[PID_AXIS_COUNT];
    uint8_t  gyro_cutoff[PID_AXIS_COUNT];

} tvPidProfile_t;

PG_DECLARE(tvPidProfile_t, tvPidProfile);

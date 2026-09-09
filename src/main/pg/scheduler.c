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

#include "platform.h"

#include "pg/pg_ids.h"
#include "pg/scheduler.h"

// v0: last version before osdRelaxDeterminism removal (trailing field, no
// offset shift for rxRelaxDeterminism, but the PG version is still bumped
// per this repo's own established convention -- see mixerConfig_t v1->v2 --
// so pgLoad() resets to defaults on upgrade instead of trusting a
// now-oversized stored record).
PG_REGISTER_WITH_RESET_TEMPLATE(schedulerConfig_t, schedulerConfig, PG_SCHEDULER_CONFIG, 1);

PG_RESET_TEMPLATE(schedulerConfig_t, schedulerConfig,
    .rxRelaxDeterminism = SCHEDULER_RELAX_RX,
);

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

#include "types.h"
#include "platform.h"

#include "common/axis.h"

#include "pg/rates.h"

#include "config/config_reset.h"


PG_REGISTER_ARRAY_WITH_RESET_FN(controlRateConfig_t, CONTROL_RATE_PROFILE_COUNT, controlRateProfiles, PG_CONTROL_RATE_PROFILES, 6);

void pgResetFn_controlRateProfiles(controlRateConfig_t *controlRateConfig)
{
    for (int i = 0; i < CONTROL_RATE_PROFILE_COUNT; i++) {
        RESET_CONFIG(controlRateConfig_t, &controlRateConfig[i],
            .profileName = INIT_ZERO,
            .rcRates[FD_ROLL] = 50,
            .rcRates[FD_PITCH] = 50,
            .rcRates[FD_YAW] = 80,
            .rcExpo[FD_ROLL] = 40,
            .rcExpo[FD_PITCH] = 40,
            .rcExpo[FD_YAW] = 50,
            .sRates[FD_ROLL] = 12,
            .sRates[FD_PITCH] = 12,
            .sRates[FD_YAW] = 12,
            .levelExpo[FD_ROLL] = 0,
            .levelExpo[FD_PITCH] = 0,
            .response_time[FD_ROLL] = 0,
            .response_time[FD_PITCH] = 0,
            .response_time[FD_YAW] = 0,
            .accel_limit[FD_ROLL] = 0,
            .accel_limit[FD_PITCH] = 0,
            .accel_limit[FD_YAW] = 0,
            .cyclic_ring = 0,
            .cyclic_polar = 0,
            .setpoint_boost_gain[FD_ROLL] = 0,
            .setpoint_boost_gain[FD_PITCH] = 0,
            .setpoint_boost_gain[FD_YAW] = 0,
            .setpoint_boost_cutoff[FD_ROLL] = 15,
            .setpoint_boost_cutoff[FD_PITCH] = 15,
            .setpoint_boost_cutoff[FD_YAW] = 90,
            .yaw_dynamic_ceiling_gain = 0,
            .yaw_dynamic_deadband_gain = 10,
            .yaw_dynamic_deadband_cutoff = 50,
            .yaw_dynamic_deadband_filter = 60,
        );
    }
}

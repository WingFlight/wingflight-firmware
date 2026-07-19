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

#include "pg/pg_ids.h"
#include "platform.h"

#include "pg/fc_link.h"

#ifdef USE_FC_LINK

PG_REGISTER_WITH_RESET_FN(fcLinkConfig_t, fcLinkConfig,
                          PG_DRIVER_FC_LINK_CONFIG, 0);

void pgResetFn_fcLinkConfig(fcLinkConfig_t *config)
{
    // Fast enough that a relaying SLAVE's output doesn't visibly step
    // between updates -- comparable to the SBUS_OUT/FBUS_MASTER cadence
    // it's standing in for.
    config->rateHz = 50;
    config->peerTimeoutMs = 500;
    config->inverted = 0;
    config->pinSwap = 0;

    // Default to today's behavior: sync everything.
    config->syncMixerServos = 1;
    config->syncPidRates = 1;
    config->syncRx = 1;
    config->syncMotor = 1;
    config->syncTelemetry = 1;
    config->syncModesAdjustments = 1;
    config->syncGps = 1;
    config->syncOsd = 1;
    config->syncVtx = 1;
    config->syncOther = 1;
}

#endif

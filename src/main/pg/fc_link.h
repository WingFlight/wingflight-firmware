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

#include "common/utils.h"
#include "pg/pg.h"

// Role is NOT stored here. It comes from which serial port function was
// assigned (FUNCTION_FC_LINK_MASTER / _SLAVE in io/serial.h), so the role
// can never drift out of sync with the port assignment.

#define FC_LINK_RATE_MIN_HZ 5
#define FC_LINK_RATE_MAX_HZ 200

#define FC_LINK_PEER_TIMEOUT_MIN_MS 100
#define FC_LINK_PEER_TIMEOUT_MAX_MS 5000

typedef struct fcLinkConfig_s {
    uint16_t rateHz;                  // heartbeat + channel relay send rate
    uint16_t peerTimeoutMs;           // time without a valid heartbeat before the peer is considered lost
    uint8_t inverted;
    uint8_t pinSwap;

    // SLAVE-side only: which base-config-sync categories this board is
    // willing to accept from a MASTER. MASTER always streams every syncable
    // PG regardless of its own values here; the SLAVE decides what to apply.
    uint8_t syncMixerServos;
    uint8_t syncPidRates;
    uint8_t syncRx;
    uint8_t syncMotor;
    uint8_t syncTelemetry;
    uint8_t syncModesAdjustments;
    uint8_t syncGps;
    uint8_t syncOsd;
    uint8_t syncVtx;
    uint8_t syncOther;
} fcLinkConfig_t;

PG_DECLARE(fcLinkConfig_t, fcLinkConfig);

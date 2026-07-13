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

#include <stdbool.h>
#include <stdint.h>

#include "common/time.h"

// Role comes solely from which port function was assigned
// (FUNCTION_FC_LINK_MASTER / FUNCTION_FC_LINK_SLAVE); it is fixed the moment
// the port opens and never renegotiated at runtime.
typedef enum {
    FC_LINK_ROLE_MASTER = 0,
    FC_LINK_ROLE_SLAVE,
} fcLinkRole_e;

typedef struct fcLinkPeerState_s {
    bool armed;
    bool failsafeActive;
    bool rxReceivingSignal;
    uint16_t seq;
} fcLinkPeerState_t;

// Covers the whole servo index space: PWM servos (0..MAX_SUPPORTED_PWM_SERVOS-1)
// plus bus servos (BUS_SERVO_OFFSET..+BUS_SERVO_CHANNELS-1), i.e. MAX_SUPPORTED_SERVOS.
// Kept as a plain constant (rather than including target headers here) and
// checked against MAX_SUPPORTED_SERVOS via _Static_assert in servos.c.
#define FC_LINK_MAX_CHANNELS 26

void fcLinkInit(void);
void fcLinkUpdate(timeUs_t currentTimeUs);

bool fcLinkIsEnabled(void);
fcLinkRole_e fcLinkGetRole(void);

// Convenience: true when disabled (standalone) or assigned MASTER.
bool fcLinkIsMaster(void);

// True if we've never heard the peer, or its last heartbeat is older than peerTimeoutMs.
bool fcLinkPeerLost(void);

const fcLinkPeerState_t *fcLinkGetPeerState(void);

// Called by whichever output stage (PWM servoUpdate(), SBUS_OUT, FBUS_MASTER)
// actually computed its own channel values this cycle, so they can be
// forwarded to the peer. startIndex is the absolute servo index of channels[0].
void fcLinkPublishChannels(uint8_t startIndex, const float *channels, uint8_t count);

// True when this board should output the peer's relayed channels instead of
// its own locally-computed ones (SLAVE, link enabled, peer heartbeat alive).
bool fcLinkShouldRelay(void);

// Fills out[0..count) with the most recently received peer channel values,
// starting at absolute servo index startIndex. Only meaningful when
// fcLinkShouldRelay() is true.
void fcLinkGetRelayChannels(uint8_t startIndex, float *out, uint8_t count);

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

// What the peer reported in its last heartbeat -- compare against this
// board's own EEPROM_CONF_VERSION/FC_VERSION_MAJOR/FC_VERSION_MINOR (see
// build/version.h, config/config_eeprom.h) to see why fcLinkTriggerConfigSync()
// might be refusing: a config sync is only offered between byte-identical
// firmware builds.
typedef struct fcLinkPeerVersionInfo_s {
    uint8_t eepromConfVersion;
    uint8_t fcVersionMajor;
    uint8_t fcVersionMinor;
} fcLinkPeerVersionInfo_t;

// Bench-debug counters for `fc_link debug` -- lets a dead/flaky link be
// diagnosed from the CLI (byte activity, framing, checksum pass/fail per
// frame type, and whether TX is even being attempted) without needing a
// logic analyzer.
typedef struct fcLinkDebugStats_s {
    uint32_t rxByteTotal;        // every byte handed to the receiver, synced or not
    uint32_t rxUnsyncedByte;     // byte seen while idle that matched no known sync value
    uint32_t rxFrameAbandoned;   // partial frame dropped by the inter-byte gap timeout
    uint32_t heartbeatOk;
    uint32_t heartbeatChecksumFail;
    uint32_t tuningOk;
    uint32_t tuningChecksumFail;
    uint32_t configOk;           // any valid CONFIG_SYNC_BYTE frame (request/start/pg/end)
    uint32_t configChecksumFail;
    uint32_t configRequestSeen;  // subset of configOk that was specifically a sync REQUEST
    uint32_t txHeartbeatSent;    // heartbeat actually handed to serialWriteBuf()
    uint32_t txHeartbeatSkipped; // heartbeat due, but serialTxBytesFree() had no room
} fcLinkDebugStats_t;

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
const fcLinkDebugStats_t *fcLinkGetDebugStats(void);
fcLinkPeerVersionInfo_t fcLinkGetPeerVersionInfo(void);

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

// Base/setup config sync (mixer, servos, ports, etc. -- everything except
// per-board identity settings like serial port function assignments). Each
// transfer is one-shot and always ends in a SLAVE EEPROM write + reboot --
// never a continuous streaming process -- but a SLAVE periodically re-checks
// whether the peer's *saved* config has since drifted (see
// fcLinkNotifyConfigSaved()) and re-triggers a fresh transfer automatically
// when it has, not just once at boot. SLAVE-only; a request is ignored by a
// MASTER-role board. Returns false if refused outright (not a SLAVE, link
// down, or peer firmware/EEPROM version doesn't match) rather than actually
// attempting the transfer.
bool fcLinkTriggerConfigSync(void);

// Called once after any real EEPROM write (config.c's writeEEPROM(), from
// whichever source -- CLI `save`, MSP_EEPROM_WRITE from the configurator or
// a Lua script, CMS menus, RX bind flows, fc_link's own SLAVE-side commit).
// Snapshots the fingerprint that gets broadcast/compared for auto-sync
// purposes, so drift detection reacts only to config that's actually been
// committed -- never to a value mid-edit in some tool that hasn't saved yet.
// A no-op if fc_link isn't enabled on this board.
void fcLinkNotifyConfigSaved(void);

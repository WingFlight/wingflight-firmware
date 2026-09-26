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

#include "platform.h"
#include "rx/rx.h"
#include "rx/sbus_channels.h"

#include "common/time.h"
#include "common/utils.h"

// Control frame: length, type, channels, rssi, crc. The length byte counts
// the channels and rssi; the crc covers type, channels and rssi.
#define FBUS_CONTROL8_LENGTH 0x0D
#define FBUS_CONTROL16_LENGTH 0x18
#define FBUS_CONTROL24_LENGTH 0x23
#define FBUS_CONTROL_TYPE_RC 0xFF

// 8-channel frame: CH1-8 analog; its flags byte is left clear.
// 16-channel frame: CH1-16 analog + CH17-18 digital (flags byte).
// 24-channel frame: CH1-24 analog; its flags byte is left clear.
#define FBUS_MASTER_CHANNELS_16_COUNT 18
#define FBUS_MASTER_CHANNELS_24_COUNT 24

#define FBUS_DOWNLINK_PAYLOAD_SIZE 0x08
#define FBUS_DOWNLINK_LENGTH 0x0A


typedef struct {
    uint8_t length;
    uint8_t type;
    sbusChannels_t channels;
    uint8_t rssi;
    uint8_t crc;
} __attribute__((__packed__)) fbusMasterControl16_t;

typedef struct {
    uint8_t length;
    uint8_t type;
    sbusChannels8ch_t channels;
    uint8_t rssi;
    uint8_t crc;
} __attribute__((__packed__)) fbusMasterControl8_t;

typedef struct {
    uint8_t length;
    uint8_t type;
    sbusChannels24ch_t channels;
    uint8_t rssi;
    uint8_t crc;
} __attribute__((__packed__)) fbusMasterControl24_t;

typedef struct {
    uint8_t length;
    uint8_t phyID;
    uint8_t prim;
    uint16_t appId ;
    uint8_t data[4];
    uint8_t crc;
} __attribute__((__packed__)) fbusMasterDownlink_t;

typedef struct {
    uint8_t length;
    uint8_t type;
    uint8_t data[24];
    uint8_t crc;
} __attribute__((__packed__)) fbusMasterOtaStart_t;

typedef struct {
    uint8_t length;
    uint8_t type;
    uint8_t data[32];
    uint8_t crc;
} __attribute__((__packed__)) fbusMasterOtaData_t;

typedef struct {
    uint8_t length;
    uint8_t type;
    uint8_t data[24];
    uint8_t crc;
} __attribute__((__packed__)) fbusMasterOtaEnd_t;

STATIC_ASSERT(sizeof(fbusMasterControl8_t) == FBUS_CONTROL8_LENGTH + 3,
              fbus_master_control8_size_mismatch);
STATIC_ASSERT(sizeof(fbusMasterControl16_t) == FBUS_CONTROL16_LENGTH + 3,
              fbus_master_control16_size_mismatch);
STATIC_ASSERT(sizeof(fbusMasterControl24_t) == FBUS_CONTROL24_LENGTH + 3,
              fbus_master_control24_size_mismatch);
STATIC_ASSERT(BUS_SERVO_CHANNELS >= FBUS_MASTER_CHANNELS_24_COUNT,
              fbus_master_bus_servo_channels_too_few);

// Routine function called by the scheduler or timer
void fbusMasterUpdate(timeUs_t currentTimeUs);

bool fbusMasterIsEnabled(void);

// Init function
void fbusMasterInit(void);

// Restarts sensor discovery (used by XACT servo programming before a scan)
void fbusMasterStartDiscovery(void);

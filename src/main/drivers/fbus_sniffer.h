/*
 * This file is part of Wingflight.
 *
 * Wingflight is free software. You can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Wingflight is distributed in the hope that it will be useful,
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

/*
  FBUS sniffer: a capture of the frames passing through the flight controller between
  the radio (the receiver's FBUS line, rx/fbus.c) and the devices on the FBUS master line
  (drivers/fbus_master.c), for debugging over MSP2_WING_FBUS_SNIFFER. Off until a client
  asks for it, and off again when the client stops asking.

  A developer option: only in builds made with OPTIONS="USE_FBUS_SNIFFER". Without it
  the capture, its buffer and the MSP command are left out, and fbusSnifferRecord() is
  an empty inline function.
 */

// Frames kept until the client reads them. Further ones are counted as dropped.
#define FBUS_SNIFFER_BUFFER_FRAMES  64

// The capture stops when no request has kept it on for this long
#define FBUS_SNIFFER_TIMEOUT_MS     5000

// Request and reply flags
#define FBUS_SNIFFER_FLAG_ON        0x01    // capture
#define FBUS_SNIFFER_FLAG_ALL       0x02    // also telemetry: data frames and their polls

// Where a frame was seen
typedef enum {
    FBUS_SNIFFER_FROM_RADIO = 0,    // a request from the radio for a device on the master line
    FBUS_SNIFFER_TO_DEVICE,         // sent on the master line
    FBUS_SNIFFER_FROM_DEVICE,       // received on the master line
    FBUS_SNIFFER_TO_RADIO,          // sent back to the radio
} fbusSnifferDirection_e;

// Or'ed into the direction of a FBUS_SNIFFER_FROM_RADIO frame that was not passed on to a
// device (none matches, or a write that two devices would act on)
#define FBUS_SNIFFER_NOT_RELAYED    0x80

typedef struct {
    uint32_t timeMs;
    uint8_t direction;      // fbusSnifferDirection_e, with FBUS_SNIFFER_NOT_RELAYED
    uint8_t phyId;          // bare physical ID, without check bits
    uint8_t prim;           // frame ID: 0x10 data, 0x30 read, 0x31 write, 0x32 response...
    uint16_t appId;
    uint32_t data;
} fbusSnifferFrame_t;

#if defined(USE_FBUS_MASTER) && defined(USE_FBUS_SNIFFER)

// A request from the client. With FBUS_SNIFFER_FLAG_ON the capture starts, or stays on for
// another FBUS_SNIFFER_TIMEOUT_MS; without it the capture stops and the buffer is emptied.
void fbusSnifferRequest(uint8_t flags, timeMs_t nowMs);

// The flags of the running capture, 0 when it is off
uint8_t fbusSnifferGetFlags(timeMs_t nowMs);

// Adds a frame if the capture is on and its filter lets it through. Null frames (frame ID
// 0x00) never are. Without FBUS_SNIFFER_FLAG_ALL, data frames (0x10) are left out, except
// the polls that collect a device's answer to a read or write (to a device, with an App ID).
// Safe from an interrupt.
void fbusSnifferRecord(timeMs_t nowMs, uint8_t direction, uint8_t phyId, uint8_t prim, uint16_t appId, uint32_t data);

// Takes up to maxFrames of the oldest frames out of the buffer. Returns how many.
uint8_t fbusSnifferRead(fbusSnifferFrame_t *frames, uint8_t maxFrames);

// Frames left in the buffer
uint8_t fbusSnifferGetCount(void);

// Frames lost to a full buffer since the last call
uint16_t fbusSnifferTakeDropped(void);

#else

static inline void fbusSnifferRecord(timeMs_t nowMs, uint8_t direction, uint8_t phyId, uint8_t prim, uint16_t appId,
                                     uint32_t data)
{
    (void)nowMs; (void)direction; (void)phyId; (void)prim; (void)appId; (void)data;
}

#endif

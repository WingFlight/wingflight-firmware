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
  CRSF sniffer: a capture of the frames the CRSF sensors port receives
  (drivers/crsf_sensors.c), whole and as received, for debugging sensors over
  MSP2_WING_CRSF_SNIFFER. Off until a client asks for it, and off again when the
  client stops asking. The same shape as the FBUS sniffer (drivers/fbus_sniffer.h).

  A developer option: only in builds made with OPTIONS="USE_CRSF_SNIFFER". Without it
  the capture, its buffer and the MSP command are left out, and crsfSnifferRecord() is
  an empty macro.
 */

// Bytes kept for frames until the client reads them: each frame takes its
// length plus CRSF_SNIFFER_FRAME_HEADER. Frames that do not fit are counted as dropped.
#define CRSF_SNIFFER_BUFFER_BYTES   768
#define CRSF_SNIFFER_FRAME_HEADER   6       // time (4), flags (1), length (1)

// The capture stops when no request has kept it on for this long
#define CRSF_SNIFFER_TIMEOUT_MS     5000

// Request and reply flags
#define CRSF_SNIFFER_FLAG_ON        0x01    // capture
#define CRSF_SNIFFER_FLAG_BAD_CRC   0x02    // also frames with a bad CRC

// Frame flags
#define CRSF_SNIFFER_CRC_OK         0x01    // the CRC matched
#define CRSF_SNIFFER_DECODED        0x02    // handed to the sensor decoder
// CRC fine, but the decoder had not taken the previous frame yet: this one was
// lost (the driver holds one frame between the receive interrupt and its task)
#define CRSF_SNIFFER_SKIPPED        0x04

#if defined(USE_CRSF_SENSORS) && defined(USE_CRSF_SNIFFER)

// A request from the client, with an optional frame type filter (0: every type).
// With CRSF_SNIFFER_FLAG_ON the capture starts, or stays on for another
// CRSF_SNIFFER_TIMEOUT_MS; without it the capture stops and the buffer is emptied.
void crsfSnifferRequest(uint8_t flags, uint8_t frameType, timeMs_t nowMs);

// The flags of the running capture, 0 when it is off
uint8_t crsfSnifferGetFlags(timeMs_t nowMs);

// Adds a received frame (address byte to CRC, length bytes) if the capture is
// on and its filters let it through. Safe from an interrupt.
void crsfSnifferRecord(timeMs_t nowMs, uint8_t frameFlags, const volatile uint8_t *frame, uint8_t length);

// Takes the oldest frames out of the buffer as long as they fit in maxBytes,
// written to dst as header + frame each. Returns the bytes written; *frames
// gets how many frames.
uint16_t crsfSnifferRead(uint8_t *dst, uint16_t maxBytes, uint8_t *frames);

// Frames left in the buffer
uint8_t crsfSnifferGetCount(void);

// Frames lost to a full buffer since the last call
uint16_t crsfSnifferTakeDropped(void);

#else

// a macro, so its arguments (millis()...) are not evaluated either: no code at all
#define crsfSnifferRecord(nowMs, frameFlags, frame, length) do { } while (0)

#endif

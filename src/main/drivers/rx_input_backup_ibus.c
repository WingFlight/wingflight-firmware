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
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_RX_INPUT_BACKUP_IBUS

#include "drivers/rx_input_backup_ibus.h"

#include "build/atomic.h"
#include "build/build_config.h"

#include "common/utils.h"

#include "drivers/nvic.h"
#include "drivers/serial.h"
#include "drivers/time.h"

#include "pg/rx_input_backup.h"

// This intentionally does not reuse rx/ibus.c's ibusInit()/ibusDataReceive() -
// that module keeps its frame-assembly state in function-local statics tied to
// the single main RX instance, plus telemetry request/response handling this
// backup link has no use for (receive-only, no telemetry, ever). This is a
// from-scratch minimal reimplementation of just the RC-frame framing, verified
// line-by-line against rx/ibus.c.
//
// Also doesn't reuse telemetry/ibus_shared.c's isChecksumOkIa6b() - pulling
// that in would drag along the whole telemetry module (and it may not even be
// compiled if USE_TELEMETRY_IBUS isn't set). The checksum itself is a few
// lines; reimplemented locally instead.
//
// Unlike the real driver, there's no rxBytesToIgnore self-echo skip logic
// here: that exists only because the main RX may just have written an IBUS
// telemetry response and needs to ignore the resulting echo on the shared
// half-duplex wire. This backup link never transmits, so there's nothing to
// echo and nothing to skip.

#define IBUS_INPUT_PORT_OPTIONS (SERIAL_STOPBITS_1 | SERIAL_PARITY_NO)
#define IBUS_INPUT_BAUDRATE 115200

// Matches rx/ibus.c's own constants exactly.
#define IBUS_INPUT_MAX_CHANNEL 18
#define IBUS_INPUT_MAX_SLOTS 14
#define IBUS_INPUT_BUFFSIZE 32
#define IBUS_INPUT_IA6B_FRAME_SIZE 32
#define IBUS_INPUT_IA6_FRAME_SIZE 31
#define IBUS_INPUT_IA6_SYNC_BYTE 0x55
#define IBUS_INPUT_FRAME_GAP_US 500

typedef enum {
    IBUS_INPUT_MODEL_NONE = 0,
    IBUS_INPUT_MODEL_IA6B,
    IBUS_INPUT_MODEL_IA6,
} ibusInputModel_e;

typedef struct ibusInputFrameData_s {
    uint8_t bytes[IBUS_INPUT_BUFFSIZE];
    volatile timeUs_t lastByteAtUs;
    volatile uint8_t position;
    volatile uint8_t syncByte;
    volatile uint8_t frameSize;
    volatile uint8_t channelOffset;
    volatile ibusInputModel_e model;
} ibusInputFrameData_t;

static ibusInputFrameData_t ibusInputFrameData;
static uint8_t ibusInputPendingFrame[IBUS_INPUT_BUFFSIZE];
static volatile uint8_t ibusInputPendingFrameSize = 0;
static volatile ibusInputModel_e ibusInputPendingModel = IBUS_INPUT_MODEL_NONE;
static volatile bool ibusInputPendingFrameReady = false;

static uint32_t ibusInputChannelData[IBUS_INPUT_MAX_CHANNEL];

static void ibusInputResetParser(void)
{
    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
        ibusInputFrameData.lastByteAtUs = 0;
        ibusInputFrameData.position = 0;
        ibusInputPendingFrameReady = false;
    }
    // Deliberately not resetting syncByte/frameSize/channelOffset/model here -
    // matches rx/ibus.c's own behaviour: once a model is detected it stays
    // "sticky" for the rest of the session (a gap-timeout mid-frame is just a
    // dropped frame, not evidence the receiver model changed).
}

static FAST_CODE void ibusInputDataReceive(uint16_t c, void *data)
{
    UNUSED(data);

    const timeUs_t nowUs = microsISR();
    const timeDelta_t byteGap = ibusInputFrameData.lastByteAtUs == 0
        ? (timeDelta_t)IBUS_INPUT_FRAME_GAP_US + 1
        : cmpTimeUs(nowUs, ibusInputFrameData.lastByteAtUs);
    ibusInputFrameData.lastByteAtUs = nowUs;

    if (byteGap > IBUS_INPUT_FRAME_GAP_US) {
        ibusInputFrameData.position = 0;
    }

    if (ibusInputFrameData.position == 0) {
        if (c == IBUS_INPUT_IA6B_FRAME_SIZE) {
            ibusInputFrameData.model = IBUS_INPUT_MODEL_IA6B;
            ibusInputFrameData.syncByte = (uint8_t)c;
            ibusInputFrameData.frameSize = IBUS_INPUT_IA6B_FRAME_SIZE;
            ibusInputFrameData.channelOffset = 2;
        } else if (ibusInputFrameData.syncByte == 0 && c == IBUS_INPUT_IA6_SYNC_BYTE) {
            ibusInputFrameData.model = IBUS_INPUT_MODEL_IA6;
            ibusInputFrameData.syncByte = IBUS_INPUT_IA6_SYNC_BYTE;
            ibusInputFrameData.frameSize = IBUS_INPUT_IA6_FRAME_SIZE;
            ibusInputFrameData.channelOffset = 1;
        } else if (ibusInputFrameData.syncByte != c) {
            return;
        }
    }

    if (ibusInputFrameData.frameSize == 0
        || ibusInputFrameData.frameSize > IBUS_INPUT_BUFFSIZE
        || ibusInputFrameData.position >= ibusInputFrameData.frameSize) {
        ibusInputFrameData.position = 0;
        return;
    }

    ibusInputFrameData.bytes[ibusInputFrameData.position++] = (uint8_t)c;

    if (ibusInputFrameData.position == ibusInputFrameData.frameSize) {
        // Snapshot into a separate holding buffer right here, rather than
        // leaving the completed frame sitting in ibusInputFrameData for the
        // consumer to read later - see the SBUS/FBUS/FPort providers' own
        // identical comment for why (torn-frame race with the next frame's
        // first byte landing back at position 0).
        memcpy(ibusInputPendingFrame, ibusInputFrameData.bytes, ibusInputFrameData.frameSize);
        ibusInputPendingFrameSize = ibusInputFrameData.frameSize;
        ibusInputPendingModel = ibusInputFrameData.model;
        ibusInputPendingFrameReady = true;
        ibusInputFrameData.position = 0;
    }
}

// Reimplementation of telemetry/ibus_shared.c's calculateChecksum()/
// isChecksumOkIa6b() (see this file's top-of-file comment for why it isn't
// reused directly). IA6B's checksum: 0xFFFF minus the sum of every byte
// except the trailing 2-byte checksum itself, stored little-endian.
static bool ibusInputChecksumOkIa6b(const uint8_t *frame, uint8_t frameSize)
{
    uint16_t checksum = 0xFFFF;
    for (uint8_t i = 0; i < frameSize - 2; i++) {
        checksum -= frame[i];
    }
    return (checksum & 0xFF) == frame[frameSize - 2] && (checksum >> 8) == frame[frameSize - 1];
}

// IA6 legacy checksum: a plain 16-bit sum (no subtraction, no sync/length
// bytes included) of the channel slots only, matching rx/ibus.c's own
// isChecksumOkIa6().
static bool ibusInputChecksumOkIa6(const uint8_t *frame, uint8_t frameSize, uint8_t channelOffset)
{
    uint16_t checksum = 0;
    uint8_t offset = channelOffset;
    for (uint8_t i = 0; i < IBUS_INPUT_MAX_SLOTS; i++, offset += 2) {
        checksum += frame[offset] + ((uint16_t)frame[offset + 1] << 8);
    }
    const uint16_t rxChecksum = frame[frameSize - 2] + ((uint16_t)frame[frameSize - 1] << 8);
    return checksum == rxChecksum;
}

// Called from the RX task (rx/rx.c, via rx_input_backup.c's poll loop), not an
// ISR - safe to do the heavier decode/convert work here.
static bool ibusInputUpdate(float *channels, uint8_t channelCount)
{
    uint8_t frame[IBUS_INPUT_BUFFSIZE];
    uint8_t frameSize = 0;
    ibusInputModel_e model = IBUS_INPUT_MODEL_NONE;
    bool haveFrame = false;

    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
        if (ibusInputPendingFrameReady) {
            frameSize = ibusInputPendingFrameSize;
            model = ibusInputPendingModel;
            memcpy(frame, ibusInputPendingFrame, frameSize);
            ibusInputPendingFrameReady = false;
            haveFrame = true;
        }
    }

    if (!haveFrame) {
        return false;
    }

    const uint8_t channelOffset = model == IBUS_INPUT_MODEL_IA6 ? 1 : 2;
    const bool checksumOk = model == IBUS_INPUT_MODEL_IA6
        ? ibusInputChecksumOkIa6(frame, frameSize, channelOffset)
        : ibusInputChecksumOkIa6b(frame, frameSize);

    if (!checksumOk) {
        return false;
    }

    // 14 channels packed as 2 bytes each (12 data bits, top nibble unused on
    // IA6/older IA6B firmware); matches rx/ibus.c's own updateChannelData().
    uint8_t offset = channelOffset;
    for (uint8_t i = 0; i < IBUS_INPUT_MAX_SLOTS; i++, offset += 2) {
        ibusInputChannelData[i] = frame[offset] + (((uint16_t)frame[offset + 1] & 0x0F) << 8);
    }

    // Newer IA6B firmware reuses the previously-unused top nibble of each
    // slot to carry 4 more channels (15-18) - harmless to run unconditionally
    // for the legacy IA6 frame too (reads stale bytes rather than overflowing
    // the fixed 32-byte buffer), same as rx/ibus.c's own driver does; not
    // fixing that inherited quirk here, just faithfully porting it.
    offset = channelOffset + 1;
    for (uint8_t i = IBUS_INPUT_MAX_SLOTS; i < IBUS_INPUT_MAX_CHANNEL; i++, offset += 6) {
        ibusInputChannelData[i] = ((frame[offset] & 0xF0) >> 4)
            | (frame[offset + 2] & 0xF0)
            | (((uint16_t)frame[offset + 4] & 0xF0) << 4);
    }

    // No scaling formula needed - unlike SBUS/FBUS/FPort's 11-bit range,
    // rx/ibus.c's own ibusReadRawRC() returns these values as-is; they're
    // already in normal ~1000-2000us units on the wire.
    for (uint8_t i = 0; i < channelCount; i++) {
        channels[i] = (float)ibusInputChannelData[i];
    }

    return true;
}

bool rxInputBackupIbusInit(rxInputBackupOps_t *ops)
{
    memset(&ibusInputFrameData, 0, sizeof(ibusInputFrameData));
    ibusInputResetParser();

    // Matches rx/ibus.c's own direction/variant exactly: IBUS's signal is
    // natively non-inverted (like FBUS/FPort/FPort2), but half-duplex uses
    // plain SERIAL_BIDIR (like SBUS), not the push-pull variant.
    ops->baudRate = IBUS_INPUT_BAUDRATE;
    ops->portOptions = IBUS_INPUT_PORT_OPTIONS
        | (rxInputBackupConfig()->inverted ? SERIAL_INVERTED : SERIAL_NOT_INVERTED)
        | (rxInputBackupConfig()->halfDuplex ? SERIAL_BIDIR : SERIAL_UNIDIR);
    ops->isrFn = ibusInputDataReceive;
    ops->channelCount = IBUS_INPUT_MAX_CHANNEL;
    ops->update = ibusInputUpdate;

    return true;
}

#endif // USE_RX_INPUT_BACKUP_IBUS

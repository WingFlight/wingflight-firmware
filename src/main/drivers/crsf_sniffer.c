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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#if defined(USE_CRSF_SENSORS) && defined(USE_CRSF_SNIFFER)

#include "build/atomic.h"

#include "common/time.h"

#include "drivers/nvic.h"
#include "drivers/crsf_sniffer.h"

// Frames are recorded from the CRSF sensors port's receive interrupt and read
// from the MSP task: the ring is only touched inside ATOMIC_BLOCK. A frame is
// stored as time (4, little-endian), flags, length, then its bytes.
static uint8_t snifferBytes[CRSF_SNIFFER_BUFFER_BYTES];
static uint16_t snifferHead = 0;    // the oldest byte
static uint16_t snifferUsed = 0;
static uint8_t snifferCount = 0;
static uint16_t snifferDropped = 0;

// 0 when off: the only thing crsfSnifferRecord() reads then
static volatile uint8_t snifferFlags = 0;
static volatile uint8_t snifferFrameType = 0;
static volatile timeMs_t snifferLastRequestMs = 0;

static void snifferClear(void)
{
    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
        snifferHead = 0;
        snifferUsed = 0;
        snifferCount = 0;
        snifferDropped = 0;
    }
}

static bool snifferTimedOut(timeMs_t nowMs)
{
    return (int32_t)(nowMs - snifferLastRequestMs) > CRSF_SNIFFER_TIMEOUT_MS;
}

void crsfSnifferRequest(uint8_t flags, uint8_t frameType, timeMs_t nowMs)
{
    if (!(flags & CRSF_SNIFFER_FLAG_ON)) {
        snifferFlags = 0;
        snifferClear();
        return;
    }

    // a capture that timed out starts again empty
    if (crsfSnifferGetFlags(nowMs) == 0) {
        snifferClear();
    }

    snifferLastRequestMs = nowMs;
    snifferFrameType = frameType;
    snifferFlags = flags & (CRSF_SNIFFER_FLAG_ON | CRSF_SNIFFER_FLAG_BAD_CRC);
}

uint8_t crsfSnifferGetFlags(timeMs_t nowMs)
{
    if (snifferFlags && snifferTimedOut(nowMs)) {
        snifferFlags = 0;
    }
    return snifferFlags;
}

static void ringPut(uint16_t offset, uint8_t byte)
{
    snifferBytes[(snifferHead + offset) % CRSF_SNIFFER_BUFFER_BYTES] = byte;
}

static uint8_t ringGet(uint16_t offset)
{
    return snifferBytes[(snifferHead + offset) % CRSF_SNIFFER_BUFFER_BYTES];
}

void crsfSnifferRecord(timeMs_t nowMs, uint8_t frameFlags, const volatile uint8_t *frame, uint8_t length)
{
    const uint8_t flags = snifferFlags;
    if (!flags || frame == NULL || length < 3) {
        return;
    }

    if (snifferTimedOut(nowMs)) {
        snifferFlags = 0;
        return;
    }

    if (!(frameFlags & CRSF_SNIFFER_CRC_OK) && !(flags & CRSF_SNIFFER_FLAG_BAD_CRC)) {
        return;
    }

    // the frame type is the third byte: address, length, type
    const uint8_t type = snifferFrameType;
    if (type != 0 && frame[2] != type) {
        return;
    }

    const uint16_t size = CRSF_SNIFFER_FRAME_HEADER + length;

    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
        if (snifferUsed + size > CRSF_SNIFFER_BUFFER_BYTES || snifferCount == UINT8_MAX) {
            if (snifferDropped < UINT16_MAX) {
                snifferDropped++;
            }
        } else {
            const uint16_t at = snifferUsed;
            ringPut(at, nowMs & 0xFF);
            ringPut(at + 1, (nowMs >> 8) & 0xFF);
            ringPut(at + 2, (nowMs >> 16) & 0xFF);
            ringPut(at + 3, (nowMs >> 24) & 0xFF);
            ringPut(at + 4, frameFlags);
            ringPut(at + 5, length);
            for (uint8_t i = 0; i < length; i++) {
                ringPut(at + CRSF_SNIFFER_FRAME_HEADER + i, frame[i]);
            }
            snifferUsed += size;
            snifferCount++;
        }
    }
}

uint16_t crsfSnifferRead(uint8_t *dst, uint16_t maxBytes, uint8_t *frames)
{
    uint16_t written = 0;
    uint8_t count = 0;

    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
        while (snifferCount > 0) {
            const uint16_t size = CRSF_SNIFFER_FRAME_HEADER + ringGet(5);
            if (written + size > maxBytes) {
                break;
            }
            for (uint16_t i = 0; i < size; i++) {
                dst[written + i] = ringGet(i);
            }
            written += size;
            count++;
            snifferHead = (snifferHead + size) % CRSF_SNIFFER_BUFFER_BYTES;
            snifferUsed -= size;
            snifferCount--;
        }
    }

    if (frames != NULL) {
        *frames = count;
    }
    return written;
}

uint8_t crsfSnifferGetCount(void)
{
    return snifferCount;
}

uint16_t crsfSnifferTakeDropped(void)
{
    uint16_t dropped = 0;

    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
        dropped = snifferDropped;
        snifferDropped = 0;
    }

    return dropped;
}

#endif // USE_CRSF_SENSORS && USE_CRSF_SNIFFER

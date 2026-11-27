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

#if defined(USE_FBUS_MASTER)

#include "build/atomic.h"

#include "common/time.h"

#include "drivers/nvic.h"
#include "drivers/fbus_sniffer.h"

#define FBUS_SNIFFER_PRIM_NULL  0x00
#define FBUS_SNIFFER_PRIM_DATA  0x10

// Frames are recorded from the master line's UART interrupt, the receiver's frame
// handling and the main loop, and read from the MSP task: the buffer is only touched
// inside ATOMIC_BLOCK.
static fbusSnifferFrame_t snifferFrames[FBUS_SNIFFER_BUFFER_FRAMES];
static uint8_t snifferHead = 0;     // the oldest frame
static uint8_t snifferCount = 0;
static uint16_t snifferDropped = 0;

// 0 when off: the only thing fbusSnifferRecord() reads then
static volatile uint8_t snifferFlags = 0;
static volatile timeMs_t snifferLastRequestMs = 0;

static void snifferClear(void)
{
    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
        snifferHead = 0;
        snifferCount = 0;
        snifferDropped = 0;
    }
}

void fbusSnifferRequest(uint8_t flags, timeMs_t nowMs)
{
    if (!(flags & FBUS_SNIFFER_FLAG_ON)) {
        snifferFlags = 0;
        snifferClear();
        return;
    }

    // a capture that timed out starts again empty
    if (fbusSnifferGetFlags(nowMs) == 0) {
        snifferClear();
    }

    snifferLastRequestMs = nowMs;
    snifferFlags = flags & (FBUS_SNIFFER_FLAG_ON | FBUS_SNIFFER_FLAG_ALL);
}

uint8_t fbusSnifferGetFlags(timeMs_t nowMs)
{
    if (snifferFlags && (int32_t)(nowMs - snifferLastRequestMs) > FBUS_SNIFFER_TIMEOUT_MS) {
        snifferFlags = 0;
    }
    return snifferFlags;
}

void fbusSnifferRecord(timeMs_t nowMs, uint8_t direction, uint8_t phyId, uint8_t prim, uint16_t appId, uint32_t data)
{
    const uint8_t flags = snifferFlags;
    if (!flags || prim == FBUS_SNIFFER_PRIM_NULL) {
        return;
    }

    if ((int32_t)(nowMs - snifferLastRequestMs) > FBUS_SNIFFER_TIMEOUT_MS) {
        snifferFlags = 0;
        return;
    }

    if (prim == FBUS_SNIFFER_PRIM_DATA && !(flags & FBUS_SNIFFER_FLAG_ALL)) {
        // telemetry, unless it is the poll for the answer to a read or write: that one
        // carries the App ID of the request, a telemetry poll carries none
        if (direction != FBUS_SNIFFER_TO_DEVICE || appId == 0) {
            return;
        }
    }

    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
        if (snifferCount >= FBUS_SNIFFER_BUFFER_FRAMES) {
            if (snifferDropped < UINT16_MAX) {
                snifferDropped++;
            }
        } else {
            fbusSnifferFrame_t *frame = &snifferFrames[(snifferHead + snifferCount) % FBUS_SNIFFER_BUFFER_FRAMES];
            frame->timeMs = nowMs;
            frame->direction = direction;
            frame->phyId = phyId;
            frame->prim = prim;
            frame->appId = appId;
            frame->data = data;
            snifferCount++;
        }
    }
}

uint8_t fbusSnifferRead(fbusSnifferFrame_t *frames, uint8_t maxFrames)
{
    uint8_t read = 0;

    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
        while (read < maxFrames && snifferCount > 0) {
            frames[read++] = snifferFrames[snifferHead];
            snifferHead = (snifferHead + 1) % FBUS_SNIFFER_BUFFER_FRAMES;
            snifferCount--;
        }
    }

    return read;
}

uint8_t fbusSnifferGetCount(void)
{
    return snifferCount;
}

uint16_t fbusSnifferTakeDropped(void)
{
    uint16_t dropped = 0;

    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
        dropped = snifferDropped;
        snifferDropped = 0;
    }

    return dropped;
}

#endif // USE_FBUS_MASTER

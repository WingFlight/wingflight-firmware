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

#ifdef USE_RX_INPUT_BACKUP_SBUS

#include "drivers/rx_input_backup_sbus.h"

#include "build/atomic.h"
#include "build/build_config.h"

#include "common/utils.h"

#include "drivers/nvic.h"
#include "drivers/serial.h"
#include "drivers/time.h"

#include "pg/rx_input_backup.h"

#include "rx/rx.h"
#include "rx/sbus_channels.h"

// This intentionally does not reuse rx/sbus.c's sbusInit()/sbusDataReceive() - that
// module keeps its frame-assembly state in function-local statics tied to the single
// main RX instance (and touches other main-RX-only globals like rssiSource), so it
// cannot be safely instantiated a second time. Only the reentrant channel decode in
// rx/sbus_channels.c (sbusChannelsDecode()) is shared between the two.

// Same fixed baud/framing as the primary SBUS receiver (rx/sbus.c): 100000 baud, 8E2.
#define SBUS_INPUT_BAUDRATE 100000
#if !defined(SBUS_INPUT_PORT_OPTIONS)
#define SBUS_INPUT_PORT_OPTIONS (SERIAL_STOPBITS_2 | SERIAL_PARITY_EVEN)
#endif

#define SBUS_INPUT_FRAME_BEGIN_BYTE 0x0F
#define SBUS_INPUT_FRAME_SIZE (SBUS_CHANNEL_DATA_LENGTH + 2)
#define SBUS_INPUT_TIME_NEEDED_PER_FRAME_US 4000

// Minimum silence before a 0x0F byte is trusted as a real frame start rather than a
// stray value occurring mid-frame. Only needs to comfortably clear normal back-to-back
// inter-byte spacing within a frame (~0us at this baud rate, since bytes are sent with
// no idle time between them) while staying well under the tightest real inter-frame
// gap this driver needs to support: a 25-byte frame takes ~3ms to transmit at 100000
// baud, and our own sbus_out_frame_rate goes up to 250Hz (4ms period), leaving only
// ~1ms of genuine idle gap at that rate - a larger threshold here would reject every
// real frame boundary and break reception entirely at the high end of a range this
// same codebase lets you configure.
#define SBUS_INPUT_INTERFRAME_GAP_US 500

typedef struct sbusInputFrame_s {
    uint8_t syncByte;
    sbusChannels_t channels;
    uint8_t endByte;
} __attribute__((__packed__)) sbusInputFrame_t;

typedef union sbusInputFrameBuf_u {
    uint8_t bytes[SBUS_INPUT_FRAME_SIZE];
    sbusInputFrame_t frame;
} sbusInputFrameBuf_t;

typedef struct sbusInputFrameData_s {
    sbusInputFrameBuf_t frame;
    volatile timeUs_t startAtUs;
    volatile timeUs_t lastByteAtUs;
    volatile uint8_t position;
    volatile bool done;
} sbusInputFrameData_t;

typedef struct sbusInputState_s {
    sbusInputFrameData_t frameData;
    sbusChannels_t pendingChannels;
    volatile bool pendingFrame;
    uint16_t channelData[RX_INPUT_BACKUP_MAX_CHANNEL];
    // Minimal rxRuntimeState_t used only to satisfy sbusChannelsDecode()'s interface -
    // only its channelData pointer is touched by that function.
    rxRuntimeState_t rxRuntimeState;
} sbusInputState_t;

// One parser per backup port, so two ports can both run SBUS. The port's
// state rides along as the ISR's data argument and update()'s state argument.
static sbusInputState_t sbusInputStates[RX_INPUT_BACKUP_COUNT];

// Note: deliberately does not validate the frame's trailing endByte against known
// SBUS1/SBUS2 markers, and does not sanity-range-check decoded channel values beyond
// what sbusChannelsDecode() itself already does. The primary RX path (rx/sbus.c) never
// rejects a frame for either reason either - it just doesn't recognize an unfamiliar
// endByte as an SBUS2 telemetry page - and being stricter here than that proven
// decoder risks discarding real, valid frames from otherwise-fine hardware for no
// actual gain: the corruption this file was originally hardened against turned out to
// be on the sender side, not a receive-side decoding gap.
static void sbusInputResetParser(sbusInputState_t *s)
{
    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
        s->frameData.startAtUs = 0;
        s->frameData.lastByteAtUs = 0;
        s->frameData.position = 0;
        s->frameData.done = false;
        s->pendingFrame = false;
    }
}

static FAST_CODE void sbusInputDataReceive(uint16_t c, void *data)
{
    sbusInputState_t *s = data;

    const timeUs_t nowUs = microsISR();
    const timeDelta_t frameTime = cmpTimeUs(nowUs, s->frameData.startAtUs);
    // No prior byte to compare against yet (right after init/reset) - treat that as
    // "definitely enough silence" rather than "definitely not enough", so the very
    // next real sync byte isn't unconditionally thrown away.
    const timeDelta_t byteGap = s->frameData.lastByteAtUs == 0
        ? (timeDelta_t)SBUS_INPUT_INTERFRAME_GAP_US
        : cmpTimeUs(nowUs, s->frameData.lastByteAtUs);
    s->frameData.lastByteAtUs = nowUs;

    if (frameTime > (long)(SBUS_INPUT_TIME_NEEDED_PER_FRAME_US + 500)) {
        s->frameData.position = 0;
    }

    if (s->frameData.position == 0) {
        if (c != SBUS_INPUT_FRAME_BEGIN_BYTE || byteGap < SBUS_INPUT_INTERFRAME_GAP_US) {
            return;
        }
        s->frameData.startAtUs = nowUs;
    }

    if (s->frameData.position < SBUS_INPUT_FRAME_SIZE) {
        s->frameData.frame.bytes[s->frameData.position++] = (uint8_t)c;
        if (s->frameData.position >= SBUS_INPUT_FRAME_SIZE) {
            // Snapshot into a separate holding buffer right here, rather than leaving
            // the completed frame sitting in sbusInputFrameData for the consumer to
            // read later - that used to leave a window for the next frame's first byte
            // (landing back at position 0) to start overwriting the same buffer being
            // decoded, tearing adjacent 11-bit channel fields across two frames.
            memcpy(&s->pendingChannels, &s->frameData.frame.frame.channels, sizeof(s->pendingChannels));
            s->pendingFrame = true;
            s->frameData.position = 0;
            s->frameData.done = true;
        } else {
            s->frameData.done = false;
        }
    }
}

// Called from the RX task (rx/rx.c, via rx_input_backup.c's poll loop), not an ISR -
// safe to do the heavier decode/convert work here. Returns true and fills channels[]
// only when a fresh, valid frame was decoded this call; the generic layer in
// rx_input_backup.c owns all freshness/staleness bookkeeping from here on.
static uint8_t sbusInputUpdate(void *state, float *channels, uint8_t channelCount)
{
    sbusInputState_t *s = state;
    // Snapshot the completed frame into a local copy under a brief interrupt mask,
    // rather than decoding directly out of sbusInputFrameData - which the receive
    // ISR owns and can start overwriting (a new frame's first byte landing at
    // position 0) at any point after `done` is observed true but before the decode
    // below finishes reading it. That window used to be able to produce a torn read
    // mixing bytes from two different frames, corrupting adjacent 11-bit channel
    // fields (e.g. one channel's movement bleeding into its neighbour).
    sbusChannels_t frame;
    bool haveFrame = false;

    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
        if (s->pendingFrame) {
            memcpy(&frame, &s->pendingChannels, sizeof(frame));
            s->pendingFrame = false;
            haveFrame = true;
        }
    }

    if (!haveFrame) {
        return 0;
    }

    const uint8_t frameStatus = sbusChannelsDecode(&s->rxRuntimeState, &frame);
    if (frameStatus & (RX_FRAME_DROPPED | RX_FRAME_FAILSAFE)) {
        // RX_FRAME_DROPPED: repeated/stale data from the satellite itself.
        // RX_FRAME_FAILSAFE: the satellite's own internal failsafe is active - it may
        // still be sending numerically valid-looking (repeated/center) channel data,
        // but this must not count as a fresh valid frame, same as rxFrameCheck() never
        // treats a main-RX RX_FRAME_FAILSAFE frame as "signal received" either. Letting
        // it through here would let a satellite that has itself lost its own uplink
        // keep reporting the fallback as active and calling failsafeOnValidDataReceived()
        // (rx.c) - suppressing real failsafe in exactly the scenario, both links
        // actually down, that it exists to catch.
        sbusInputResetParser(s);
        return 0;
    }

    for (uint8_t i = 0; i < channelCount; i++) {
        channels[i] = (5.0f * (float)s->channelData[i] / 8.0f) + 880.0f;
    }

    return channelCount;
}

bool rxInputBackupSbusInit(rxInputBackupOps_t *ops, const rxInputBackupPortConfig_t *config, uint8_t index)
{
    sbusInputState_t *s = &sbusInputStates[index];
    s->rxRuntimeState.channelData = s->channelData;
    sbusInputResetParser(s);

    // Matches rx/sbus.c's own direction/variant exactly: SBUS's signal is
    // natively inverted, so inverted=OFF (normal SBUS wiring) applies hardware
    // inversion by default, and half-duplex uses plain SERIAL_BIDIR (no _PP).
    ops->baudRate = SBUS_INPUT_BAUDRATE;
    ops->portOptions = SBUS_INPUT_PORT_OPTIONS
        | (config->inverted ? SERIAL_NOT_INVERTED : SERIAL_INVERTED)
        | (config->halfDuplex ? SERIAL_BIDIR : SERIAL_UNIDIR);
    ops->isrFn = sbusInputDataReceive;
    ops->state = s;
    ops->channelCount = SBUS_MAX_CHANNEL;   // 16 + 2 digital
    ops->update = sbusInputUpdate;

    return true;
}

#endif // USE_RX_INPUT_BACKUP_SBUS

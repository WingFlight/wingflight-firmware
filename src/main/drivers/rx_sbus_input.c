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

#ifdef USE_RX_SBUS_INPUT

#include "drivers/rx_sbus_input.h"

#include "build/atomic.h"
#include "build/build_config.h"

#include "common/utils.h"

#include "drivers/time.h"

#include "io/serial.h"

#include "pg/rx_sbus_input.h"

#include "rx/rx.h"
#include "rx/sbus_channels.h"

// Same fixed baud/framing as the primary SBUS receiver (rx/sbus.c): 100000 baud, 8E2.
// Inversion/pin-swap are this port's own settings (pg/rx_sbus_input.h), independent
// of the main RX's serialrx_inverted/serialrx_pinswap - different physical UART.
#define SBUS_INPUT_BAUDRATE 100000
#if !defined(SBUS_INPUT_PORT_OPTIONS)
#define SBUS_INPUT_PORT_OPTIONS (SERIAL_STOPBITS_2 | SERIAL_PARITY_EVEN)
#endif

#define SBUS_INPUT_FRAME_BEGIN_BYTE 0x0F
#define SBUS_INPUT_FRAME_SIZE (SBUS_CHANNEL_DATA_LENGTH + 2)
#define SBUS_INPUT_TIME_NEEDED_PER_FRAME_US 4000

// How long without a decoded frame before the SBUS-in link is considered down.
// ~3 missed SBUS frames at the fastest common frame rate (~6-14ms/frame).
#define SBUS_INPUT_STALE_MS 50

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
    volatile uint8_t position;
    volatile bool done;
} sbusInputFrameData_t;

static serialPort_t *sbusInputPort = NULL;

static sbusInputFrameData_t sbusInputFrameData;

static uint16_t sbusInputChannelData[SBUS_INPUT_MAX_CHANNEL];
static float sbusInputChannel[SBUS_INPUT_MAX_CHANNEL];
static timeMs_t sbusInputLastValidFrameMs = 0;

// False until the first genuinely valid (non-dropped, non-failsafe) frame has been
// decoded. Without this, sbusInputIsActive() would read as "active" for up to
// SBUS_INPUT_STALE_MS right after boot/config-change purely because
// sbusInputLastValidFrameMs's zero-init happens to be within that window of
// millis()'s own startup value - reporting fallback available (and, if the main
// link were already down at that moment, feeding zeroed channels) before any real
// frame has ever been seen.
static bool sbusInputHasValidFrame = false;

// Minimal rxRuntimeState_t used only to satisfy sbusChannelsDecode()'s interface -
// only its channelData pointer is touched by that function.
static rxRuntimeState_t sbusInputRxRuntimeState;

static FAST_CODE void sbusInputDataReceive(uint16_t c, void *data)
{
    UNUSED(data);

    const timeUs_t nowUs = microsISR();
    const timeDelta_t frameTime = cmpTimeUs(nowUs, sbusInputFrameData.startAtUs);

    if (frameTime > (long)(SBUS_INPUT_TIME_NEEDED_PER_FRAME_US + 500)) {
        sbusInputFrameData.position = 0;
    }

    if (sbusInputFrameData.position == 0) {
        if (c != SBUS_INPUT_FRAME_BEGIN_BYTE) {
            return;
        }
        sbusInputFrameData.startAtUs = nowUs;
    }

    if (sbusInputFrameData.position < SBUS_INPUT_FRAME_SIZE) {
        sbusInputFrameData.frame.bytes[sbusInputFrameData.position++] = (uint8_t)c;
        sbusInputFrameData.done = (sbusInputFrameData.position >= SBUS_INPUT_FRAME_SIZE);
    }
}

// Called from the RX task (rx/rx.c), not an ISR - safe to do the heavier decode/convert work here.
static void sbusInputUpdate(void)
{
    // Snapshot the completed frame into a local copy under a brief interrupt mask,
    // rather than decoding directly out of sbusInputFrameData - which the receive
    // ISR owns and can start overwriting (a new frame's first byte landing at
    // position 0) at any point after `done` is observed true but before the decode
    // below finishes reading it. That window used to be able to produce a torn read
    // mixing bytes from two different frames, corrupting adjacent 11-bit channel
    // fields (e.g. one channel's movement bleeding into its neighbour).
    sbusChannels_t channels;
    bool haveFrame = false;

    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
        if (sbusInputFrameData.done) {
            memcpy(&channels, &sbusInputFrameData.frame.frame.channels, sizeof(channels));
            sbusInputFrameData.done = false;
            haveFrame = true;
        }
    }

    if (!haveFrame) {
        return;
    }

    const uint8_t frameStatus = sbusChannelsDecode(&sbusInputRxRuntimeState, &channels);
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
        return;
    }

    for (int i = 0; i < SBUS_INPUT_MAX_CHANNEL; i++) {
        sbusInputChannel[i] = (5.0f * (float)sbusInputChannelData[i] / 8) + 880;
    }

    sbusInputHasValidFrame = true;
    sbusInputLastValidFrameMs = millis();
}

bool sbusInputIsEnabled(void)
{
    return sbusInputPort != NULL;
}

// Decodes any newly-completed frame and updates the channel/freshness state.
// Must be called every cycle regardless of whether the main RX link is up or
// the fallback is currently "needed" - it used to be called only as a side
// effect of sbusInputIsActive(), which rx.c only evaluates once the main
// link is already down (short-circuiting `!rxSignalReceived && ...`). That
// starved this of any real-time decoding whenever the main link was healthy,
// leaving diagnostics/MSP polling as the only thing driving it (once every
// poll interval instead of every cycle) and meaning the very first fallback
// frame used at the instant of a real failover could already be stale.
void sbusInputPoll(void)
{
    if (!sbusInputIsEnabled()) {
        return;
    }

    sbusInputUpdate();
}

bool sbusInputIsActive(void)
{
    if (!sbusInputIsEnabled() || !sbusInputHasValidFrame) {
        return false;
    }

    return (timeMs_t)(millis() - sbusInputLastValidFrameMs) < SBUS_INPUT_STALE_MS;
}

uint8_t sbusInputGetChannelCount(void)
{
    return SBUS_INPUT_MAX_CHANNEL;
}

float sbusInputGetChannel(uint8_t channel)
{
    if (channel >= SBUS_INPUT_MAX_CHANNEL) {
        return 0;
    }
    return sbusInputChannel[channel];
}

void sbusInputInit(void)
{
    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_RX_SBUS_INPUT);
    if (!portConfig) {
        sbusInputPort = NULL;
        return;
    }

    sbusInputRxRuntimeState.channelData = sbusInputChannelData;
    sbusInputFrameData.position = 0;
    sbusInputFrameData.done = false;
    sbusInputLastValidFrameMs = 0;
    sbusInputHasValidFrame = false;

    sbusInputPort = openSerialPort(portConfig->identifier,
        FUNCTION_RX_SBUS_INPUT,
        sbusInputDataReceive,
        NULL,
        SBUS_INPUT_BAUDRATE,
        MODE_RX,
        SBUS_INPUT_PORT_OPTIONS |
            (sbusInputConfig()->inverted ? SERIAL_NOT_INVERTED : SERIAL_INVERTED) |
            (sbusInputConfig()->pinSwap ? SERIAL_PINSWAP : SERIAL_NOSWAP));
}

#endif // USE_RX_SBUS_INPUT

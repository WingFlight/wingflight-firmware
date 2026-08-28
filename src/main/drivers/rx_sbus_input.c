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

#include "platform.h"

#ifdef USE_RX_SBUS_INPUT

#include "drivers/rx_sbus_input.h"

#include "build/build_config.h"

#include "common/utils.h"

#include "drivers/time.h"

#include "io/serial.h"

#include "pg/rx.h"

#include "rx/rx.h"
#include "rx/sbus_channels.h"

// Same electrical parameters as the primary SBUS receiver (rx/sbus.c): 100000 baud,
// 8E2, normally inverted. Only inversion is shared with the primary RX config -
// SBUS wiring polarity is a board property, not a per-link one.
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
    if (!sbusInputFrameData.done) {
        return;
    }
    sbusInputFrameData.done = false;

    const uint8_t frameStatus = sbusChannelsDecode(&sbusInputRxRuntimeState, &sbusInputFrameData.frame.frame.channels);
    if (frameStatus & RX_FRAME_DROPPED) {
        // Repeated/stale data from the satellite itself - don't treat as a fresh valid frame.
        return;
    }

    for (int i = 0; i < SBUS_INPUT_MAX_CHANNEL; i++) {
        sbusInputChannel[i] = (5.0f * (float)sbusInputChannelData[i] / 8) + 880;
    }

    sbusInputLastValidFrameMs = millis();
}

bool sbusInputIsEnabled(void)
{
    return sbusInputPort != NULL;
}

bool sbusInputIsActive(void)
{
    if (!sbusInputIsEnabled()) {
        return false;
    }

    sbusInputUpdate();

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

    sbusInputPort = openSerialPort(portConfig->identifier,
        FUNCTION_RX_SBUS_INPUT,
        sbusInputDataReceive,
        NULL,
        SBUS_INPUT_BAUDRATE,
        MODE_RX,
        SBUS_INPUT_PORT_OPTIONS |
            (rxConfig()->serialrx_inverted ? SERIAL_NOT_INVERTED : SERIAL_INVERTED));
}

#endif // USE_RX_SBUS_INPUT

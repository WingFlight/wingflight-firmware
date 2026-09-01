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

#ifdef USE_RX_INPUT_BACKUP_SPEKTRUM

#include "drivers/rx_input_backup_spektrum.h"

#include "build/atomic.h"
#include "build/build_config.h"

#include "common/utils.h"

#include "drivers/nvic.h"
#include "drivers/serial.h"
#include "drivers/time.h"

#include "pg/rx_input_backup.h"

// This intentionally does not reuse rx/spektrum.c's spektrumInit()/
// spektrumDataReceive() - that module keeps its frame-assembly state in
// function-local statics tied to the single main RX instance, plus VTX
// control and RSSI-over-Spektrum-channel handling this backup link has no use
// for (receive-only, no telemetry or VTX control, ever). This is a
// from-scratch minimal reimplementation of just the RC-frame framing,
// verified line-by-line against rx/spektrum.c.
//
// Spektrum DSM and FBUS/FPort2 share the same "one file, two Init entry
// points differing only by a constant" shape - see rx_input_backup_fbus.c.

#define SPEKTRUM_INPUT_PORT_OPTIONS (SERIAL_STOPBITS_1 | SERIAL_PARITY_NO)
#define SPEKTRUM_INPUT_BAUDRATE 115200

// Matches rx/spektrum.c's own SPEK_FRAME_SIZE/SPEKTRUM_NEEDED_FRAME_INTERVAL.
// 2 header bytes (fades/system) + up to 7 channel slots (2 bytes each).
#define SPEKTRUM_INPUT_FRAME_SIZE 16
#define SPEKTRUM_INPUT_FRAME_GAP_US 5000

typedef struct spektrumInputFrameData_s {
    uint8_t bytes[SPEKTRUM_INPUT_FRAME_SIZE];
    volatile timeUs_t lastByteAtUs;
    volatile uint8_t position;
} spektrumInputFrameData_t;

static spektrumInputFrameData_t spektrumInputFrameData;
static uint8_t spektrumInputPendingFrame[SPEKTRUM_INPUT_FRAME_SIZE];
static volatile bool spektrumInputPendingFrameReady = false;

static uint32_t spektrumInputChannelData[RX_INPUT_BACKUP_MAX_CHANNEL];

// Set once at Init by whichever entry point was called - see this file's
// header comment for why resolution is two providers, not a config flag.
static uint8_t spektrumInputChanShift;
static uint8_t spektrumInputChanMask;
static uint8_t spektrumInputChannelCount;
static bool spektrumInputHiRes;

static void spektrumInputResetParser(void)
{
    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
        spektrumInputFrameData.lastByteAtUs = 0;
        spektrumInputFrameData.position = 0;
        spektrumInputPendingFrameReady = false;
    }
}

// Receive ISR callback. Unlike every other provider in this framework,
// rx/spektrum.c's own receiver has no leading sync byte or frame-start marker
// to validate against at all - it relies purely on fixed-frame-size timing
// (a byte arriving more than SPEKTRUM_INPUT_FRAME_GAP_US after the last one
// starts a new frame). This is a faithful port of that same, admittedly
// minimal, framing - not something being loosened here.
static FAST_CODE void spektrumInputDataReceive(uint16_t c, void *data)
{
    UNUSED(data);

    const timeUs_t nowUs = microsISR();
    const timeDelta_t byteGap = spektrumInputFrameData.lastByteAtUs == 0
        ? (timeDelta_t)SPEKTRUM_INPUT_FRAME_GAP_US + 1
        : cmpTimeUs(nowUs, spektrumInputFrameData.lastByteAtUs);
    spektrumInputFrameData.lastByteAtUs = nowUs;

    if (byteGap > SPEKTRUM_INPUT_FRAME_GAP_US) {
        spektrumInputFrameData.position = 0;
    }

    if (spektrumInputFrameData.position < SPEKTRUM_INPUT_FRAME_SIZE) {
        spektrumInputFrameData.bytes[spektrumInputFrameData.position++] = (uint8_t)c;
        if (spektrumInputFrameData.position == SPEKTRUM_INPUT_FRAME_SIZE) {
            // Snapshot into a separate holding buffer right here, rather than
            // leaving the completed frame sitting in spektrumInputFrameData
            // for the consumer to read later - see the SBUS/FBUS/FPort/IBUS/
            // SUMD providers' own identical comment for why.
            memcpy(spektrumInputPendingFrame, spektrumInputFrameData.bytes, SPEKTRUM_INPUT_FRAME_SIZE);
            spektrumInputPendingFrameReady = true;
            spektrumInputFrameData.position = 0;
        }
    }
}

// Called from the RX task (rx/rx.c, via rx_input_backup.c's poll loop), not an
// ISR - safe to do the heavier decode/convert work here.
static bool spektrumInputUpdate(float *channels, uint8_t channelCount)
{
    uint8_t frame[SPEKTRUM_INPUT_FRAME_SIZE];
    bool haveFrame = false;

    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
        if (spektrumInputPendingFrameReady) {
            memcpy(frame, spektrumInputPendingFrame, SPEKTRUM_INPUT_FRAME_SIZE);
            spektrumInputPendingFrameReady = false;
            haveFrame = true;
        }
    }

    if (!haveFrame) {
        return false;
    }

    // No checksum to verify - see this file's top-of-file comment. Matches
    // rx/spektrum.c's own channel-extraction loop exactly (minus the VTX
    // control frame special-case and RSSI-channel exclusion, neither of which
    // apply to this receive-only, telemetry-free link).
    for (uint8_t b = 3; b < SPEKTRUM_INPUT_FRAME_SIZE; b += 2) {
        const uint8_t spekChannel = 0x0F & (frame[b - 1] >> spektrumInputChanShift);
        if (spekChannel < spektrumInputChannelCount && spekChannel < RX_INPUT_BACKUP_MAX_CHANNEL) {
            spektrumInputChannelData[spekChannel] = ((uint32_t)(frame[b - 1] & spektrumInputChanMask) << 8) + frame[b];
        }
    }

    for (uint8_t i = 0; i < channelCount; i++) {
        channels[i] = spektrumInputHiRes
            ? 0.5f * (float)spektrumInputChannelData[i] + 988.0f
            : (float)spektrumInputChannelData[i] + 988.0f;
    }

    return true;
}

static bool spektrumInputInitCommon(rxInputBackupOps_t *ops, uint8_t chanShift, uint8_t chanMask, uint8_t channelCount, bool hiRes)
{
    memset(&spektrumInputFrameData, 0, sizeof(spektrumInputFrameData));
    spektrumInputResetParser();

    spektrumInputChanShift = chanShift;
    spektrumInputChanMask = chanMask;
    spektrumInputChannelCount = channelCount;
    spektrumInputHiRes = hiRes;

    // Matches rx/spektrum.c's own direction/variant exactly (identical for
    // both resolutions): Spektrum DSM's signal is natively non-inverted (like
    // FBUS/FPort/FPort2/IBUS/SUMD), plain SERIAL_BIDIR for half-duplex (like
    // SBUS/IBUS/SUMD, not the push-pull variant).
    ops->baudRate = SPEKTRUM_INPUT_BAUDRATE;
    ops->portOptions = SPEKTRUM_INPUT_PORT_OPTIONS
        | (rxInputBackupConfig()->inverted ? SERIAL_INVERTED : SERIAL_NOT_INVERTED)
        | (rxInputBackupConfig()->halfDuplex ? SERIAL_BIDIR : SERIAL_UNIDIR);
    ops->isrFn = spektrumInputDataReceive;
    ops->channelCount = channelCount;
    ops->update = spektrumInputUpdate;

    return true;
}

bool rxInputBackupSpektrum1024Init(rxInputBackupOps_t *ops)
{
    return spektrumInputInitCommon(ops, 2, 0x03, 7, false);
}

bool rxInputBackupSpektrum2048Init(rxInputBackupOps_t *ops)
{
    return spektrumInputInitCommon(ops, 3, 0x07, 12, true);
}

#endif // USE_RX_INPUT_BACKUP_SPEKTRUM

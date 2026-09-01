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

#ifdef USE_RX_INPUT_BACKUP_SUMD

#include "drivers/rx_input_backup_sumd.h"

#include "build/atomic.h"
#include "build/build_config.h"

#include "common/crc.h"
#include "common/maths.h"
#include "common/utils.h"

#include "drivers/nvic.h"
#include "drivers/serial.h"
#include "drivers/time.h"

#include "pg/rx_input_backup.h"

// This intentionally does not reuse rx/sumd.c's sumdInit()/sumdDataReceive() -
// that module keeps its frame-assembly state in function-local statics tied to
// the single main RX instance. This is a from-scratch minimal
// reimplementation of just the RC-frame framing, verified line-by-line
// against rx/sumd.c, reusing only the generic, already-reentrant
// common/crc.c's crc16_ccitt() the real driver itself builds on.

#define SUMD_INPUT_PORT_OPTIONS (SERIAL_STOPBITS_1 | SERIAL_PARITY_NO)
#define SUMD_INPUT_BAUDRATE 115200
#define SUMD_INPUT_TIME_NEEDED_PER_FRAME_US 4000

// Matches rx/sumd.c's own constants exactly. The wire format can declare up
// to 32 channels regardless of how many this framework actually keeps
// (RX_INPUT_BACKUP_MAX_CHANNEL, 18) - the parser/buffer must still be sized
// for the full protocol max so a real 32-channel frame is consumed correctly
// (header+checksum included), even though only the first 18 decoded channels
// ever reach the caller.
#define SUMD_INPUT_SYNC_BYTE 0xA8
#define SUMD_INPUT_MAX_CHANNEL 32
#define SUMD_INPUT_BUFFSIZE (SUMD_INPUT_MAX_CHANNEL * 2 + 5)
#define SUMD_INPUT_HEADER_LENGTH 3
#define SUMD_INPUT_CRC_LENGTH 2
#define SUMD_INPUT_OFFSET_CHANNEL_1_HIGH 3
#define SUMD_INPUT_OFFSET_CHANNEL_1_LOW 4
#define SUMD_INPUT_BYTES_PER_CHANNEL 2
#define SUMD_INPUT_SYNC_BYTE_INDEX 0
#define SUMD_INPUT_CHANNEL_COUNT_INDEX 2

#define SUMD_INPUT_FRAME_STATE_V1_OK 0x01
#define SUMD_INPUT_FRAME_STATE_V3_OK 0x03
#define SUMD_INPUT_FRAME_STATE_FAILSAFE 0x81

typedef struct sumdInputFrameData_s {
    uint8_t bytes[SUMD_INPUT_BUFFSIZE];
    volatile timeUs_t lastByteAtUs;
    volatile uint16_t crc;
    volatile uint8_t position;
    volatile uint8_t channelCount;
} sumdInputFrameData_t;

static sumdInputFrameData_t sumdInputFrameData;
static uint8_t sumdInputPendingFrame[SUMD_INPUT_BUFFSIZE];
static volatile uint8_t sumdInputPendingChannelCount = 0;
static volatile bool sumdInputPendingFrameReady = false;

static uint16_t sumdInputChannelData[RX_INPUT_BACKUP_MAX_CHANNEL];

static void sumdInputResetParser(void)
{
    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
        sumdInputFrameData.lastByteAtUs = 0;
        sumdInputFrameData.position = 0;
        sumdInputPendingFrameReady = false;
    }
}

static FAST_CODE void sumdInputDataReceive(uint16_t c, void *data)
{
    UNUSED(data);

    const timeUs_t nowUs = microsISR();

    // Matches rx/sumd.c's own gap check exactly: measured per-byte against the
    // total frame-time budget, not a shorter inter-byte gap - SUMD has no
    // per-byte silence guarantee to rely on the way FBUS's interbyte timeout
    // does.
    if (sumdInputFrameData.lastByteAtUs != 0
        && cmpTimeUs(nowUs, sumdInputFrameData.lastByteAtUs) > SUMD_INPUT_TIME_NEEDED_PER_FRAME_US) {
        sumdInputFrameData.position = 0;
    }
    sumdInputFrameData.lastByteAtUs = nowUs;

    if (sumdInputFrameData.position == SUMD_INPUT_SYNC_BYTE_INDEX) {
        if (c != SUMD_INPUT_SYNC_BYTE) {
            return;
        }
        sumdInputPendingFrameReady = false; // lazy consumer didn't fetch the previous frame
        sumdInputFrameData.crc = 0;
    } else if (sumdInputFrameData.position == SUMD_INPUT_CHANNEL_COUNT_INDEX) {
        if (c == 0 || c > SUMD_INPUT_MAX_CHANNEL) {
            sumdInputFrameData.position = 0;
            return;
        }
        sumdInputFrameData.channelCount = (uint8_t)c;
    }

    if (sumdInputFrameData.position < SUMD_INPUT_BUFFSIZE) {
        sumdInputFrameData.bytes[sumdInputFrameData.position] = (uint8_t)c;
    }
    sumdInputFrameData.position++;

    const uint8_t crcSpan = sumdInputFrameData.channelCount * SUMD_INPUT_BYTES_PER_CHANNEL + SUMD_INPUT_HEADER_LENGTH;
    if (sumdInputFrameData.position <= crcSpan) {
        sumdInputFrameData.crc = crc16_ccitt(sumdInputFrameData.crc, (uint8_t)c);
    } else if (sumdInputFrameData.position == crcSpan + SUMD_INPUT_CRC_LENGTH) {
        // Snapshot into a separate holding buffer right here, rather than
        // leaving the completed frame sitting in sumdInputFrameData for the
        // consumer to read later - see the SBUS/FBUS/FPort/IBUS providers'
        // own identical comment for why.
        memcpy(sumdInputPendingFrame, sumdInputFrameData.bytes, crcSpan + SUMD_INPUT_CRC_LENGTH);
        sumdInputPendingChannelCount = sumdInputFrameData.channelCount;
        sumdInputPendingFrameReady = true;
        sumdInputFrameData.position = 0;
    }
}

// Called from the RX task (rx/rx.c, via rx_input_backup.c's poll loop), not an
// ISR - safe to do the heavier decode/convert work here.
static bool sumdInputUpdate(float *channels, uint8_t channelCount)
{
    uint8_t frame[SUMD_INPUT_BUFFSIZE];
    uint8_t sumdChannelCount = 0;
    bool haveFrame = false;

    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
        if (sumdInputPendingFrameReady) {
            sumdChannelCount = sumdInputPendingChannelCount;
            memcpy(frame, sumdInputPendingFrame, sumdChannelCount * SUMD_INPUT_BYTES_PER_CHANNEL + SUMD_INPUT_HEADER_LENGTH + SUMD_INPUT_CRC_LENGTH);
            sumdInputPendingFrameReady = false;
            haveFrame = true;
        }
    }

    if (!haveFrame) {
        return false;
    }

    uint16_t crc = 0;
    const uint8_t crcSpan = sumdChannelCount * SUMD_INPUT_BYTES_PER_CHANNEL + SUMD_INPUT_HEADER_LENGTH;
    for (uint8_t i = 0; i < crcSpan; i++) {
        crc = crc16_ccitt(crc, frame[i]);
    }
    const uint16_t rxCrc = ((uint16_t)frame[crcSpan] << 8) | frame[crcSpan + 1];
    if (crc != rxCrc) {
        return false;
    }

    // Failsafe frames from the satellite itself must not count as a fresh
    // valid frame - same rationale as every other provider here. Anything
    // other than a recognized OK/failsafe status byte is treated as garbage
    // and rejected too, matching rx/sumd.c's own switch (no default case
    // sets RX_FRAME_COMPLETE).
    if (frame[1] == SUMD_INPUT_FRAME_STATE_FAILSAFE) {
        return false;
    }
    if (frame[1] != SUMD_INPUT_FRAME_STATE_V1_OK && frame[1] != SUMD_INPUT_FRAME_STATE_V3_OK) {
        return false;
    }

    const uint8_t channelsToProcess = MIN(sumdChannelCount, RX_INPUT_BACKUP_MAX_CHANNEL);
    for (uint8_t i = 0; i < channelsToProcess; i++) {
        sumdInputChannelData[i] = ((uint16_t)frame[SUMD_INPUT_BYTES_PER_CHANNEL * i + SUMD_INPUT_OFFSET_CHANNEL_1_HIGH] << 8)
            | frame[SUMD_INPUT_BYTES_PER_CHANNEL * i + SUMD_INPUT_OFFSET_CHANNEL_1_LOW];
    }

    // No scaling formula needed - unlike SBUS/FBUS/FPort's 11-bit range,
    // rx/sumd.c's own sumdReadRawRC() just divides by 8; these values are
    // already in normal ~1000-2000us units once scaled that way.
    for (uint8_t i = 0; i < channelCount; i++) {
        channels[i] = (float)sumdInputChannelData[i] / 8.0f;
    }

    return true;
}

bool rxInputBackupSumdInit(rxInputBackupOps_t *ops)
{
    memset(&sumdInputFrameData, 0, sizeof(sumdInputFrameData));
    sumdInputResetParser();

    // Matches rx/sumd.c's own direction/variant exactly: SUMD's signal is
    // natively non-inverted (like FBUS/FPort/FPort2/IBUS), plain SERIAL_BIDIR
    // for half-duplex (like SBUS/IBUS, not the push-pull variant).
    ops->baudRate = SUMD_INPUT_BAUDRATE;
    ops->portOptions = SUMD_INPUT_PORT_OPTIONS
        | (rxInputBackupConfig()->inverted ? SERIAL_INVERTED : SERIAL_NOT_INVERTED)
        | (rxInputBackupConfig()->halfDuplex ? SERIAL_BIDIR : SERIAL_UNIDIR);
    ops->isrFn = sumdInputDataReceive;
    ops->channelCount = RX_INPUT_BACKUP_MAX_CHANNEL;
    ops->update = sumdInputUpdate;

    return true;
}

#endif // USE_RX_INPUT_BACKUP_SUMD

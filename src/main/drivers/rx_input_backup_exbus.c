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

#ifdef USE_RX_INPUT_BACKUP_EXBUS

#include "drivers/rx_input_backup_exbus.h"

#include "build/atomic.h"
#include "build/build_config.h"

#include "common/utils.h"

#include "drivers/nvic.h"
#include "drivers/serial.h"
#include "drivers/time.h"

#include "pg/rx_input_backup.h"

// This intentionally does not reuse rx/jetiexbus.c's jetiExBusInit()/
// jetiExBusDataReceive() - that module keeps its frame-assembly state in
// function-local/file-scope statics tied to the single main RX instance,
// plus a whole second buffer and state machine for telemetry/Jetibox request
// frames this backup link has no use for (receive-only, no telemetry,
// ever). This is a from-scratch minimal reimplementation of just the
// channel-data framing, verified line-by-line against rx/jetiexbus.c,
// reusing only its exact CRC16 algorithm (jetiExBusCalcCRC16 - a bespoke
// Jeti-specific bit-manipulation, not one of common/crc.c's standard
// polynomials, so it's copied here verbatim rather than shared).

#define EXBUS_INPUT_PORT_OPTIONS (SERIAL_STOPBITS_1 | SERIAL_PARITY_NO)
#define EXBUS_INPUT_BAUDRATE 125000
#define EXBUS_INPUT_MIN_FRAME_GAP_US 1000

// Matches rx/jetiexbus.c's own constants exactly.
#define EXBUS_INPUT_CHANNEL_COUNT 16
#define EXBUS_INPUT_HEADER_LEN 6
#define EXBUS_INPUT_CRC_LEN 2
#define EXBUS_INPUT_FRAME_SIZE (EXBUS_INPUT_HEADER_LEN + EXBUS_INPUT_CHANNEL_COUNT * 2 + EXBUS_INPUT_CRC_LEN) // 40

#define EXBUS_INPUT_START_CHANNEL_FRAME 0x3E
#define EXBUS_INPUT_REQ_WITH_TELEMETRY 0x01
#define EXBUS_INPUT_REQ_NO_TELEMETRY 0x03
#define EXBUS_INPUT_DATA_ID_CHANNEL 0x31

// Header layout, matching rx/jetiexbus.h's exBusHeader_e exactly.
#define EXBUS_INPUT_HEADER_SYNC 0
#define EXBUS_INPUT_HEADER_REQ 1
#define EXBUS_INPUT_HEADER_MSG_LEN 2
#define EXBUS_INPUT_HEADER_DATA_ID 4

typedef struct exbusInputFrameData_s {
    uint8_t bytes[EXBUS_INPUT_FRAME_SIZE];
    volatile timeUs_t lastByteAtUs;
    volatile uint8_t position;
} exbusInputFrameData_t;

typedef struct exbusInputState_s {
    exbusInputFrameData_t frameData;
    uint8_t pendingFrame[EXBUS_INPUT_FRAME_SIZE];
    volatile bool pendingFrameReady;
    uint16_t channelData[EXBUS_INPUT_CHANNEL_COUNT];
} exbusInputState_t;

// One parser per backup port, so two ports can both run EX Bus.
static exbusInputState_t exbusInputStates[RX_INPUT_BACKUP_COUNT];

static void exbusInputResetParser(exbusInputState_t *s)
{
    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
        s->frameData.lastByteAtUs = 0;
        s->frameData.position = 0;
        s->pendingFrameReady = false;
    }
}

// Reimplementation of rx/jetiexbus.c's jetiExBusCalcCRC16() - a bespoke
// Jeti-specific CRC, not a standard polynomial. A frame (including its own
// trailing 2-byte CRC) is valid iff this returns 0 when run across the whole
// thing, same "residue check" convention rx/jetiexbus.c itself relies on.
static uint16_t exbusInputCalcCRC16(const uint8_t *frame, uint8_t length)
{
    uint16_t crc = 0;
    for (uint8_t i = 0; i < length; i++) {
        uint8_t value = frame[i] ^ (uint8_t)crc;
        value ^= value << 4;
        crc = (((uint16_t)value << 8) | ((crc & 0xFF00) >> 8))
            ^ (uint8_t)(value >> 4)
            ^ ((uint16_t)value << 3);
    }
    return crc;
}

static FAST_CODE void exbusInputDataReceive(uint16_t c, void *data)
{
    exbusInputState_t *s = data;

    const timeUs_t nowUs = microsISR();
    const timeDelta_t byteGap = s->frameData.lastByteAtUs == 0
        ? (timeDelta_t)EXBUS_INPUT_MIN_FRAME_GAP_US + 1
        : cmpTimeUs(nowUs, s->frameData.lastByteAtUs);
    s->frameData.lastByteAtUs = nowUs;

    if (byteGap > EXBUS_INPUT_MIN_FRAME_GAP_US) {
        s->frameData.position = 0;
    }

    if (s->frameData.position == 0) {
        // Only the channel-data frame start byte is ever recognized - the
        // other EX Bus frame start (0x3D, a request addressed to the FC) is
        // never something this receive-only link needs to answer, so it's
        // simply never treated as a valid frame start here at all, unlike
        // rx/jetiexbus.c's own driver which tracks both in parallel.
        if (c != EXBUS_INPUT_START_CHANNEL_FRAME) {
            return;
        }
    }

    if (s->frameData.position >= EXBUS_INPUT_FRAME_SIZE) {
        s->frameData.position = 0;
        return;
    }

    s->frameData.bytes[s->frameData.position++] = (uint8_t)c;

    if (s->frameData.position == EXBUS_INPUT_HEADER_LEN) {
        // Reject early rather than continuing to fill a buffer sized only
        // for the one frame shape this provider supports (the fixed
        // 16-channel channel-data frame).
        if (s->frameData.bytes[EXBUS_INPUT_HEADER_MSG_LEN] != EXBUS_INPUT_FRAME_SIZE
            || s->frameData.bytes[EXBUS_INPUT_HEADER_DATA_ID] != EXBUS_INPUT_DATA_ID_CHANNEL
            || (s->frameData.bytes[EXBUS_INPUT_HEADER_REQ] != EXBUS_INPUT_REQ_WITH_TELEMETRY
                && s->frameData.bytes[EXBUS_INPUT_HEADER_REQ] != EXBUS_INPUT_REQ_NO_TELEMETRY)) {
            s->frameData.position = 0;
            return;
        }
    }

    if (s->frameData.position == EXBUS_INPUT_FRAME_SIZE) {
        // Snapshot into a separate holding buffer right here, rather than
        // leaving the completed frame sitting in exbusInputFrameData for the
        // consumer to read later - see the SBUS/FBUS/FPort providers' own
        // identical comment for why (torn-frame race with the next frame's
        // first byte landing back at position 0).
        memcpy(s->pendingFrame, s->frameData.bytes, EXBUS_INPUT_FRAME_SIZE);
        s->pendingFrameReady = true;
        s->frameData.position = 0;
    }
}

// Called from the RX task (rx/rx.c, via rx_input_backup.c's poll loop), not an
// ISR - safe to do the heavier decode/convert work here.
static uint8_t exbusInputUpdate(void *state, float *channels, uint8_t channelCount)
{
    exbusInputState_t *s = state;
    uint8_t frame[EXBUS_INPUT_FRAME_SIZE];
    bool haveFrame = false;

    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
        if (s->pendingFrameReady) {
            memcpy(frame, s->pendingFrame, EXBUS_INPUT_FRAME_SIZE);
            s->pendingFrameReady = false;
            haveFrame = true;
        }
    }

    if (!haveFrame) {
        return 0;
    }

    // Residue check across the whole frame including its own trailing CRC -
    // matches rx/jetiexbus.c's own jetiExBusCalcCRC16(frame, msg_len) == 0.
    if (exbusInputCalcCRC16(frame, EXBUS_INPUT_FRAME_SIZE) != 0) {
        return 0;
    }

    // 16 channels, little-endian 16-bit each, right-shifted by 3 to this
    // framework's usual ~1000-2000us-ish raw scale - matches rx/jetiexbus.c's
    // own jetiExBusDecodeChannelFrame()/jetiExBusReadRawRC() exactly (no
    // added offset at read time, unlike SBUS/FBUS/FPort's 11-bit formula).
    for (uint8_t i = 0; i < EXBUS_INPUT_CHANNEL_COUNT; i++) {
        const uint8_t frameAddr = EXBUS_INPUT_HEADER_LEN + i * 2;
        const uint16_t value = frame[frameAddr] | ((uint16_t)frame[frameAddr + 1] << 8);
        s->channelData[i] = value >> 3;
    }

    for (uint8_t i = 0; i < channelCount; i++) {
        channels[i] = (float)s->channelData[i];
    }

    return channelCount;
}

bool rxInputBackupExbusInit(rxInputBackupOps_t *ops, const rxInputBackupPortConfig_t *config, uint8_t index)
{
    exbusInputState_t *s = &exbusInputStates[index];
    memset(&s->frameData, 0, sizeof(s->frameData));
    exbusInputResetParser(s);

    // Matches rx/jetiexbus.c's own direction: EX Bus's signal is natively
    // non-inverted (like FBUS/FPort/FPort2). Half-duplex is left
    // user-configurable here (plain SERIAL_BIDIR, like SBUS) rather than
    // forced on unconditionally the way rx/jetiexbus.c itself always does -
    // that driver hardcodes it because the main RX use case may need to
    // reply with telemetry on the same wire; this receive-only link never
    // does, so a genuinely two-wire backup wiring (RX only, TX left
    // disconnected) is equally valid here.
    ops->baudRate = EXBUS_INPUT_BAUDRATE;
    ops->portOptions = EXBUS_INPUT_PORT_OPTIONS
        | (config->inverted ? SERIAL_INVERTED : SERIAL_NOT_INVERTED)
        | (config->halfDuplex ? SERIAL_BIDIR : SERIAL_UNIDIR);
    ops->isrFn = exbusInputDataReceive;
    ops->state = s;
    ops->channelCount = EXBUS_INPUT_CHANNEL_COUNT;
    ops->update = exbusInputUpdate;

    return true;
}

#endif // USE_RX_INPUT_BACKUP_EXBUS

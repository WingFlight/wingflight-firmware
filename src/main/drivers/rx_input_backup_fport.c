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

#ifdef USE_RX_INPUT_BACKUP_FPORT

#include "drivers/rx_input_backup_fport.h"

#include "build/atomic.h"
#include "build/build_config.h"

#include "common/utils.h"

#include "drivers/nvic.h"
#include "drivers/serial.h"
#include "drivers/time.h"

#include "pg/rx_input_backup.h"

#include "rx/frsky_crc.h"
#include "rx/rx.h"
#include "rx/sbus_channels.h"

// This intentionally does not reuse rx/fport.c's fportRxInit()/fportDataReceive() -
// that module keeps its frame-assembly state in function-local statics tied to
// the single main RX instance, plus a ring buffer and telemetry-response state
// we have no use for here (this link is receive-only, no telemetry, ever). This
// is a from-scratch minimal reimplementation of just the RC-frame framing,
// verified line-by-line against rx/fport.c, reusing only the same reentrant
// rx/sbus_channels.c decode already shared by every provider in this framework,
// plus rx/frsky_crc.c's checksum helper fport.c itself builds on.
//
// Unlike FBUS's simple length-prefixed framing, FPort uses HDLC-style byte-
// stuffing: 0x7E is an unconditional frame delimiter (doubles as both the end
// of the previous frame and the start of the next - there is no separate end
// marker) and 0x7D escapes the following byte via XOR 0x20. Because the sender
// always escapes any literal occurrence of 0x7E/0x7D before transmission, a
// received 0x7E can never be mistaken for in-frame data - this makes frame
// boundaries self-synchronizing, unlike FBUS's length-byte-value heuristic
// (rx_input_backup_fbus.c), so no interbyte-silence gate is needed here.

#define FPORT_INPUT_BAUDRATE 115200
#define FPORT_INPUT_PORT_OPTIONS (SERIAL_STOPBITS_1 | SERIAL_PARITY_NO)

#define FPORT_INPUT_FRAME_MARKER 0x7E
#define FPORT_INPUT_ESCAPE_CHAR 0x7D
#define FPORT_INPUT_ESCAPE_MASK 0x20

#define FPORT_INPUT_FRAME_TYPE_CONTROL 0x00

// [length][type][23-byte sbusChannels_t][rssi][checksum], de-stuffed. The
// length byte's own value is always payload+checksum, i.e. total-2: matches
// rx/fport.c's FPORT_FRAME_PAYLOAD_LENGTH_CONTROL (1 type byte + 23-byte
// sbusChannels_t + 1 rssi byte = 25) for a control frame specifically.
#define FPORT_INPUT_PAYLOAD_LENGTH_CONTROL 25
#define FPORT_INPUT_FRAME_SIZE (FPORT_INPUT_PAYLOAD_LENGTH_CONTROL + 2) // 27

// rx/fport.c's own total-frame-duration watchdog (FPORT_TIME_NEEDED_PER_FRAME_US
// + the same 500us margin it uses) - measured from the opening 0x7E, not a
// per-byte gap, since 0x7E's escaping guarantee already makes byte-value
// framing unambiguous (no per-byte gap heuristic needed the way FBUS needs one).
#define FPORT_INPUT_TIME_NEEDED_PER_FRAME_US 3000

typedef struct fportInputFrameData_s {
    uint8_t bytes[FPORT_INPUT_FRAME_SIZE];
    volatile timeUs_t startAtUs;
    volatile uint8_t position;
    volatile bool escapePending;
} fportInputFrameData_t;

typedef struct fportInputState_s {
    fportInputFrameData_t frameData;
    uint8_t pendingFrame[FPORT_INPUT_FRAME_SIZE];
    volatile uint8_t pendingLength;
    volatile bool pendingFrameReady;
    uint16_t channelData[RX_INPUT_BACKUP_MAX_CHANNEL];
    rxRuntimeState_t rxRuntimeState;
} fportInputState_t;

// One parser per backup port, so two ports can both run F.Port.
static fportInputState_t fportInputStates[RX_INPUT_BACKUP_COUNT];

static void fportInputResetParser(fportInputState_t *s)
{
    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
        s->frameData.startAtUs = 0;
        s->frameData.position = 0;
        s->frameData.escapePending = false;
        s->pendingFrameReady = false;
    }
}

static FAST_CODE void fportInputDataReceive(uint16_t c, void *data)
{
    fportInputState_t *s = data;

    const timeUs_t nowUs = microsISR();

    // Total-duration watchdog, checked before handling this byte - matches
    // rx/fport.c's own framePosition>1 (1-based: "at least one byte already
    // stored") check, which is position>0 here since this position is 0-based
    // (position IS the count of bytes already stored, not offset-by-one).
    if (s->frameData.position > 0
        && cmpTimeUs(nowUs, s->frameData.startAtUs) > FPORT_INPUT_TIME_NEEDED_PER_FRAME_US + 500) {
        s->frameData.position = 0;
    }

    const uint8_t val = (uint8_t)c;

    if (val == FPORT_INPUT_FRAME_MARKER) {
        if (s->frameData.position > 0) {
            ATOMIC_BLOCK(NVIC_PRIO_MAX) {
                memcpy(s->pendingFrame, s->frameData.bytes, s->frameData.position);
                s->pendingLength = s->frameData.position;
                s->pendingFrameReady = true;
            }
        }
        s->frameData.position = 0;
        s->frameData.startAtUs = nowUs;
        // Unconditional reset (rx/fport.c only clears this inside the
        // "position>1" branch, which can leak a stale escape flag into the next
        // frame after a malformed 0x7D immediately followed by 0x7E) - a
        // deliberate, safe divergence, not a fidelity gap.
        s->frameData.escapePending = false;
        return;
    }

    if (val == FPORT_INPUT_ESCAPE_CHAR) {
        s->frameData.escapePending = true;
        return;
    }

    if (s->frameData.position >= FPORT_INPUT_FRAME_SIZE) {
        // Overflow (e.g. a telemetry-request or otherwise-oversized frame we
        // have no interest in) - stop storing until the next marker resets us.
        return;
    }

    uint8_t byteValue = val;
    if (s->frameData.escapePending) {
        byteValue ^= FPORT_INPUT_ESCAPE_MASK;
        s->frameData.escapePending = false;
    }

    s->frameData.bytes[s->frameData.position++] = byteValue;
}

// Called from the RX task (rx/rx.c, via rx_input_backup.c's poll loop), not an
// ISR - safe to do the heavier decode/convert work here.
static uint8_t fportInputUpdate(void *state, float *channels, uint8_t channelCount)
{
    fportInputState_t *s = state;
    uint8_t frame[FPORT_INPUT_FRAME_SIZE];
    uint8_t length = 0;
    bool haveFrame = false;

    ATOMIC_BLOCK(NVIC_PRIO_MAX) {
        if (s->pendingFrameReady) {
            length = s->pendingLength;
            memcpy(frame, s->pendingFrame, length);
            s->pendingFrameReady = false;
            haveFrame = true;
        }
    }

    if (!haveFrame) {
        return 0;
    }

    // Generic size self-consistency (matches rx/fport.c's frameLength !=
    // bufferLength - 2 check), then the control-frame-specific size (matches
    // its FPORT_FRAME_PAYLOAD_LENGTH_CONTROL check) and type byte - anything
    // else (telemetry-request frames, garbage) is silently ignored, same as a
    // rejected/dropped frame elsewhere in this framework.
    if (length != FPORT_INPUT_FRAME_SIZE
        || frame[0] != length - 2
        || frame[0] != FPORT_INPUT_PAYLOAD_LENGTH_CONTROL
        || frame[1] != FPORT_INPUT_FRAME_TYPE_CONTROL) {
        return 0;
    }

    // Checksum covers the full de-stuffed span, length byte included - unlike
    // FBUS's checksum, which skips its own length byte (rx_input_backup_fbus.c).
    // Confirmed as a genuine per-protocol difference, not an inconsistency:
    // rx/fport.c's own frskyCheckSumIsGood(&data[0], bufferLength) does the
    // same full-span check.
    if (!frskyCheckSumIsGood(frame, length)) {
        fportInputResetParser(s);
        return 0;
    }

    // Copied into a genuine sbusChannels_t object rather than pointer-cast
    // straight out of the uint8_t frame buffer - see rx_input_backup_fbus.c's
    // identical fix for why (C11 effective-type rules; matches the SBUS
    // provider's own union-based approach).
    sbusChannels_t wireChannels;
    memcpy(&wireChannels, &frame[2], sizeof(wireChannels));
    const uint8_t frameStatus = sbusChannelsDecode(&s->rxRuntimeState, &wireChannels);
    if (frameStatus & (RX_FRAME_DROPPED | RX_FRAME_FAILSAFE)) {
        fportInputResetParser(s);
        return 0;
    }

    for (uint8_t i = 0; i < channelCount; i++) {
        channels[i] = (5.0f * (float)s->channelData[i] / 8.0f) + 880.0f;
    }

    return channelCount;
}

bool rxInputBackupFportInit(rxInputBackupOps_t *ops, const rxInputBackupPortConfig_t *config, uint8_t index)
{
    fportInputState_t *s = &fportInputStates[index];
    s->rxRuntimeState.channelData = s->channelData;
    fportInputResetParser(s);

    // Matches rx/fport.c's own direction/variant exactly - identical to FBUS's
    // (rx_input_backup_fbus.c): natively non-inverted, SERIAL_BIDIR|SERIAL_BIDIR_PP
    // for half-duplex.
    ops->baudRate = FPORT_INPUT_BAUDRATE;
    ops->portOptions = FPORT_INPUT_PORT_OPTIONS
        | (config->inverted ? SERIAL_INVERTED : SERIAL_NOT_INVERTED)
        | (config->halfDuplex ? (SERIAL_BIDIR | SERIAL_BIDIR_PP) : SERIAL_UNIDIR);
    ops->isrFn = fportInputDataReceive;
    ops->state = s;
    ops->channelCount = SBUS_MAX_CHANNEL;   // 16 + 2 digital
    ops->update = fportInputUpdate;

    return true;
}

#endif // USE_RX_INPUT_BACKUP_FPORT

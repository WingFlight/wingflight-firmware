/*
 * This file is part of Rotorflight.
 *
 * Rotorflight is free software. You can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Rotorflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software. If not, see <https://www.gnu.org/licenses/>.
 */

#include <string.h>

#include "platform.h"

#ifdef USE_FBUS_MASTER

#include "common/maths.h"
#include "common/utils.h"

#include "drivers/fbus_ota.h"

#include "rx/frsky_crc.h"

#define FBUS_OTA_TYPE_START         0xF0
#define FBUS_OTA_TYPE_DATA          0xF1
#define FBUS_OTA_TYPE_END           0xF2
#define FBUS_OTA_START_END_LENGTH   24
#define FBUS_OTA_DOWNLINK_SIZE      10

// Start frame Data1/Data2: reply after 50 us (Data1 = 0), within 5 ms (Data2 x 100 us)
#define FBUS_OTA_REPLY_DELAY        0
#define FBUS_OTA_REPLY_WINDOW       50

// A frame is sent again when no answer came within this time. Longer than the
// reply window, so a late answer cannot collide with the repeat. A device
// erasing flash skips its answer and gets the frame again.
#define FBUS_OTA_REPEAT_US          12000
// The device has this long to enter its update mode (it may reset into a bootloader)
#define FBUS_OTA_START_TIMEOUT_US   5000000
// Longest time without an acknowledged data frame
#define FBUS_OTA_ACK_TIMEOUT_US     3000000
// The device verifies the image before acknowledging the end frame
#define FBUS_OTA_END_TIMEOUT_US     3000000
// Longest wait for image data from the Configurator
#define FBUS_OTA_STALL_TIMEOUT_US   5000000

static struct {
    fbusOtaState_e state;
    fbusOtaError_e error;
    uint8_t phyId;
    uint32_t size;
    uint32_t acked;
    uint32_t received;
    uint16_t retries;

    bool clockStarted;
    bool sent;              // a frame of the current step went out
    bool awaiting;          // waiting for the answer to it
    volatile bool echoReceived;
    volatile bool dataReceived;
    timeUs_t lastSendUs;
    timeUs_t progressUs;    // last answer, image data or step change

    uint8_t downlink[FBUS_OTA_DOWNLINK_SIZE];
    uint8_t buffer[FBUS_OTA_BUFFER_SIZE];
} ota;

static uint8_t phyIdWithCheckBits(uint8_t phyId)
{
    // Same encoding as the S.Port data ID and the master's telemetry polls
    const uint8_t b0 = phyId & 1, b1 = (phyId >> 1) & 1, b2 = (phyId >> 2) & 1;
    const uint8_t b3 = (phyId >> 3) & 1, b4 = (phyId >> 4) & 1;
    return phyId | (b0 ^ b1 ^ b2) << 5 | (b2 ^ b3 ^ b4) << 6 | (b0 ^ b2 ^ b4) << 7;
}

static void fail(fbusOtaError_e error)
{
    ota.state = FBUS_OTA_STATE_FAILED;
    ota.error = error;
    ota.awaiting = false;
}

static void nextStep(fbusOtaState_e state, timeUs_t currentTimeUs)
{
    ota.state = state;
    ota.sent = false;
    ota.awaiting = false;
    ota.progressUs = currentTimeUs;
}

bool fbusOtaStart(uint8_t phyId, uint32_t size)
{
    if (fbusOtaIsActive() || phyId > FBUS_OTA_MAX_PHY_ID || size == 0 || size > FBUS_OTA_MAX_IMAGE_SIZE) {
        return false;
    }

    memset(&ota, 0, sizeof(ota));
    ota.state = FBUS_OTA_STATE_STARTING;
    ota.phyId = phyId;
    ota.size = size;
    return true;
}

bool fbusOtaWrite(uint32_t offset, const uint8_t *data, uint8_t length)
{
    if ((ota.state != FBUS_OTA_STATE_STARTING && ota.state != FBUS_OTA_STATE_TRANSFER) ||
        length == 0 || length > FBUS_OTA_MAX_WRITE) {
        return false;
    }
    if (offset < ota.received && offset + length <= ota.received) {
        return true; // repeat of data already received, e.g. after a lost MSP reply
    }
    if (offset != ota.received || offset + length > ota.size ||
        ota.received + length - ota.acked > FBUS_OTA_BUFFER_SIZE) {
        return false;
    }

    for (unsigned i = 0; i < length; i++) {
        ota.buffer[(offset + i) % FBUS_OTA_BUFFER_SIZE] = data[i];
    }
    ota.received += length;
    ota.dataReceived = true;
    return true;
}

void fbusOtaAbort(void)
{
    if (fbusOtaIsActive()) {
        fail(FBUS_OTA_ERROR_ABORTED);
    }
}

bool fbusOtaIsActive(void)
{
    return ota.state == FBUS_OTA_STATE_STARTING || ota.state == FBUS_OTA_STATE_TRANSFER ||
           ota.state == FBUS_OTA_STATE_ENDING;
}

void fbusOtaGetStatus(fbusOtaStatus_t *status)
{
    status->state = ota.state;
    status->error = ota.error;
    status->phyId = ota.phyId;
    status->size = ota.size;
    status->acked = ota.acked;
    status->received = ota.received;
    const uint32_t room = FBUS_OTA_BUFFER_SIZE - (ota.received - ota.acked);
    status->bufferFree = fbusOtaIsActive() ? MIN(room, ota.size - ota.received) : 0;
    status->retries = ota.retries;
}

static size_t buildFrame(uint8_t *frame, uint8_t type, const uint8_t *data, uint8_t length,
                         uint32_t value, timeUs_t currentTimeUs)
{
    frame[0] = length;
    frame[1] = type;
    memcpy(&frame[2], data, length);
    frame[2 + length] = frskyCheckSum(&frame[1], length + 1);

    uint8_t *downlink = &frame[3 + length];
    downlink[0] = FBUS_OTA_DOWNLINK_SIZE - 2;
    downlink[1] = phyIdWithCheckBits(ota.phyId);
    downlink[2] = type;
    downlink[3] = 0;
    downlink[4] = 0;
    downlink[5] = value & 0xFF;
    downlink[6] = (value >> 8) & 0xFF;
    downlink[7] = (value >> 16) & 0xFF;
    downlink[8] = (value >> 24) & 0xFF;
    downlink[9] = frskyCheckSum(&downlink[1], FBUS_OTA_DOWNLINK_SIZE - 2);
    memcpy(ota.downlink, downlink, FBUS_OTA_DOWNLINK_SIZE);

    if (ota.sent) {
        ota.retries++;
    }
    ota.sent = true;
    ota.awaiting = true;
    ota.echoReceived = false;
    ota.lastSendUs = currentTimeUs;
    return 3 + length + FBUS_OTA_DOWNLINK_SIZE;
}

static void processEcho(timeUs_t currentTimeUs)
{
    ota.echoReceived = false;
    if (!ota.awaiting) {
        return;
    }
    ota.awaiting = false;

    switch (ota.state) {
    case FBUS_OTA_STATE_STARTING:
        nextStep(FBUS_OTA_STATE_TRANSFER, currentTimeUs);
        break;
    case FBUS_OTA_STATE_TRANSFER:
        ota.acked += MIN((uint32_t)FBUS_OTA_CHUNK_SIZE, ota.size - ota.acked);
        ota.sent = false;
        ota.progressUs = currentTimeUs;
        if (ota.acked >= ota.size) {
            nextStep(FBUS_OTA_STATE_ENDING, currentTimeUs);
        }
        break;
    case FBUS_OTA_STATE_ENDING:
        nextStep(FBUS_OTA_STATE_DONE, currentTimeUs);
        break;
    default:
        break;
    }
}

static bool waitingForAnswer(timeUs_t currentTimeUs)
{
    return ota.awaiting && cmpTimeUs(currentTimeUs, ota.lastSendUs) < FBUS_OTA_REPEAT_US;
}

size_t fbusOtaPrepareFrame(uint8_t *frame, timeUs_t currentTimeUs)
{
    if (!fbusOtaIsActive()) {
        return 0;
    }
    if (!ota.clockStarted) {
        ota.clockStarted = true;
        ota.progressUs = currentTimeUs;
    }
    if (ota.echoReceived) {
        processEcho(currentTimeUs);
    }
    if (ota.dataReceived) {
        ota.dataReceived = false;
        if (!ota.awaiting) {
            ota.progressUs = currentTimeUs;
        }
    }

    const timeDelta_t sinceProgress = cmpTimeUs(currentTimeUs, ota.progressUs);
    static const uint8_t noData[FBUS_OTA_START_END_LENGTH] = { 0 };

    switch (ota.state) {
    case FBUS_OTA_STATE_STARTING: {
        if (sinceProgress > FBUS_OTA_START_TIMEOUT_US) {
            fail(FBUS_OTA_ERROR_NO_RESPONSE);
            return 0;
        }
        if (waitingForAnswer(currentTimeUs)) {
            return 0;
        }
        uint8_t data[FBUS_OTA_START_END_LENGTH] = { FBUS_OTA_REPLY_DELAY, FBUS_OTA_REPLY_WINDOW };
        return buildFrame(frame, FBUS_OTA_TYPE_START, data, sizeof(data), 0, currentTimeUs);
    }

    case FBUS_OTA_STATE_TRANSFER: {
        if (ota.awaiting) {
            if (sinceProgress > FBUS_OTA_ACK_TIMEOUT_US) {
                fail(FBUS_OTA_ERROR_TIMEOUT);
                return 0;
            }
            if (waitingForAnswer(currentTimeUs)) {
                return 0;
            }
        } else {
            const uint32_t needed = MIN((uint32_t)FBUS_OTA_CHUNK_SIZE, ota.size - ota.acked);
            if (ota.received - ota.acked < needed) {
                if (sinceProgress > FBUS_OTA_STALL_TIMEOUT_US) {
                    fail(FBUS_OTA_ERROR_STALLED);
                }
                return 0;
            }
        }
        uint8_t chunk[FBUS_OTA_CHUNK_SIZE];
        for (unsigned i = 0; i < FBUS_OTA_CHUNK_SIZE; i++) {
            const uint32_t offset = ota.acked + i;
            chunk[i] = offset < ota.size ? ota.buffer[offset % FBUS_OTA_BUFFER_SIZE] : 0xFF;
        }
        return buildFrame(frame, FBUS_OTA_TYPE_DATA, chunk, sizeof(chunk), ota.acked, currentTimeUs);
    }

    case FBUS_OTA_STATE_ENDING:
        if (sinceProgress > FBUS_OTA_END_TIMEOUT_US) {
            fail(FBUS_OTA_ERROR_REJECTED);
            return 0;
        }
        if (waitingForAnswer(currentTimeUs)) {
            return 0;
        }
        return buildFrame(frame, FBUS_OTA_TYPE_END, noData, sizeof(noData), 0, currentTimeUs);

    default:
        return 0;
    }
}

void fbusOtaProcessReply(const uint8_t *frame)
{
    // The answer echoes our downlink frame. Compare the physical ID without
    // its check bits, in case a device echoes it plain.
    if (!ota.awaiting || frame[0] != FBUS_OTA_DOWNLINK_SIZE - 2 ||
        (frame[1] & 0x1F) != ota.phyId ||
        memcmp(&frame[2], &ota.downlink[2], FBUS_OTA_DOWNLINK_SIZE - 3) != 0 ||
        frskyCheckSum((uint8_t *)&frame[1], FBUS_OTA_DOWNLINK_SIZE - 2) != frame[9]) {
        return;
    }
    ota.echoReceived = true;
}

#endif

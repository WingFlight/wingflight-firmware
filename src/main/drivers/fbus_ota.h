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

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "platform.h"

#include "common/time.h"

// Firmware update of an FBUS device, with the FC as FBUS master (FrSky FBUS
// protocol V1.0.0, "Procedure of OTA"). Each step is a control frame followed
// by a downlink frame whose PRIM equals the control type; the device answers
// by echoing the downlink frame:
//
//   start: 18 F0 <Data1 reply delay> <Data2 reply window> 0..  + 08 PHY F0 00 00 00000000 CRC
//   data:  20 F1 <32 bytes>                                   + 08 PHY F1 00 00 <offset> CRC
//   end:   18 F2 0..                                          + 08 PHY F2 00 00 00000000 CRC
//
// While an update runs, the master sends no channel frames: the spec has the
// device abort the update when it sees one. Bus servos get no signal meanwhile.
//
// The image comes from the Configurator over MSP (MSP2_WING_FBUS_OTA_*) into a
// ring buffer and is sent verbatim, 32 bytes per data frame; a short last
// chunk is padded with 0xFF.

#define FBUS_OTA_CHUNK_SIZE      32
#define FBUS_OTA_BUFFER_SIZE     1024
#define FBUS_OTA_MAX_WRITE       128
#define FBUS_OTA_MAX_IMAGE_SIZE  (1024 * 1024)
#define FBUS_OTA_MAX_PHY_ID      0x1E
// Longest frame produced: data control frame plus downlink frame
#define FBUS_OTA_MAX_FRAME_SIZE  (FBUS_OTA_CHUNK_SIZE + 3 + 10)

typedef enum {
    FBUS_OTA_STATE_IDLE = 0,
    FBUS_OTA_STATE_STARTING,    // sending the start frame until the device answers
    FBUS_OTA_STATE_TRANSFER,    // sending data frames
    FBUS_OTA_STATE_ENDING,      // sending the end frame until the device answers
    FBUS_OTA_STATE_DONE,        // device acknowledged the end frame
    FBUS_OTA_STATE_FAILED,
} fbusOtaState_e;

typedef enum {
    FBUS_OTA_ERROR_NONE = 0,
    FBUS_OTA_ERROR_NO_RESPONSE, // device did not acknowledge the start frame
    FBUS_OTA_ERROR_TIMEOUT,     // device stopped acknowledging data frames
    FBUS_OTA_ERROR_REJECTED,    // device did not acknowledge the end frame (image refused)
    FBUS_OTA_ERROR_STALLED,     // the Configurator stopped sending image data
    FBUS_OTA_ERROR_ABORTED,     // aborted over MSP
} fbusOtaError_e;

typedef struct {
    fbusOtaState_e state;
    fbusOtaError_e error;
    uint8_t phyId;
    uint32_t size;          // image size
    uint32_t acked;         // bytes acknowledged by the device
    uint32_t received;      // bytes received from the Configurator (next expected offset)
    uint16_t bufferFree;    // room for more image data
    uint16_t retries;       // frames sent again because no answer came in time
} fbusOtaStatus_t;

// Starts an update of the device with this physical ID. Fails if one is
// already running or the arguments are out of range.
bool fbusOtaStart(uint8_t phyId, uint32_t size);

// Stores image data. offset must be the next expected one; a repeat of data
// already received is accepted and ignored. Fails if the data does not fit.
bool fbusOtaWrite(uint32_t offset, const uint8_t *data, uint8_t length);

void fbusOtaAbort(void);

// True while frames are being exchanged (starting, transfer, ending)
#ifdef USE_FBUS_MASTER
bool fbusOtaIsActive(void);
#else
static inline bool fbusOtaIsActive(void) { return false; }
#endif

void fbusOtaGetStatus(fbusOtaStatus_t *status);

// Builds the next frame to send into frame (FBUS_OTA_MAX_FRAME_SIZE bytes).
// Returns its length, or 0 if nothing is to be sent now.
size_t fbusOtaPrepareFrame(uint8_t *frame, timeUs_t currentTimeUs);

// Processes a 10-byte downlink-format frame received from the bus.
void fbusOtaProcessReply(const uint8_t *frame);

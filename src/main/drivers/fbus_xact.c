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

#include "platform.h"

#include "fbus_master.h"
#include "fbus_xact.h"

#include <stdbool.h>
#include <string.h>

#include "common/time.h"
#include "common/utils.h"
#include "rx/frsky_crc.h"
#include "rx/fbus.h"

// XACT read/write frames ride on the same downlink frame used for telemetry polling.
#define FBUS_FRAME_ID_DATA 0x10

// XACT module state
static bool xactInitialized = false;

// XACT servo programming queue
#define XACT_QUEUE_SIZE 8
static xactServoParam_t xactQueue[XACT_QUEUE_SIZE];
static uint8_t xactQueueHead = 0;
static uint8_t xactQueueTail = 0;
static uint8_t xactQueueCount = 0;

// XACT servo tracking (separate from general sensor discovery)
static xactServo_t xactServos[XACT_MAX_SERVOS];
static uint8_t xactServoCount = 0;

// XACT servo parameter storage (indexed by physical ID)
static xactServoParams_t xactServoParams[XACT_MAX_SERVOS];

// Parameter reading state machine
typedef enum {
    XACT_READ_STATE_IDLE = 0,
    XACT_READ_STATE_READING,
    XACT_READ_STATE_WAIT_POLL,  // Wait to send 0x10 poll frame after 0x30 read
    XACT_READ_STATE_COMPLETE
} xactReadState_e;

static xactReadState_e xactReadState = XACT_READ_STATE_IDLE;
static uint8_t xactReadServoIndex = 0;
static uint8_t xactReadParamIndex = 0;
static uint8_t xactLastReadPhyID = 0;  // Track last read physical ID for polling
static uint8_t xactLastReadFieldId = 0;  // Track last read field ID for response matching

// Field IDs to read in order
static const uint8_t xactReadFieldIds[] = {
    XACT_FIELD_PHYSICAL_ID,
    XACT_FIELD_APP_ID_BASE,
    XACT_FIELD_DATA_RATE,
    XACT_FIELD_RANGE,
    XACT_FIELD_DIRECTION,
    XACT_FIELD_PULSE_TYPE,
    XACT_FIELD_CHANNEL,
    XACT_FIELD_CENTER,
    XACT_FIELD_P1,
    XACT_FIELD_P2,
    XACT_FIELD_D1,
    XACT_FIELD_TB,
    XACT_FIELD_POT_GAP
};
#define XACT_READ_PARAM_COUNT (sizeof(xactReadFieldIds) / sizeof(xactReadFieldIds[0]))

// Helper function to fill physical ID check bits
static void xactPhyIDFillCheckBits(uint8_t *phyIDByte)
{
    *phyIDByte |= (fbusGetBit(*phyIDByte, 0) ^ fbusGetBit(*phyIDByte, 1) ^ fbusGetBit(*phyIDByte, 2)) << 5;
    *phyIDByte |= (fbusGetBit(*phyIDByte, 2) ^ fbusGetBit(*phyIDByte, 3) ^ fbusGetBit(*phyIDByte, 4)) << 6;
    *phyIDByte |= (fbusGetBit(*phyIDByte, 0) ^ fbusGetBit(*phyIDByte, 2) ^ fbusGetBit(*phyIDByte, 4)) << 7;
}

// Initialize XACT servo module
void fbusXactInit(void)
{
    xactInitialized = true;
    xactQueueHead = 0;
    xactQueueTail = 0;
    xactQueueCount = 0;
    memset(xactQueue, 0, sizeof(xactQueue));
    memset(xactServos, 0, sizeof(xactServos));
    memset(xactServoParams, 0, sizeof(xactServoParams));
    xactServoCount = 0;
    xactReadState = XACT_READ_STATE_IDLE;
}

// Clear all discovered XACT servos
void fbusXactClearDiscoveredServos(void)
{
    memset(xactServos, 0, sizeof(xactServos));
    xactServoCount = 0;
    xactReadState = XACT_READ_STATE_IDLE;

    // Restart FBUS master sensor discovery so it re-scans physical IDs
    fbusMasterStartDiscovery();
}

// Track an XACT servo (called when servo data is received)
void fbusXactTrackServo(uint8_t phyID, uint16_t appId, timeUs_t currentTimeUs)
{
    // Check if this servo is already tracked
    for (uint8_t i = 0; i < xactServoCount; i++) {
        if (xactServos[i].phyID == phyID) {
            // Update existing servo
            xactServos[i].appId = appId;
            xactServos[i].lastSeenUs = currentTimeUs;
            return;
        }
    }

    // Add new servo if space available
    if (xactServoCount < XACT_MAX_SERVOS) {
        xactServos[xactServoCount].phyID = phyID;
        xactServos[xactServoCount].appId = appId;
        xactServos[xactServoCount].lastSeenUs = currentTimeUs;
        xactServoCount++;

        // Start reading parameters for the first servo
        if (xactServoCount == 1 && xactReadState == XACT_READ_STATE_IDLE) {
            xactReadState = XACT_READ_STATE_READING;
            xactReadServoIndex = 0;
            xactReadParamIndex = 0;
        }
    }
}

// Start a new sensor discovery phase
void fbusXactStartSensorDiscovery(void)
{
    // Clear existing discovered XACT servos and restart FBUS master scanning
    fbusXactClearDiscoveredServos();
}

// Write a parameter to an XACT servo using physical ID
bool fbusXactWriteUplinkFramePhyID(uint8_t phyID, uint8_t fieldId, uint16_t appId, uint16_t data)
{
    if (!xactInitialized) {
        return false;
    }

    // Check if queue is full
    if (xactQueueCount >= XACT_QUEUE_SIZE) {
        return false;
    }

    // Add to queue
    xactQueue[xactQueueTail].phyID = phyID;
    xactQueue[xactQueueTail].fieldId = fieldId;
    xactQueue[xactQueueTail].appId = appId;
    xactQueue[xactQueueTail].data = data;

    xactQueueTail = (xactQueueTail + 1) % XACT_QUEUE_SIZE;
    xactQueueCount++;

    return true;
}

// Get the number of discovered XACT servos
uint8_t fbusXactGetDiscoveredServoCount(void)
{
    return xactServoCount;
}

// Get a discovered XACT servo physical ID by index
uint8_t fbusXactGetDiscoveredServoPhyID(uint8_t index)
{
    if (index >= xactServoCount) {
        return 0;
    }
    return xactServos[index].phyID;
}

// Check if XACT module is initialized
bool fbusXactIsInitialized(void)
{
    return xactInitialized;
}

// Process XACT servo programming queue (called from FBUS master update)
// Frame format: Length PhysID PRIM FIELDID APPID2 APPID1 DATA1 CRC
bool fbusXactProcessQueue(fbusMasterDownlink_t *downlink)
{
    if (!xactInitialized) {
        return false;
    }

    // Get servo pointer for both read and poll operations
    xactServo_t *servo = (xactServoCount > 0) ? &xactServos[xactReadServoIndex] : NULL;

    // Priority 1: Check if we need to send 0x10 poll frame after 0x30 read
    if (xactReadState == XACT_READ_STATE_WAIT_POLL && servo != NULL) {
        // Send 0x10 DATA frame to poll for the response
        downlink->length = 0x08;
        downlink->phyID = xactLastReadPhyID;
        downlink->prim = FBUS_FRAME_ID_DATA;  // 0x10 to poll for response
        downlink->appId = servo->appId;       // Include servo App ID
        downlink->data[0] = 0;
        downlink->data[1] = 0;
        downlink->data[2] = 0;
        downlink->data[3] = 0;

        xactPhyIDFillCheckBits(&downlink->phyID);
        uint8_t crc = frskyCheckSum((uint8_t *)&downlink->phyID, 0x08);
        downlink->crc = crc;

        // Stay in WAIT_POLL state - will be advanced by fbusXactNotifyResponse()

        return true;
    }

    // Priority 2: Check if we need to read parameters
    if (xactReadState == XACT_READ_STATE_READING && servo != NULL) {
        if (xactReadParamIndex < XACT_READ_PARAM_COUNT) {
            // Send READ command for next parameter
            uint8_t fieldId = xactReadFieldIds[xactReadParamIndex];

            downlink->length = 0x08;
            downlink->phyID = servo->phyID;
            downlink->prim = XACT_FRAME_TYPE_READ;  // 0x30 for read
            downlink->appId = servo->appId;         // Use appId field for App ID
            downlink->data[0] = fieldId;            // Field ID in data[0]
            downlink->data[1] = 0;                  // Unused
            downlink->data[2] = 0;                  // Unused for read
            downlink->data[3] = 0;                  // Unused

            xactPhyIDFillCheckBits(&downlink->phyID);
            uint8_t crc = frskyCheckSum((uint8_t *)&downlink->phyID, 0x08);
            downlink->crc = crc;

            // Save the physical ID and field ID for the next poll frame
            xactLastReadPhyID = servo->phyID;
            xactLastReadFieldId = fieldId;

            // Transition to wait for poll state (don't increment param index yet)
            xactReadState = XACT_READ_STATE_WAIT_POLL;

            return true;
        }
    }

    // Priority 3: Process write queue
    if (xactQueueCount > 0) {
        // Get next item from queue
        xactServoParam_t *param = &xactQueue[xactQueueHead];

        // Prepare downlink frame for XACT servo programming
        downlink->length = 0x08;  // 8 bytes payload
        downlink->phyID = param->phyID;
        downlink->prim = XACT_FRAME_TYPE_WRITE;  // 0x31 for write

        // Pack the frame data
        downlink->appId = param->appId;          // Use appId field for App ID
        downlink->data[0] = param->fieldId;      // Field ID in data[0]
        downlink->data[1] = param->data & 0xFF;  // DATA1 low byte in data[1]
        downlink->data[2] = (param->data >> 8) & 0xFF; // DATA1 high byte in data[2]
        downlink->data[3] = 0;                   // Unused

        // Fill physical ID check bits
        xactPhyIDFillCheckBits(&downlink->phyID);

        // Calculate CRC
        uint8_t crc = frskyCheckSum((uint8_t *)&downlink->phyID, 0x08);
        downlink->crc = crc;

        // Remove from queue
        xactQueueHead = (xactQueueHead + 1) % XACT_QUEUE_SIZE;
        xactQueueCount--;

        return true;
    }

    return false;
}

// Get queue count (for monitoring)
uint8_t fbusXactGetQueueCount(void)
{
    return xactQueueCount;
}

// Get servo parameters for a specific physical ID
bool fbusXactGetServoParams(uint8_t phyID, xactServoParams_t *params)
{
    if (!xactInitialized || params == NULL) {
        return false;
    }

    // Find the servo by physical ID
    for (uint8_t i = 0; i < xactServoCount; i++) {
        if (xactServos[i].phyID == phyID) {
            // Copy the stored parameters
            memcpy(params, &xactServoParams[i], sizeof(xactServoParams_t));
            return true;
        }
    }

    return false;
}

// Set a specific servo parameter field (stores value from read response)
bool fbusXactSetServoParam(uint8_t phyID, uint8_t fieldId, uint16_t appId, uint16_t data)
{
    UNUSED(appId);
    if (!xactInitialized) {
        return false;
    }

    // Find the servo by physical ID
    int8_t servoIndex = -1;
    for (uint8_t i = 0; i < xactServoCount; i++) {
        if (xactServos[i].phyID == phyID) {
            servoIndex = i;
            break;
        }
    }

    if (servoIndex < 0) {
        return false;
    }

    // Update the parameter in storage based on field ID
    xactServoParams_t *params = &xactServoParams[servoIndex];
    switch (fieldId) {
        case XACT_FIELD_PHYSICAL_ID:
            params->physicalId = data;
            break;
        case XACT_FIELD_APP_ID_BASE:
            params->appIdOffset = (uint8_t)data;
            break;
        case XACT_FIELD_DATA_RATE:
            params->dataRate = (uint16_t)data;
            break;
        case XACT_FIELD_RANGE:
            params->range = (uint8_t)data;
            break;
        case XACT_FIELD_DIRECTION:
            params->direction = (uint8_t)data;
            break;
        case XACT_FIELD_PULSE_TYPE:
            params->pulseType = (uint8_t)data;
            break;
        case XACT_FIELD_CHANNEL:
            params->channel = (uint8_t)data;
            break;
        case XACT_FIELD_CENTER:
            params->center = (uint8_t)data;
            break;
        case XACT_FIELD_P1:
            params->p1 = (uint8_t)data;
            break;
        case XACT_FIELD_P2:
            params->p2 = (uint8_t)data;
            break;
        case XACT_FIELD_D1:
            params->d1 = (uint8_t)data;
            break;
        case XACT_FIELD_TB:
            params->tb = (uint8_t)data;
            break;
        case XACT_FIELD_POT_GAP:
            params->potGap = (uint8_t)data;
            break;
        default:
            return false;
    }

    // Only store the value - do NOT queue write command
    // Writing is triggered explicitly via MSP command calling fbusXactWriteUplinkFramePhyID
    return true;
}

// Compare and write all parameters if different from cache
bool fbusXactCompareAndWriteParams(uint8_t phyID, uint16_t appId, const xactServoParams_t *newParams)
{
    if (!xactInitialized || newParams == NULL) {
        return false;
    }

    // Find the servo by physical ID
    int8_t servoIndex = -1;
    for (uint8_t i = 0; i < xactServoCount; i++) {
        if (xactServos[i].phyID == phyID) {
            servoIndex = i;
            break;
        }
    }

    if (servoIndex < 0) {
        return false;
    }

    // Get cached parameters
    xactServoParams_t *cachedParams = &xactServoParams[servoIndex];
    bool hasChanges = false;

    // Track current phyID and appId - these may be updated if PHYSICAL_ID or APP_ID_BASE change
    uint8_t currentPhyID = phyID;
    uint16_t currentAppId = appId;

    // Compare each parameter and queue writes for differences
    if (cachedParams->physicalId != newParams->physicalId) {
        fbusXactWriteUplinkFramePhyID(currentPhyID, XACT_FIELD_PHYSICAL_ID, currentAppId, newParams->physicalId);
        cachedParams->physicalId = newParams->physicalId;
        // Update currentPhyID for subsequent writes
        currentPhyID = newParams->physicalId;
        hasChanges = true;
    }

    if (cachedParams->appIdOffset != newParams->appIdOffset) {
        fbusXactWriteUplinkFramePhyID(currentPhyID, XACT_FIELD_APP_ID_BASE, currentAppId, newParams->appIdOffset);
        cachedParams->appIdOffset = newParams->appIdOffset;
        // Update currentAppId for subsequent writes (base + offset)
        currentAppId = FBUS_SERVO_DATA_BASE + newParams->appIdOffset;
        hasChanges = true;
    }

    if (cachedParams->dataRate != newParams->dataRate) {
        // Data rate is 16-bit, write low byte then high byte
        fbusXactWriteUplinkFramePhyID(currentPhyID, XACT_FIELD_DATA_RATE, currentAppId, newParams->dataRate);
        cachedParams->dataRate = newParams->dataRate;
        hasChanges = true;
    }

    if (cachedParams->range != newParams->range) {
        fbusXactWriteUplinkFramePhyID(currentPhyID, XACT_FIELD_RANGE, currentAppId, newParams->range);
        cachedParams->range = newParams->range;
        hasChanges = true;
    }

    if (cachedParams->direction != newParams->direction) {
        fbusXactWriteUplinkFramePhyID(currentPhyID, XACT_FIELD_DIRECTION, currentAppId, newParams->direction);
        cachedParams->direction = newParams->direction;
        hasChanges = true;
    }

    if (cachedParams->pulseType != newParams->pulseType) {
        fbusXactWriteUplinkFramePhyID(currentPhyID, XACT_FIELD_PULSE_TYPE, currentAppId, newParams->pulseType);
        cachedParams->pulseType = newParams->pulseType;
        hasChanges = true;
    }

    if (cachedParams->channel != newParams->channel) {
        fbusXactWriteUplinkFramePhyID(currentPhyID, XACT_FIELD_CHANNEL, currentAppId, newParams->channel);
        cachedParams->channel = newParams->channel;
        hasChanges = true;
    }

    if (cachedParams->center != newParams->center) {
        fbusXactWriteUplinkFramePhyID(currentPhyID, XACT_FIELD_CENTER, currentAppId, newParams->center);
        cachedParams->center = newParams->center;
        hasChanges = true;
    }

    if (cachedParams->p1 != newParams->p1) {
        fbusXactWriteUplinkFramePhyID(currentPhyID, XACT_FIELD_P1, currentAppId, newParams->p1);
        cachedParams->p1 = newParams->p1;
        hasChanges = true;
    }

    if (cachedParams->p2 != newParams->p2) {
        fbusXactWriteUplinkFramePhyID(currentPhyID, XACT_FIELD_P2, currentAppId, newParams->p2);
        cachedParams->p2 = newParams->p2;
        hasChanges = true;
    }

    if (cachedParams->d1 != newParams->d1) {
        fbusXactWriteUplinkFramePhyID(currentPhyID, XACT_FIELD_D1, currentAppId, newParams->d1);
        cachedParams->d1 = newParams->d1;
        hasChanges = true;
    }

    if (cachedParams->tb != newParams->tb) {
        fbusXactWriteUplinkFramePhyID(currentPhyID, XACT_FIELD_TB, currentAppId, newParams->tb);
        cachedParams->tb = newParams->tb;
        hasChanges = true;
    }

    if (cachedParams->potGap != newParams->potGap) {
        fbusXactWriteUplinkFramePhyID(currentPhyID, XACT_FIELD_POT_GAP, currentAppId, newParams->potGap);
        cachedParams->potGap = newParams->potGap;
        hasChanges = true;
    }

    // If any parameters were changed, queue a write flash command to save them
    if (hasChanges) {
        fbusXactWriteUplinkFramePhyID(currentPhyID, XACT_FIELD_WRITE_FLASH, currentAppId, 0);
    }

    return hasChanges;
}

// Check if XACT is currently busy (reading or writing)
bool fbusXactIsBusy(void)
{
    if (!xactInitialized) {
        return false;
    }

    // XACT is busy if:
    // 1. Currently reading parameters
    // 2. Waiting for a read response
    // 3. Has items in the write queue
    return (xactReadState == XACT_READ_STATE_READING ||
            xactReadState == XACT_READ_STATE_WAIT_POLL ||
            xactQueueCount > 0);
}

// Notify that a response was received for the current read operation
void fbusXactNotifyResponse(uint8_t phyID, uint8_t fieldId)
{
    if (!xactInitialized) {
        return;
    }

    // Only process if we're waiting for a response
    if (xactReadState != XACT_READ_STATE_WAIT_POLL) {
        return;
    }

    // Verify this response matches what we're waiting for
    if (phyID != xactLastReadPhyID || fieldId != xactLastReadFieldId) {
        return;
    }

    // Move to next parameter
    xactReadParamIndex++;

    // Check if we've read all parameters
    if (xactReadParamIndex >= XACT_READ_PARAM_COUNT) {
        xactReadState = XACT_READ_STATE_COMPLETE;
    } else {
        // Continue reading next parameter
        xactReadState = XACT_READ_STATE_READING;
    }
}

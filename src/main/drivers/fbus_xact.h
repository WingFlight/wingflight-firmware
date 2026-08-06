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

#include "platform.h"
#include "common/time.h"
#include "drivers/fbus_master.h"

// XACT servo programming frame types (PRIM field)
#define XACT_FRAME_TYPE_READ     0x30
#define XACT_FRAME_TYPE_WRITE    0x31
#define XACT_FRAME_TYPE_RESPONSE 0x32

// XACT servo field IDs (FIELDID)
// Frame format: Length PhysID PRIM APPID(16-bit) FIELDID DATA1 ... CRC
#define XACT_FIELD_PHYSICAL_ID    0x00  // Physical ID
#define XACT_FIELD_APP_ID_BASE    0x01  // App ID base addr + offset
#define XACT_FIELD_DATA_RATE      0x02  // Data rate (ms)
#define XACT_FIELD_RANGE          0x04  // Range: 0=120°, 1=90°, 2=180°
#define XACT_FIELD_DIRECTION      0x05  // Direction: 0=clockwise, 1=counter-clockwise
#define XACT_FIELD_PULSE_TYPE     0x06  // Pulse type: 0=1500us, 1=760us
#define XACT_FIELD_CHANNEL        0x07  // Channel: 0=CH1
#define XACT_FIELD_CENTER         0x08  // Center
#define XACT_FIELD_P1             0x11  // P1 gain
#define XACT_FIELD_P2             0x12  // P2 gain
#define XACT_FIELD_D1             0x13  // D1 gain
#define XACT_FIELD_TB             0x15  // TB
#define XACT_FIELD_POT_GAP        0x21  // POT Gap
#define XACT_FIELD_WRITE_FLASH    0x30  // Save changes to flash

// XACT servo data ID range (from fbus_sensor.h)
#define FBUS_SERVO_DATA_BASE 0x6800
#define FBUS_SERVO_DATA_END  0x680F

// Maximum number of XACT servos to track
#define XACT_MAX_SERVOS 16

// XACT servo programming structure
// All fields are uint8_t to match the frame format
typedef struct {
    uint8_t phyID;        // Physical ID
    uint8_t fieldId;      // Field ID (parameter to write)
    uint16_t appId;       // Application ID (APPID2 << 8 | APPID1)
    uint16_t data;         // Data value (DATA1)
} xactServoParam_t;

// XACT servo tracking structure
typedef struct {
    uint8_t phyID;
    uint16_t appId;
    timeUs_t lastSeenUs;
} xactServo_t;

// XACT servo parameter storage
typedef struct {
    uint8_t physicalId;      // 0x00
    uint8_t appIdOffset;     // 0x01
    uint16_t dataRate;       // 0x02 (16-bit value)
    uint8_t range;           // 0x04
    uint8_t direction;       // 0x05
    uint8_t pulseType;       // 0x06
    uint8_t channel;         // 0x07
    uint8_t center;          // 0x08
    uint8_t p1;              // 0x11
    uint8_t p2;              // 0x12
    uint8_t d1;              // 0x13
    uint8_t tb;              // 0x15
    uint8_t potGap;          // 0x21
} xactServoParams_t;

// Initialize XACT servo module
void fbusXactInit(void);

// Clear all discovered XACT servos
void fbusXactClearDiscoveredServos(void);

// Start a new sensor discovery phase
void fbusXactStartSensorDiscovery(void);

// Track an XACT servo (called when servo data is received)
void fbusXactTrackServo(uint8_t phyID, uint16_t appId, timeUs_t currentTimeUs);

// Write a parameter to an XACT servo using physical ID
// fieldId: XACT_FIELD_* constant
// appId: Application ID (16-bit)
// data: Data value (8-bit)
bool fbusXactWriteUplinkFramePhyID(uint8_t phyID, uint8_t fieldId, uint16_t appId, uint16_t data);

// Get the number of discovered XACT servos
uint8_t fbusXactGetDiscoveredServoCount(void);

// Get a discovered XACT servo physical ID by index
uint8_t fbusXactGetDiscoveredServoPhyID(uint8_t index);

// Get servo parameters for a specific physical ID
bool fbusXactGetServoParams(uint8_t phyID, xactServoParams_t *params);

// Set a specific servo parameter field
bool fbusXactSetServoParam(uint8_t phyID, uint8_t fieldId, uint16_t appId, uint16_t data);

// Compare and write all parameters if different from cache
bool fbusXactCompareAndWriteParams(uint8_t phyID, uint16_t appId, const xactServoParams_t *newParams);

// Check if XACT module is initialized
bool fbusXactIsInitialized(void);

// Process XACT servo programming queue (internal use)
bool fbusXactProcessQueue(fbusMasterDownlink_t *downlink);

// Get queue count (for monitoring)
uint8_t fbusXactGetQueueCount(void);

// Check if XACT is currently busy (reading or writing)
bool fbusXactIsBusy(void);

// Notify that a response was received for the current read operation
void fbusXactNotifyResponse(uint8_t phyID, uint8_t fieldId);

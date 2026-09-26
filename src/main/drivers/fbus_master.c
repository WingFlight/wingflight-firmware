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

 //this uses SBUS out and SPORT/FBUS_in

#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "platform.h"

#include "pg/fbus_master.h"
#include "pg/sbus_output.h"
#include "pg/servos.h"
#include "pg/bus_servo.h"

#include "common/maths.h"
#include "common/time.h"

#include "drivers/time.h"
#include "drivers/sbus_output.h"
#include "drivers/fbus_master.h"
#include "drivers/fbus_sensor.h"
#include "drivers/fbus_xact.h"

#include "flight/mixer.h"
#include "flight/servos.h"

#include "fc/runtime_config.h"
#include "build/build_config.h"
#include "rx/frsky_crc.h"
#include "rx/fbus.h"
#include "io/serial.h"
#define FBUS_MASTER_BUFFER_SIZE 64

enum {
    FBUS_FRAME_ID_NULL = 0x00,
    FBUS_FRAME_ID_DATA = 0x10,
    FBUS_FRAME_ID_WORKING_STATE = 0x21,
    FBUS_FRAME_IDLE_STATE = 0x22,
    FBUS_FRAME_ID_READ = 0x30,
    FBUS_FRAME_ID_WRITE = 0x31,
    FBUS_FRAME_ID_RESPONSE = 0x32,
    FBUS_FRAME_ID_OTA_START = 0xF0,
    FBUS_FRAME_ID_OTA_DATA = 0xF1,
    FBUS_FRAME_ID_OTA_STOP = 0xF2
};

serialPort_t *fbusMasterPort = NULL;

#define FC_COMMON_ID 0x1B
typedef enum {
    FBUS_MASTER_SCAN_PHY_ID = 0,
    FBUS_MASTER_QUERY_PHY_ID,
}fbusMasterTelemetryState_e;

typedef enum {
    FBUS_MASTER_TELEMETRY = 0,
    FBUS_MASTER_OTA,
}fbusMasterPayloadState_e;

uint8_t phsIdList[FBUS_MAX_PHYS_ID] = {0};
uint8_t physIdsfound = 0;
uint8_t physIdCnt = 0;
uint8_t currentPhysId = 0;
uint8_t readIngoreBytes = 0;
uint8_t readBytes = 0;
uint8_t buffer[FBUS_MASTER_BUFFER_SIZE] = {0};

static timeUs_t nextTelemetryPollTimeUs = 0;
static timeUs_t sensorDiscoveryEndTimeUs = 0;

fbusMasterPayloadState_e fbusMasterPayloadState = FBUS_MASTER_TELEMETRY;
fbusMasterTelemetryState_e fbusMasterTelemetryState = FBUS_MASTER_SCAN_PHY_ID;

static uint16_t fbusMasterTelemetryRateHz(void)
{
    return constrain(fbusMasterConfig()->telemetryRate, FBUS_MASTER_TELEMETRY_RATE_MIN_HZ, FBUS_MASTER_TELEMETRY_RATE_MAX_HZ);
}

static uint16_t fbusMasterDiscoveryTimeMs(void)
{
    return constrain(fbusMasterConfig()->sensorDiscoveryTimeMs, FBUS_MASTER_DISCOVERY_TIME_MIN_MS, FBUS_MASTER_DISCOVERY_TIME_MAX_MS);
}

static timeDelta_t fbusMasterTelemetryPeriodUs(void)
{
    return 1000000 / fbusMasterTelemetryRateHz();
}

static void fbusMasterStartDiscoveryWindow(timeUs_t currentTimeUs)
{
    sensorDiscoveryEndTimeUs = currentTimeUs + ((timeUs_t)fbusMasterDiscoveryTimeMs() * 1000);
}

// Resets phy-ID scanning state and re-arms the discovery window. Exposed so XACT servo
// programming can force a fresh scan (e.g. after a servo's physical ID has been reprogrammed).
void fbusMasterStartDiscovery(void)
{
    physIdsfound = 0;
    physIdCnt = 0;
    currentPhysId = 0;
    fbusMasterTelemetryState = FBUS_MASTER_SCAN_PHY_ID;
    fbusMasterStartDiscoveryWindow(micros());
}

static uint8_t fbusMasterTakeNextScanPhysId(void)
{
    if (currentPhysId >= FBUS_MAX_PHYS_ID) {
        currentPhysId = 0;
    }

    const uint8_t phyId = currentPhysId;
    currentPhysId++;
    return phyId;
}

static void smartportMasterPhyIDFillCheckBits(uint8_t *phyIDByte)
{
    *phyIDByte |= (fbusGetBit(*phyIDByte, 0) ^ fbusGetBit(*phyIDByte, 1) ^ fbusGetBit(*phyIDByte, 2)) << 5;
    *phyIDByte |= (fbusGetBit(*phyIDByte, 2) ^ fbusGetBit(*phyIDByte, 3) ^ fbusGetBit(*phyIDByte, 4)) << 6;
    *phyIDByte |= (fbusGetBit(*phyIDByte, 0) ^ fbusGetBit(*phyIDByte, 2) ^ fbusGetBit(*phyIDByte, 4)) << 7;
}

static int8_t smartportMasterStripPhyIDCheckBits(uint8_t phyID)
{
    uint8_t smartportPhyID = phyID & 0x1F;
    uint8_t phyIDCheck = smartportPhyID;
    smartportMasterPhyIDFillCheckBits(&phyIDCheck);
    return phyID == phyIDCheck ? smartportPhyID : -1;
}

// channels[] holds 8 values: CH1-8 analog
static void fbusMasterPrepareControl8(fbusMasterControl8_t *frame, const uint16_t *channels)
{
    memset(frame, 0, sizeof(*frame));

    frame->length = FBUS_CONTROL8_LENGTH;
    frame->type = FBUS_CONTROL_TYPE_RC;

    frame->channels.chan0 = channels[0];
    frame->channels.chan1 = channels[1];
    frame->channels.chan2 = channels[2];
    frame->channels.chan3 = channels[3];
    frame->channels.chan4 = channels[4];
    frame->channels.chan5 = channels[5];
    frame->channels.chan6 = channels[6];
    frame->channels.chan7 = channels[7];

    frame->rssi = 100; //ToDo

    frame->crc = frskyCheckSum(&frame->type, sizeof(*frame) - 2);
}

// channels[] holds FBUS_MASTER_CHANNELS_16_COUNT values: CH1-16 analog, CH17-18 digital
static void fbusMasterPrepareControl16(fbusMasterControl16_t *frame, const uint16_t *channels)
{
    memset(frame, 0, sizeof(*frame));

    frame->length = FBUS_CONTROL16_LENGTH;
    frame->type = FBUS_CONTROL_TYPE_RC;

    frame->channels.chan0 = channels[0];
    frame->channels.chan1 = channels[1];
    frame->channels.chan2 = channels[2];
    frame->channels.chan3 = channels[3];
    frame->channels.chan4 = channels[4];
    frame->channels.chan5 = channels[5];
    frame->channels.chan6 = channels[6];
    frame->channels.chan7 = channels[7];
    frame->channels.chan8 = channels[8];
    frame->channels.chan9 = channels[9];
    frame->channels.chan10 = channels[10];
    frame->channels.chan11 = channels[11];
    frame->channels.chan12 = channels[12];
    frame->channels.chan13 = channels[13];
    frame->channels.chan14 = channels[14];
    frame->channels.chan15 = channels[15];

    frame->channels.flags = channels[16] ? BIT(0) : 0;
    frame->channels.flags |= channels[17] ? BIT(1) : 0;

    frame->rssi = 100; //ToDo

    frame->crc = frskyCheckSum(&frame->type, sizeof(*frame) - 2);
}

// channels[] holds FBUS_MASTER_CHANNELS_24_COUNT values: CH1-24 analog
static void fbusMasterPrepareControl24(fbusMasterControl24_t *frame, const uint16_t *channels)
{
    memset(frame, 0, sizeof(*frame));

    frame->length = FBUS_CONTROL24_LENGTH;
    frame->type = FBUS_CONTROL_TYPE_RC;

    frame->channels.chan0 = channels[0];
    frame->channels.chan1 = channels[1];
    frame->channels.chan2 = channels[2];
    frame->channels.chan3 = channels[3];
    frame->channels.chan4 = channels[4];
    frame->channels.chan5 = channels[5];
    frame->channels.chan6 = channels[6];
    frame->channels.chan7 = channels[7];
    frame->channels.chan8 = channels[8];
    frame->channels.chan9 = channels[9];
    frame->channels.chan10 = channels[10];
    frame->channels.chan11 = channels[11];
    frame->channels.chan12 = channels[12];
    frame->channels.chan13 = channels[13];
    frame->channels.chan14 = channels[14];
    frame->channels.chan15 = channels[15];
    frame->channels.chan16 = channels[16];
    frame->channels.chan17 = channels[17];
    frame->channels.chan18 = channels[18];
    frame->channels.chan19 = channels[19];
    frame->channels.chan20 = channels[20];
    frame->channels.chan21 = channels[21];
    frame->channels.chan22 = channels[22];
    frame->channels.chan23 = channels[23];

    frame->rssi = 100; //ToDo

    frame->crc = frskyCheckSum(&frame->type, sizeof(*frame) - 2);
}

static void fbusMasterPrepareDownlink(fbusMasterDownlink_t *downlink, timeUs_t currentTimeUs)
{
    uint8_t crc;

    memset(downlink, 0, sizeof(fbusMasterDownlink_t));

    switch (fbusMasterPayloadState) {
        case FBUS_MASTER_TELEMETRY:
            downlink->length = FBUS_DOWNLINK_PAYLOAD_SIZE;

            // XACT servo programming (read/write) takes priority over telemetry polling and
            // is not subject to the telemetry rate throttle below, so reads/writes complete quickly.
            if (fbusXactIsBusy() && fbusXactProcessQueue(downlink)) {
                smartportMasterPhyIDFillCheckBits(&downlink->phyID);
                crc = frskyCheckSum(&downlink->phyID, FBUS_DOWNLINK_PAYLOAD_SIZE);
                downlink->crc = crc;
                break;
            }

            if (cmpTimeUs(currentTimeUs, nextTelemetryPollTimeUs) < 0) {
                downlink->phyID = 0;
                downlink->prim = FBUS_FRAME_ID_NULL;
                crc = frskyCheckSum(&downlink->phyID, FBUS_DOWNLINK_PAYLOAD_SIZE);
                downlink->crc = crc;
                break;
            }

            nextTelemetryPollTimeUs = currentTimeUs + fbusMasterTelemetryPeriodUs();

            if (fbusMasterTelemetryState == FBUS_MASTER_SCAN_PHY_ID && cmpTimeUs(currentTimeUs, sensorDiscoveryEndTimeUs) >= 0) {
                fbusMasterTelemetryState = FBUS_MASTER_QUERY_PHY_ID;
                physIdCnt = 0;
            }
            
            switch (fbusMasterTelemetryState) {
                case FBUS_MASTER_SCAN_PHY_ID:
                    downlink->phyID = fbusMasterTakeNextScanPhysId();
                    downlink->prim = FBUS_FRAME_ID_DATA;
                    break;
                case FBUS_MASTER_QUERY_PHY_ID:
                    if (physIdsfound == 0) {
                        fbusMasterTelemetryState = FBUS_MASTER_SCAN_PHY_ID;
                        currentPhysId = 0;
                        fbusMasterStartDiscoveryWindow(currentTimeUs);
                        downlink->phyID = 0;
                        downlink->prim = FBUS_FRAME_ID_NULL;
                        break;
                    }

                    if (physIdCnt >= physIdsfound) {
                        physIdCnt = 0;
                    }

                    currentPhysId = phsIdList[physIdCnt];
                    downlink->phyID = currentPhysId;
                    downlink->prim = FBUS_FRAME_ID_DATA;
                    physIdCnt++;
                    break;
                
                default:
                    break;
            }
    
            smartportMasterPhyIDFillCheckBits(&downlink->phyID);
            crc = frskyCheckSum(&downlink->phyID, FBUS_DOWNLINK_PAYLOAD_SIZE);
            downlink->crc = crc;
    
            break;
    
        case FBUS_MASTER_OTA:
            //ToDo
            break;
        
        default:
            break;
    }

}

static void processDownlinkFrame(uint8_t *data)
{
    fbusMasterDownlink_t downlink;
    memcpy(&downlink, data, sizeof(downlink));
    uint8_t chkSum = frskyCheckSum((uint8_t *)&downlink.phyID, FBUS_DOWNLINK_PAYLOAD_SIZE);
    if (chkSum == downlink.crc) {
        const int8_t decodedPhyId = smartportMasterStripPhyIDCheckBits(downlink.phyID);
        if (decodedPhyId < 0) {
            return;
        }

        downlink.phyID = decodedPhyId;
        if (fbusMasterTelemetryState == FBUS_MASTER_SCAN_PHY_ID) {
            bool alreadyInList = false;
            for (uint8_t i = 0; i < physIdsfound; i++) {
                if (phsIdList[i] == downlink.phyID) {
                    alreadyInList = true;
                    break;
                }
            }
            if (!alreadyInList && physIdsfound < ARRAYLEN(phsIdList)) {
                phsIdList[physIdsfound++] = downlink.phyID;
            }
        }
        // Process sensor data for observation tracking and forwarding
        // Only process if it's a data frame (not null/poll frames)
        if (downlink.prim == FBUS_FRAME_ID_DATA && downlink.phyID != FC_COMMON_ID) {
            // Convert 4-byte array to uint32_t (little-endian)
            uint32_t sensorData = downlink.data[0] | (downlink.data[1] << 8) |
                                  (downlink.data[2] << 16) | (downlink.data[3] << 24);
            fbusSensorProcessData(downlink.phyID, downlink.appId, sensorData);
        }
        // XACT servo programming response frame:
        // appId = App ID (16-bit), data[0] = FIELDID, data[1..2] = DATA1 (little-endian)
        else if (downlink.prim == FBUS_FRAME_ID_RESPONSE) {
            uint16_t appId = downlink.appId;
            uint8_t fieldId = downlink.data[0];
            uint16_t dataValue = (uint16_t)downlink.data[1] | ((uint16_t)downlink.data[2] << 8);

            fbusXactSetServoParam(downlink.phyID, fieldId, appId, dataValue);
            fbusXactNotifyResponse(downlink.phyID, fieldId);
        }
    }
}

static FAST_CODE void dataReceive(uint16_t c, void *data)
{
    UNUSED(data);
    // don't listen to self
    if (readIngoreBytes > 0) {
        readIngoreBytes--;
        return;
    }

    buffer[readBytes++] = c;

    if (readBytes >= FBUS_MASTER_BUFFER_SIZE) {
        //sync error
        readBytes = 0;
    } else {
        if (readBytes >= 10 && buffer[0] == FBUS_DOWNLINK_PAYLOAD_SIZE) {
            //process frame, reset readBytes
            processDownlinkFrame(buffer);
            readBytes = 0;
        }
    }
}

// Speed-limit state for F.Bus output, separate from SBUS output's so each
// output steps only on its own frames.
static float fbusMasterServoInput[FBUS_MASTER_CHANNELS_24_COUNT];
static timeUs_t fbusMasterLastFrameUs = 0;

static float fbusMasterGetChannelValue(uint8_t channel, float dt)
{
    return sbusOutGetValueMixer(channel, &fbusMasterServoInput[channel], dt);
}

static uint16_t fbusMasterConvertToSbus(float value, bool digital)
{
    // Digital channels (the two in the flags byte) are on at 1500us and above
    if (digital) {
        return (value >= 1500) ? 1 : 0;
    }

    // Analog channels: bus servo range (1000 -> BUS_SERVO_MIN_SIGNAL) to
    // (2000 -> BUS_SERVO_MAX_SIGNAL) -> SBUS 192-1792
    const float scaledValue = scaleRangef(value, BUS_SERVO_MIN_SIGNAL, BUS_SERVO_MAX_SIGNAL, 192, 1792);
    return constrain(nearbyintf(scaledValue), FBUS_MIN, FBUS_MAX);
}

void fbusMasterUpdate(timeUs_t currentTimeUs)
{
    if (!fbusMasterPort)
        return;

    // Keep derived FBUS sensor states (timeouts/GPS mirrors) updated.
    fbusSensorUpdate(currentTimeUs);

    // fbus_master_channels: 8 -> 8-channel frame, 12/16 -> 16-channel frame
    // (at 12, CH13-16 are sent at center), 24 -> 24-channel frame. The
    // digital channels (CH17-18) are only used with all 16.
    const int count = busOutChannelCount(fbusMasterConfig()->channels);
    const int frameChannels = (count <= 8) ? 8 : (count <= 16) ? FBUS_MASTER_CHANNELS_16_COUNT : FBUS_MASTER_CHANNELS_24_COUNT;
    const size_t controlSize = (count <= 8) ? sizeof(fbusMasterControl8_t) :
                               (count <= 16) ? sizeof(fbusMasterControl16_t) : sizeof(fbusMasterControl24_t);

    // Check TX Buff is free
    if (serialTxBytesFree(fbusMasterPort) <= controlSize + sizeof(fbusMasterDownlink_t)) {
        return;
    }

    // Time since the previous frame, for the speed limit. The first frame
    // assumes the configured frame rate; a long gap is capped at 100ms.
    const float dt = fbusMasterLastFrameUs ?
        constrainf(cmpTimeUs(currentTimeUs, fbusMasterLastFrameUs) * 1e-6f, 0.0f, 0.1f) :
        1.0f / fbusMasterConfig()->frameRate;
    fbusMasterLastFrameUs = currentTimeUs;

    // Start sending.
    uint16_t channels[FBUS_MASTER_CHANNELS_24_COUNT];
    for (int ch = 0; ch < frameChannels; ch++) {
        const bool digital = (count == 16) && (ch >= 16);
        const bool used = (ch < count) || digital;
        if (!used) {
            channels[ch] = (ch >= 16) ? 0 : fbusMasterConvertToSbus(1500, false);
            continue;
        }

        const float value = fbusMasterGetChannelValue(ch, dt);
        channels[ch] = fbusMasterConvertToSbus(value, digital);

        // Store the output value for getServoOutput() to retrieve
        setBusServoOutput(ch, value);
    }

    // Control frame followed by the downlink slot, sent as one write
    uint8_t frame[sizeof(fbusMasterControl24_t) + sizeof(fbusMasterDownlink_t)];
    if (count <= 8) {
        fbusMasterControl8_t control;
        fbusMasterPrepareControl8(&control, channels);
        memcpy(frame, &control, sizeof(control));
    } else if (count <= 16) {
        fbusMasterControl16_t control;
        fbusMasterPrepareControl16(&control, channels);
        memcpy(frame, &control, sizeof(control));
    } else {
        fbusMasterControl24_t control;
        fbusMasterPrepareControl24(&control, channels);
        memcpy(frame, &control, sizeof(control));
    }

    fbusMasterDownlink_t downlink;
    fbusMasterPrepareDownlink(&downlink, currentTimeUs);
    memcpy(frame + controlSize, &downlink, sizeof(downlink));

    const size_t frameSize = controlSize + sizeof(downlink);

    // serial output
    serialWriteBuf(fbusMasterPort, frame, frameSize);
    readIngoreBytes = frameSize;
    readBytes = 0;
}

bool fbusMasterIsEnabled(void)
{
    return fbusMasterPort != NULL;
}

void fbusMasterInit(void)
{
    const serialPortConfig_t *portConfig =
        findSerialPortConfig(FUNCTION_FBUS_MASTER);

    if (!portConfig) {
        fbusMasterPort = NULL;
        return;
    }

    nextTelemetryPollTimeUs = 0;
    fbusMasterStartDiscovery();

    // Initialize XACT servo programming module
    fbusXactInit();

    serialReceiveCallbackPtr callback = dataReceive;
    fbusMasterPort = openSerialPort(
        portConfig->identifier, FUNCTION_FBUS_MASTER, callback, NULL, 460800, MODE_RXTX,
        SERIAL_STOPBITS_1 | SERIAL_PARITY_NO |
            (fbusMasterConfig()->inverted ? SERIAL_INVERTED : SERIAL_NOT_INVERTED) |
            SERIAL_BIDIR |
            (fbusMasterConfig()->pinSwap ? SERIAL_PINSWAP : SERIAL_NOSWAP));
}

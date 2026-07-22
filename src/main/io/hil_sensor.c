/*
 * This file is part of Wingflight.
 *
 * Wingflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Wingflight is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <string.h>
#include <math.h>

#include "platform.h"

#ifdef USE_HIL_SENSOR_OVERRIDE

#include "build/debug.h"

#include "common/axis.h"
#include "common/crc.h"
#include "common/utils.h"

#include "drivers/time.h"
#include "drivers/serial.h"

#include "drivers/barometer/barometer_fake.h"

#include "io/serial.h"
#include "io/hil_sensor.h"

// See hitl/bridge/PROTOCOL.md for the full wire format spec.

#define HIL_SENSOR_SYNC1                0x48 // 'H'
#define HIL_SENSOR_SYNC2                0x53 // 'S'
#define HIL_SENSOR_PROTO_VERSION        1
#define HIL_SENSOR_TYPE_CORE_V1         0x01
#define HIL_SENSOR_CORE_V1_PAYLOAD_LEN  36
#define HIL_SENSOR_MAX_PAYLOAD          36
#define HIL_SENSOR_BAUDRATE             921600
#define HIL_SENSOR_TIMEOUT_MS           250

typedef enum {
    HIL_WAIT_SYNC1 = 0,
    HIL_WAIT_SYNC2,
    HIL_WAIT_VERSION,
    HIL_WAIT_TYPE,
    HIL_WAIT_LEN,
    HIL_WAIT_PAYLOAD,
    HIL_WAIT_CRC1,
    HIL_WAIT_CRC2,
} hilParserState_e;

typedef struct {
    hilParserState_e state;
    uint8_t type;
    uint8_t len;
    uint8_t payload[HIL_SENSOR_MAX_PAYLOAD];
    uint8_t payloadIndex;
    uint16_t crcCalc;
    uint16_t crcRx;
} hilParser_t;

static serialPort_t *hilSensorPort = NULL;
static hilParser_t hilParser;

static float hilGyroDps[XYZ_AXIS_COUNT];
static float hilAccG[XYZ_AXIS_COUNT];
static timeMs_t hilLastPacketMs = 0;
static uint32_t hilLastSeq = 0;
static uint32_t hilPacketCount = 0;
static uint32_t hilCrcErrorCount = 0;

bool hilSensorIsActive(void)
{
    if (!hilSensorPort || hilPacketCount == 0) {
        return false;
    }

    return cmp32(millis(), hilLastPacketMs) <= HIL_SENSOR_TIMEOUT_MS;
}

static void hilSensorResetParser(void)
{
    hilParser.state = HIL_WAIT_SYNC1;
    hilParser.payloadIndex = 0;
    hilParser.crcCalc = 0;
}

static void hilSensorApplyPacket(void)
{
    const uint8_t *p = hilParser.payload;
    uint32_t seq;
    float gyro[XYZ_AXIS_COUNT];
    float accel[XYZ_AXIS_COUNT];
    int32_t baroPressurePa;
    int32_t baroTempCentiC;

    memcpy(&seq, p, sizeof(seq));
    p += sizeof(seq);
    memcpy(gyro, p, sizeof(gyro));
    p += sizeof(gyro);
    memcpy(accel, p, sizeof(accel));
    p += sizeof(accel);
    memcpy(&baroPressurePa, p, sizeof(baroPressurePa));
    p += sizeof(baroPressurePa);
    memcpy(&baroTempCentiC, p, sizeof(baroTempCentiC));

    if (hilPacketCount > 0 && seq != hilLastSeq + 1) {
        DEBUG_SET(DEBUG_HIL_SENSOR, 3, (int16_t)(seq - hilLastSeq - 1)); // dropped packet count
    }
    hilLastSeq = seq;
    hilPacketCount++;

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        hilGyroDps[axis] = gyro[axis];
        hilAccG[axis] = accel[axis];
    }

#ifdef USE_FAKE_BARO
    fakeBaroSet(baroPressurePa, baroTempCentiC);
#endif

    hilLastPacketMs = millis();

    DEBUG_SET(DEBUG_HIL_SENSOR, 0, lrintf(hilGyroDps[X]));
    DEBUG_SET(DEBUG_HIL_SENSOR, 1, lrintf(hilAccG[Z] * 1000));
    DEBUG_SET(DEBUG_HIL_SENSOR, 2, (int16_t)hilPacketCount);
}

static void hilSensorParseByte(uint8_t c)
{
    switch (hilParser.state) {
    case HIL_WAIT_SYNC1:
        if (c == HIL_SENSOR_SYNC1) {
            hilParser.state = HIL_WAIT_SYNC2;
        }
        break;

    case HIL_WAIT_SYNC2:
        if (c == HIL_SENSOR_SYNC2) {
            hilParser.state = HIL_WAIT_VERSION;
        } else {
            hilSensorResetParser();
        }
        break;

    case HIL_WAIT_VERSION:
        if (c != HIL_SENSOR_PROTO_VERSION) {
            hilSensorResetParser();
            break;
        }
        hilParser.crcCalc = crc16_ccitt(0, c);
        hilParser.state = HIL_WAIT_TYPE;
        break;

    case HIL_WAIT_TYPE:
        hilParser.type = c;
        hilParser.crcCalc = crc16_ccitt(hilParser.crcCalc, c);
        hilParser.state = HIL_WAIT_LEN;
        break;

    case HIL_WAIT_LEN:
        hilParser.len = c;
        hilParser.crcCalc = crc16_ccitt(hilParser.crcCalc, c);
        hilParser.payloadIndex = 0;
        if (hilParser.len == 0 || hilParser.len > HIL_SENSOR_MAX_PAYLOAD) {
            hilSensorResetParser();
            break;
        }
        hilParser.state = HIL_WAIT_PAYLOAD;
        break;

    case HIL_WAIT_PAYLOAD:
        hilParser.payload[hilParser.payloadIndex++] = c;
        hilParser.crcCalc = crc16_ccitt(hilParser.crcCalc, c);
        if (hilParser.payloadIndex >= hilParser.len) {
            hilParser.state = HIL_WAIT_CRC1;
        }
        break;

    case HIL_WAIT_CRC1:
        hilParser.crcRx = (uint16_t)c << 8;
        hilParser.state = HIL_WAIT_CRC2;
        break;

    case HIL_WAIT_CRC2:
        hilParser.crcRx |= c;
        if (hilParser.crcRx == hilParser.crcCalc &&
            hilParser.type == HIL_SENSOR_TYPE_CORE_V1 &&
            hilParser.len == HIL_SENSOR_CORE_V1_PAYLOAD_LEN) {
            hilSensorApplyPacket();
        } else {
            hilCrcErrorCount++;
        }
        hilSensorResetParser();
        break;
    }
}

void hilSensorInit(void)
{
    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_HIL_SENSOR);
    if (!portConfig) {
        return;
    }

    hilSensorResetParser();

    hilSensorPort = openSerialPort(portConfig->identifier, FUNCTION_HIL_SENSOR, NULL, NULL,
        HIL_SENSOR_BAUDRATE, MODE_RX, SERIAL_NOT_INVERTED);
}

void hilSensorUpdate(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    if (!hilSensorPort) {
        return;
    }

    while (serialRxBytesWaiting(hilSensorPort)) {
        hilSensorParseByte(serialRead(hilSensorPort));
    }
}

void hilSensorOverrideGyro(float gyroADC[3], float scale)
{
    if (!hilSensorIsActive() || scale == 0.0f) {
        return;
    }

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        gyroADC[axis] = hilGyroDps[axis] / scale;
    }
}

void hilSensorOverrideAcc(float accADC[3], uint16_t acc1G)
{
    if (!hilSensorIsActive()) {
        return;
    }

    for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
        accADC[axis] = hilAccG[axis] * acc1G;
    }
}

#endif // USE_HIL_SENSOR_OVERRIDE

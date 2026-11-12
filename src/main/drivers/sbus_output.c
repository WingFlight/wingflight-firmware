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

#include "sbus_output.h"

#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "build/build_config.h"
#include "common/curve.h"
#include "common/maths.h"
#include "common/time.h"
#include "fc/runtime_config.h"
#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/servos.h"
#include "io/serial.h"
#include "pg/bus_servo.h"
#include "pg/sbus_output.h"
#include "pg/servos.h"
#include "platform.h"
#include "rx/rx.h"

// SBUS payload value range for analog channels
#define SBUS_MIN 192
#define SBUS_MAX 1792

static serialPort_t *sbusOutPort = NULL;

static void sbusOutPrepareSbusFrame(sbusOutFrame_t *frame,
                                    uint16_t *channels)
{
    frame->syncByte = 0x0F;

    // There's no way to make a bit field array, so we have to go tedious.
    frame->chan0 = channels[0];
    frame->chan1 = channels[1];
    frame->chan2 = channels[2];
    frame->chan3 = channels[3];
    frame->chan4 = channels[4];
    frame->chan5 = channels[5];
    frame->chan6 = channels[6];
    frame->chan7 = channels[7];
    frame->chan8 = channels[8];
    frame->chan9 = channels[9];
    frame->chan10 = channels[10];
    frame->chan11 = channels[11];
    frame->chan12 = channels[12];
    frame->chan13 = channels[13];
    frame->chan14 = channels[14];
    frame->chan15 = channels[15];

    frame->flags = channels[16] ? BIT(0) : 0;
    frame->flags |= channels[17] ? BIT(1) : 0;
    // Other flags?

    frame->endByte = 0;
}

float sbusOutGetRX(uint8_t channel)
{
    if (channel < MAX_SUPPORTED_RC_CHANNEL_COUNT)
        return rcChannel[channel];
    return 0;
}

// The value sent on a bus channel: servo N's finished output (flight/servos.c),
// the same one that goes to PWM output N on boards that have one, clipped to
// the range the bus carries. Shared with F.Bus output.
float sbusOutGetServoValue(uint8_t channel)
{
    return constrainf(getServoOutputFloat(channel), BUS_SERVO_MIN_SIGNAL, BUS_SERVO_MAX_SIGNAL);
}

static uint16_t sbusOutConvertToSbus(uint8_t channel, float pwm)
{
    // For digital channels (16-17), convert to 0 or 1
    if (channel >= 16) {
        // Assume threshold at 1500us
        return (pwm >= 1500) ? 1 : 0;
    }

    // For analog channels (0-15), convert microseconds to SBUS range (192-1792)
    // Bus servo range: (1000 -> BUS_SERVO_MIN_SIGNAL) to (2000 -> BUS_SERVO_MAX_SIGNAL) -> SBUS 192-1792
    const float value = scaleRangef(pwm, BUS_SERVO_MIN_SIGNAL, BUS_SERVO_MAX_SIGNAL, SBUS_MIN, SBUS_MAX);
    return constrain(nearbyintf(value), SBUS_MIN, SBUS_MAX);
}

void sbusOutUpdate(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    if (!sbusOutPort)
        return;

    // Check TX Buff is free
    if (serialTxBytesFree(sbusOutPort) <= sizeof(sbusOutFrame_t))
        return;

    // sbus_out_channels: CH1-count carry S1-count, the rest are sent at
    // center. The digital channels (17-18) are only used with all 16.
    const int count = MIN(busOutChannelCount(sbusOutConfig()->channels), 16);
    // Prepare SBUS frame
    sbusOutFrame_t frame;
    uint16_t channels[SBUS_OUT_CHANNELS];
    for (int ch = 0; ch < SBUS_OUT_CHANNELS; ch++) {
        const bool used = (ch < count) || (count == 16);
        if (!used) {
            channels[ch] = (ch >= 16) ? 0 : sbusOutConvertToSbus(ch, 1500);
            continue;
        }

        channels[ch] = sbusOutConvertToSbus(ch, sbusOutGetServoValue(ch));
    }
    sbusOutPrepareSbusFrame(&frame, channels);

    // serial output
    serialWriteBuf(sbusOutPort, (const uint8_t *)&frame, sizeof(frame));
}

bool sbusOutIsEnabled(void)
{ 
    return sbusOutPort != NULL;
}

void sbusOutInit(void)
{
    const serialPortConfig_t *portConfig =
        findSerialPortConfig(FUNCTION_SBUS_OUT);

    if (!portConfig) {
        sbusOutPort = NULL;
        return;
    }

    sbusOutPort = openSerialPort(
        portConfig->identifier, FUNCTION_SBUS_OUT, NULL, NULL, 100000, MODE_TX,
        SERIAL_STOPBITS_2 | SERIAL_PARITY_EVEN |
            (sbusOutConfig()->inverted ? SERIAL_INVERTED : SERIAL_NOT_INVERTED) |
            SERIAL_UNIDIR |
            (sbusOutConfig()->pinSwap ? SERIAL_PINSWAP : SERIAL_NOSWAP));
}

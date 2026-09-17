/*
 * This file is part of Wingflight.
 *
 * Wingflight is free software. You can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Wingflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software. If not, see <https://www.gnu.org/licenses/>.
 */

/*
 * iCE40 FBUS/PWM output mux (wingflight-fbus-mux, RF007-V3).
 *
 * The FPGA sits between the MCU timer outputs and the servo header and has no
 * configuration flash. At boot the bitstream is loaded in SPI slave mode
 * (Lattice TN1248), then the channel modes are sent as a 32-bit word on a
 * UART (115200 8N1, low byte first) and the FPGA echoes the accepted word:
 *
 *   bits 2i+1:2i  channel i+1: 00 FBUS, 01 PWM, 10 bidirectional DShot
 *   bit  16       DShot600 timing (else DShot300)
 *   bit  17       FBUS line not inverted (idle high)
 *
 * Until configured, the FPGA outputs are high-Z; afterwards all channels
 * default to PWM passthrough.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#ifdef USE_FBUS_MUX_FPGA

#include "common/maths.h"
#include "common/time.h"

#include "drivers/bus_spi.h"
#include "drivers/io.h"
#include "drivers/motor.h"
#include "drivers/resource.h"
#include "drivers/serial.h"
#include "drivers/time.h"

#include "io/serial.h"

#include "pg/fbus_master.h"
#include "pg/motor.h"

#include "fbus_mux_fpga.h"

#define FPGA_SPI_CLOCK_HZ           4000000
#define FPGA_RESET_TIME_US          1000
#define FPGA_CLEAR_TIME_US          2000    // >= 1200 us after CRESET_B rises
#define FPGA_CDONE_TIMEOUT_MS       20
#define FPGA_LOAD_ATTEMPTS          3

#define MUX_UART_BAUD               115200
#define MUX_WORD_BYTES              4
#define MUX_ECHO_TIMEOUT_MS         30      // 4 bytes at 115200 take ~3.5 ms
#define MUX_RESYNC_MS               10      // the FPGA drops partial words after 5 ms
#define MUX_CONFIG_ATTEMPTS         3

#define MUX_WORD_MODE_FBUS          0x0
#define MUX_WORD_MODE_PWM           0x1
#define MUX_WORD_MODE_DSHOT         0x2
#define MUX_WORD_DSHOT600           (1UL << 16)
#define MUX_WORD_FBUS_IDLE_HIGH     (1UL << 17)

static fbusMuxFpgaStatus_e status = FBUS_MUX_FPGA_DISABLED;
static uint32_t modeWord;
static fbusMuxMode_e channelMode[FBUS_MUX_FPGA_CHANNELS];

static extDevice_t fpgaSpi;
static IO_t csPin = IO_NONE;
static IO_t cresetPin = IO_NONE;
static IO_t cdonePin = IO_NONE;
static serialPort_t *muxPort = NULL;

static uint8_t lastEcho[8];
static uint8_t lastEchoCount;

static const char * const statusNames[FBUS_MUX_FPGA_STATUS_COUNT] = {
    [FBUS_MUX_FPGA_DISABLED]       = "DISABLED",
    [FBUS_MUX_FPGA_ERROR_RESOURCE] = "RESOURCE ERROR",
    [FBUS_MUX_FPGA_ERROR_CDONE]    = "LOAD FAILED",
    [FBUS_MUX_FPGA_ERROR_ECHO]     = "NO CONFIG ECHO",
    [FBUS_MUX_FPGA_READY]          = "READY",
};

static IO_t claimPin(ioTag_t tag, resourceOwner_e owner, ioConfig_t cfg)
{
    IO_t io = IOGetByTag(tag);

    if (!io || !IOIsFreeOrPreinit(io)) {
        return IO_NONE;
    }

    IOInit(io, owner, 0);
    IOConfigGPIO(io, cfg);

    return io;
}

static void sendDummyClocks(uint8_t bytes)
{
    uint8_t buf[16];

    memset(buf, 0xFF, sizeof(buf));

    while (bytes) {
        const uint8_t len = MIN(bytes, sizeof(buf));
        spiReadWriteBuf(&fpgaSpi, buf, NULL, len);
        bytes -= len;
    }
}

static bool loadBitstream(void)
{
    // SPI slave configuration: SS_B low while CRESET_B rises
    IOLo(csPin);
    IOLo(cresetPin);
    delayMicroseconds(FPGA_RESET_TIME_US);
    IOHi(cresetPin);
    delayMicroseconds(FPGA_CLEAR_TIME_US);

    IOHi(csPin);
    sendDummyClocks(1);

    IOLo(csPin);
    spiReadWriteBuf(&fpgaSpi, (uint8_t *)fbusMuxFpgaBitstream, NULL, fbusMuxFpgaBitstreamSize);
    IOHi(csPin);

    // >= 49 clocks to finish the configuration
    sendDummyClocks(13);

    const timeMs_t start = millis();
    while (!IORead(cdonePin)) {
        if (cmp32(millis(), start) > FPGA_CDONE_TIMEOUT_MS) {
            return false;
        }
        delay(1);
    }

    // >= 49 more clocks to activate the user I/O
    sendDummyClocks(7);

    return true;
}

static uint32_t buildModeWord(void)
{
    const motorDevConfig_t *motor = &motorConfig()->dev;
    const bool dshotBidir = motor->useDshotTelemetry &&
        (motor->motorPwmProtocol == PWM_TYPE_DSHOT300 || motor->motorPwmProtocol == PWM_TYPE_DSHOT600);
    const bool fbusAvailable = findSerialPortConfig(FUNCTION_FBUS_MASTER) && fbusMasterConfig()->sendPin;

    uint32_t word = 0;

    for (int ch = 0; ch < FBUS_MUX_FPGA_CHANNELS; ch++) {
        fbusMuxMode_e mode = fbusMuxFpgaConfig()->mode[ch];

        // Fall back to PWM where the requested mode cannot work: FBUS needs the
        // master port and its send pin, the DShot bridge needs bidirectional
        // DShot300/600 and a bridge in the bitstream.
        if (mode == FBUS_MUX_MODE_FBUS && !fbusAvailable) {
            mode = FBUS_MUX_MODE_PWM;
        }
        if (mode == FBUS_MUX_MODE_DSHOT && !(dshotBidir && (fbusMuxFpgaDshotChannelMask & (1 << ch)))) {
            mode = FBUS_MUX_MODE_PWM;
        }
        if (mode >= FBUS_MUX_MODE_COUNT) {
            mode = FBUS_MUX_MODE_PWM;
        }
        channelMode[ch] = mode;

        uint32_t bits;
        switch (mode) {
            case FBUS_MUX_MODE_FBUS:
                bits = MUX_WORD_MODE_FBUS;
                break;
            case FBUS_MUX_MODE_DSHOT:
                bits = MUX_WORD_MODE_DSHOT;
                break;
            default:
                bits = MUX_WORD_MODE_PWM;
                break;
        }
        word |= bits << (2 * ch);
    }

    if (motor->motorPwmProtocol == PWM_TYPE_DSHOT600) {
        word |= MUX_WORD_DSHOT600;
    }
    if (!fbusMasterConfig()->inverted) {
        word |= MUX_WORD_FBUS_IDLE_HIGH;
    }

    return word;
}

static bool sendModeWord(uint32_t word)
{
    uint8_t tx[MUX_WORD_BYTES];

    for (int i = 0; i < MUX_WORD_BYTES; i++) {
        tx[i] = word >> (8 * i);
    }

    for (int attempt = 0; attempt < MUX_CONFIG_ATTEMPTS; attempt++) {
        // Let the FPGA discard any partial word
        delay(MUX_RESYNC_MS);
        while (serialRxBytesWaiting(muxPort)) {
            serialRead(muxPort);
        }

        serialWriteBuf(muxPort, tx, sizeof(tx));

        // Collect everything that arrives in the window (garbage helps diagnosis)
        uint8_t count = 0;
        const timeMs_t start = millis();

        while (cmp32(millis(), start) <= MUX_ECHO_TIMEOUT_MS) {
            if (serialRxBytesWaiting(muxPort)) {
                const uint8_t c = serialRead(muxPort);
                if (count < sizeof(lastEcho)) {
                    lastEcho[count] = c;
                }
                count++;
                if (count == MUX_WORD_BYTES && memcmp(lastEcho, tx, sizeof(tx)) == 0) {
                    break;
                }
            } else {
                delay(1);
            }
        }
        lastEchoCount = count;

        if (count == MUX_WORD_BYTES && memcmp(lastEcho, tx, sizeof(tx)) == 0) {
            return true;
        }
    }

    return false;
}

static void resetChannelModes(void)
{
    for (int ch = 0; ch < FBUS_MUX_FPGA_CHANNELS; ch++) {
        channelMode[ch] = FBUS_MUX_MODE_PWM;
    }
}

static bool initResources(const fbusMuxFpgaConfig_t *config)
{
    const serialPortIdentifier_e portId = SERIAL_PORT_USART1 + config->serialPort - 1;
    const serialPortConfig_t *portConfig = serialFindPortConfiguration(portId);

    if (!config->serialPort || !portConfig || portConfig->functionMask != FUNCTION_NONE) {
        return false;
    }

    if (!spiSetBusInstance(&fpgaSpi, config->spiDevice)) {
        return false;
    }
    fpgaSpi.useDMA = false;
    fpgaSpi.busType_u.spi.csnPin = IO_NONE;     // SS_B is driven explicitly
    spiSetClkDivisor(&fpgaSpi, spiCalculateDivider(FPGA_SPI_CLOCK_HZ));
    spiSetClkPhasePolarity(&fpgaSpi, false);    // mode 3, SCK idles high

    csPin = claimPin(config->csTag, OWNER_FPGA_CS, SPI_IO_CS_CFG);
    cresetPin = claimPin(config->cresetTag, OWNER_FPGA_CRESET, IOCFG_OUT_PP);
    cdonePin = claimPin(config->cdoneTag, OWNER_FPGA_CDONE, IOCFG_IPU);

    if (!csPin || !cresetPin || !cdonePin) {
        return false;
    }

    IOHi(csPin);
    IOHi(cresetPin);

    muxPort = openSerialPort(portId, FUNCTION_NONE, NULL, NULL, MUX_UART_BAUD, MODE_RXTX, SERIAL_NOT_INVERTED);

    return muxPort != NULL;
}

void fbusMuxFpgaInit(void)
{
    const fbusMuxFpgaConfig_t *config = fbusMuxFpgaConfig();

    resetChannelModes();

    if (!config->cresetTag) {
        status = FBUS_MUX_FPGA_DISABLED;
        return;
    }

    if (!initResources(config)) {
        status = FBUS_MUX_FPGA_ERROR_RESOURCE;
        return;
    }

    status = FBUS_MUX_FPGA_ERROR_CDONE;
    for (int attempt = 0; attempt < FPGA_LOAD_ATTEMPTS; attempt++) {
        if (loadBitstream()) {
            status = FBUS_MUX_FPGA_ERROR_ECHO;
            break;
        }
    }
    if (status == FBUS_MUX_FPGA_ERROR_CDONE) {
        return;
    }

    modeWord = buildModeWord();
    if (sendModeWord(modeWord)) {
        status = FBUS_MUX_FPGA_READY;
    } else {
        // The FPGA keeps its reset configuration (all PWM)
        resetChannelModes();
    }
}

bool fbusMuxFpgaReload(void)
{
    if (!csPin || !cresetPin || !cdonePin || !muxPort) {
        return false;
    }

    if (!loadBitstream()) {
        status = FBUS_MUX_FPGA_ERROR_CDONE;
        resetChannelModes();
        return false;
    }

    modeWord = buildModeWord();
    if (sendModeWord(modeWord)) {
        status = FBUS_MUX_FPGA_READY;
        return true;
    }

    status = FBUS_MUX_FPGA_ERROR_ECHO;
    resetChannelModes();
    return false;
}

bool fbusMuxFpgaSendRawWord(uint32_t word)
{
    return muxPort && sendModeWord(word);
}

uint8_t fbusMuxFpgaGetLastEcho(uint8_t *buf, uint8_t size)
{
    const uint8_t n = MIN(MIN(lastEchoCount, sizeof(lastEcho)), size);
    memcpy(buf, lastEcho, n);
    return lastEchoCount;
}

bool fbusMuxFpgaIsEnabled(void)
{
    return status != FBUS_MUX_FPGA_DISABLED;
}

bool fbusMuxFpgaIsReady(void)
{
    return status == FBUS_MUX_FPGA_READY;
}

fbusMuxFpgaStatus_e fbusMuxFpgaGetStatus(void)
{
    return status;
}

const char *fbusMuxFpgaGetStatusName(void)
{
    return statusNames[status];
}

uint32_t fbusMuxFpgaGetModeWord(void)
{
    return modeWord;
}

fbusMuxMode_e fbusMuxFpgaGetChannelMode(uint8_t channel)
{
    return (channel < FBUS_MUX_FPGA_CHANNELS) ? channelMode[channel] : FBUS_MUX_MODE_PWM;
}

#endif // USE_FBUS_MUX_FPGA

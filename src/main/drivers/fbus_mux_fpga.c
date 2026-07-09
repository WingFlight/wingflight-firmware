/*
 * This file is part of Wingflight.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "common/time.h"

#ifdef USE_FBUS_MUX_FPGA

#include "common/utils.h"

#include "drivers/bus_spi.h"
#include "drivers/fbus_mux_fpga.h"
#include "drivers/io.h"
#include "drivers/time.h"
#include "drivers/serial.h"

#include "io/serial.h"

#include "pg/fbus_mux_fpga.h"

#include "drivers/fbus_mux_ice40lp384_bitstream.h"

static extDevice_t fbusMuxFpgaDevice;
static bool fbusMuxFpgaReady;
static IO_t fbusMuxFpgaSendInPin = IO_NONE;
static serialPort_t *fbusMuxFpgaConfigUartPort;
static bool fbusMuxFpgaTxPulseActive;
static timeUs_t fbusMuxFpgaTxPulseEndUs;

static uint16_t fbusMuxFpgaBuildConfigWord(const fbusMuxFpgaConfig_t *config)
{
    uint16_t configWord = config->muxConfigWord;
    const unsigned portCount = MIN(config->muxPortCount, ARRAYLEN(config->muxPortModePwm));

    for (unsigned i = 0; i < portCount; i++) {
        if (config->muxPortModePwm[i]) {
            configWord |= (uint16_t)BIT(i);
        } else {
            configWord &= (uint16_t)~BIT(i);
        }
    }

    return configWord;
}

static uint32_t fbusMuxFpgaComputeTxDurationUs(uint16_t txBytes, uint32_t baudRate)
{
    if (baudRate == 0 || txBytes == 0) {
        return 0;
    }

    // 8N1 UART frame timing: 10 bits per byte.
    const uint64_t bits = (uint64_t)txBytes * 10ULL;
    const uint64_t durationUs = (bits * 1000000ULL + baudRate - 1U) / baudRate;
    return (uint32_t)durationUs;
}

static void fbusMuxFpgaInitRuntimeIo(const fbusMuxFpgaConfig_t *config)
{
    fbusMuxFpgaSendInPin = IO_NONE;
    fbusMuxFpgaTxPulseActive = false;
    fbusMuxFpgaTxPulseEndUs = 0;

    if (config->sendInTag) {
        IO_t sendInPin = IOGetByTag(config->sendInTag);
        if (sendInPin != IO_NONE && IOIsFreeOrPreinit(sendInPin)) {
            IOInit(sendInPin, OWNER_SYSTEM, 0);
            IOConfigGPIO(sendInPin, IOCFG_OUT_PP);
            IOLo(sendInPin);
            fbusMuxFpgaSendInPin = sendInPin;
        }
    }
}

static void fbusMuxFpgaInitConfigUart(const fbusMuxFpgaConfig_t *config)
{
    fbusMuxFpgaConfigUartPort = NULL;

    if (config->configUartPort < 0 || config->configUartBaud == 0) {
        return;
    }

    const serialPortIdentifier_e identifier = (serialPortIdentifier_e)config->configUartPort;
    if (!serialIsPortAvailable(identifier)) {
        return;
    }

    fbusMuxFpgaConfigUartPort = openSerialPort(
        identifier,
        FUNCTION_NONE,
        NULL,
        NULL,
        config->configUartBaud,
        MODE_TX,
        SERIAL_STOPBITS_1 | SERIAL_PARITY_NO | SERIAL_NOT_INVERTED | SERIAL_NOSWAP);
}

static bool fbusMuxFpgaProgramBitstream(const fbusMuxFpgaConfig_t *config)
{
    if (!spiSetBusInstance(&fbusMuxFpgaDevice, config->spiDevice)) {
        return false;
    }

    fbusMuxFpgaDevice.busType_u.spi.csnPin = IOGetByTag(config->csTag);
    if (!IOIsFreeOrPreinit(fbusMuxFpgaDevice.busType_u.spi.csnPin)) {
        return false;
    }

    IOInit(fbusMuxFpgaDevice.busType_u.spi.csnPin, OWNER_SPI_CS, 0);
    IOConfigGPIO(fbusMuxFpgaDevice.busType_u.spi.csnPin, SPI_IO_CS_CFG);
    IOHi(fbusMuxFpgaDevice.busType_u.spi.csnPin);

    IO_t cresetPin = IOGetByTag(config->cresetTag);
    if (cresetPin == IO_NONE) {
        return false;
    }

    IOInit(cresetPin, OWNER_SYSTEM, 0);
    IOConfigGPIO(cresetPin, IOCFG_OUT_PP);
    IOHi(cresetPin);

    IO_t cdonePin = IO_NONE;
    if (config->cdoneTag) {
        cdonePin = IOGetByTag(config->cdoneTag);
        if (cdonePin != IO_NONE) {
            IOInit(cdonePin, OWNER_SYSTEM, 0);
            IOConfigGPIO(cdonePin, IOCFG_IPU);
        }
    }

    spiSetClkDivisor(&fbusMuxFpgaDevice, spiCalculateDivider((uint32_t)config->spiClockKhz * 1000U));
    spiSetClkPhasePolarity(&fbusMuxFpgaDevice, true);

    // Reset iCE40 and enter SRAM programming mode.
    IOLo(cresetPin);
    delayMicroseconds(10);
    IOHi(cresetPin);
    delayMicroseconds(1200);

    IOLo(fbusMuxFpgaDevice.busType_u.spi.csnPin);

    // Initial clocks before sending the bitstream payload.
    for (int i = 0; i < 8; i++) {
        spiWrite(&fbusMuxFpgaDevice, 0x00);
    }

    uint32_t address = 0;
    for (unsigned i = 0; i < FBUS_MUX_ICE40LP384_BITSTREAM_DATA_COUNT; i++) {
        const bitstream_tuple_t *tuple = &FBUS_MUX_ICE40LP384_BITSTREAM_DATA[i];
        while (address < tuple->address && address < FBUS_MUX_ICE40LP384_BITSTREAM_SIZE_BYTES) {
            spiWrite(&fbusMuxFpgaDevice, 0x00);
            address++;
        }

        if (tuple->address < FBUS_MUX_ICE40LP384_BITSTREAM_SIZE_BYTES) {
            spiWrite(&fbusMuxFpgaDevice, tuple->data);
            address = tuple->address + 1U;
        }
    }

    while (address < FBUS_MUX_ICE40LP384_BITSTREAM_SIZE_BYTES) {
        spiWrite(&fbusMuxFpgaDevice, 0x00);
        address++;
    }

    IOHi(fbusMuxFpgaDevice.busType_u.spi.csnPin);

    // Final clocks to latch configuration.
    for (int i = 0; i < 8; i++) {
        spiWrite(&fbusMuxFpgaDevice, 0x00);
    }

    delayMicroseconds(100);

    if (cdonePin != IO_NONE && !IORead(cdonePin)) {
        return false;
    }

    return true;
}

void fbusMuxFpgaInit(void)
{
    const fbusMuxFpgaConfig_t *config = fbusMuxFpgaConfig();
    const uint16_t configWord = fbusMuxFpgaBuildConfigWord(config);
    fbusMuxFpgaReady = false;

    fbusMuxFpgaInitRuntimeIo(config);
    fbusMuxFpgaInitConfigUart(config);

    if (!config->enabled || !config->loadAtStartup) {
        fbusMuxFpgaSendConfigWord(configWord);
        return;
    }

    if (!config->csTag || !config->cresetTag || config->spiDevice == SPI_DEV_TO_CFG(SPIINVALID)) {
        fbusMuxFpgaSendConfigWord(configWord);
        return;
    }

    fbusMuxFpgaReady = fbusMuxFpgaProgramBitstream(config);
    fbusMuxFpgaSendConfigWord(configWord);
}

bool fbusMuxFpgaIsReady(void)
{
    return fbusMuxFpgaReady;
}

uint16_t fbusMuxFpgaGetConfigWord(void)
{
    return fbusMuxFpgaBuildConfigWord(fbusMuxFpgaConfig());
}

void fbusMuxFpgaSendConfigWord(uint16_t configWord)
{
    if (!fbusMuxFpgaConfigUartPort) {
        return;
    }

    if (serialTxBytesFree(fbusMuxFpgaConfigUartPort) < 2) {
        return;
    }

    uint8_t frame[2];
    frame[0] = (uint8_t)(configWord & 0xFFU);
    frame[1] = (uint8_t)((configWord >> 8) & 0xFFU);
    serialWriteBuf(fbusMuxFpgaConfigUartPort, frame, sizeof(frame));
}

void fbusMuxFpgaOnFbusTxStart(timeUs_t currentTimeUs, uint16_t txBytes, uint32_t baudRate)
{
    if (!fbusMuxFpgaSendInPin) {
        return;
    }

    IOHi(fbusMuxFpgaSendInPin);
    fbusMuxFpgaTxPulseActive = true;

    const uint32_t durationUs = fbusMuxFpgaComputeTxDurationUs(txBytes, baudRate);
    fbusMuxFpgaTxPulseEndUs = currentTimeUs + durationUs + 20U;
}

void fbusMuxFpgaOnFbusTxUpdate(timeUs_t currentTimeUs, bool txBufferEmpty)
{
    if (!fbusMuxFpgaTxPulseActive) {
        return;
    }

    if (txBufferEmpty && cmpTimeUs(currentTimeUs, fbusMuxFpgaTxPulseEndUs) >= 0) {
        IOLo(fbusMuxFpgaSendInPin);
        fbusMuxFpgaTxPulseActive = false;
    }
}

#else

void fbusMuxFpgaInit(void)
{
}

bool fbusMuxFpgaIsReady(void)
{
    return false;
}

uint16_t fbusMuxFpgaGetConfigWord(void)
{
    return 0;
}

void fbusMuxFpgaSendConfigWord(uint16_t configWord)
{
    UNUSED(configWord);
}

void fbusMuxFpgaOnFbusTxStart(timeUs_t currentTimeUs, uint16_t txBytes, uint32_t baudRate)
{
    UNUSED(currentTimeUs);
    UNUSED(txBytes);
    UNUSED(baudRate);
}

void fbusMuxFpgaOnFbusTxUpdate(timeUs_t currentTimeUs, bool txBufferEmpty)
{
    UNUSED(currentTimeUs);
    UNUSED(txBufferEmpty);
}

#endif

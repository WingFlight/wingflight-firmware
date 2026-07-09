/*
 * This file is part of Wingflight.
 */

#include "platform.h"

#ifdef USE_FBUS_MUX_FPGA

#include "common/utils.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "drivers/bus_spi.h"

#include "fbus_mux_fpga.h"

PG_REGISTER_WITH_RESET_FN(fbusMuxFpgaConfig_t, fbusMuxFpgaConfig,
                          PG_DRIVER_FBUS_MUX_FPGA_CONFIG, 0);

void pgResetFn_fbusMuxFpgaConfig(fbusMuxFpgaConfig_t *config)
{
    uint16_t defaultMuxConfigWord;
    const uint8_t portCount = constrain(FBUS_MUX_FPGA_PORT_COUNT, 1, FBUS_MUX_FPGA_MAX_PORT_COUNT);

    config->enabled = 0;
    config->loadAtStartup = 1;
    config->muxPortCount = portCount;
    config->csTag = IO_TAG(FBUS_MUX_FPGA_CS_PIN);
    config->cresetTag = IO_TAG(FBUS_MUX_FPGA_CRESET_PIN);
    config->cdoneTag = IO_TAG(FBUS_MUX_FPGA_CDONE_PIN);
    config->sendInTag = IO_TAG(FBUS_MUX_FPGA_SEND_IN_PIN);
#ifdef FBUS_MUX_FPGA_CONFIG_UART_PORT
    config->configUartPort = FBUS_MUX_FPGA_CONFIG_UART_PORT;
#else
    config->configUartPort = -1;
#endif
#ifdef FBUS_MUX_FPGA_CONFIG_WORD
    defaultMuxConfigWord = FBUS_MUX_FPGA_CONFIG_WORD;
#else
    defaultMuxConfigWord = 0;
#endif
    config->muxConfigWord = defaultMuxConfigWord;

    for (unsigned i = 0; i < ARRAYLEN(config->muxPortModePwm); i++) {
        config->muxPortModePwm[i] = (i < portCount) ? ((defaultMuxConfigWord >> i) & 0x01U) : 0;
    }

#if defined(USE_SPI) && defined(FBUS_MUX_FPGA_SPI_INSTANCE)
    config->spiDevice = SPI_DEV_TO_CFG(spiDeviceByInstance(FBUS_MUX_FPGA_SPI_INSTANCE));
#else
    config->spiDevice = SPI_DEV_TO_CFG(SPIINVALID);
#endif
    config->spiClockKhz = FBUS_MUX_FPGA_SPI_CLOCK_KHZ;
#ifdef FBUS_MUX_FPGA_CONFIG_UART_BAUD
    config->configUartBaud = FBUS_MUX_FPGA_CONFIG_UART_BAUD;
#else
    config->configUartBaud = 115200;
#endif
}

#endif

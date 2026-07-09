/*
 * This file is part of Wingflight.
 */

#pragma once

#include "drivers/io_types.h"
#include "pg/pg.h"

#define FBUS_MUX_FPGA_MAX_PORT_COUNT 16

#if FBUS_MUX_FPGA_PORT_COUNT > FBUS_MUX_FPGA_MAX_PORT_COUNT
#error "FBUS_MUX_FPGA_PORT_COUNT exceeds FBUS_MUX_FPGA_MAX_PORT_COUNT"
#endif

typedef struct fbusMuxFpgaConfig_s {
    uint8_t enabled;
    uint8_t loadAtStartup;
    uint8_t muxPortCount;
    uint8_t muxPortModePwm[FBUS_MUX_FPGA_MAX_PORT_COUNT];
    uint8_t spiDevice;
    ioTag_t csTag;
    ioTag_t cresetTag;
    ioTag_t cdoneTag;
    ioTag_t sendInTag;
    int8_t configUartPort;
    uint16_t muxConfigWord;
    uint16_t spiClockKhz;
    uint32_t configUartBaud;
} fbusMuxFpgaConfig_t;

PG_DECLARE(fbusMuxFpgaConfig_t, fbusMuxFpgaConfig);

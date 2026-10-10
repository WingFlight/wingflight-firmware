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

#pragma once

#include "pg/pg.h"
#include "drivers/io_types.h"

// Output channels routed through the FPGA (RF007-V3: S1, S2, S3, TAIL, ESC)
#define FBUS_MUX_FPGA_CHANNELS  5

typedef enum {
    FBUS_MUX_MODE_PWM = 0,      // MCU timer output passed through (any one-way protocol)
    FBUS_MUX_MODE_FBUS,         // Output carries the FBUS master line
    FBUS_MUX_MODE_DSHOT,        // Bidirectional DShot passthrough (bitstream dependent channels)
    FBUS_MUX_MODE_COUNT
} fbusMuxMode_e;

typedef struct fbusMuxFpgaConfig_s {
    ioTag_t csTag;              // SPI_SS_B of the FPGA configuration interface
    ioTag_t cresetTag;          // CRESET_B
    ioTag_t cdoneTag;           // CDONE
    uint8_t spiDevice;          // CLI bus number (1-based), 0 = none
    uint8_t serialPort;         // UART number of the mux configuration port (1-based), 0 = none
    uint8_t mode[FBUS_MUX_FPGA_CHANNELS];   // fbusMuxMode_e per channel
} fbusMuxFpgaConfig_t;

PG_DECLARE(fbusMuxFpgaConfig_t, fbusMuxFpgaConfig);

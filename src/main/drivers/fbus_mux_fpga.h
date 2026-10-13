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

#include <stdbool.h>
#include <stdint.h>

#include "pg/fbus_mux_fpga.h"

typedef enum {
    FBUS_MUX_FPGA_DISABLED = 0,     // no pins configured
    FBUS_MUX_FPGA_ERROR_RESOURCE,   // pin, SPI bus or serial port missing / in use
    FBUS_MUX_FPGA_ERROR_CDONE,      // bitstream not accepted (CDONE stayed low)
    FBUS_MUX_FPGA_ERROR_ECHO,       // configured, but the mode word was not confirmed
    FBUS_MUX_FPGA_READY,
    FBUS_MUX_FPGA_STATUS_COUNT
} fbusMuxFpgaStatus_e;

// Bitstream (drivers/fbus_mux_fpga_bitstream.c)
extern const uint8_t fbusMuxFpgaBitstream[];
extern const uint32_t fbusMuxFpgaBitstreamSize;
extern const char * const fbusMuxFpgaBitstreamInfo;
// Channels with a DShot passthrough bridge in the bitstream (bit 0 = channel 1)
extern const uint8_t fbusMuxFpgaDshotChannelMask;

// Loads the bitstream and applies the channel modes. Blocks for up to ~0.3 s.
void fbusMuxFpgaInit(void);

bool fbusMuxFpgaIsEnabled(void);
bool fbusMuxFpgaIsReady(void);

fbusMuxFpgaStatus_e fbusMuxFpgaGetStatus(void);
const char *fbusMuxFpgaGetStatusName(void);

// Mode word sent to the FPGA and the effective mode per channel
uint32_t fbusMuxFpgaGetModeWord(void);

// Diagnostics (CLI "fpga"): reload the bitstream, send a raw word, last echo received
bool fbusMuxFpgaReload(void);
bool fbusMuxFpgaSendRawWord(uint32_t word);
uint8_t fbusMuxFpgaGetLastEcho(uint8_t *buf, uint8_t size);
fbusMuxMode_e fbusMuxFpgaGetChannelMode(uint8_t channel);

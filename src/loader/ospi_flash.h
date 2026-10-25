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

#define OSPI_FLASH_BASE         0x90000000
#define OSPI_FLASH_SIZE         (2 * 1024 * 1024)
#define OSPI_FLASH_BLOCK_SIZE   (64 * 1024)
#define OSPI_FLASH_SECTOR_SIZE  (4 * 1024)

#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

// Offsets are relative to the start of the flash
bool ospiFlashInit(void);
bool ospiFlashRead(uint32_t offset, uint8_t *data, uint32_t length);
bool ospiFlashEraseBlock(uint32_t offset);
bool ospiFlashEraseSector(uint32_t offset);
bool ospiFlashWrite(uint32_t offset, const uint8_t *data, uint32_t length);
bool ospiFlashMemoryMapped(void);

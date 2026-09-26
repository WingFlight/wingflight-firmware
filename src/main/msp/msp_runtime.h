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

#include "common/streambuf.h"
#include "msp/msp.h"

// Flags in the MSP2_WING_TASK_INFO reply header.
#define MSP_TASK_INFO_FLAG_STATISTICS   (1 << 0) // system.task_statistics is on
#define MSP_TASK_INFO_FLAG_LATE_STATS   (1 << 1) // run/late/exec fields are real

// Handles the MSP2_WING_* runtime diagnostic opcodes. Returns false if cmdMSP
// is not one of them, leaving the caller to try the next handler.
bool mspRuntimeCommand(int16_t cmdMSP, sbuf_t *src, sbuf_t *dst, mspResult_e *result);

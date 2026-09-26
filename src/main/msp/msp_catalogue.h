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

// The MSP config catalogue (msp_catalogue.c), which step 5 of
// docs/parameter-addressing-design.md deletes. msp.c's dispatchers hand
// each opcode they do not handle to the matching one of these.
bool mspCatalogueProcessOutCommand(int16_t cmdMSP, sbuf_t *dst);
mspResult_e mspCatalogueProcessOutCommandWithArg(mspDescriptor_t srcDesc, int16_t cmdMSP, sbuf_t *src, sbuf_t *dst);
mspResult_e mspCatalogueProcessInCommand(mspDescriptor_t srcDesc, int16_t cmdMSP, sbuf_t *src);

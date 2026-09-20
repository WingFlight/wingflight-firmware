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

// Wire format version for the MSP2_WING_PARAM_* family. Bumped only if the
// shape of these replies changes incompatibly; the configurator refuses a
// version it does not know rather than misreading a reply.
#define MSP_PARAM_PROTOCOL_VERSION 1

// Capability bits in the MSP2_WING_BUILD_ID reply.
#define MSP_PARAM_CAP_EMBEDDED_MANIFEST (1 << 0)
// Clear while the build ID is still the placeholder, so a client can tell an
// unbound build from one whose manifest it should go and fetch.
#define MSP_PARAM_CAP_BUILD_ID_VALID    (1 << 1)

// Scratch space for materialising a parameter group's defaults. Parameter
// groups reset as a unit -- pgResetInstance() writes the whole struct -- so a
// default cannot be computed for a slice in isolation, and 53 of the groups
// compute their defaults in a reset function rather than copying a template.
//
// This cannot borrow pg->copy: cli.c uses that as a backup of the *live*
// config for the duration of a dump/diff (see backupConfigs()), so writing
// defaults into it would corrupt a CLI session running on another port.
//
// A group larger than this cannot answer MSP2_WING_PG_DEFAULT. That is a build
// error waiting to happen rather than a runtime condition, so the manifest CI
// check asserts max(pgSize) <= MSP_PARAM_STAGING_SIZE; the runtime guard below
// exists so the failure is a clean error rather than a stack smash if the two
// ever drift.
#define MSP_PARAM_STAGING_SIZE 512

// Handles the MSP2_WING_PARAM_* opcodes. Returns false if cmdMSP is not one of
// them, leaving the caller to try the next handler.
bool mspParamCommand(int16_t cmdMSP, sbuf_t *src, sbuf_t *dst, mspResult_e *result);

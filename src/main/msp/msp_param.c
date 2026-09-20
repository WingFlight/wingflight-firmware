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
 * Generic parameter addressing over MSP.
 *
 * Configuration is addressed by (pgn, offset, length) against the parameter
 * group registry rather than by a hand-written opcode per field. The registry
 * is already in the firmware -- it is how config is loaded and saved -- so
 * these six opcodes replace the per-field marshalling in msp.c and the name
 * and range tables in cli/settings.c, both of which exist only to be read by
 * something on the other end of the wire.
 *
 * The client learns what the offsets mean from a build manifest generated from
 * the ELF at build time (src/utils/wf_manifest.py), keyed to the firmware by
 * the build ID reported here. See parameter-addressing-design.md.
 *
 * This file is deliberately separate from msp.c: msp.c's config catalogue is
 * on its way out, and keeping the replacement in its own translation unit
 * means the eventual deletion is a file removal rather than surgery.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "build/version.h"

#include "common/streambuf.h"
#include "common/utils.h"

#include "fc/runtime_config.h"

#include "msp/msp.h"
#include "msp/msp_param.h"
#include "msp/msp_protocol.h"

#include "pg/pg.h"

#define WF_BUILD_ID_LENGTH 8

/*
 * Hash of the build manifest, which the client uses to pick the manifest that
 * describes this exact firmware.
 *
 * Still a placeholder: wiring the generated value into the build is a separate
 * change (the generator has to run against the linked ELF, and the result then
 * has to get back into the binary). Until then this stays zero and the reply
 * clears MSP_PARAM_CAP_BUILD_ID_VALID, so a client can tell "not bound yet"
 * from "bound to something I should go and find" instead of chasing a manifest
 * whose hash is eight zero bytes.
 */
static const uint8_t wfBuildId[WF_BUILD_ID_LENGTH] = { 0 };

#ifdef USE_EMBEDDED_MANIFEST
extern const uint8_t __wf_manifest_start[];
extern const uint8_t __wf_manifest_end[];
#endif

// See the header: this cannot borrow pg->copy, which cli.c owns for the
// duration of a dump/diff.
static uint8_t pgDefaultStaging[MSP_PARAM_STAGING_SIZE];

static bool buildIdIsValid(void)
{
    for (unsigned i = 0; i < WF_BUILD_ID_LENGTH; i++) {
        if (wfBuildId[i]) {
            return true;
        }
    }
    return false;
}

static uint32_t embeddedManifestSize(void)
{
#ifdef USE_EMBEDDED_MANIFEST
    return (uint32_t)(__wf_manifest_end - __wf_manifest_start);
#else
    return 0;
#endif
}

/*
 * Common bounds check for every addressed access.
 *
 * pgSize() is the whole group, arrays included, which is what makes element N
 * of a repeated struct addressable as (index * stride + field offset) without
 * the firmware needing to know anything about the struct. Arithmetic is in
 * uint32_t so that offset + length cannot wrap a uint16_t back into range.
 */
static const pgRegistry_t *pgFindChecked(uint16_t pgn, uint32_t offset, uint32_t length)
{
    const pgRegistry_t *reg = pgFind(pgn);
    if (!reg) {
        return NULL;
    }
    if (offset + length > pgSize(reg)) {
        return NULL;
    }
    return reg;
}

static mspResult_e mspParamRead(sbuf_t *src, sbuf_t *dst)
{
    if (sbufBytesRemaining(src) < 6) {
        return MSP_RESULT_ERROR;
    }
    const uint16_t pgn = sbufReadU16(src);
    const uint16_t offset = sbufReadU16(src);
    const uint16_t length = sbufReadU16(src);

    const pgRegistry_t *reg = pgFindChecked(pgn, offset, length);
    if (!reg) {
        return MSP_RESULT_ERROR;
    }
    if (length > (uint32_t)sbufBytesRemaining(dst)) {
        // The client is responsible for chunking to the negotiated MSP buffer
        // size; refusing is better than a truncated reply it cannot detect.
        return MSP_RESULT_ERROR;
    }

    sbufWriteData(dst, reg->address + offset, length);
    return MSP_RESULT_ACK;
}

static mspResult_e mspParamWrite(sbuf_t *src)
{
    if (sbufBytesRemaining(src) < 4) {
        return MSP_RESULT_ERROR;
    }
    const uint16_t pgn = sbufReadU16(src);
    const uint16_t offset = sbufReadU16(src);
    const uint32_t length = sbufBytesRemaining(src);

    if (ARMING_FLAG(ARMED)) {
        // Config writes while armed are how you change a mixer rule mid-flight.
        return MSP_RESULT_ERROR;
    }

    const pgRegistry_t *reg = pgFindChecked(pgn, offset, length);
    if (!reg) {
        return MSP_RESULT_ERROR;
    }

    sbufReadData(src, reg->address + offset, length);
    sbufAdvance(src, length);
    return MSP_RESULT_ACK;
}

static mspResult_e mspParamDefault(sbuf_t *src, sbuf_t *dst)
{
    if (sbufBytesRemaining(src) < 6) {
        return MSP_RESULT_ERROR;
    }
    const uint16_t pgn = sbufReadU16(src);
    const uint16_t offset = sbufReadU16(src);
    const uint16_t length = sbufReadU16(src);

    const pgRegistry_t *reg = pgFindChecked(pgn, offset, length);
    if (!reg) {
        return MSP_RESULT_ERROR;
    }
    if (pgSize(reg) > sizeof(pgDefaultStaging)) {
        // Should be unreachable -- CI asserts max(pgSize) fits. See the header.
        return MSP_RESULT_ERROR;
    }
    if (length > (uint32_t)sbufBytesRemaining(dst)) {
        return MSP_RESULT_ERROR;
    }

    // Defaults are produced a whole group at a time: 53 of the groups compute
    // them in a reset function rather than copying a template, so there is no
    // way to ask for just the requested slice.
    pgResetInstance(reg, pgDefaultStaging);
    sbufWriteData(dst, pgDefaultStaging + offset, length);
    return MSP_RESULT_ACK;
}

/*
 * The registry, paged.
 *
 * ~110 groups at 6 bytes each overflows even the larger MSP output buffer, so
 * the client asks for a starting index and is told the total. Paging on index
 * rather than byte offset keeps records whole.
 */
static mspResult_e mspParamPgList(sbuf_t *src, sbuf_t *dst)
{
    if (sbufBytesRemaining(src) < 2) {
        return MSP_RESULT_ERROR;
    }
    const uint16_t first = sbufReadU16(src);
    const uint16_t total = PG_REGISTRY_SIZE;

    if (first > total) {
        return MSP_RESULT_ERROR;
    }

    // Header is total + first + count; count is backfilled once known.
    sbufWriteU16(dst, total);
    sbufWriteU16(dst, first);
    uint8_t *countPtr = sbufPtr(dst);
    sbufWriteU8(dst, 0);

    uint8_t count = 0;
    uint16_t index = 0;
    PG_FOREACH(reg) {
        if (index++ < first) {
            continue;
        }
        if (sbufBytesRemaining(dst) < 6 || count == UINT8_MAX) {
            break;
        }
        sbufWriteU16(dst, pgN(reg));
        sbufWriteU8(dst, pgVersion(reg));
        sbufWriteU16(dst, pgSize(reg));
        sbufWriteU8(dst, reg->length);
        count++;
    }
    *countPtr = count;

    return MSP_RESULT_ACK;
}

static mspResult_e mspParamBuildId(sbuf_t *dst)
{
    uint16_t capabilities = 0;
    if (embeddedManifestSize()) {
        capabilities |= MSP_PARAM_CAP_EMBEDDED_MANIFEST;
    }
    if (buildIdIsValid()) {
        capabilities |= MSP_PARAM_CAP_BUILD_ID_VALID;
    }

    sbufWriteU8(dst, MSP_PARAM_PROTOCOL_VERSION);
    sbufWriteData(dst, wfBuildId, WF_BUILD_ID_LENGTH);
    sbufWriteU16(dst, capabilities);
    sbufWriteU32(dst, embeddedManifestSize());
    sbufWriteStringWithZeroTerminator(dst, targetName);
    sbufWriteStringWithZeroTerminator(dst, FC_VERSION_STRING);
    sbufWriteStringWithZeroTerminator(dst, shortGitRevision);

    return MSP_RESULT_ACK;
}

static mspResult_e mspParamManifestRead(sbuf_t *src, sbuf_t *dst)
{
#ifdef USE_EMBEDDED_MANIFEST
    if (sbufBytesRemaining(src) < 6) {
        return MSP_RESULT_ERROR;
    }
    const uint32_t offset = sbufReadU32(src);
    const uint16_t length = sbufReadU16(src);
    const uint32_t size = embeddedManifestSize();

    if (offset + length > size) {
        return MSP_RESULT_ERROR;
    }
    if (length > (uint32_t)sbufBytesRemaining(dst)) {
        return MSP_RESULT_ERROR;
    }

    sbufWriteData(dst, __wf_manifest_start + offset, length);
    return MSP_RESULT_ACK;
#else
    UNUSED(src);
    UNUSED(dst);
    return MSP_RESULT_ERROR;
#endif
}

bool mspParamCommand(int16_t cmdMSP, sbuf_t *src, sbuf_t *dst, mspResult_e *result)
{
    switch (cmdMSP) {
    case MSP2_WING_BUILD_ID:
        *result = mspParamBuildId(dst);
        return true;

    case MSP2_WING_PG_LIST:
        *result = mspParamPgList(src, dst);
        return true;

    case MSP2_WING_PARAM_READ:
        *result = mspParamRead(src, dst);
        return true;

    case MSP2_WING_PARAM_WRITE:
        *result = mspParamWrite(src);
        return true;

    case MSP2_WING_PG_DEFAULT:
        *result = mspParamDefault(src, dst);
        return true;

    case MSP2_WING_MANIFEST_READ:
        *result = mspParamManifestRead(src, dst);
        return true;

    default:
        return false;
    }
}

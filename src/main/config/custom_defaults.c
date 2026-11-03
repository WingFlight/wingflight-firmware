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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "common/crc.h"
#include "common/streambuf.h"

#include "config/build_id.h"
#include "config/config.h"
#include "config/custom_defaults.h"

#include "pg/pg.h"

typedef struct customDefaultsHeader_s {
    uint32_t magic;
    uint8_t formatVersion;
    uint16_t payloadLength;
    const uint8_t *buildId;
    uint16_t crc;
    const uint8_t *payload;
} customDefaultsHeader_t;

static customDefaultsStatus_e readHeader(const uint8_t *area, uint32_t areaSize, customDefaultsHeader_t *header)
{
    if (!area || areaSize < CUSTOM_DEFAULTS_HEADER_SIZE) {
        return CUSTOM_DEFAULTS_ABSENT;
    }

    sbuf_t buf = { .ptr = (uint8_t *)area, .end = (uint8_t *)area + CUSTOM_DEFAULTS_HEADER_SIZE };
    header->magic = sbufReadU32(&buf);
    header->formatVersion = sbufReadU8(&buf);
    sbufAdvance(&buf, 1); // reserved
    header->payloadLength = sbufReadU16(&buf);
    header->buildId = sbufPtr(&buf);
    sbufAdvance(&buf, WF_BUILD_ID_LENGTH);
    header->crc = sbufReadU16(&buf);
    header->payload = area + CUSTOM_DEFAULTS_HEADER_SIZE;

    if (header->magic != CUSTOM_DEFAULTS_MAGIC) {
        return CUSTOM_DEFAULTS_ABSENT;
    }
    if (header->formatVersion != CUSTOM_DEFAULTS_FORMAT_VERSION) {
        return CUSTOM_DEFAULTS_UNSUPPORTED;
    }
    if (header->payloadLength > areaSize - CUSTOM_DEFAULTS_HEADER_SIZE) {
        return CUSTOM_DEFAULTS_CORRUPT;
    }
    return CUSTOM_DEFAULTS_VALID;
}

static uint16_t areaCrc(const uint8_t *area, const customDefaultsHeader_t *header)
{
    uint16_t crc = crc16_ccitt_update(0, area, CUSTOM_DEFAULTS_CRC_COVERED_HEADER);
    return crc16_ccitt_update(crc, header->payload, header->payloadLength);
}

/*
 * Walks the records, applying each one when `apply` is set. With `apply`
 * clear this is the check that has to pass before anything is written.
 */
static bool walkRecords(const customDefaultsHeader_t *header, bool apply)
{
    sbuf_t buf = { .ptr = (uint8_t *)header->payload, .end = (uint8_t *)header->payload + header->payloadLength };

    while (sbufBytesRemaining(&buf) > 0) {
        if (sbufBytesRemaining(&buf) < CUSTOM_DEFAULTS_RECORD_HEADER_SIZE) {
            return false;
        }
        const uint16_t pgn = sbufReadU16(&buf);
        const uint16_t offset = sbufReadU16(&buf);
        const uint8_t length = sbufReadU8(&buf);
        const uint8_t flags = sbufReadU8(&buf);

        if (length == 0 || (flags & ~CUSTOM_DEFAULTS_RECORD_FLAGS_KNOWN)) {
            return false;
        }
        const bool masked = flags & CUSTOM_DEFAULTS_RECORD_MASKED;
        const uint32_t recordBytes = masked ? 2u * length : length;
        if ((uint32_t)sbufBytesRemaining(&buf) < recordBytes) {
            return false;
        }

        const pgRegistry_t *reg = pgFindSpan(pgn, offset, length);
        if (!reg) {
            return false;
        }

        const uint8_t *data = sbufPtr(&buf);
        const uint8_t *mask = masked ? data + length : NULL;
        if (apply) {
            uint8_t *target = reg->address + offset;
            for (unsigned i = 0; i < length; i++) {
                target[i] = mask ? (uint8_t)((target[i] & ~mask[i]) | (data[i] & mask[i])) : data[i];
            }
        }
        sbufAdvance(&buf, recordBytes);
    }
    return true;
}

customDefaultsStatus_e customDefaultsCheck(const uint8_t *area, uint32_t areaSize, const uint8_t *buildId)
{
    customDefaultsHeader_t header;
    const customDefaultsStatus_e status = readHeader(area, areaSize, &header);
    if (status != CUSTOM_DEFAULTS_VALID) {
        return status;
    }
    if (areaCrc(area, &header) != header.crc) {
        return CUSTOM_DEFAULTS_CORRUPT;
    }
    if (memcmp(header.buildId, buildId, WF_BUILD_ID_LENGTH) != 0) {
        return CUSTOM_DEFAULTS_OTHER_BUILD;
    }
    return walkRecords(&header, false) ? CUSTOM_DEFAULTS_VALID : CUSTOM_DEFAULTS_CORRUPT;
}

customDefaultsStatus_e customDefaultsApply(const uint8_t *area, uint32_t areaSize, const uint8_t *buildId)
{
    const customDefaultsStatus_e status = customDefaultsCheck(area, areaSize, buildId);
    if (status != CUSTOM_DEFAULTS_VALID) {
        return status;
    }
    customDefaultsHeader_t header;
    readHeader(area, areaSize, &header);
    walkRecords(&header, true);
    return CUSTOM_DEFAULTS_VALID;
}

#if defined(USE_CUSTOM_DEFAULTS_ADDRESS)
/*
 * Where the configurator finds the area in a hex: two words at a fixed address
 * (the linker scripts' .custom_defaults section), start then end, as with the
 * CLI's custom defaults. The words after `end` are new: they tell the
 * configurator to compile records rather than insert text, and for which
 * build, so it can check the manifest it compiles against is this hex's.
 */
typedef struct customDefaultsLocator_s {
    const uint8_t *end;
    uint32_t magic;
    uint8_t formatVersion;
    uint8_t reserved[3];
    uint8_t buildId[WF_BUILD_ID_LENGTH];
} customDefaultsLocator_t;

#if defined(USE_CUSTOM_DEFAULTS)
extern const uint8_t __custom_defaults_start;
extern const uint8_t __custom_defaults_end;
#define CUSTOM_DEFAULTS_START (&__custom_defaults_start)
#define CUSTOM_DEFAULTS_END (&__custom_defaults_end)
#else
#define CUSTOM_DEFAULTS_START ((const uint8_t *)0)
#define CUSTOM_DEFAULTS_END ((const uint8_t *)0)
#endif

static const uint8_t * const customDefaultsStart __attribute__ ((section(".custom_defaults_start_address"), used)) = CUSTOM_DEFAULTS_START;
static const customDefaultsLocator_t customDefaultsLocator __attribute__ ((section(".custom_defaults_end_address"), used)) = {
    .end = CUSTOM_DEFAULTS_END,
    .magic = CUSTOM_DEFAULTS_MAGIC,
    .formatVersion = CUSTOM_DEFAULTS_FORMAT_VERSION,
    .buildId = { WF_BUILD_ID_BYTES },
};
#endif

#if defined(USE_CUSTOM_DEFAULTS)
static uint32_t areaSize(void)
{
    return (uint32_t)(customDefaultsLocator.end - customDefaultsStart);
}

bool hasCustomDefaults(void)
{
    return wfBuildIdIsValid()
        && customDefaultsCheck(customDefaultsStart, areaSize(), wfBuildId) == CUSTOM_DEFAULTS_VALID;
}

bool resetConfigToCustomDefaults(void)
{
    resetConfig();
    if (wfBuildIdIsValid()) {
        customDefaultsApply(customDefaultsStart, areaSize(), wfBuildId);
    }
    return true;
}
#endif

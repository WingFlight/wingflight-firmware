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
 * Custom defaults: a board's configuration, carried in the hex, that a reset
 * to defaults lands on instead of the bare MCU defaults.
 *
 * They used to be CLI command text replayed through the on-board CLI. With the
 * CLI gone they are compiled field writes instead: the configurator runs the
 * board's config text through its own CLI when it flashes, collects the bytes
 * each command writes, and stores them as records against this build's layout.
 * The firmware only has to apply them, with the same bounds check as
 * MSP2_WING_PARAM_WRITE. See docs/parameter-addressing-design.md.
 *
 * Area layout, little-endian:
 *
 *   header  magic u32 "WFCD", format u8, reserved u8, payload length u16,
 *           build ID [8], CRC u16, reserved u16
 *   payload records, back to back:
 *           pgn u16, offset u16, length u8, flags u8, data [length],
 *           then mask [length] if flags has CUSTOM_DEFAULTS_RECORD_MASKED
 *
 * A record writes `length` bytes at `offset` into group `pgn`. A masked record
 * changes only the bits set in its mask, which is what a command like
 * `feature X` compiles to: one bit of a word whose other bits keep the reset
 * value. The CRC is CRC16-CCITT (initial value 0) over the first 16 header
 * bytes and the payload.
 *
 * Records are checked as a whole before any is applied: a build ID from
 * another layout, a bad CRC or one record out of range leaves the plain
 * defaults, never a half-applied set.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#define CUSTOM_DEFAULTS_MAGIC              0x44434657 // "WFCD"
#define CUSTOM_DEFAULTS_FORMAT_VERSION     1

#define CUSTOM_DEFAULTS_HEADER_SIZE        20
#define CUSTOM_DEFAULTS_CRC_COVERED_HEADER 16 // header bytes before the CRC
#define CUSTOM_DEFAULTS_RECORD_HEADER_SIZE 6

#define CUSTOM_DEFAULTS_RECORD_MASKED      (1 << 0)
#define CUSTOM_DEFAULTS_RECORD_FLAGS_KNOWN (CUSTOM_DEFAULTS_RECORD_MASKED)

typedef enum {
    CUSTOM_DEFAULTS_VALID = 0,
    CUSTOM_DEFAULTS_ABSENT,      // no header: an empty area, or text from an older configurator
    CUSTOM_DEFAULTS_UNSUPPORTED, // a format version this firmware does not read
    CUSTOM_DEFAULTS_OTHER_BUILD, // compiled against another config layout
    CUSTOM_DEFAULTS_CORRUPT,     // length, CRC, or a record out of range
} customDefaultsStatus_e;

// Whether `area` holds custom defaults this build can apply.
customDefaultsStatus_e customDefaultsCheck(const uint8_t *area, uint32_t areaSize, const uint8_t *buildId);

// Checks, then applies every record to the live config. Nothing is written
// unless the whole area is valid.
customDefaultsStatus_e customDefaultsApply(const uint8_t *area, uint32_t areaSize, const uint8_t *buildId);

#if defined(USE_CUSTOM_DEFAULTS)
// Whether this board's hex carries custom defaults for this build.
bool hasCustomDefaults(void);

// Resets the config, then applies the custom defaults if there are any. A
// board without them gets the plain defaults, as with the CLI's custom
// defaults, so a reset asking for custom defaults always succeeds.
bool resetConfigToCustomDefaults(void);
#endif

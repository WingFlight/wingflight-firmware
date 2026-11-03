/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stddef.h>
#include <string.h>
#include <stdint.h>

#include "platform.h"

#include "common/crc.h"
#include "common/maths.h"

#include "pg.h"

const pgRegistry_t* pgFind(pgn_t pgn)
{
    PG_FOREACH(reg) {
        if (pgN(reg) == pgn) {
            return reg;
        }
    }
    return NULL;
}

/*
 * The group a byte span lives in, or NULL if the span is not inside one. This
 * is the bounds check for everything that writes config by address: addressed
 * MSP access and custom defaults.
 *
 * pgSize() is the whole group, arrays included, which is what makes element N
 * of a repeated struct addressable as (index * stride + field offset) without
 * the firmware needing to know anything about the struct. The length is
 * compared with what is left after the offset, so no offset or length a caller
 * passes can wrap the sum back into range.
 */
const pgRegistry_t *pgFindSpan(pgn_t pgn, uint32_t offset, uint32_t length)
{
    const pgRegistry_t *reg = pgFind(pgn);
    if (!reg) {
        return NULL;
    }
    const uint32_t size = pgSize(reg);
    if (offset > size || length > size - offset) {
        return NULL;
    }
    return reg;
}

static uint8_t *pgOffset(const pgRegistry_t* reg)
{
    return reg->address;
}

void pgResetInstance(const pgRegistry_t *reg, uint8_t *base)
{
    const uint16_t regSize = pgSize(reg);

    memset(base, 0, regSize);
    if (reg->reset.ptr >= (void*)__pg_resetdata_start && reg->reset.ptr < (void*)__pg_resetdata_end) {
        // pointer points to resetdata section, to it is data template
        memcpy(base, reg->reset.ptr, regSize);
    } else if (reg->reset.fn) {
        // reset function, call it
        reg->reset.fn(base);
    }
}

void pgReset(const pgRegistry_t* reg)
{
    pgResetInstance(reg, pgOffset(reg));
}

bool pgResetCopy(void *copy, pgn_t pgn)
{
    const pgRegistry_t *reg = pgFind(pgn);
    if (reg) {
        pgResetInstance(reg, copy);
        return true;
    }
    return false;
}

bool pgLoad(const pgRegistry_t* reg, const void *from, int size, int version)
{
    pgResetInstance(reg, pgOffset(reg));
    // restore only matching version, keep defaults otherwise
    if (version == pgVersion(reg)) {
        const int take = MIN(size, pgSize(reg));
        memcpy(pgOffset(reg), from, take);

        *reg->fnv_hash = fnv_update(FNV_OFFSET_BASIS, from, take);

        return true;
    }

    return false;
}

int pgStore(const pgRegistry_t* reg, void *to, int size)
{
    const int take = MIN(size, pgSize(reg));
    memcpy(to, pgOffset(reg), take);
    return take;
}

void pgResetAll(void)
{
    PG_FOREACH(reg) {
        pgReset(reg);
    }
}

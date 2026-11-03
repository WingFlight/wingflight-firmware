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
 * Custom defaults as compiled field writes (config/custom_defaults.c).
 *
 * The area comes out of a hex the configurator wrote, so nothing in it is
 * trusted: every way a record can point outside its group has to be refused,
 * and a refusal must leave the config exactly as the reset left it.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <vector>

extern "C" {
    #include "platform.h"

    #include "common/crc.h"
    #include "config/build_id.h"
    #include "config/custom_defaults.h"
    #include "pg/pg.h"
    #include "pg/pg_ids.h"

    // A plain group, an array group, and a run of bytes for bit-level writes.
    typedef struct testConfig_s {
        uint8_t  a;
        int16_t  b;
        uint32_t c;
    } testConfig_t;

    typedef struct testProfile_s {
        uint16_t p;
        int8_t   q;
    } testProfile_t;

    typedef struct testBytes_s {
        uint8_t bytes[8];
    } testBytes_t;

    // PG_DECLARE defines every accessor; clang reports the ones a test does
    // not call.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
    PG_DECLARE(testConfig_t, testConfig);
    PG_DECLARE_ARRAY(testProfile_t, 4, testProfiles);
    PG_DECLARE(testBytes_t, testBytes);
#pragma GCC diagnostic pop

    PG_REGISTER_WITH_RESET_TEMPLATE(testConfig_t, testConfig, PG_RESERVED_FOR_TESTING_1, 0);
    PG_RESET_TEMPLATE(testConfig_t, testConfig,
        .a = 7,
        .b = -1234,
        .c = 0xDEADBEEF
    );

    PG_REGISTER_ARRAY(testProfile_t, 4, testProfiles, PG_RESERVED_FOR_TESTING_2, 0);

    PG_REGISTER_WITH_RESET_TEMPLATE(testBytes_t, testBytes, PG_RESERVED_FOR_TESTING_3, 0);
    PG_RESET_TEMPLATE(testBytes_t, testBytes,
        .bytes = { 0x10, 0x11, 0x12, 0x12, 0x14, 0x15, 0x16, 0x17 }
    );
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

static const uint8_t BUILD_ID[WF_BUILD_ID_LENGTH] = { 1, 2, 3, 4, 5, 6, 7, 8 };
static const uint8_t OTHER_BUILD_ID[WF_BUILD_ID_LENGTH] = { 8, 7, 6, 5, 4, 3, 2, 1 };

/*
 * The vector the configurator's compiler is checked against too
 * (src/js/param/custom_defaults.selftest.mjs): one plain record setting byte 0
 * of PG_RESERVED_FOR_TESTING_3 to 0x2a, and one masked record setting bit 0 of
 * byte 3 and bit 7 of byte 6, for build 01..08.
 */
static const uint8_t SHARED_VECTOR[] = {
    0x57, 0x46, 0x43, 0x44, 0x01, 0x00, 0x15, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06,
    0x07, 0x08, 0x30, 0xe1, 0x00, 0x00, 0xfd, 0x0f, 0x00, 0x00, 0x01, 0x00, 0x2a, 0xfd,
    0x0f, 0x03, 0x00, 0x04, 0x01, 0x01, 0x00, 0x00, 0x80, 0x01, 0x00, 0x00, 0x80,
};

/** Builds an area the way the configurator does. */
class Area
{
public:
    std::vector<uint8_t> payload;

    Area &record(uint16_t pgn, uint16_t offset, std::vector<uint8_t> data, std::vector<uint8_t> mask = {})
    {
        put16(payload, pgn);
        put16(payload, offset);
        payload.push_back((uint8_t)data.size());
        payload.push_back(mask.empty() ? 0 : CUSTOM_DEFAULTS_RECORD_MASKED);
        payload.insert(payload.end(), data.begin(), data.end());
        payload.insert(payload.end(), mask.begin(), mask.end());
        return *this;
    }

    std::vector<uint8_t> bytes(const uint8_t *buildId = BUILD_ID, uint8_t version = CUSTOM_DEFAULTS_FORMAT_VERSION) const
    {
        std::vector<uint8_t> out;
        put32(out, CUSTOM_DEFAULTS_MAGIC);
        out.push_back(version);
        out.push_back(0);
        put16(out, (uint16_t)payload.size());
        out.insert(out.end(), buildId, buildId + WF_BUILD_ID_LENGTH);
        uint16_t crc = crc16_ccitt_update(0, out.data(), CUSTOM_DEFAULTS_CRC_COVERED_HEADER);
        crc = crc16_ccitt_update(crc, payload.data(), payload.size());
        put16(out, crc);
        put16(out, 0);
        out.insert(out.end(), payload.begin(), payload.end());
        return out;
    }

private:
    static void put16(std::vector<uint8_t> &out, uint16_t value)
    {
        out.push_back(value & 0xff);
        out.push_back(value >> 8);
    }

    static void put32(std::vector<uint8_t> &out, uint32_t value)
    {
        put16(out, value & 0xffff);
        put16(out, value >> 16);
    }
};

class CustomDefaultsTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        pgResetAll();
    }

    static customDefaultsStatus_e apply(const std::vector<uint8_t> &area, const uint8_t *buildId = BUILD_ID)
    {
        return customDefaultsApply(area.data(), area.size(), buildId);
    }

    static void expectUntouched()
    {
        EXPECT_EQ(7, testConfig()->a);
        EXPECT_EQ(-1234, testConfig()->b);
        EXPECT_EQ(0xDEADBEEF, testConfig()->c);
        EXPECT_EQ(0x10, testBytes()->bytes[0]);
    }
};

// --- the shared vector ----------------------------------------------------------

TEST_F(CustomDefaultsTest, BuildsTheSharedVector)
{
    Area area;
    area.record(PG_RESERVED_FOR_TESTING_3, 0, { 0x2a })
        .record(PG_RESERVED_FOR_TESTING_3, 3, { 0x01, 0x00, 0x00, 0x80 }, { 0x01, 0x00, 0x00, 0x80 });

    const std::vector<uint8_t> expected(SHARED_VECTOR, SHARED_VECTOR + sizeof(SHARED_VECTOR));
    EXPECT_EQ(expected, area.bytes());
}

TEST_F(CustomDefaultsTest, AppliesTheSharedVector)
{
    ASSERT_EQ(CUSTOM_DEFAULTS_VALID, customDefaultsApply(SHARED_VECTOR, sizeof(SHARED_VECTOR), BUILD_ID));

    EXPECT_EQ(0x2a, testBytes()->bytes[0]);
    EXPECT_EQ(0x11, testBytes()->bytes[1]);
    EXPECT_EQ(0x13, testBytes()->bytes[3]); // bit 0 set
    EXPECT_EQ(0x14, testBytes()->bytes[4]); // mask 0: kept
    EXPECT_EQ(0x96, testBytes()->bytes[6]); // bit 7 set
    EXPECT_EQ(0x17, testBytes()->bytes[7]);
}

// --- applying -------------------------------------------------------------------

TEST_F(CustomDefaultsTest, WritesFieldsAndArrayElements)
{
    Area area;
    area.record(PG_RESERVED_FOR_TESTING_1, offsetof(testConfig_t, b), { 0x39, 0x30 }) // 12345
        .record(PG_RESERVED_FOR_TESTING_2, 2 * sizeof(testProfile_t) + offsetof(testProfile_t, q), { 0xfb }); // element 2

    ASSERT_EQ(CUSTOM_DEFAULTS_VALID, apply(area.bytes()));
    EXPECT_EQ(12345, testConfig()->b);
    EXPECT_EQ(7, testConfig()->a);
    EXPECT_EQ(-5, testProfiles(2)->q);
    EXPECT_EQ(0, testProfiles(1)->q);
}

TEST_F(CustomDefaultsTest, AMaskedRecordClearsBitsToo)
{
    Area area;
    area.record(PG_RESERVED_FOR_TESTING_3, 2, { 0x00 }, { 0x02 }); // clear bit 1 of 0x12

    ASSERT_EQ(CUSTOM_DEFAULTS_VALID, apply(area.bytes()));
    EXPECT_EQ(0x10, testBytes()->bytes[2]);
}

TEST_F(CustomDefaultsTest, AnEmptyPayloadIsValidAndChangesNothing)
{
    EXPECT_EQ(CUSTOM_DEFAULTS_VALID, apply(Area().bytes()));
    expectUntouched();
}

// --- refusing -------------------------------------------------------------------

TEST_F(CustomDefaultsTest, AnEmptyOrTextAreaIsAbsent)
{
    EXPECT_EQ(CUSTOM_DEFAULTS_ABSENT, customDefaultsCheck(NULL, 0, BUILD_ID));

    std::vector<uint8_t> blank(64, 0xff);
    EXPECT_EQ(CUSTOM_DEFAULTS_ABSENT, apply(blank));

    // What an older configurator inserted: CLI text.
    const char *text = "## Wingflight Custom Defaults\nset gyro_lpf1_static_hz = 250\n";
    std::vector<uint8_t> textArea(text, text + strlen(text));
    EXPECT_EQ(CUSTOM_DEFAULTS_ABSENT, apply(textArea));

    std::vector<uint8_t> tooShort(Area().bytes());
    tooShort.pop_back();
    EXPECT_EQ(CUSTOM_DEFAULTS_ABSENT, apply(tooShort));
}

TEST_F(CustomDefaultsTest, RefusesAnotherFormatVersion)
{
    Area area;
    area.record(PG_RESERVED_FOR_TESTING_1, 0, { 99 });

    EXPECT_EQ(CUSTOM_DEFAULTS_UNSUPPORTED, apply(area.bytes(BUILD_ID, CUSTOM_DEFAULTS_FORMAT_VERSION + 1)));
    expectUntouched();
}

TEST_F(CustomDefaultsTest, RefusesRecordsForAnotherLayout)
{
    Area area;
    area.record(PG_RESERVED_FOR_TESTING_1, 0, { 99 });

    EXPECT_EQ(CUSTOM_DEFAULTS_OTHER_BUILD, apply(area.bytes(OTHER_BUILD_ID)));
    expectUntouched();
}

TEST_F(CustomDefaultsTest, RefusesABadCrc)
{
    Area area;
    area.record(PG_RESERVED_FOR_TESTING_1, 0, { 99 });
    std::vector<uint8_t> bytes = area.bytes();
    bytes.back() ^= 0x01;

    EXPECT_EQ(CUSTOM_DEFAULTS_CORRUPT, apply(bytes));
    expectUntouched();
}

TEST_F(CustomDefaultsTest, RefusesAPayloadLongerThanTheArea)
{
    Area area;
    area.record(PG_RESERVED_FOR_TESTING_1, 0, { 99 });
    std::vector<uint8_t> bytes = area.bytes();
    bytes.pop_back();

    EXPECT_EQ(CUSTOM_DEFAULTS_CORRUPT, apply(bytes));
    expectUntouched();
}

TEST_F(CustomDefaultsTest, OneRecordOutOfRangeRefusesThemAll)
{
    Area area;
    area.record(PG_RESERVED_FOR_TESTING_1, 0, { 99 })                       // valid, and first
        .record(PG_RESERVED_FOR_TESTING_3, sizeof(testBytes_t) - 1, { 1, 2 }); // one byte past the end

    EXPECT_EQ(CUSTOM_DEFAULTS_CORRUPT, apply(area.bytes()));
    expectUntouched();
}

TEST_F(CustomDefaultsTest, RefusesAnUnknownGroup)
{
    Area area;
    area.record(PG_RESERVED_FOR_TESTING_1 - 10, 0, { 1 });

    EXPECT_EQ(CUSTOM_DEFAULTS_CORRUPT, apply(area.bytes()));
}

TEST_F(CustomDefaultsTest, RefusesMalformedRecords)
{
    Area empty;
    empty.record(PG_RESERVED_FOR_TESTING_1, 0, {});
    EXPECT_EQ(CUSTOM_DEFAULTS_CORRUPT, apply(empty.bytes()));

    Area unknownFlag;
    unknownFlag.record(PG_RESERVED_FOR_TESTING_1, 0, { 1 });
    unknownFlag.payload[5] = 0x80;
    EXPECT_EQ(CUSTOM_DEFAULTS_CORRUPT, apply(unknownFlag.bytes()));

    Area truncated;
    truncated.record(PG_RESERVED_FOR_TESTING_1, 0, { 1, 2 });
    truncated.payload.pop_back();
    EXPECT_EQ(CUSTOM_DEFAULTS_CORRUPT, apply(truncated.bytes()));

    Area shortHeader;
    shortHeader.payload = { 0xff, 0x0f, 0x00 };
    EXPECT_EQ(CUSTOM_DEFAULTS_CORRUPT, apply(shortHeader.bytes()));

    expectUntouched();
}

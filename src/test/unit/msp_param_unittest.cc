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
 * Generic parameter addressing (msp/msp_param.c).
 *
 * These opcodes let a client read and write arbitrary byte ranges of the
 * configuration. The only thing standing between a malformed request and the
 * rest of RAM is the bounds check, so that is what most of this file is about:
 * every way a request can be out of range is tried, and has to be refused.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

extern "C" {
    #include "platform.h"

    // build/version.h is deliberately not included: it carries a
    // STATIC_ASSERT that needs common/utils.h pulled in first, and all this
    // file needs from it are the three strings msp_param.c reads, which are
    // defined below.
    #include "common/streambuf.h"
    #include "fc/runtime_config.h"
    #include "msp/msp.h"
    #include "msp/msp_param.h"
    #include "msp/msp_protocol.h"
    #include "pg/pg.h"
    #include "pg/pg_ids.h"

    // Two groups to address: one plain, one an array, so that the element
    // arithmetic a client does is exercised against a real registry entry.
    typedef struct testConfig_s {
        uint8_t  a;
        int16_t  b;
        uint32_t c;
    } testConfig_t;

    typedef struct testProfile_s {
        uint16_t p;
        int8_t   q;
    } testProfile_t;

    PG_DECLARE(testConfig_t, testConfig);
    PG_DECLARE_ARRAY(testProfile_t, 4, testProfiles);

    PG_REGISTER_WITH_RESET_TEMPLATE(testConfig_t, testConfig, PG_RESERVED_FOR_TESTING_1, 3);
    PG_RESET_TEMPLATE(testConfig_t, testConfig,
        .a = 7,
        .b = -1234,
        .c = 0xDEADBEEF
    );

    PG_REGISTER_ARRAY(testProfile_t, 4, testProfiles, PG_RESERVED_FOR_TESTING_2, 1);

    // `extern` is required: this file is compiled as C++, where a const object
    // at namespace scope has internal linkage by default and msp_param.c would
    // not find it. extern "C" fixes the name mangling, not the linkage.
    extern const char * const targetName;
    extern const char * const shortGitRevision;

    const char * const targetName = "TESTTARGET";
    const char * const shortGitRevision = "abc1234";

    uint8_t armingFlags;
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

static const uint16_t TEST_PGN = PG_RESERVED_FOR_TESTING_1;
static const uint16_t ARRAY_PGN = PG_RESERVED_FOR_TESTING_2;

class MspParamTest : public ::testing::Test
{
protected:
    uint8_t requestBuffer[128];
    uint8_t replyBuffer[256];
    sbuf_t request;
    sbuf_t reply;

    void SetUp() override
    {
        armingFlags = 0;
        pgResetAll();
        memset(requestBuffer, 0, sizeof(requestBuffer));
        memset(replyBuffer, 0, sizeof(replyBuffer));
    }

    /** Build a request payload and dispatch it, returning the MSP result. */
    mspResult_e call(int16_t cmd, const uint8_t *payload, size_t length)
    {
        memcpy(requestBuffer, payload, length);
        sbufInit(&request, requestBuffer, requestBuffer + length);
        sbufInit(&reply, replyBuffer, replyBuffer + sizeof(replyBuffer));

        mspResult_e result = MSP_RESULT_ERROR;
        EXPECT_TRUE(mspParamCommand(cmd, &request, &reply, &result));
        return result;
    }

    size_t replyLength() const { return reply.ptr - replyBuffer; }

    static void put16(uint8_t *at, uint16_t value)
    {
        at[0] = value & 0xff;
        at[1] = (value >> 8) & 0xff;
    }
};

// --- dispatch ---------------------------------------------------------------

TEST_F(MspParamTest, IgnoresOpcodesItDoesNotOwn)
{
    sbufInit(&request, requestBuffer, requestBuffer);
    sbufInit(&reply, replyBuffer, replyBuffer + sizeof(replyBuffer));
    mspResult_e result = MSP_RESULT_ERROR;

    // Must decline rather than claim it, or it would shadow the real handler.
    EXPECT_FALSE(mspParamCommand(MSP_API_VERSION, &request, &reply, &result));
}

// --- reading ----------------------------------------------------------------

TEST_F(MspParamTest, ReadsAWholeGroup)
{
    uint8_t payload[6];
    put16(payload + 0, TEST_PGN);
    put16(payload + 2, 0);
    put16(payload + 4, sizeof(testConfig_t));

    EXPECT_EQ(MSP_RESULT_ACK, call(MSP2_WING_PARAM_READ, payload, sizeof(payload)));
    EXPECT_EQ(sizeof(testConfig_t), replyLength());
    EXPECT_EQ(0, memcmp(replyBuffer, testConfig(), sizeof(testConfig_t)));
}

TEST_F(MspParamTest, ReadsOneFieldByOffset)
{
    testConfigMutable()->b = -4321;

    uint8_t payload[6];
    put16(payload + 0, TEST_PGN);
    put16(payload + 2, offsetof(testConfig_t, b));
    put16(payload + 4, sizeof(int16_t));

    EXPECT_EQ(MSP_RESULT_ACK, call(MSP2_WING_PARAM_READ, payload, sizeof(payload)));
    ASSERT_EQ(sizeof(int16_t), replyLength());

    int16_t value;
    memcpy(&value, replyBuffer, sizeof(value));
    EXPECT_EQ(-4321, value);
}

TEST_F(MspParamTest, ReadsAnElementOfAnArrayGroup)
{
    // Element 2 of a 4-element group: exactly the (index * stride + offset)
    // arithmetic a client performs, checked against the real layout.
    testProfilesMutable(2)->p = 0xBEEF;

    const uint16_t stride = sizeof(testProfile_t);
    uint8_t payload[6];
    put16(payload + 0, ARRAY_PGN);
    put16(payload + 2, 2 * stride + offsetof(testProfile_t, p));
    put16(payload + 4, sizeof(uint16_t));

    EXPECT_EQ(MSP_RESULT_ACK, call(MSP2_WING_PARAM_READ, payload, sizeof(payload)));
    ASSERT_EQ(sizeof(uint16_t), replyLength());

    uint16_t value;
    memcpy(&value, replyBuffer, sizeof(value));
    EXPECT_EQ(0xBEEF, value);
}

// --- the bounds check -------------------------------------------------------

TEST_F(MspParamTest, RefusesAnUnknownGroup)
{
    // Assert the premise rather than assume it: picking a pgn that turns out
    // to be registered would make this test pass for the wrong reason.
    ASSERT_TRUE(pgFind(PG_RESERVED_FOR_TESTING_3) == NULL);

    uint8_t payload[6];
    put16(payload + 0, PG_RESERVED_FOR_TESTING_3);
    put16(payload + 2, 0);
    put16(payload + 4, 1);

    EXPECT_EQ(MSP_RESULT_ERROR, call(MSP2_WING_PARAM_READ, payload, sizeof(payload)));
    EXPECT_EQ(0u, replyLength());
}

TEST_F(MspParamTest, RefusesReadingPastTheEndOfAGroup)
{
    uint8_t payload[6];
    put16(payload + 0, TEST_PGN);
    put16(payload + 2, 0);
    put16(payload + 4, sizeof(testConfig_t) + 1);

    EXPECT_EQ(MSP_RESULT_ERROR, call(MSP2_WING_PARAM_READ, payload, sizeof(payload)));
}

TEST_F(MspParamTest, RefusesAnOffsetAtTheEndOfAGroup)
{
    uint8_t payload[6];
    put16(payload + 0, TEST_PGN);
    put16(payload + 2, sizeof(testConfig_t));
    put16(payload + 4, 1);

    EXPECT_EQ(MSP_RESULT_ERROR, call(MSP2_WING_PARAM_READ, payload, sizeof(payload)));
}

TEST_F(MspParamTest, RefusesAnOffsetThatWouldWrapAround)
{
    // offset + length overflows 16 bits. If the check were done in uint16_t it
    // would wrap back to a small number and pass, handing out whatever follows
    // the group in RAM.
    uint8_t payload[6];
    put16(payload + 0, TEST_PGN);
    put16(payload + 2, 0xFFFF);
    put16(payload + 4, 2);

    EXPECT_EQ(MSP_RESULT_ERROR, call(MSP2_WING_PARAM_READ, payload, sizeof(payload)));
}

TEST_F(MspParamTest, RefusesATruncatedRequest)
{
    uint8_t payload[3] = { 0, 0, 0 };
    EXPECT_EQ(MSP_RESULT_ERROR, call(MSP2_WING_PARAM_READ, payload, sizeof(payload)));
}

// --- writing ----------------------------------------------------------------

TEST_F(MspParamTest, WritesOneField)
{
    uint8_t payload[6];
    put16(payload + 0, TEST_PGN);
    put16(payload + 2, offsetof(testConfig_t, b));
    put16(payload + 4, (uint16_t)-999);

    EXPECT_EQ(MSP_RESULT_ACK, call(MSP2_WING_PARAM_WRITE, payload, sizeof(payload)));
    EXPECT_EQ(-999, testConfig()->b);
}

TEST_F(MspParamTest, WritingDoesNotDisturbNeighbours)
{
    const uint8_t a = testConfig()->a;
    const uint32_t c = testConfig()->c;

    uint8_t payload[6];
    put16(payload + 0, TEST_PGN);
    put16(payload + 2, offsetof(testConfig_t, b));
    put16(payload + 4, 0x1234);

    EXPECT_EQ(MSP_RESULT_ACK, call(MSP2_WING_PARAM_WRITE, payload, sizeof(payload)));
    EXPECT_EQ(0x1234, testConfig()->b);
    EXPECT_EQ(a, testConfig()->a);
    EXPECT_EQ(c, testConfig()->c);
}

TEST_F(MspParamTest, RefusesAWriteThatRunsPastTheGroup)
{
    const int16_t before = testConfig()->b;

    uint8_t payload[4 + sizeof(testConfig_t) + 1];
    memset(payload, 0xAA, sizeof(payload));
    put16(payload + 0, TEST_PGN);
    put16(payload + 2, 0);

    EXPECT_EQ(MSP_RESULT_ERROR, call(MSP2_WING_PARAM_WRITE, payload, sizeof(payload)));
    EXPECT_EQ(before, testConfig()->b);
}

TEST_F(MspParamTest, RefusesToWriteWhileArmed)
{
    const int16_t before = testConfig()->b;
    armingFlags = ARMED;

    uint8_t payload[6];
    put16(payload + 0, TEST_PGN);
    put16(payload + 2, offsetof(testConfig_t, b));
    put16(payload + 4, 0x4242);

    EXPECT_EQ(MSP_RESULT_ERROR, call(MSP2_WING_PARAM_WRITE, payload, sizeof(payload)));
    EXPECT_EQ(before, testConfig()->b);
}

// --- defaults ---------------------------------------------------------------

TEST_F(MspParamTest, ReportsDefaultsWithoutChangingTheLiveValue)
{
    testConfigMutable()->b = 31;

    uint8_t payload[6];
    put16(payload + 0, TEST_PGN);
    put16(payload + 2, offsetof(testConfig_t, b));
    put16(payload + 4, sizeof(int16_t));

    EXPECT_EQ(MSP_RESULT_ACK, call(MSP2_WING_PG_DEFAULT, payload, sizeof(payload)));
    ASSERT_EQ(sizeof(int16_t), replyLength());

    int16_t value;
    memcpy(&value, replyBuffer, sizeof(value));
    EXPECT_EQ(-1234, value);       // from the reset template
    EXPECT_EQ(31, testConfig()->b); // and the live value is untouched
}

TEST_F(MspParamTest, RefusesDefaultsPastTheEndOfAGroup)
{
    uint8_t payload[6];
    put16(payload + 0, TEST_PGN);
    put16(payload + 2, sizeof(testConfig_t) - 1);
    put16(payload + 4, 4);

    EXPECT_EQ(MSP_RESULT_ERROR, call(MSP2_WING_PG_DEFAULT, payload, sizeof(payload)));
}

// --- the registry -----------------------------------------------------------

TEST_F(MspParamTest, ListsTheRegistryAndAgreesWithIt)
{
    uint8_t payload[2];
    put16(payload, 0);

    EXPECT_EQ(MSP_RESULT_ACK, call(MSP2_WING_PG_LIST, payload, sizeof(payload)));
    ASSERT_GE(replyLength(), 5u);

    const uint16_t total = replyBuffer[0] | (replyBuffer[1] << 8);
    const uint16_t first = replyBuffer[2] | (replyBuffer[3] << 8);
    const uint8_t count = replyBuffer[4];

    EXPECT_EQ(PG_REGISTRY_SIZE, total);
    EXPECT_EQ(0, first);
    EXPECT_GT(count, 0);
    EXPECT_EQ(5u + count * 6u, replyLength());

    // Every record must match the registry it claims to describe.
    for (unsigned i = 0; i < count; i++) {
        const uint8_t *record = replyBuffer + 5 + i * 6;
        const uint16_t pgn = record[0] | (record[1] << 8);
        const pgRegistry_t *reg = pgFind(pgn);
        ASSERT_TRUE(reg != NULL) << "pgn " << pgn << " is not in the registry";
        EXPECT_EQ(pgVersion(reg), record[2]);
        EXPECT_EQ(pgSize(reg), (uint16_t)(record[3] | (record[4] << 8)));
        EXPECT_EQ(reg->length, record[5]);
    }
}

TEST_F(MspParamTest, RefusesARegistryPageStartingPastTheEnd)
{
    uint8_t payload[2];
    put16(payload, PG_REGISTRY_SIZE + 1);

    EXPECT_EQ(MSP_RESULT_ERROR, call(MSP2_WING_PG_LIST, payload, sizeof(payload)));
}

// --- build id ---------------------------------------------------------------

TEST_F(MspParamTest, ReportsTheBuildIdAndItsValidity)
{
    EXPECT_EQ(MSP_RESULT_ACK, call(MSP2_WING_BUILD_ID, NULL, 0));
    ASSERT_GE(replyLength(), 15u);

    EXPECT_EQ(MSP_PARAM_PROTOCOL_VERSION, replyBuffer[0]);

    bool anyIdByteSet = false;
    for (int i = 1; i <= 8; i++) {
        anyIdByteSet = anyIdByteSet || replyBuffer[i];
    }
    const uint16_t capabilities = replyBuffer[9] | (replyBuffer[10] << 8);

    // The valid bit must agree with the bytes. A test build has no generated
    // build id, so this says "not bound to a manifest" rather than offering
    // eight zero bytes as though they meant something.
    EXPECT_EQ(anyIdByteSet, (capabilities & MSP_PARAM_CAP_BUILD_ID_VALID) != 0);

    // targetName follows the fixed header.
    EXPECT_STREQ("TESTTARGET", (const char *)(replyBuffer + 15));
}

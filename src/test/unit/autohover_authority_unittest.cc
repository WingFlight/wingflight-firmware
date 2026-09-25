#include <cstdint>
#include "gtest/gtest.h"

extern "C" {
#include "flight/autohover_authority.h"
}

// Output authority the yaw mixer input can reach = (clamped input) * rate / 1000, and the clamp is
// widened by autoHoverYawAuthorityScale(rate). For default min/max (+-1000) that must come out at
// exactly full authority whatever the rate, and never less than the normal path allows.

TEST(AutoHoverAuthorityTest, UnityRateIsUnchanged)
{
    EXPECT_FLOAT_EQ(1.0f, autoHoverYawAuthorityScale(1000));
}

TEST(AutoHoverAuthorityTest, ReducedRateWidensClampSoOutputReachesFullTravel)
{
    // The configuration the hover blackbox was flown with: SY rate 350.
    const float scale = autoHoverYawAuthorityScale(350);
    EXPECT_FLOAT_EQ(1000.0f / 350.0f, scale);

    const float clampedInput = 1.0f * scale;            // input min/max +-1000 -> +-1.0, widened
    const float output = clampedInput * 350 / 1000.0f;  // mixerUpdateRules(): input * rate / 1000
    EXPECT_NEAR(1.0f, output, 1e-6f);
}

TEST(AutoHoverAuthorityTest, NormalPathCeilingWasRateNotFullTravel)
{
    // Documents the behaviour being changed: without the widening, the same input clamp tops out at
    // rate/1000 (35%) of full travel.
    const float unwidenedOutput = 1.0f * 350 / 1000.0f;
    EXPECT_NEAR(0.35f, unwidenedOutput, 1e-6f);
    EXPECT_GT(1.0f, unwidenedOutput);
}

TEST(AutoHoverAuthorityTest, InputBelowTheOldCeilingIsNotClampedByEitherPath)
{
    // A PID output that the old clamp already passed through (|x| <= 1.0) is far inside the widened
    // clamp too, so the linear region -- and therefore the loop gain -- is identical.
    const float widenedMax = 1.0f * autoHoverYawAuthorityScale(350);
    for (float x = -1.0f; x <= 1.0f; x += 0.125f) {
        EXPECT_LE(x, widenedMax);
        EXPECT_GE(x, -widenedMax);
    }
}

TEST(AutoHoverAuthorityTest, NegativeRateWidensByItsMagnitude)
{
    EXPECT_FLOAT_EQ(autoHoverYawAuthorityScale(350), autoHoverYawAuthorityScale(-350));
    EXPECT_FLOAT_EQ(1000.0f / 350.0f, autoHoverYawAuthorityScale(-350));
}

TEST(AutoHoverAuthorityTest, ZeroRateStaysDisabled)
{
    // rate 0 disables the axis; there is nothing to divide by and nothing to widen.
    EXPECT_FLOAT_EQ(1.0f, autoHoverYawAuthorityScale(0));
}

TEST(AutoHoverAuthorityTest, RatesAtOrAboveUnityAreNotNarrowed)
{
    EXPECT_FLOAT_EQ(1.0f, autoHoverYawAuthorityScale(1000));
    EXPECT_FLOAT_EQ(1.0f, autoHoverYawAuthorityScale(1500));
    EXPECT_FLOAT_EQ(1.0f, autoHoverYawAuthorityScale(INT16_MAX));
    EXPECT_FLOAT_EQ(1.0f, autoHoverYawAuthorityScale(-1000));
    EXPECT_FLOAT_EQ(1.0f, autoHoverYawAuthorityScale(INT16_MIN)); // -(-32768) must not overflow
}

TEST(AutoHoverAuthorityTest, ScaleIsNeverBelowOneForAnyInt16Rate)
{
    for (int r = INT16_MIN; r <= INT16_MAX; r++) {
        ASSERT_GE(autoHoverYawAuthorityScale((int16_t)r), 1.0f) << "rate " << r;
    }
}

TEST(AutoHoverAuthorityTest, ReachableOutputIsFullTravelForEveryNarrowingRate)
{
    // For every rate the widening applies to (0 < |rate| < 1000), clamp * rate / 1000 lands on full
    // travel: not below it (would leave authority on the table) and not above it (would exceed the
    // limit min/max define).
    for (int r = -999; r <= 999; r++) {
        if (r == 0) {
            continue;
        }
        const float magnitude = (float)(r < 0 ? -r : r);
        const float output = 1.0f * autoHoverYawAuthorityScale((int16_t)r) * magnitude / 1000.0f;
        ASSERT_NEAR(1.0f, output, 1e-5f) << "rate " << r;
    }
}

TEST(AutoHoverAuthorityTest, SmallestRatesGiveFiniteScales)
{
    EXPECT_FLOAT_EQ(1000.0f, autoHoverYawAuthorityScale(1));
    EXPECT_FLOAT_EQ(1000.0f, autoHoverYawAuthorityScale(-1));
    EXPECT_GT(autoHoverYawAuthorityScale(999), 1.0f);
}

/*
 * This file is part of Rotorflight.
 *
 * Rotorflight is free software. You can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Rotorflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software. If not, see <https://www.gnu.org/licenses/>.
 */

#include <cmath>

extern "C" {
    #include "common/curve.h"
}

#include "gtest/gtest.h"

TEST(CurveTest, DisabledCurveReturnsFallback)
{
    const curvePoint_t points[2] = { { -1000, -1000 }, { 1000, 1000 } };

    EXPECT_FLOAT_EQ(evaluateCurvePoints(points, 0, 123.0f, 123.0f), 123.0f);
    EXPECT_FLOAT_EQ(evaluateCurvePoints(points, 1, -456.0f, -456.0f), -456.0f);
}

TEST(CurveTest, IdentityCurvePassesThrough)
{
    const curvePoint_t points[2] = { { -1000, -1000 }, { 1000, 1000 } };

    EXPECT_FLOAT_EQ(evaluateCurvePoints(points, 2, 0.0f, 0.0f), 0.0f);
    EXPECT_FLOAT_EQ(evaluateCurvePoints(points, 2, 500.0f, 500.0f), 500.0f);
    EXPECT_FLOAT_EQ(evaluateCurvePoints(points, 2, -500.0f, -500.0f), -500.0f);
}

TEST(CurveTest, ClampsBeyondEndpoints)
{
    const curvePoint_t points[3] = { { -500, -200 }, { 0, 0 }, { 500, 800 } };

    EXPECT_FLOAT_EQ(evaluateCurvePoints(points, 3, -1000.0f, 0.0f), -200.0f);
    EXPECT_FLOAT_EQ(evaluateCurvePoints(points, 3, 1000.0f, 0.0f), 800.0f);
}

TEST(CurveTest, InterpolatesBetweenPoints)
{
    const curvePoint_t points[3] = { { -1000, -1000 }, { 0, 500 }, { 1000, 1000 } };

    // Midpoint of the second segment: x=500 is halfway between (0,500) and (1000,1000)
    EXPECT_FLOAT_EQ(evaluateCurvePoints(points, 3, 500.0f, 0.0f), 750.0f);

    // Midpoint of the first segment: x=-500 is halfway between (-1000,-1000) and (0,500)
    EXPECT_FLOAT_EQ(evaluateCurvePoints(points, 3, -500.0f, 0.0f), -250.0f);

    // Exactly on a point
    EXPECT_FLOAT_EQ(evaluateCurvePoints(points, 3, 0.0f, 0.0f), 500.0f);
}

TEST(CurveTest, DegenerateSegmentDoesNotDivideByZero)
{
    // Two points with identical x (shouldn't normally happen given the
    // configurator's endpoint clamping, but must not crash/NaN if it does).
    const curvePoint_t points[2] = { { 0, 100 }, { 0, 900 } };

    const float result = evaluateCurvePoints(points, 2, 0.0f, 0.0f);
    EXPECT_FALSE(std::isnan(result));
}

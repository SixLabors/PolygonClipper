// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

namespace PolygonClipper.Tests;

public class FloatExtensionsTests
{
    public static TheoryData<double, double, double> NextAfterTestData => new()
    {
        { 0.0, double.PositiveInfinity, double.Epsilon },
        { -0.0, double.PositiveInfinity, double.Epsilon },
        { 0.0, double.NegativeInfinity, -double.Epsilon },
        { -0.0, double.NegativeInfinity, -double.Epsilon },
        { 1.0, double.PositiveInfinity, 1.0000000000000002 },
        { 1.0, double.NegativeInfinity, 0.9999999999999999 },
        { -1.0, double.PositiveInfinity, -0.9999999999999999 },
        { -1.0, double.NegativeInfinity, -1.0000000000000002 },
        { double.MaxValue, double.PositiveInfinity, double.PositiveInfinity },
        { double.MinValue, double.NegativeInfinity, double.NegativeInfinity },
        { double.PositiveInfinity, double.PositiveInfinity, double.PositiveInfinity },
        { double.NegativeInfinity, double.NegativeInfinity, double.NegativeInfinity },
        { double.NaN, double.PositiveInfinity, double.NaN },
        { double.NaN, double.NegativeInfinity, double.NaN },
    };

    [Theory]
    [MemberData(nameof(NextAfterTestData))]
    public void NextAfter_ShouldReturnCorrectResult(double input, double target, double expected)
    {
        double result = input.NextAfter(target);
        Assert.Equal(expected, result);
    }
}

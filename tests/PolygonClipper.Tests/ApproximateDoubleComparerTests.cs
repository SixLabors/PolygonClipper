// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

namespace SixLabors.PolygonClipper.Tests;

/// <summary>
/// Checks the approximate comparer through xUnit equality assertions.
/// </summary>
public class ApproximateDoubleComparerTests
{
    /// <summary>
    /// Checks finite differences, equal infinities and matching NaNs.
    /// </summary>
    /// <param name="first">The first value.</param>
    /// <param name="second">The second value.</param>
    /// <param name="equal">Whether the values compare equal within one eighth.</param>
    [Theory]
    [InlineData(0D, 0D, true)]
    [InlineData(1D, 1.125D, true)]
    [InlineData(-1D, -1.125D, true)]
    [InlineData(1D, 1.25D, false)]
    [InlineData(double.NaN, double.NaN, true)]
    [InlineData(double.PositiveInfinity, double.PositiveInfinity, true)]
    [InlineData(double.NegativeInfinity, double.NegativeInfinity, true)]
    [InlineData(double.PositiveInfinity, double.NegativeInfinity, false)]
    [InlineData(double.PositiveInfinity, double.MaxValue, false)]
    public void Equals_Doubles_UsesSpecifiedError(double first, double second, bool equal)
    {
        ApproximateDoubleComparer comparer = new(0.125D);
        if (equal)
        {
            Assert.Equal(first, second, comparer);
        }
        else
        {
            Assert.NotEqual(first, second, comparer);
        }
    }

    /// <summary>
    /// Checks that both vertex coordinates must compare equal.
    /// </summary>
    /// <param name="x">The second vertex's X coordinate.</param>
    /// <param name="y">The second vertex's Y coordinate.</param>
    /// <param name="equal">Whether both coordinates are within one eighth of zero.</param>
    [Theory]
    [InlineData(0.125D, -0.125D, true)]
    [InlineData(0.25D, 0D, false)]
    [InlineData(0D, 0.25D, false)]
    public void Equals_Vertices_ChecksBothCoordinates(double x, double y, bool equal)
    {
        ApproximateDoubleComparer comparer = new(0.125D);
        Vertex first = new(0, 0);
        Vertex second = new(x, y);
        if (equal)
        {
            Assert.Equal(first, second, comparer);
        }
        else
        {
            Assert.NotEqual(first, second, comparer);
        }
    }
}

// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

namespace SixLabors.PolygonClipper.Tests;

/// <summary>
/// Tests arithmetic and retained low-order bits using exact binary identities.
/// </summary>
public class ExtendedPrecisionDoubleTests
{
    /// <summary>
    /// Checks addition for exactly representable operands and results.
    /// </summary>
    /// <param name="left">The first value.</param>
    /// <param name="right">The second value.</param>
    /// <param name="expected">The exact sum.</param>
    [Theory]
    [InlineData(6D, 3D, 9D)]
    [InlineData(-6D, 3D, -3D)]
    [InlineData(6D, -3D, 3D)]
    [InlineData(0D, 3D, 3D)]
    public void Addition_ReturnsExactSum(double left, double right, double expected)
    {
        ExtendedPrecisionDouble result = new ExtendedPrecisionDouble(left) + new ExtendedPrecisionDouble(right);

        Assert.Equal(expected, result.Value);
    }

    /// <summary>
    /// Checks subtraction for exactly representable operands and results.
    /// </summary>
    /// <param name="left">The value to subtract from.</param>
    /// <param name="right">The value to subtract.</param>
    /// <param name="expected">The exact difference.</param>
    [Theory]
    [InlineData(6D, 3D, 3D)]
    [InlineData(-6D, 3D, -9D)]
    [InlineData(6D, -3D, 9D)]
    [InlineData(0D, 3D, -3D)]
    public void Subtraction_ReturnsExactDifference(double left, double right, double expected)
    {
        ExtendedPrecisionDouble result = new ExtendedPrecisionDouble(left) - new ExtendedPrecisionDouble(right);

        Assert.Equal(expected, result.Value);
    }

    /// <summary>
    /// Checks multiplication for exactly representable operands and results.
    /// </summary>
    /// <param name="left">The first factor.</param>
    /// <param name="right">The second factor.</param>
    /// <param name="expected">The exact product.</param>
    [Theory]
    [InlineData(6D, 3D, 18D)]
    [InlineData(-6D, 3D, -18D)]
    [InlineData(6D, -3D, -18D)]
    [InlineData(0D, 3D, 0D)]
    public void Multiplication_ReturnsExactProduct(double left, double right, double expected)
    {
        ExtendedPrecisionDouble result = new ExtendedPrecisionDouble(left) * new ExtendedPrecisionDouble(right);

        Assert.Equal(expected, result.Value);
    }

    /// <summary>
    /// Checks division for exactly representable operands and quotients.
    /// </summary>
    /// <param name="left">The dividend.</param>
    /// <param name="right">The divisor.</param>
    /// <param name="expected">The exact quotient.</param>
    [Theory]
    [InlineData(6D, 3D, 2D)]
    [InlineData(-6D, 3D, -2D)]
    [InlineData(6D, -3D, -2D)]
    [InlineData(0D, 3D, 0D)]
    public void Division_ReturnsExactQuotient(double left, double right, double expected)
    {
        ExtendedPrecisionDouble result = new ExtendedPrecisionDouble(left) / new ExtendedPrecisionDouble(right);

        Assert.Equal(expected, result.Value);
    }

    /// <summary>
    /// Checks that addition retains a term smaller than the leading value's spacing.
    /// </summary>
    [Fact]
    public void Addition_RetainsLowOrderTermAfterCancellation()
    {
        ExtendedPrecisionDouble large = new(Math.ScaleB(1D, 53));
        ExtendedPrecisionDouble one = new(1D);
        ExtendedPrecisionDouble sum = large + one;

        // A double rounds 2^53 + 1 to 2^53, but the retained term must survive cancellation.
        Assert.Equal(Math.ScaleB(1D, 53), sum.Value);
        Assert.Equal(1D, (sum - large).Value);
    }

    /// <summary>
    /// Checks subtraction when the leading components cancel and both residuals remain.
    /// </summary>
    [Fact]
    public void Subtraction_RetainsBothLowComponents()
    {
        ExtendedPrecisionDouble large = new(Math.ScaleB(1D, 54));
        ExtendedPrecisionDouble one = new(1D);
        ExtendedPrecisionDouble first = large + one;
        ExtendedPrecisionDouble second = large - one;

        // (2^54 + 1) - (2^54 - 1) is exactly 2, though each operand rounds to 2^54.
        Assert.Equal(2D, (first - second).Value);
    }

    /// <summary>
    /// Checks that coordinate subtraction retains bits lost by a double subtraction.
    /// </summary>
    [Fact]
    public void Difference_RetainsSubtractionError()
    {
        double large = Math.ScaleB(1D, 54);
        ExtendedPrecisionDouble difference = ExtendedPrecisionDouble.Difference(large, 1D);

        Assert.Equal(-1D, (difference - new ExtendedPrecisionDouble(large)).Value);
    }

    /// <summary>
    /// Checks multiplication against an exact difference-of-squares identity.
    /// </summary>
    [Fact]
    public void Multiplication_RetainsProductRoundingError()
    {
        double increment = Math.ScaleB(1D, -27);
        ExtendedPrecisionDouble left = new(1D + increment);
        ExtendedPrecisionDouble right = new(1D - increment);
        ExtendedPrecisionDouble difference = (left * right) - new ExtendedPrecisionDouble(1D);

        // (1 + 2^-27)(1 - 2^-27) - 1 is exactly -2^-54.
        Assert.Equal(-Math.ScaleB(1D, -54), difference.Value);
    }

    /// <summary>
    /// Checks that multiplication carries both operands' low components into the product.
    /// </summary>
    [Fact]
    public void Multiplication_RetainsBothLowComponents()
    {
        ExtendedPrecisionDouble value = new ExtendedPrecisionDouble(1D) + new ExtendedPrecisionDouble(Math.ScaleB(1D, -54));
        ExtendedPrecisionDouble difference = (value * value) - new ExtendedPrecisionDouble(1D);

        // (1 + 2^-54)^2 - 1 rounds to 2^-53; its 2^-108 term is below that double's rounding threshold.
        Assert.Equal(Math.ScaleB(1D, -53), difference.Value);
    }

    /// <summary>
    /// Checks quotient correction against the exact rounding error of one third.
    /// </summary>
    [Fact]
    public void Division_RetainsQuotientRoundingError()
    {
        ExtendedPrecisionDouble quotient = new ExtendedPrecisionDouble(1D) / new ExtendedPrecisionDouble(3D);
        ExtendedPrecisionDouble difference = quotient - new ExtendedPrecisionDouble(1D / 3D);
        ExtendedPrecisionDouble scaled = difference * new ExtendedPrecisionDouble(Math.ScaleB(1D, 54));

        // The nearest double is 1/(3 * 2^54) below 1/3. Scaling that residual by 2^54 gives 1/3.
        Assert.Equal(1D / 3D, scaled.Value);
    }

    /// <summary>
    /// Checks division when the dividend and divisor both contain retained low components.
    /// </summary>
    [Fact]
    public void Division_RetainsBothLowComponents()
    {
        ExtendedPrecisionDouble large = new(Math.ScaleB(1D, 54));
        ExtendedPrecisionDouble one = new(1D);
        ExtendedPrecisionDouble quotient = (large + one) / (large - one);
        ExtendedPrecisionDouble difference = quotient - one;
        ExtendedPrecisionDouble scaled = difference * new ExtendedPrecisionDouble(Math.ScaleB(1D, 53));

        // ((2^54 + 1)/(2^54 - 1) - 1) * 2^53 rounds to 1.
        Assert.Equal(1D, scaled.Value);
    }
}

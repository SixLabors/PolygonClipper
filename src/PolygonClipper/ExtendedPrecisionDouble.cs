// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System.Runtime.CompilerServices;

namespace SixLabors.PolygonClipper;

/// <summary>
/// Stores a number as the sum of two doubles to retain arithmetic rounding error.
/// </summary>
/// <remarks>
/// The high component holds the rounded leading value. The low component holds the remaining low-order bits.
/// Intersection calculations carry both components until <see cref="Value"/> produces a double coordinate.
/// This increases precision but does not extend the double exponent range.
/// Arithmetic requires finite inputs and finite intermediate results.
/// </remarks>
internal readonly struct ExtendedPrecisionDouble
{
    /// <summary>
    /// The rounded leading component of the value.
    /// </summary>
    private readonly double high;

    /// <summary>
    /// The residual added to the leading component to retain low-order bits.
    /// </summary>
    private readonly double low;

    /// <summary>
    /// Initializes a new instance of the <see cref="ExtendedPrecisionDouble"/> struct from a double.
    /// </summary>
    /// <param name="value">The exact starting value, with no residual component.</param>
    public ExtendedPrecisionDouble(double value)
        : this(value, 0)
    {
    }

    /// <summary>
    /// Initializes a new instance of the <see cref="ExtendedPrecisionDouble"/> struct from a normalized pair.
    /// </summary>
    /// <param name="high">The rounded leading component.</param>
    /// <param name="low">The residual that cannot be represented in the leading component.</param>
    private ExtendedPrecisionDouble(double high, double low)
    {
        this.high = high;
        this.low = low;
    }

    /// <summary>
    /// Gets the sum of both components rounded to one double.
    /// </summary>
    public double Value => this.high + this.low;

    /// <summary>
    /// Adds two values and retains their rounding error.
    /// </summary>
    /// <param name="left">The first value.</param>
    /// <param name="right">The second value.</param>
    /// <returns>The sum with its residual component.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static ExtendedPrecisionDouble operator +(in ExtendedPrecisionDouble left, in ExtendedPrecisionDouble right)
    {
        // Two-sum recovers the parts of the leading operands lost when their
        // sum is rounded. Include both existing residuals before renormalizing.
        double sum = left.high + right.high;
        double virtualRight = sum - left.high;
        double error = (left.high - (sum - virtualRight)) + (right.high - virtualRight) + left.low + right.low;

        // Move the representable part of the accumulated error into high.
        // Keep the rest in low for subsequent operations.
        double high = sum + error;
        return new(high, error - (high - sum));
    }

    /// <summary>
    /// Subtracts the second value from the first and retains the rounding error.
    /// </summary>
    /// <param name="left">The value to subtract from.</param>
    /// <param name="right">The value to subtract.</param>
    /// <returns>The difference with its residual component.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static ExtendedPrecisionDouble operator -(in ExtendedPrecisionDouble left, in ExtendedPrecisionDouble right)
    {
        // Negate both components so addition also carries the right residual.
        ExtendedPrecisionDouble negativeRight = new(-right.high, -right.low);
        return left + negativeRight;
    }

    /// <summary>
    /// Multiplies two values and retains the rounding error.
    /// </summary>
    /// <param name="left">The first factor.</param>
    /// <param name="right">The second factor.</param>
    /// <returns>The product with its residual component.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static ExtendedPrecisionDouble operator *(in ExtendedPrecisionDouble left, in ExtendedPrecisionDouble right)
    {
        double product = left.high * right.high;

        // FMA recovers the leading product's rounding error. The cross terms
        // carry each low component into the result; low * low is below the retained precision.
        double error = Math.FusedMultiplyAdd(left.high, right.high, -product) + (left.high * right.low) + (left.low * right.high);
        double high = product + error;
        return new(high, error - (high - product));
    }

    /// <summary>
    /// Divides two values and corrects the leading quotient with its remainder.
    /// </summary>
    /// <param name="left">The dividend.</param>
    /// <param name="right">The divisor, whose leading component must be non-zero.</param>
    /// <returns>The quotient with its residual component.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static ExtendedPrecisionDouble operator /(in ExtendedPrecisionDouble left, in ExtendedPrecisionDouble right)
    {
        // The first quotient uses the leading components. Divide the remaining
        // error by the leading divisor to recover the next quotient component.
        ExtendedPrecisionDouble quotient = new(left.high / right.high);
        ExtendedPrecisionDouble product = right * quotient;
        ExtendedPrecisionDouble remainder = left - product;
        ExtendedPrecisionDouble correction = new(remainder.Value / right.high);
        return quotient + correction;
    }

    /// <summary>
    /// Subtracts double coordinates without discarding their subtraction error.
    /// </summary>
    /// <param name="first">The coordinate to subtract from.</param>
    /// <param name="second">The coordinate to subtract.</param>
    /// <returns>The difference with its residual component.</returns>
    public static ExtendedPrecisionDouble Difference(double first, double second)
        => new ExtendedPrecisionDouble(first) - new ExtendedPrecisionDouble(second);
}

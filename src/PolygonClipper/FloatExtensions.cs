// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System;
using System.Runtime.CompilerServices;

namespace PolygonClipper;

/// <summary>
/// Provides extension methods for floating-point numbers.
/// </summary>
internal static class FloatExtensions
{
    /// <summary>
    /// Returns the next representable double value in the direction of y.
    /// </summary>
    /// <remarks><see href="https://docs.rs/float_next_after/latest/src/float_next_after/lib.rs.html"/></remarks>
    /// <param name="x">The starting floating-point number.</param>
    /// <param name="y">The target floating-point number.</param>
    /// <returns>The next representable value of x towards y.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static double NextAfter(this double x, double y)
    {
        // Special cases
        if (double.IsNaN(x) || double.IsNaN(y))
        {
            return double.NaN;
        }

        if (x == y)
        {
            return y;
        }

        if (double.IsPositiveInfinity(x))
        {
            return double.PositiveInfinity;
        }

        if (double.IsNegativeInfinity(x))
        {
            return double.NegativeInfinity;
        }

        // Handle stepping from zero
        if (x == 0D)
        {
            return Math.CopySign(double.Epsilon, y); // Smallest positive subnormal double
        }

        // Convert double to raw bits
        long bits = BitConverter.DoubleToInt64Bits(x);

        // Adjust bits to get the next representable value
        if ((y > x) == (x > 0D)) // Moving in the same sign direction
        {
            bits++;
        }
        else
        {
            bits--;
        }

        // Convert bits back to double
        double next = BitConverter.Int64BitsToDouble(bits);

        // Ensure correct handling of signed zeros
        if (next == 0D)
        {
            return Math.CopySign(next, x);
        }

        return next;
    }
}

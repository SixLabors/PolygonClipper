// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

namespace SixLabors.PolygonClipper;

/// <summary>Provides compensated construction of geometric intersections.</summary>
internal static partial class PolygonUtilities
{
    /// <summary>Finds segment intersections with compensated construction of a crossing point.</summary>
    /// <param name="first">The first source segment.</param>
    /// <param name="second">The second source segment.</param>
    /// <param name="point">The first intersection.</param>
    /// <param name="last">The final intersection of an overlap.</param>
    /// <returns>The number of intersections.</returns>
    public static int FindIntersectionPrecise(in Segment first, in Segment second, out Vertex point, out Vertex last)
    {
        int count = FindIntersection(first, second, out point, out last);
        if (count != 1 || point == first.Source || point == first.Target || point == second.Source || point == second.Target)
        {
            return count;
        }

        // Intersection parameters are quotients of determinants. Retain the
        // multiplication and addition roundoff until the final coordinate is
        // rounded, rather than rounding the parameter and then each multiply/add.
        // Independent pairs at a multiway junction then construct the same node.
        CompensatedDouble ax = CompensatedDouble.Difference(first.Target.X, first.Source.X);
        CompensatedDouble ay = CompensatedDouble.Difference(first.Target.Y, first.Source.Y);
        CompensatedDouble bx = CompensatedDouble.Difference(second.Target.X, second.Source.X);
        CompensatedDouble by = CompensatedDouble.Difference(second.Target.Y, second.Source.Y);
        CompensatedDouble ex = CompensatedDouble.Difference(second.Source.X, first.Source.X);
        CompensatedDouble ey = CompensatedDouble.Difference(second.Source.Y, first.Source.Y);
        CompensatedDouble denominator = ax.Multiply(by).Subtract(ay.Multiply(bx));
        if (denominator.Value != 0)
        {
            CompensatedDouble numerator = ex.Multiply(by).Subtract(ey.Multiply(bx));
            CompensatedDouble parameter = numerator.Divide(denominator);
            point = new Vertex(parameter.Multiply(ax).Add(new(first.Source.X)).Value, parameter.Multiply(ay).Add(new(first.Source.Y)).Value);
            Box2 bounds = new(
                Vertex.Max(Vertex.Min(first.Source, first.Target), Vertex.Min(second.Source, second.Target)),
                Vertex.Min(Vertex.Max(first.Source, first.Target), Vertex.Max(second.Source, second.Target)));
            point = ConstrainToBoundingBox(point, bounds);
        }

        return count;
    }

    /// <summary>Retains arithmetic roundoff as a second double during intersection construction.</summary>
    private readonly struct CompensatedDouble
    {
        private readonly double high;
        private readonly double low;

        /// <summary>Initializes a new instance of the <see cref="CompensatedDouble"/> struct.</summary>
        /// <param name="high">The leading value.</param>
        /// <param name="low">The retained roundoff.</param>
        public CompensatedDouble(double high, double low = 0)
        {
            this.high = high;
            this.low = low;
        }

        /// <summary>Gets the value rounded to one double.</summary>
        public double Value => this.high + this.low;

        /// <summary>Subtracts two coordinates without discarding the subtraction roundoff.</summary>
        /// <param name="first">The first coordinate.</param>
        /// <param name="second">The coordinate to subtract.</param>
        /// <returns>The compensated difference.</returns>
        public static CompensatedDouble Difference(double first, double second) => new CompensatedDouble(first).Subtract(new(second));

        /// <summary>Adds two compensated values using the error-free two-sum identity.</summary>
        /// <param name="other">The value to add.</param>
        /// <returns>The compensated sum.</returns>
        public CompensatedDouble Add(CompensatedDouble other)
        {
            double sum = this.high + other.high;
            double virtualOther = sum - this.high;
            double error = (this.high - (sum - virtualOther)) + (other.high - virtualOther) + this.low + other.low;
            double high = sum + error;
            return new(high, error - (high - sum));
        }

        /// <summary>Subtracts a compensated value.</summary>
        /// <param name="other">The value to subtract.</param>
        /// <returns>The compensated difference.</returns>
        public CompensatedDouble Subtract(CompensatedDouble other) => this.Add(new(-other.high, -other.low));

        /// <summary>Multiplies values while retaining the leading product's roundoff with FMA.</summary>
        /// <param name="other">The multiplier.</param>
        /// <returns>The compensated product.</returns>
        public CompensatedDouble Multiply(CompensatedDouble other)
        {
            double product = this.high * other.high;
            double error = Math.FusedMultiplyAdd(this.high, other.high, -product) + (this.high * other.low) + (this.low * other.high);
            double high = product + error;
            return new(high, error - (high - product));
        }

        /// <summary>Corrects a leading quotient using its compensated residual.</summary>
        /// <param name="other">The non-zero divisor.</param>
        /// <returns>The compensated quotient.</returns>
        public CompensatedDouble Divide(CompensatedDouble other)
        {
            CompensatedDouble quotient = new(this.high / other.high);
            CompensatedDouble remainder = this.Subtract(other.Multiply(quotient));
            return quotient.Add(new(remainder.Value / other.high));
        }
    }
}

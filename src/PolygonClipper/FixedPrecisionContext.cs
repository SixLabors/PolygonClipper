// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System.Runtime.CompilerServices;

namespace SixLabors.PolygonClipper;

/// <summary>
/// Provides fixed-precision scaling and coordinate conversion utilities.
/// </summary>
internal readonly struct FixedPrecisionContext
{
    private const long MaxInt64 = long.MaxValue;
    internal const long MaxCoord = (MaxInt64 / 3) * 2;
    private const double MaxCoordDouble = MaxCoord;
    private const double DoubleExactIntLimit = 9007199254740992D; // 2^53

    public FixedPrecisionContext(double scale, Vertex origin)
    {
        this.Scale = scale;
        this.InvScale = 1D / scale;
        this.Origin = origin;
    }

    /// <summary>
    /// Gets the scale factor used to convert from double to fixed precision.
    /// </summary>
    public double Scale { get; }

    /// <summary>
    /// Gets the inverse scale used to convert back to double coordinates.
    /// </summary>
    public double InvScale { get; }

    /// <summary>
    /// Gets the translation origin used to normalize coordinates before scaling.
    /// </summary>
    public Vertex Origin { get; }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vertex64 Quantize(in Vertex value)
        => new(
            CheckCast((value.X - this.Origin.X) * this.Scale),
            CheckCast((value.Y - this.Origin.Y) * this.Scale));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Vertex Dequantize(in Vertex64 value)
        => new(
            (value.X * this.InvScale) + this.Origin.X,
            (value.Y * this.InvScale) + this.Origin.Y);

    public static FixedPrecisionContext Create(ClipperOptions? options, ReadOnlySpan<Polygon> polygons)
    {
        ClipperOptions resolved = options ?? ClipperOptions.Default;
        GetCoordinateStats(polygons, out double minX, out double minY, out double maxX, out double maxY, out double minDelta);
        Vertex origin = default;
        double maxAbsCoord = Math.Max(
            Math.Max(Math.Abs(minX), Math.Abs(maxX)),
            Math.Max(Math.Abs(minY), Math.Abs(maxY)));
        double scale = ResolveScale(resolved, maxAbsCoord, minDelta);
        return new FixedPrecisionContext(scale, origin);
    }

    private static double ResolveScale(ClipperOptions options, double maxAbsCoord, double minDelta)
    {
        double scale = options.ScaleMode switch
        {
            ClipperScaleMode.FixedPrecision => GetPrecisionScale(options.Precision),
            ClipperScaleMode.FixedScale => GetFixedScale(options.FixedScale),
            ClipperScaleMode.Auto => GetPrecisionScale(options.Precision),
            _ => GetPrecisionScale(options.Precision)
        };

        if (options.ScaleMode == ClipperScaleMode.Auto && maxAbsCoord > 0D)
        {
            double maxScale = (MaxCoordDouble - 1D) / maxAbsCoord;
            if (maxScale < scale)
            {
                scale = maxScale;
            }

            if (minDelta > 0D)
            {
                double minDeltaScale = 1D / minDelta;
                if (minDeltaScale > scale)
                {
                    scale = Math.Min(minDeltaScale, maxScale);
                }
            }
        }

        if (scale <= 0D)
        {
            throw new ArgumentOutOfRangeException(nameof(options), "Scale must be greater than zero.");
        }

        if (options.ScaleMode != ClipperScaleMode.Auto && maxAbsCoord > 0D &&
            maxAbsCoord * scale >= MaxCoordDouble)
        {
            throw new ArgumentOutOfRangeException(nameof(options), "Input coordinates exceed fixed-precision range.");
        }

        return scale;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static double GetPrecisionScale(int precision)
    {
        if (precision < -16 || precision > 16)
        {
            throw new ArgumentOutOfRangeException(nameof(precision), "Precision must be between -16 and 16.");
        }

        return Math.Pow(10D, precision);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static double GetFixedScale(double scale)
    {
        if (scale <= 0D)
        {
            throw new ArgumentOutOfRangeException(nameof(scale), "Fixed scale must be greater than zero.");
        }

        return scale;
    }

    private static void GetCoordinateStats(
        ReadOnlySpan<Polygon> polygons,
        out double minX,
        out double minY,
        out double maxX,
        out double maxY,
        out double minDelta)
    {
        minX = double.PositiveInfinity;
        minY = double.PositiveInfinity;
        maxX = double.NegativeInfinity;
        maxY = double.NegativeInfinity;
        minDelta = double.PositiveInfinity;

        for (int i = 0; i < polygons.Length; i++)
        {
            Polygon polygon = polygons[i];
            for (int j = 0; j < polygon.Count; j++)
            {
                Contour contour = polygon[j];
                int count = contour.Count;
                if (count == 0)
                {
                    continue;
                }

                Vertex prev = contour[0];
                for (int k = 0; k < count; k++)
                {
                    Vertex vertex = contour[k];
                    double x = vertex.X;
                    double y = vertex.Y;
                    if (x < minX)
                    {
                        minX = x;
                    }

                    if (x > maxX)
                    {
                        maxX = x;
                    }

                    if (y < minY)
                    {
                        minY = y;
                    }

                    if (y > maxY)
                    {
                        maxY = y;
                    }

                    if (k > 0)
                    {
                        double dx = Math.Abs(vertex.X - prev.X);
                        double dy = Math.Abs(vertex.Y - prev.Y);
                        if (dx > 0D && dx < minDelta)
                        {
                            minDelta = dx;
                        }

                        if (dy > 0D && dy < minDelta)
                        {
                            minDelta = dy;
                        }
                    }

                    prev = vertex;
                }
            }
        }

        if (double.IsPositiveInfinity(minDelta))
        {
            minDelta = 0D;
        }

        if (double.IsPositiveInfinity(minX))
        {
            minX = 0D;
            minY = 0D;
            maxX = 0D;
            maxY = 0D;
        }
    }


    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static long CheckCast(double value)
    {
        if (value > MaxCoordDouble || value < -MaxCoordDouble)
        {
            throw new ArgumentOutOfRangeException(nameof(value), "Fixed-precision coordinate is out of range.");
        }

        if (value <= -DoubleExactIntLimit || value >= DoubleExactIntLimit)
        {
            return (long)Math.Round((decimal)value, MidpointRounding.AwayFromZero);
        }

        return (long)Math.Round(value, MidpointRounding.AwayFromZero);
    }
}

// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System.Buffers;
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
        GetCoordinateStats(
            polygons,
            resolved.ScaleMode == ClipperScaleMode.Auto,
            out double minX,
            out double minY,
            out double maxX,
            out double maxY,
            out double minDelta);
        Vertex origin = new((minX + maxX) * 0.5D, (minY + maxY) * 0.5D);
        double maxAbsCoord = Math.Max(
            Math.Max(Math.Abs(minX - origin.X), Math.Abs(maxX - origin.X)),
            Math.Max(Math.Abs(minY - origin.Y), Math.Abs(maxY - origin.Y)));
        double scale;

        if (resolved.ScaleMode == ClipperScaleMode.Auto)
        {
            double desiredScale = GetPrecisionScale(resolved.Precision);
            if (minDelta > 0D)
            {
                double minDeltaScale = 1D / minDelta;
                if (minDeltaScale > desiredScale)
                {
                    desiredScale = minDeltaScale;
                }
            }

            double maxScale = maxAbsCoord > 0D
                ? (MaxCoordDouble - 1D) / maxAbsCoord
                : double.PositiveInfinity;
            scale = Math.Min(desiredScale, maxScale);
        }
        else
        {
            scale = resolved.ScaleMode switch
            {
                ClipperScaleMode.FixedPrecision => GetPrecisionScale(resolved.Precision),
                ClipperScaleMode.FixedScale => GetFixedScale(resolved.FixedScale),
                _ => GetPrecisionScale(resolved.Precision)
            };

            if (scale <= 0D)
            {
                throw new ArgumentOutOfRangeException(nameof(options), "Scale must be greater than zero.");
            }

            if (maxAbsCoord > 0D && maxAbsCoord * scale >= MaxCoordDouble)
            {
                throw new ArgumentOutOfRangeException(nameof(options), "Input coordinates exceed fixed-precision range.");
            }
        }

        if (scale <= 0D)
        {
            throw new ArgumentOutOfRangeException(nameof(options), "Scale must be greater than zero.");
        }

        return new FixedPrecisionContext(scale, origin);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static double GetPrecisionScale(int precision)
    {
        Guard.MustBeBetweenOrEqualTo(precision, -16, 16, nameof(precision));
        return Math.Pow(10D, precision);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static double GetFixedScale(double scale)
    {
        Guard.MustBeGreaterThan(scale, 0D, nameof(scale));
        return scale;
    }

    private static void GetCoordinateStats(
        ReadOnlySpan<Polygon> polygons,
        bool computeMinDelta,
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

        if (!computeMinDelta)
        {
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
                    }
                }
            }

            if (double.IsPositiveInfinity(minX))
            {
                minX = 0D;
                minY = 0D;
                maxX = 0D;
                maxY = 0D;
            }

            minDelta = 0D;
            return;
        }

        int totalVertices = 0;

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

                totalVertices += count;
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
                }
            }
        }

        if (totalVertices == 0)
        {
            minDelta = 0D;
            minX = 0D;
            minY = 0D;
            maxX = 0D;
            maxY = 0D;
            return;
        }

        double[] xs = ArrayPool<double>.Shared.Rent(totalVertices);
        double[] ys = ArrayPool<double>.Shared.Rent(totalVertices);
        int index = 0;

        try
        {
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

                    for (int k = 0; k < count; k++)
                    {
                        Vertex vertex = contour[k];
                        xs[index] = vertex.X;
                        ys[index] = vertex.Y;
                        index++;
                    }
                }
            }

            Array.Sort(xs, 0, index);
            Array.Sort(ys, 0, index);

            minDelta = FindMinPositiveDelta(xs, index);
            double minDeltaY = FindMinPositiveDelta(ys, index);
            if (minDeltaY < minDelta)
            {
                minDelta = minDeltaY;
            }
        }
        finally
        {
            ArrayPool<double>.Shared.Return(xs);
            ArrayPool<double>.Shared.Return(ys);
        }

        if (double.IsPositiveInfinity(minDelta))
        {
            minDelta = 0D;
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static double FindMinPositiveDelta(double[] values, int count)
    {
        if (count < 2)
        {
            return double.PositiveInfinity;
        }

        double minDelta = double.PositiveInfinity;
        double prev = values[0];

        for (int i = 1; i < count; i++)
        {
            double current = values[i];
            double delta = current - prev;
            if (delta > 0D && delta < minDelta)
            {
                minDelta = delta;
            }

            prev = current;
        }

        return minDelta;
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static long CheckCast(double value)
    {
        if (value is > MaxCoordDouble or < -MaxCoordDouble)
        {
            throw new ArgumentOutOfRangeException(nameof(value), "Fixed-precision coordinate is out of range.");
        }

        return (long)Math.Round(value, MidpointRounding.AwayFromZero);
    }
}

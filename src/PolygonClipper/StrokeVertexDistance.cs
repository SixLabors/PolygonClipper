// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System.Runtime.CompilerServices;

namespace SixLabors.PolygonClipper;

/// <summary>
/// Represents a stroke-processing vertex with a cached outgoing segment length.
/// </summary>
/// <remarks>
/// This is an internal mutable value used by <see cref="PolygonStroker"/> while
/// normalizing source contours and computing joins/caps.
/// </remarks>
internal struct StrokeVertexDistance
{
    private const double VertexDistanceEpsilon = 1E-14D;
    private const double Dd = 1D / VertexDistanceEpsilon;

    /// <summary>
    /// The X-coordinate.
    /// </summary>
    public double X;

    /// <summary>
    /// The Y-coordinate.
    /// </summary>
    public double Y;

    /// <summary>
    /// Cached distance to another vertex measured by <see cref="Measure(in StrokeVertexDistance)"/>.
    /// </summary>
    public double Distance;

    /// <summary>
    /// Initializes a new instance of the <see cref="StrokeVertexDistance"/> struct.
    /// </summary>
    /// <param name="x">The X-coordinate.</param>
    /// <param name="y">The Y-coordinate.</param>
    /// <param name="distance">Initial cached distance value.</param>
    public StrokeVertexDistance(double x, double y, double distance)
    {
        this.X = x;
        this.Y = y;
        this.Distance = distance;
    }

    /// <summary>
    /// Measures the Euclidean distance from this vertex to <paramref name="vd"/> and stores it in <see cref="Distance"/>.
    /// </summary>
    /// <param name="vd">The vertex to measure to.</param>
    /// <returns>
    /// <see langword="true"/> when the measured distance is greater than the internal epsilon;
    /// otherwise <see langword="false"/>.
    /// </returns>
    /// <remarks>
    /// When points are closer than epsilon, <see cref="Distance"/> is set to a large sentinel value
    /// to avoid divide-by-near-zero behavior in downstream stroker math.
    /// </remarks>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Measure(in StrokeVertexDistance vd)
    {
        bool ret = (this.Distance = Vertex.Distance(new Vertex(this.X, this.Y), new Vertex(vd.X, vd.Y))) > VertexDistanceEpsilon;
        if (!ret)
        {
            this.Distance = Dd;
        }

        return ret;
    }
}

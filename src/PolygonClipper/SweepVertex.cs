// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

namespace SixLabors.PolygonClipper;

/// <summary>
/// Represents a vertex in the input contour linked list used by the sweep.
/// </summary>
/// <remarks>
/// Vertices are linked in a circular doubly linked list (see <see cref="Prev" /> and
/// <see cref="Next" />) so the sweep can traverse ascending/descending bounds and
/// detect local minima/maxima efficiently.
/// </remarks>
internal sealed class SweepVertex
{
#pragma warning disable SA1401 // Hot sweep vertex state uses fields to avoid accessor overhead.
    /// <summary>
    /// The vertex position.
    /// </summary>
    public Vertex64 Point;

    /// <summary>
    /// The next vertex in the contour.
    /// </summary>
    public SweepVertex? Next;

    /// <summary>
    /// The previous vertex in the contour.
    /// </summary>
    public SweepVertex? Prev;

    /// <summary>
    /// Flags describing sweep-related classification.
    /// </summary>
    public VertexFlags Flags;
#pragma warning restore SA1401

    /// <summary>
    /// Initializes a new instance of the <see cref="SweepVertex"/> class.
    /// </summary>
    public SweepVertex(Vertex64 point, VertexFlags flags, SweepVertex? prev)
    {
        this.Point = point;
        this.Flags = flags;
        this.Next = null;
        this.Prev = prev;
    }

    /// <summary>
    /// Gets a value indicating whether this vertex is marked as a local maxima.
    /// </summary>
    public bool IsMaxima => (this.Flags & VertexFlags.LocalMax) != 0;
}

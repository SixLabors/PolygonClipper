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
    /// <summary>
    /// Initializes a new instance of the <see cref="SweepVertex"/> class.
    /// </summary>
    public SweepVertex(Vertex point, VertexFlags flags, SweepVertex? prev)
    {
        this.Point = point;
        this.Flags = flags;
        this.Next = null;
        this.Prev = prev;
    }

    /// <summary>
    /// Gets or sets the vertex position.
    /// </summary>
    public Vertex Point { get; set; }

    /// <summary>
    /// Gets or sets the next vertex in the contour.
    /// </summary>
    public SweepVertex? Next { get; set; }

    /// <summary>
    /// Gets or sets the previous vertex in the contour.
    /// </summary>
    public SweepVertex? Prev { get; set; }

    /// <summary>
    /// Gets or sets the flags describing sweep-related classification.
    /// </summary>
    public VertexFlags Flags { get; set; }

    /// <summary>
    /// Gets a value indicating whether this vertex is marked as a local maxima.
    /// </summary>
    public bool IsMaxima => (this.Flags & VertexFlags.LocalMax) != 0;
}

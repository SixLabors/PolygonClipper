// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

namespace SixLabors.PolygonClipper;

/// <summary>
/// Represents a pending intersection between two active edges.
/// </summary>
/// <remarks>
/// Intersections are sorted and processed from higher scanlines to lower ones so
/// that edge order in the AEL remains consistent as the sweep descends.
/// </remarks>
internal readonly struct IntersectNode
{
    /// <summary>
    /// Initializes a new instance of the <see cref="IntersectNode"/> struct.
    /// </summary>
    internal IntersectNode(Vertex64 point, ActiveEdge edge1, ActiveEdge edge2)
    {
        this.Point = point;
        this.Edge1 = edge1;
        this.Edge2 = edge2;
    }

    /// <summary>
    /// Gets the intersection point between <see cref="Edge1"/> and <see cref="Edge2"/>.
    /// </summary>
    public Vertex64 Point { get; }

    /// <summary>
    /// Gets the first active edge participating in the intersection.
    /// </summary>
    public ActiveEdge Edge1 { get; }

    /// <summary>
    /// Gets the second active edge participating in the intersection.
    /// </summary>
    public ActiveEdge Edge2 { get; }
}

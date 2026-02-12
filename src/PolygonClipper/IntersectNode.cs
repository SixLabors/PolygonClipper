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
#pragma warning disable SA1401 // Hot path intersection sorting benefits from field access.
    /// <summary>
    /// Gets the intersection point between <see cref="Edge1"/> and <see cref="Edge2"/>.
    /// </summary>
    public readonly Vertex Point;

    /// <summary>
    /// Gets the first active edge participating in the intersection.
    /// </summary>
    public readonly ActiveEdge Edge1;

    /// <summary>
    /// Gets the second active edge participating in the intersection.
    /// </summary>
    public readonly ActiveEdge Edge2;
#pragma warning restore SA1401

    /// <summary>
    /// Initializes a new instance of the <see cref="IntersectNode"/> struct.
    /// </summary>
    internal IntersectNode(Vertex point, ActiveEdge edge1, ActiveEdge edge2)
    {
        this.Point = point;
        this.Edge1 = edge1;
        this.Edge2 = edge2;
    }
}

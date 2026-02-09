// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

namespace SixLabors.PolygonClipper;

// IntersectNode: a structure representing 2 intersecting edges.
// Intersections must be sorted so they are processed from the largest
// Y coordinates to the smallest while keeping edges adjacent.
internal readonly struct IntersectNode
{
    internal IntersectNode(Vertex point, ActiveEdge edge1, ActiveEdge edge2)
    {
        this.Point = point;
        this.Edge1 = edge1;
        this.Edge2 = edge2;
    }

    internal Vertex Point { get; }

    internal ActiveEdge Edge1 { get; }

    internal ActiveEdge Edge2 { get; }
}

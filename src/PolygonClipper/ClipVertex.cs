// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

namespace SixLabors.PolygonClipper;

internal sealed class ClipVertex
{
    internal ClipVertex(Vertex point, VertexFlags flags, ClipVertex? prev)
    {
        this.Point = point;
        this.Flags = flags;
        this.Next = null;
        this.Prev = prev;
    }

    internal Vertex Point { get; set; }

    internal ClipVertex? Next { get; set; }

    internal ClipVertex? Prev { get; set; }

    internal VertexFlags Flags { get; set; }

    internal bool IsMaxima => (this.Flags & VertexFlags.LocalMax) != 0;

    internal bool IsOpenEnd => (this.Flags & (VertexFlags.OpenStart | VertexFlags.OpenEnd)) != VertexFlags.None;
}

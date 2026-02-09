// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

namespace SixLabors.PolygonClipper;

internal readonly struct LocalMinima
{
    internal LocalMinima(ClipVertex vertex, ClipperPathType pathType, bool isOpen = false)
    {
        this.Vertex = vertex;
        this.PathType = pathType;
        this.IsOpen = isOpen;
    }

    internal ClipVertex Vertex { get; }

    internal ClipperPathType PathType { get; }

    internal bool IsOpen { get; }

    public static bool operator ==(LocalMinima lm1, LocalMinima lm2) => ReferenceEquals(lm1.Vertex, lm2.Vertex);

    public static bool operator !=(LocalMinima lm1, LocalMinima lm2) => !(lm1 == lm2);

    public override bool Equals(object? obj) => obj is LocalMinima minima && this == minima;

    public override int GetHashCode() => this.Vertex.GetHashCode();
}

internal struct LocalMinimaComparer : IComparer<LocalMinima>
{
    public readonly int Compare(LocalMinima locMin1, LocalMinima locMin2) => locMin2.Vertex.Point.Y.CompareTo(locMin1.Vertex.Point.Y);
}

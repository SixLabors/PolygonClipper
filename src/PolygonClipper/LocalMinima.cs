// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

namespace SixLabors.PolygonClipper;

/// <summary>
/// Describes the lowest vertex of an edge bound for the sweep line.
/// </summary>
internal readonly struct LocalMinima : IEquatable<LocalMinima>
{
    /// <summary>
    /// Initializes a new instance of the <see cref="LocalMinima"/> struct.
    /// </summary>
    internal LocalMinima(ClipVertex vertex) => this.Vertex = vertex;

    /// <summary>
    /// Gets the vertex associated with this local minima.
    /// </summary>
    internal ClipVertex Vertex { get; }

    public static bool operator ==(LocalMinima lm1, LocalMinima lm2) => lm1.Equals(lm2);

    public static bool operator !=(LocalMinima lm1, LocalMinima lm2) => !(lm1 == lm2);

    public override bool Equals(object? obj) => obj is LocalMinima minima && this.Equals(minima);

    public override int GetHashCode() => this.Vertex.GetHashCode();

    public bool Equals(LocalMinima other) => ReferenceEquals(this.Vertex, other.Vertex);
}

/// <summary>
/// Orders local minima so higher Y-values are processed first during the sweep.
/// </summary>
internal struct LocalMinimaComparer : IComparer<LocalMinima>
{
    public readonly int Compare(LocalMinima locMin1, LocalMinima locMin2)
        => locMin2.Vertex.Point.Y.CompareTo(locMin1.Vertex.Point.Y);
}

// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System.Runtime.CompilerServices;

namespace SixLabors.PolygonClipper;

/// <summary>
/// Represents a line segment on a plane.
/// </summary>
internal readonly struct Segment : IEquatable<Segment>
{
    /// <summary>
    /// Initializes a new instance of the <see cref="Segment"/> struct.
    /// </summary>
    /// <param name="source">The segment source.</param>
    /// <param name="target">The segment target.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Segment(in Vertex source, in Vertex target)
    {
        this.Source = source;
        this.Target = target;
        this.Min = Vertex.Min(source, target);
        this.Max = Vertex.Max(source, target);
    }

    /// <summary>
    /// Gets the segment source vector.
    /// </summary>
    public Vertex Source { get; }

    /// <summary>
    /// Gets the segment target vector.
    /// </summary>
    public Vertex Target { get; }

    /// <summary>
    /// Gets the point of the segment with lexicographically smallest coordinate.
    /// </summary>
    public Vertex Min { get; }

    /// <summary>
    /// Gets the point of the segment with lexicographically largest coordinate.
    /// </summary>
    public Vertex Max { get; }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator ==(in Segment left, in Segment right)
        => left.Equals(right);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator !=(in Segment left, in Segment right)
        => !(left == right);

    /// <summary>
    /// Gets a value indicating whether the segment is degenerate.
    /// </summary>
    /// <returns>
    /// <see langword="true"/> if the segment is degenerate; otherwise <see langword="false"/>.
    /// </returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool IsDegenerate() => this.Source.Equals(this.Target);

    /// <summary>
    /// Gets a value indicating whether the segment is vertical.
    /// </summary>
    /// <returns>
    /// <see langword="true"/> if the segment is vertical; otherwise <see langword="false"/>.
    /// </returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool IsVertical() => this.Source.X == this.Target.X;

    /// <summary>
    /// Changes the segment orientation.
    /// </summary>
    /// <returns>The <see cref="Segment"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Segment Reverse()
        => new(this.Target, this.Source);

    /// <inheritdoc/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override bool Equals(object? obj)
        => obj is Segment segment && this.Equals(segment);

    /// <inheritdoc/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Equals(Segment other)
        => this.Source.Equals(other.Source) && this.Target.Equals(other.Target);

    /// <inheritdoc/>
    public override int GetHashCode()
        => HashCode.Combine(this.Source, this.Target);
}

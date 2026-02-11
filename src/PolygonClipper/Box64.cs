// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System.Runtime.CompilerServices;

namespace SixLabors.PolygonClipper;

/// <summary>
/// Represents a fixed-precision bounding box.
/// </summary>
internal readonly struct Box64 : IEquatable<Box64>
{
    /// <summary>
    /// Gets the minimum xy-coordinate.
    /// </summary>
#pragma warning disable CA1051 // Do not declare visible instance fields
    public readonly Vertex64 Min;

    /// <summary>
    /// Gets the maximum xy-coordinate.
    /// </summary>
    public readonly Vertex64 Max;
#pragma warning restore CA1051 // Do not declare visible instance fields

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Box64(in Vertex64 vector)
        : this(vector, vector)
    {
    }

    public Box64(in Vertex64 min, in Vertex64 max)
    {
        this.Min = min;
        this.Max = max;
    }

    public static Box64 Invalid { get; } = new(
        new Vertex64(long.MaxValue, long.MaxValue),
        new Vertex64(long.MinValue, long.MinValue));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator ==(in Box64 left, in Box64 right) => left.Equals(right);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator !=(in Box64 left, in Box64 right) => !(left == right);

    public bool IsEmpty() => this.Max.X <= this.Min.X || this.Max.Y <= this.Min.Y;

    public bool Contains(in Vertex64 point)
        => point.X > this.Min.X && point.X < this.Max.X && point.Y > this.Min.Y && point.Y < this.Max.Y;

    public bool Contains(in Box64 bounds)
        => bounds.Min.X >= this.Min.X && bounds.Max.X <= this.Max.X &&
           bounds.Min.Y >= this.Min.Y && bounds.Max.Y <= this.Max.Y;

    public bool Intersects(in Box64 bounds)
        => Math.Max(this.Min.X, bounds.Min.X) <= Math.Min(this.Max.X, bounds.Max.X) &&
           Math.Max(this.Min.Y, bounds.Min.Y) <= Math.Min(this.Max.Y, bounds.Max.Y);

    public Vertex64 MidPoint() => new((this.Min.X + this.Max.X) / 2, (this.Min.Y + this.Max.Y) / 2);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Box64 Add(in Box64 other)
        => new(Vertex64.Min(this.Min, other.Min), Vertex64.Max(this.Max, other.Max));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override bool Equals(object? obj) => obj is Box64 box && this.Equals(box);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Equals(Box64 other) => this.Min == other.Min && this.Max == other.Max;

    public override int GetHashCode() => HashCode.Combine(this.Min, this.Max);
}

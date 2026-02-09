// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System;
using System.Runtime.CompilerServices;

namespace SixLabors.PolygonClipper;

/// <summary>
/// Represents a bounding box.
/// </summary>
public readonly struct Box2 : IEquatable<Box2>
{
    /// <summary>
    /// Initializes a new instance of the <see cref="Box2"/> struct.
    /// </summary>
    /// <param name="vector">The xy-coordinate.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Box2(in Vertex vector)
        : this(vector, vector)
    {
    }

    /// <summary>
    /// Initializes a new instance of the <see cref="Box2"/> struct.
    /// </summary>
    /// <param name="min">The minimum xy-coordinate.</param>
    /// <param name="max">The maximum xy-coordinate.</param>
    public Box2(in Vertex min, in Vertex max)
    {
        this.Min = min;
        this.Max = max;
    }

    /// <summary>
    /// Gets an invalid bounds instance.
    /// </summary>
    public static Box2 Invalid { get; } = new(
        new Vertex(double.MaxValue, double.MaxValue),
        new Vertex(-double.MaxValue, -double.MaxValue));

    /// <summary>
    /// Gets the minimum xy-coordinate.
    /// </summary>
    public Vertex Min { get; }

    /// <summary>
    /// Gets the maximum xy-coordinate.
    /// </summary>
    public Vertex Max { get; }

    /// <summary>
    /// Compares two <see cref="Box2"/> instances for equality.
    /// </summary>
    /// <param name="left">The left <see cref="Box2"/> object.</param>
    /// <param name="right">The right <see cref="Box2"/> object.</param>
    /// <returns><c>true</c> if both boxes are equal; otherwise, <c>false</c>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator ==(in Box2 left, in Box2 right)
        => left.Equals(right);

    /// <summary>
    /// Determines whether two <see cref="Box2"/> instances are not equal.
    /// </summary>
    /// <param name="left">The left <see cref="Box2"/> object.</param>
    /// <param name="right">The right <see cref="Box2"/> object.</param>
    /// <returns><c>true</c> if the boxes are not equal; otherwise, <c>false</c>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool operator !=(in Box2 left, in Box2 right)
        => !(left == right);

    /// <summary>
    /// Returns true if the box is empty.
    /// </summary>
    /// <returns><see langword="true"/> if the box is empty; otherwise, <see langword="false"/>.</returns>
    public bool IsEmpty() => this.Max.X <= this.Min.X || this.Max.Y <= this.Min.Y;

    /// <summary>
    /// Returns true if the point lies within the box.
    /// </summary>
    /// <param name="point">The point to test.</param>
    /// <returns><see langword="true"/> if the point lies within the box; otherwise, <see langword="false"/>.</returns>
    public bool Contains(in Vertex point)
        => point.X > this.Min.X && point.X < this.Max.X && point.Y > this.Min.Y && point.Y < this.Max.Y;

    /// <summary>
    /// Returns true if the box contains another box.
    /// </summary>
    /// <param name="bounds">The other box.</param>
    /// <returns><see langword="true"/> if the box contains the other box; otherwise, <see langword="false"/>.</returns>
    public bool Contains(in Box2 bounds)
        => bounds.Min.X >= this.Min.X && bounds.Max.X <= this.Max.X &&
           bounds.Min.Y >= this.Min.Y && bounds.Max.Y <= this.Max.Y;

    /// <summary>
    /// Returns true if the boxes intersect.
    /// </summary>
    /// <param name="bounds">The other box.</param>
    /// <returns><see langword="true"/> if the boxes intersect; otherwise, <see langword="false"/>.</returns>
    public bool Intersects(in Box2 bounds)
        => Math.Max(this.Min.X, bounds.Min.X) <= Math.Min(this.Max.X, bounds.Max.X) &&
           Math.Max(this.Min.Y, bounds.Min.Y) <= Math.Min(this.Max.Y, bounds.Max.Y);

    /// <summary>
    /// Returns the midpoint of the box.
    /// </summary>
    /// <returns>The midpoint.</returns>
    public Vertex MidPoint() => new((this.Min.X + this.Max.X) / 2D, (this.Min.Y + this.Max.Y) / 2D);

    /// <summary>
    /// Adds another bounding box to this instance.
    /// </summary>
    /// <param name="other">The other box.</param>
    /// <returns>The summed <see cref="Box2"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Box2 Add(in Box2 other)
        => new(Vertex.Min(this.Min, other.Min), Vertex.Max(this.Max, other.Max));

    /// <inheritdoc/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public override bool Equals(object? obj)
        => obj is Box2 box
        && this.Equals(box);

    /// <inheritdoc/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool Equals(Box2 other)
        => this.Min == other.Min && this.Max == other.Max;

    /// <inheritdoc/>
    public override int GetHashCode() => HashCode.Combine(this.Min, this.Max);
}

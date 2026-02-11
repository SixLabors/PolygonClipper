// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System.Runtime.CompilerServices;
using System.Runtime.Intrinsics;

namespace SixLabors.PolygonClipper;

/// <summary>
/// Represents a two-dimensional fixed-precision vertex with X and Y coordinates.
/// </summary>
internal readonly struct Vertex64 : IEquatable<Vertex64>
{
    /// <summary>
    /// Gets the X-coordinate of the vertex.
    /// </summary>
#pragma warning disable CA1051 // Do not declare visible instance fields
    public readonly long X;

    /// <summary>
    /// Gets the Y-coordinate of the vertex.
    /// </summary>
    public readonly long Y;
#pragma warning restore CA1051 // Do not declare visible instance fields

    /// <summary>
    /// Initializes a new instance of the <see cref="Vertex64"/> struct.
    /// </summary>
    /// <param name="xy">The X and Y coordinates of the vertex.</param>
    public Vertex64(long xy)
    {
        this.X = xy;
        this.Y = xy;
    }

    /// <summary>
    /// Initializes a new instance of the <see cref="Vertex64"/> struct.
    /// </summary>
    /// <param name="x">The X-coordinate of the vertex.</param>
    /// <param name="y">The Y-coordinate of the vertex.</param>
    public Vertex64(long x, long y)
    {
        this.X = x;
        this.Y = y;
    }

    /// <summary>
    /// Adds two vertices together.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vertex64 operator +(in Vertex64 left, in Vertex64 right)
        => AsVertexUnsafe(AsVector128Unsafe(left) + AsVector128Unsafe(right));

    /// <summary>
    /// Subtracts the second vertex from the first.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vertex64 operator -(in Vertex64 left, in Vertex64 right)
        => AsVertexUnsafe(AsVector128Unsafe(left) - AsVector128Unsafe(right));

    /// <summary>
    /// Determines whether two vertices are equal.
    /// </summary>
    public static bool operator ==(in Vertex64 left, in Vertex64 right) => left.Equals(right);

    /// <summary>
    /// Determines whether two vertices are not equal.
    /// </summary>
    public static bool operator !=(in Vertex64 left, in Vertex64 right) => !left.Equals(right);

    /// <summary>
    /// Returns the dot product of two vertices.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Int128 Dot(in Vertex64 a, in Vertex64 b)
        => ((Int128)a.X * b.X) + ((Int128)a.Y * b.Y);

    /// <summary>
    /// Returns the cross product of two vertices.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Int128 Cross(in Vertex64 a, in Vertex64 b)
        => ((Int128)a.X * b.Y) - ((Int128)a.Y * b.X);

    /// <summary>
    /// Returns a vertex whose elements are the minimum of each component pair.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vertex64 Min(in Vertex64 a, in Vertex64 b)
        => AsVertexUnsafe(Vector128.Min(AsVector128Unsafe(a), AsVector128Unsafe(b)));

    /// <summary>
    /// Returns a vertex whose elements are the maximum of each component pair.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vertex64 Max(in Vertex64 a, in Vertex64 b)
        => AsVertexUnsafe(Vector128.Max(AsVector128Unsafe(a), AsVector128Unsafe(b)));

    /// <summary>
    /// Computes the absolute value of each element in a specified vertex.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vertex64 Abs(in Vertex64 value)
        => AsVertexUnsafe(Vector128.Abs(AsVector128Unsafe(value)));

    /// <inheritdoc/>
    public bool Equals(Vertex64 other)
        => this.X == other.X && this.Y == other.Y;

    /// <inheritdoc/>
    public override bool Equals(object? obj)
        => obj is Vertex64 other && this.Equals(other);

    /// <inheritdoc/>
    public override int GetHashCode()
        => HashCode.Combine(this.X, this.Y);

    /// <inheritdoc/>
    public override string ToString() => $"Vertex64 [ X={this.X}, Y={this.Y} ]";

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vector128<long> AsVector128Unsafe(in Vertex64 value)
        => Unsafe.BitCast<Vertex64, Vector128<long>>(value);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vertex64 AsVertexUnsafe(Vector128<long> value)
        => Unsafe.BitCast<Vector128<long>, Vertex64>(value);
}

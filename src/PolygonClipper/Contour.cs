// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System.Collections;
using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace SixLabors.PolygonClipper;

/// <summary>
/// Represents a simple polygon. The edges of the contours are interior disjoint.
/// </summary>
[DebuggerDisplay("Count = {Count}")]
#pragma warning disable CA1710 // Identifiers should have correct suffix
public sealed class Contour : IReadOnlyCollection<Vertex>
#pragma warning restore CA1710 // Identifiers should have correct suffix
{
    private bool hasCachedOrientation;
    private bool cachedCounterClockwise;

    /// <summary>
    /// Set of vertices conforming the external contour
    /// </summary>
    private readonly List<Vertex> vertices = [];

    /// <summary>
    /// Holes of the contour. They are stored as the indexes of
    /// the holes in a polygon class
    /// </summary>
    private readonly List<int> holeIndices = [];

    /// <summary>
    /// Gets the number of vertices.
    /// </summary>
    public int Count
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => this.vertices.Count;
    }

    /// <summary>
    /// Gets the number of holes.
    /// </summary>
    public int HoleCount => this.holeIndices.Count;

    /// <summary>
    /// Gets a value indicating whether the contour is external (not a hole).
    /// </summary>
    public bool IsExternal => this.ParentIndex == null;

    /// <summary>
    /// Gets or sets the index of the parent contour in the polygon if this contour is a hole.
    /// </summary>
    public int? ParentIndex { get; set; }

    /// <summary>
    /// Gets or sets the depth of the contour.
    /// </summary>
    public int Depth { get; set; }

    /// <summary>
    /// Gets the vertex at the specified index.
    /// </summary>
    /// <param name="index">The index of the vertex.</param>
    /// <returns>The <see cref="Vertex"/> at the specified index.</returns>
    public Vertex this[int index]
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => this.vertices[index];
    }

    /// <summary>
    /// Gets the hole index at the specified position in the contour.
    /// </summary>
    /// <param name="index">The index of the hole.</param>
    /// <returns>The hole index.</returns>
    public int GetHoleIndex(int index) => this.holeIndices[index];

    /// <summary>
    /// Gets the segment at the specified index of the contour.
    /// </summary>
    /// <param name="index">The index of the segment.</param>
    /// <returns>The <see cref="Segment"/>.</returns>
    internal Segment GetSegment(int index)
        => (index == this.Count - 1)
        ? new Segment(this.vertices[^1], this.vertices[0])
        : new Segment(this.vertices[index], this.vertices[index + 1]);

    /// <summary>
    /// Gets the bounding box of the contour.
    /// </summary>
    /// <returns>The <see cref="Box2"/>.</returns>
    public Box2 GetBoundingBox()
    {
        if (this.Count == 0)
        {
            return default;
        }

        List<Vertex> points = this.vertices;
        Box2 b = new(points[0]);
        for (int i = 1; i < points.Count; ++i)
        {
            b = b.Add(new Box2(points[i]));
        }

        return b;
    }

    /// <summary>
    /// Gets a value indicating whether the contour is counterclockwise oriented
    /// </summary>
    /// <returns>
    /// <see langword="true"/> if the contour is counterclockwise oriented; otherwise <see langword="false"/>.
    /// </returns>
    public bool IsCounterClockwise()
    {
        if (this.hasCachedOrientation)
        {
            return this.cachedCounterClockwise;
        }

        this.hasCachedOrientation = true;

        double area = 0;
        Vertex c;
        Vertex c1;

        List<Vertex> points = this.vertices;
        for (int i = 0; i < points.Count - 1; i++)
        {
            c = points[i];
            c1 = points[i + 1];
            area += Vertex.Cross(c, c1);
        }

        c = points[^1];
        c1 = points[0];
        area += Vertex.Cross(c, c1);
        return this.cachedCounterClockwise = area >= 0;
    }

    /// <summary>
    /// Gets a value indicating whether the contour is clockwise oriented
    /// </summary>
    /// <returns>
    /// <see langword="true"/> if the contour is clockwise oriented; otherwise <see langword="false"/>.
    /// </returns>
    public bool IsClockwise() => !this.IsCounterClockwise();

    /// <summary>
    /// Reverses the orientation of the contour.
    /// </summary>
    public void Reverse()
    {
        this.vertices.Reverse();
        this.cachedCounterClockwise = !this.cachedCounterClockwise;
    }

    /// <summary>
    /// Sets the contour to clockwise orientation.
    /// </summary>
    public void SetClockwise()
    {
        if (this.IsCounterClockwise())
        {
            this.Reverse();
        }
    }

    /// <summary>
    /// Sets the contour to counterclockwise orientation.
    /// </summary>
    public void SetCounterClockwise()
    {
        if (this.IsClockwise())
        {
            this.Reverse();
        }
    }

    /// <summary>
    /// Translates the contour by the specified x and y values.
    /// </summary>
    /// <param name="x">The x-coordinate offset.</param>
    /// <param name="y">The y-coordinate offset.</param>
    public void Translate(double x, double y)
    {
        List<Vertex> points = this.vertices;
        for (int i = 0; i < points.Count; i++)
        {
            points[i] += new Vertex(x, y);
        }
    }

    /// <summary>
    /// Adds a vertex to the end of the vertices collection.
    /// </summary>
    /// <param name="vertex">The vertex to add.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Add(in Vertex vertex) => this.vertices.Add(vertex);

    /// <summary>
    /// Removes the vertex at the specified index from the contour.
    /// </summary>
    /// <param name="index">The index of the vertex to remove.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void RemoveVertexAt(int index) => this.vertices.RemoveAt(index);

    /// <summary>
    /// Clears all vertices and holes from the contour.
    /// </summary>
    public void Clear()
    {
        this.vertices.Clear();
        this.holeIndices.Clear();
    }

    /// <summary>
    /// Clears all holes from the contour.
    /// </summary>
    public void ClearHoles() => this.holeIndices.Clear();

    /// <summary>
    /// Gets the last vertex in the contour.
    /// </summary>
    /// <returns>The last <see cref="Vertex"/> in the contour.</returns>
    public Vertex GetLastVertex() => this.vertices[^1];

    /// <summary>
    /// Adds a hole index to the contour.
    /// </summary>
    /// <param name="index">The index of the hole to add.</param>
    public void AddHoleIndex(int index) => this.holeIndices.Add(index);

    /// <inheritdoc/>
    public IEnumerator<Vertex> GetEnumerator()
        => ((IEnumerable<Vertex>)this.vertices).GetEnumerator();

    /// <inheritdoc/>
    IEnumerator IEnumerable.GetEnumerator()
        => ((IEnumerable)this.vertices).GetEnumerator();
}

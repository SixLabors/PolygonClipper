// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace PolygonClipper;

/// <summary>
/// Represents a simple polygon. The edges of the contours are interior disjoint.
/// </summary>
[DebuggerDisplay("Count = {VertexCount}")]
public sealed class Contour
{
    private bool precomputeCC;
    private bool cc;

    /// <summary>
    /// Set of points conforming the external contour
    /// </summary>
    private readonly List<Vertex> points = [];

    /// <summary>
    /// Holes of the contour. They are stored as the indexes of
    /// the holes in a polygon class
    /// </summary>
    private readonly List<int> holes = [];

    /// <summary>
    /// Gets the number of vertices.
    /// </summary>
    public int VertexCount => this.points.Count;

    /// <summary>
    /// Gets the number of edges.
    /// </summary>
    public int EdgeCount => this.points.Count;

    /// <summary>
    /// Gets the number of holes.
    /// </summary>
    public int HoleCount => this.holes.Count;

    /// <summary>
    /// Gets a value indicating whether the contour is external (not a hole).
    /// </summary>
    public bool IsExternal => this.HoleOf == null;

    /// <summary>
    /// Gets or sets the ID of the parent contour if this contour is a hole.
    /// </summary>
    public int? HoleOf { get; set; }

    /// <summary>
    /// Gets or sets the depth of the contour.
    /// </summary>
    public int Depth { get; set; }

    /// <summary>
    /// Gets the vertex at the specified index of the external contour.
    /// </summary>
    /// <param name="index">The index of the vertex.</param>
    /// <returns>The <see cref="Vertex"/>.</returns>
    public Vertex GetVertex(int index) => this.points[index];

    /// <summary>
    /// Gets the hole index at the specified position in the contour.
    /// </summary>
    /// <param name="index">The index of the hole.</param>
    /// <returns>The hole index.</returns>
    public int GetHoleIndex(int index) => this.holes[index];

    /// <summary>
    /// Gets the segment at the specified index of the contour.
    /// </summary>
    /// <param name="index">The index of the segment.</param>
    /// <returns>The <see cref="Segment"/>.</returns>
    internal Segment Segment(int index)
        => (index == this.VertexCount - 1)
        ? new Segment(this.points[^1], this.points[0])
        : new Segment(this.points[index], this.points[index + 1]);

    /// <summary>
    /// Gets the bounding box of the contour.
    /// </summary>
    /// <returns>The <see cref="Box2"/>.</returns>
    public Box2 GetBoundingBox()
    {
        if (this.VertexCount == 0)
        {
            return default;
        }

        List<Vertex> points = this.points;
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
        if (this.precomputeCC)
        {
            return this.cc;
        }

        this.precomputeCC = true;

        double area = 0;
        Vertex c;
        Vertex c1;

        List<Vertex> points = this.points;
        for (int i = 0; i < points.Count - 1; i++)
        {
            c = points[i];
            c1 = points[i + 1];
            area += Vertex.Cross(c, c1);
        }

        c = points[^1];
        c1 = points[0];
        area += Vertex.Cross(c, c1);
        return this.cc = area >= 0;
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
        this.points.Reverse();
        this.cc = !this.cc;
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
        List<Vertex> points = this.points;
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
    public void AddVertex(in Vertex vertex) => this.points.Add(vertex);

    /// <summary>
    /// Removes the vertex at the specified index from the contour.
    /// </summary>
    /// <param name="index">The index of the vertex to remove.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void RemoveVertexAt(int index) => this.points.RemoveAt(index);

    /// <summary>
    /// Clears all vertices and holes from the contour.
    /// </summary>
    public void Clear()
    {
        this.points.Clear();
        this.holes.Clear();
    }

    /// <summary>
    /// Clears all holes from the contour.
    /// </summary>
    public void ClearHoles() => this.holes.Clear();

    /// <summary>
    /// Gets the last vertex in the contour.
    /// </summary>
    /// <returns>The last <see cref="Vertex"/> in the contour.</returns>
    public Vertex GetLastVertex() => this.points[^1];

    /// <summary>
    /// Adds a hole index to the contour.
    /// </summary>
    /// <param name="index">The index of the hole to add.</param>
    public void AddHoleIndex(int index) => this.holes.Add(index);
}

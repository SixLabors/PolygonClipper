// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System.Collections;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace SixLabors.PolygonClipper;

/// <summary>
/// Represents an open or closed sequence of vertices.
/// </summary>
/// <remarks>
/// Stroking respects <see cref="IsClosed"/>. Filled-area operations implicitly close the
/// contour regardless of that value. A repeated closing vertex is optional on input.
/// </remarks>
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
    /// Initializes a new instance of the <see cref="Contour"/> class.
    /// </summary>
    public Contour()
        => this.vertices = [];

    /// <summary>
    /// Initializes a new instance of the <see cref="Contour"/> class with a vertex capacity.
    /// </summary>
    /// <param name="capacity">The initial vertex capacity.</param>
    public Contour(int capacity)
        => this.vertices = new List<Vertex>(capacity);

    /// <summary>
    /// Gets the number of stored vertices.
    /// </summary>
    public int Count
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => this.vertices.Count;
    }

    /// <summary>
    /// Gets or sets a value indicating whether the source contour is closed.
    /// </summary>
    /// <remarks>
    /// The default is true. Stroking respects this value.
    /// Filled-area operations implicitly close the contour regardless of this value.
    /// </remarks>
    public bool IsClosed { get; set; } = true;

    /// <summary>
    /// Gets the number of directly nested child contours.
    /// </summary>
    public int HoleCount => this.holeIndices.Count;

    /// <summary>
    /// Gets a value indicating whether the contour has no parent contour.
    /// </summary>
    /// <remarks>In a full containment hierarchy, even depths identify exterior contours and islands.</remarks>
    public bool IsExternal => this.ParentIndex == null;

    /// <summary>
    /// Gets or sets the index of the contour that directly contains this contour.
    /// </summary>
    public int? ParentIndex { get; set; }

    /// <summary>
    /// Gets or sets the number of ancestors of this contour.
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
    /// Gets the child contour index at the specified position.
    /// </summary>
    /// <param name="index">The position in the child contour list.</param>
    /// <returns>The child's index in the owning polygon.</returns>
    public int GetHoleIndex(int index) => this.holeIndices[index];

    /// <summary>
    /// Gets the filled-area boundary segment at the specified index.
    /// </summary>
    /// <param name="index">The index of the segment.</param>
    /// <returns>The <see cref="Segment"/>. The final segment wraps from last vertex to first vertex.</returns>
    internal Segment GetSegment(int index)
        => (index == this.Count - 1)
        ? new Segment(this.vertices[^1], this.vertices[0])
        : new Segment(this.vertices[index], this.vertices[index + 1]);

    /// <summary>
    /// Gets a read-only span of the vertices in this contour.
    /// </summary>
    /// <returns>The vertex span.</returns>
    /// <remarks>Do not add or remove vertices while using the span.</remarks>
    internal ReadOnlySpan<Vertex> AsVertexSpan() => CollectionsMarshal.AsSpan(this.vertices);

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
    /// Determines whether the implicitly closed contour has counterclockwise orientation.
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

        return this.cachedCounterClockwise = PolygonUtilities.Area(this.vertices) >= 0;
    }

    /// <summary>
    /// Determines whether the implicitly closed contour has clockwise orientation.
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
    public void Add(in Vertex vertex)
    {
        this.vertices.Add(vertex);
        this.hasCachedOrientation = false;
    }

    /// <summary>
    /// Removes the vertex at the specified index from the contour.
    /// </summary>
    /// <param name="index">The index of the vertex to remove.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void RemoveVertexAt(int index)
    {
        this.vertices.RemoveAt(index);
        this.hasCachedOrientation = false;
    }

    /// <summary>
    /// Clears all vertices and child contour indices.
    /// </summary>
    /// <remarks>The closure state is unchanged.</remarks>
    public void Clear()
    {
        this.vertices.Clear();
        this.holeIndices.Clear();
        this.hasCachedOrientation = false;
    }

    /// <summary>
    /// Clears all child contour indices.
    /// </summary>
    public void ClearHoles() => this.holeIndices.Clear();

    /// <summary>
    /// Gets the last vertex in the contour.
    /// </summary>
    /// <returns>The last <see cref="Vertex"/> in the contour.</returns>
    public Vertex GetLastVertex() => this.vertices[^1];

    /// <summary>
    /// Adds an index to the child contour list.
    /// </summary>
    /// <param name="index">The child's index in the owning polygon.</param>
    public void AddHoleIndex(int index) => this.holeIndices.Add(index);

    /// <summary>
    /// Moves hierarchy indices into a polygon that already contains contours.
    /// </summary>
    /// <param name="offset">The number of preceding contours in the destination.</param>
    internal void OffsetHierarchyIndices(int offset)
    {
        if (this.ParentIndex is int parent)
        {
            this.ParentIndex = parent + offset;
        }

        // Update the owning list in place. Join shares contour instances, so clearing
        // and then reading the source's holes would erase the same list.
        for (int i = 0; i < this.holeIndices.Count; i++)
        {
            this.holeIndices[i] += offset;
        }
    }

    /// <summary>
    /// Remaps hierarchy indices after contours have been reordered.
    /// </summary>
    /// <param name="indices">The destination index for each original contour index.</param>
    internal void RemapHierarchyIndices(ReadOnlySpan<int> indices)
    {
        if (this.ParentIndex is int parent)
        {
            this.ParentIndex = indices[parent];
        }

        for (int i = 0; i < this.holeIndices.Count; i++)
        {
            this.holeIndices[i] = indices[this.holeIndices[i]];
        }
    }

    /// <summary>
    /// Creates a deep copy of this contour.
    /// </summary>
    /// <returns>A detached contour copy.</returns>
    public Contour DeepClone()
    {
        Contour clone = new(this.vertices.Count)
        {
            IsClosed = this.IsClosed,
            ParentIndex = this.ParentIndex,
            Depth = this.Depth,
            hasCachedOrientation = this.hasCachedOrientation,
            cachedCounterClockwise = this.cachedCounterClockwise
        };

        clone.vertices.AddRange(this.vertices);
        clone.holeIndices.AddRange(this.holeIndices);

        return clone;
    }

    /// <inheritdoc/>
    public IEnumerator<Vertex> GetEnumerator()
        => ((IEnumerable<Vertex>)this.vertices).GetEnumerator();

    /// <inheritdoc/>
    IEnumerator IEnumerable.GetEnumerator()
        => ((IEnumerable)this.vertices).GetEnumerator();
}

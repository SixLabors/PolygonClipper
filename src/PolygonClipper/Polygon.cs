// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System.Collections;
using System.Runtime.CompilerServices;
using System.Text;

namespace SixLabors.PolygonClipper;

/// <summary>
/// Represents a complex polygon.
/// </summary>
#pragma warning disable CA1710 // Identifiers should have correct suffix
public sealed class Polygon : IReadOnlyCollection<Contour>
#pragma warning restore CA1710 // Identifiers should have correct suffix
{
    /// <summary>
    /// The collection of contours that make up the polygon.
    /// </summary>
    private readonly List<Contour> contours;

    /// <summary>
    /// Initializes a new instance of the <see cref="Polygon"/> class.
    /// </summary>
    public Polygon()
        => this.contours = [];

    /// <summary>
    /// Initializes a new instance of the <see cref="Polygon"/> class with a contour capacity.
    /// </summary>
    /// <param name="capacity">The initial contour capacity.</param>
    public Polygon(int capacity)
        => this.contours = new List<Contour>(capacity);

    /// <summary>
    /// Gets the number of contours in the polygon.
    /// </summary>
    public int Count
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => this.contours.Count;
    }

    /// <summary>
    /// Gets the total number of vertices across all contours in the polygon.
    /// </summary>
    /// <returns>The total vertex count.</returns>
    public int VertexCount
    {
        get
        {
            int count = 0;
            for (int i = 0; i < this.contours.Count; i++)
            {
                count += this.contours[i].Count;
            }

            return count;
        }
    }

    /// <summary>
    /// Gets the contour at the specified index.
    /// </summary>
    /// <param name="index">The index of the contour.</param>
    /// <returns>The <see cref="Contour"/> at the given index.</returns>
    public Contour this[int index]
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => this.contours[index];
    }

    /// <summary>
    /// Joins another polygon to this instance.
    /// </summary>
    /// <param name="polygon">The polygon to join.</param>
    public void Join(Polygon polygon)
    {
        ArgumentNullException.ThrowIfNull(polygon);

        int sourceCount = polygon.contours.Count;
        int contourIndexOffset = this.contours.Count;
        for (int i = 0; i < sourceCount; ++i)
        {
            this.contours.Add(polygon.contours[i].DeepClone(contourIndexOffset));
        }
    }

    /// <summary>
    /// Gets the bounding box.
    /// </summary>
    /// <returns>The <see cref="Box2"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Box2 GetBoundingBox()
    {
        if (this.Count == 0)
        {
            return default;
        }

        Box2 b = this.contours[0].GetBoundingBox();
        for (int i = 1; i < this.Count; i++)
        {
            b = b.Add(this.contours[i].GetBoundingBox());
        }

        return b;
    }

    /// <summary>
    /// Translates the polygon by the specified x and y values.
    /// </summary>
    /// <param name="x">The x-coordinate offset.</param>
    /// <param name="y">The y-coordinate offset.</param>
    public void Translate(double x, double y)
    {
        for (int i = 0; i < this.contours.Count; i++)
        {
            this.contours[i].Translate(x, y);
        }
    }

    /// <summary>
    /// Adds a contour to the end of the contour collection.
    /// </summary>
    /// <param name="contour">The contour to add.</param>
    public void Add(Contour contour) => this.contours.Add(contour);

    /// <summary>
    /// Gets the last contour in the polygon.
    /// </summary>
    /// <returns>The last <see cref="Contour"/> in the collection.</returns>
    public Contour GetLastContour() => this.contours[^1];

    /// <summary>
    /// Clears all contours from the polygon.
    /// </summary>
    public void Clear() => this.contours.Clear();

    /// <summary>
    /// Creates a deep copy of this polygon and all of its contours.
    /// </summary>
    /// <returns>A detached polygon copy.</returns>
    internal Polygon DeepClone()
    {
        Polygon clone = new(this.contours.Count);
        for (int i = 0; i < this.contours.Count; i++)
        {
            clone.contours.Add(this.contours[i].DeepClone());
        }

        return clone;
    }

    /// <inheritdoc/>
    public IEnumerator<Contour> GetEnumerator()
        => ((IEnumerable<Contour>)this.contours).GetEnumerator();

    /// <inheritdoc/>
    IEnumerator IEnumerable.GetEnumerator()
        => ((IEnumerable)this.contours).GetEnumerator();

    /// <summary>
    /// Creates a string useful for debugging.
    /// </summary>
    /// <returns>The <see cref="string"/>.</returns>
    public string ToDebugString()
    {
        StringBuilder stringBuilder = new();
        stringBuilder.AppendLine("[");

        foreach (Contour contour in this.contours)
        {
            stringBuilder.AppendLine("    [");
            foreach (Vertex vertex in contour)
            {
                stringBuilder.AppendLine("        new Vertex(" + vertex.X + ", " + vertex.Y + "),");
            }

            stringBuilder.AppendLine("    ],");
        }

        stringBuilder.AppendLine("];");

        return stringBuilder.ToString();
    }
}

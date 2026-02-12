// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System.Collections.Generic;

namespace SixLabors.PolygonClipper;

/// <summary>
/// Captures a clipped contour, its topology, and its ownership hierarchy.
/// </summary>
internal sealed class OutputRecord
{
    /// <summary>
    /// Gets or sets the stable index assigned when the record is pooled.
    /// </summary>
    public int Index { get; set; }

    /// <summary>
    /// Gets or sets the number of output points in the contour.
    /// </summary>
    public int OutputPointCount { get; set; }

    /// <summary>
    /// Gets or sets the containing output record, if any.
    /// </summary>
    public OutputRecord? Owner { get; set; }

    /// <summary>
    /// Gets or sets the front edge that defines the output orientation.
    /// </summary>
    public ActiveEdge? FrontEdge { get; set; }

    /// <summary>
    /// Gets or sets the back edge that defines the output orientation.
    /// </summary>
    public ActiveEdge? BackEdge { get; set; }

    /// <summary>
    /// Gets or sets the circular linked list of output points.
    /// </summary>
    public OutputPoint? Points { get; set; }

    /// <summary>
    /// Gets or sets the cached bounds for ownership tests.
    /// </summary>
    public Box64 Bounds { get; set; }

    /// <summary>
    /// Gets or sets the temporary contour used during bounds checks.
    /// </summary>
    public List<Vertex64> Path { get; set; } = [];

    /// <summary>
    /// Gets or sets split indices used to resolve complex self-intersections.
    /// </summary>
    public List<int>? Splits { get; set; }

    /// <summary>
    /// Gets or sets the cached split ownership used to avoid recursion.
    /// </summary>
    public OutputRecord? RecursiveSplit { get; set; }
}

/// <summary>
/// Represents a vertex in the output contour linked list.
/// </summary>
internal sealed class OutputPoint
{
#pragma warning disable SA1401 // Hot output ring traversal uses fields to avoid accessor overhead.
    /// <summary>
    /// The vertex coordinate.
    /// </summary>
    public Vertex64 Point;

    /// <summary>
    /// The next point in the linked list.
    /// </summary>
    public OutputPoint? Next;

    /// <summary>
    /// The previous point in the linked list.
    /// </summary>
    public OutputPoint Prev;

    /// <summary>
    /// The owning output record.
    /// </summary>
    public OutputRecord OutputRecord;

    /// <summary>
    /// The horizontal segment reference used for joins.
    /// </summary>
    public HorizontalSegment? HorizontalSegment;
#pragma warning restore SA1401

    /// <summary>
    /// Initializes a new instance of the <see cref="OutputPoint"/> class.
    /// </summary>
    public OutputPoint(Vertex64 point, OutputRecord outputRecord)
    {
        this.Point = point;
        this.OutputRecord = outputRecord;
        this.Next = this;
        this.Prev = this;
        this.HorizontalSegment = null;
    }
}

/// <summary>
/// Captures a pending horizontal segment to be joined.
/// </summary>
internal sealed class HorizontalSegment
{
    /// <summary>
    /// Initializes a new instance of the <see cref="HorizontalSegment"/> class.
    /// </summary>
    public HorizontalSegment(OutputPoint op)
    {
        this.LeftPoint = op;
        this.RightPoint = null;
        this.LeftToRight = true;
    }

    /// <summary>
    /// Gets or sets the left-most point of the segment.
    /// </summary>
    public OutputPoint? LeftPoint { get; set; }

    /// <summary>
    /// Gets or sets the right-most point of the segment.
    /// </summary>
    public OutputPoint? RightPoint { get; set; }

    /// <summary>
    /// Gets or sets a value indicating whether the segment runs left-to-right.
    /// </summary>
    public bool LeftToRight { get; set; }
}

/// <summary>
/// Stores a pair of horizontal edges to be joined.
/// </summary>
internal sealed class HorizontalJoin
{
    /// <summary>
    /// Initializes a new instance of the <see cref="HorizontalJoin"/> class.
    /// </summary>
    public HorizontalJoin(OutputPoint leftToRight, OutputPoint rightToLeft)
    {
        this.LeftToRight = leftToRight;
        this.RightToLeft = rightToLeft;
    }

    /// <summary>
    /// Gets or sets the left-to-right point of the join.
    /// </summary>
    public OutputPoint? LeftToRight { get; set; }

    /// <summary>
    /// Gets or sets the right-to-left point of the join.
    /// </summary>
    public OutputPoint? RightToLeft { get; set; }
}

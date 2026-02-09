// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

namespace SixLabors.PolygonClipper;

/// <summary>
/// Captures a clipped contour, its topology, and its ownership hierarchy.
/// </summary>
internal sealed class OutputRecord
{
    /// <summary>
    /// Gets or sets the stable index assigned when the record is pooled.
    /// </summary>
    internal int Index { get; set; }

    /// <summary>
    /// Gets or sets the number of output points in the contour.
    /// </summary>
    internal int OutputPointCount { get; set; }

    /// <summary>
    /// Gets or sets the containing output record, if any.
    /// </summary>
    internal OutputRecord? Owner { get; set; }

    /// <summary>
    /// Gets or sets the front edge that defines the output orientation.
    /// </summary>
    internal ActiveEdge? FrontEdge { get; set; }

    /// <summary>
    /// Gets or sets the back edge that defines the output orientation.
    /// </summary>
    internal ActiveEdge? BackEdge { get; set; }

    /// <summary>
    /// Gets or sets the circular linked list of output points.
    /// </summary>
    internal OutputPoint? Points { get; set; }

    /// <summary>
    /// Gets or sets the cached bounds for ownership tests.
    /// </summary>
    internal Box2 Bounds { get; set; }

    /// <summary>
    /// Gets or sets the temporary contour used during bounds checks.
    /// </summary>
    internal Contour Path { get; set; } = [];

    /// <summary>
    /// Gets or sets split indices used to resolve complex self-intersections.
    /// </summary>
    internal List<int>? Splits { get; set; }

    /// <summary>
    /// Gets or sets the cached split ownership used to avoid recursion.
    /// </summary>
    internal OutputRecord? RecursiveSplit { get; set; }
}

/// <summary>
/// Represents a vertex in the output contour linked list.
/// </summary>
internal sealed class OutputPoint
{
    /// <summary>
    /// Initializes a new instance of the <see cref="OutputPoint"/> class.
    /// </summary>
    internal OutputPoint(Vertex point, OutputRecord outputRecord)
    {
        this.Point = point;
        this.OutputRecord = outputRecord;
        this.Next = this;
        this.Prev = this;
        this.HorizontalSegment = null;
    }

    /// <summary>
    /// Gets or sets the vertex coordinate.
    /// </summary>
    internal Vertex Point { get; set; }

    /// <summary>
    /// Gets or sets the next point in the linked list.
    /// </summary>
    internal OutputPoint? Next { get; set; }

    /// <summary>
    /// Gets or sets the previous point in the linked list.
    /// </summary>
    internal OutputPoint Prev { get; set; }

    /// <summary>
    /// Gets or sets the owning output record.
    /// </summary>
    internal OutputRecord OutputRecord { get; set; }

    /// <summary>
    /// Gets or sets the horizontal segment reference used for joins.
    /// </summary>
    internal HorizontalSegment? HorizontalSegment { get; set; }
}

/// <summary>
/// Captures a pending horizontal segment to be joined.
/// </summary>
internal sealed class HorizontalSegment
{
    /// <summary>
    /// Initializes a new instance of the <see cref="HorizontalSegment"/> class.
    /// </summary>
    internal HorizontalSegment(OutputPoint op)
    {
        this.LeftPoint = op;
        this.RightPoint = null;
        this.LeftToRight = true;
    }

    /// <summary>
    /// Gets or sets the left-most point of the segment.
    /// </summary>
    internal OutputPoint? LeftPoint { get; set; }

    /// <summary>
    /// Gets or sets the right-most point of the segment.
    /// </summary>
    internal OutputPoint? RightPoint { get; set; }

    /// <summary>
    /// Gets or sets a value indicating whether the segment runs left-to-right.
    /// </summary>
    internal bool LeftToRight { get; set; }
}

/// <summary>
/// Stores a pair of horizontal edges to be joined.
/// </summary>
internal sealed class HorizontalJoin
{
    /// <summary>
    /// Initializes a new instance of the <see cref="HorizontalJoin"/> class.
    /// </summary>
    internal HorizontalJoin(OutputPoint leftToRight, OutputPoint rightToLeft)
    {
        this.LeftToRight = leftToRight;
        this.RightToLeft = rightToLeft;
    }

    /// <summary>
    /// Gets or sets the left-to-right point of the join.
    /// </summary>
    internal OutputPoint? LeftToRight { get; set; }

    /// <summary>
    /// Gets or sets the right-to-left point of the join.
    /// </summary>
    internal OutputPoint? RightToLeft { get; set; }
}

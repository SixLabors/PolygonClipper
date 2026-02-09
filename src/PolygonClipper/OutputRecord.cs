// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

namespace SixLabors.PolygonClipper;

// OutputRecord: path data structure for clipping solutions
internal sealed class OutputRecord
{
    internal int Index { get; set; }

    internal int OutputPointCount { get; set; }

    internal OutputRecord? Owner { get; set; }

    internal Active? FrontEdge { get; set; }

    internal Active? BackEdge { get; set; }

    internal OutputPoint? Points { get; set; }

    internal Box2 Bounds { get; set; }

    internal Contour Path { get; set; } = [];

    internal bool IsOpen { get; set; }

    internal List<int>? Splits { get; set; }

    internal OutputRecord? RecursiveSplit { get; set; }
}

// OutputPoint: ClipVertex data structure for clipping solutions
internal sealed class OutputPoint
{
    internal OutputPoint(Vertex point, OutputRecord outputRecord)
    {
        this.Point = point;
        this.OutputRecord = outputRecord;
        this.Next = this;
        this.Prev = this;
        this.HorizontalSegment = null;
    }

    internal Vertex Point { get; set; }

    internal OutputPoint? Next { get; set; }

    internal OutputPoint Prev { get; set; }

    internal OutputRecord OutputRecord { get; set; }

    internal HorizontalSegment? HorizontalSegment { get; set; }
}

internal sealed class HorizontalSegment
{
    internal HorizontalSegment(OutputPoint op)
    {
        this.LeftPoint = op;
        this.RightPoint = null;
        this.LeftToRight = true;
    }

    internal OutputPoint? LeftPoint { get; set; }

    internal OutputPoint? RightPoint { get; set; }

    internal bool LeftToRight { get; set; }
}

internal sealed class HorizontalJoin
{
    internal HorizontalJoin(OutputPoint leftToRight, OutputPoint rightToLeft)
    {
        this.LeftToRight = leftToRight;
        this.RightToLeft = rightToLeft;
    }

    internal OutputPoint? LeftToRight { get; set; }

    internal OutputPoint? RightToLeft { get; set; }
}

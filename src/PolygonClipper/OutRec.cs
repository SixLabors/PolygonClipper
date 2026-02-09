// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

namespace SixLabors.PolygonClipper;

// OutRec: path data structure for clipping solutions
internal sealed class OutRec
{
    internal int Index { get; set; }

    internal int OutPointCount { get; set; }

    internal OutRec? Owner { get; set; }

    internal Active? FrontEdge { get; set; }

    internal Active? BackEdge { get; set; }

    internal OutPt? Points { get; set; }

    internal PolyPathBase? PolyPath { get; set; }

    internal Box2 Bounds { get; set; }

    internal Contour Path { get; set; } = [];

    internal bool IsOpen { get; set; }

    internal List<int>? Splits { get; set; }

    internal OutRec? RecursiveSplit { get; set; }
}

// OutPt: ClipVertex data structure for clipping solutions
internal sealed class OutPt
{
    internal OutPt(Vertex point, OutRec outrec)
    {
        this.Point = point;
        this.OutRec = outrec;
        this.Next = this;
        this.Prev = this;
        this.Horz = null;
    }

    internal Vertex Point { get; set; }

    internal OutPt? Next { get; set; }

    internal OutPt Prev { get; set; }

    internal OutRec OutRec { get; set; }

    internal HorzSegment? Horz { get; set; }
}

internal sealed class HorzSegment
{
    internal HorzSegment(OutPt op)
    {
        this.LeftOp = op;
        this.RightOp = null;
        this.LeftToRight = true;
    }

    internal OutPt? LeftOp { get; set; }

    internal OutPt? RightOp { get; set; }

    internal bool LeftToRight { get; set; }
}

internal sealed class HorzJoin
{
    internal HorzJoin(OutPt leftToRight, OutPt rightToLeft)
    {
        this.Op1 = leftToRight;
        this.Op2 = rightToLeft;
    }

    internal OutPt? Op1 { get; set; }

    internal OutPt? Op2 { get; set; }
}

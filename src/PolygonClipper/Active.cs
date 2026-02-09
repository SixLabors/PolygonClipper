// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System.Runtime.CompilerServices;

namespace SixLabors.PolygonClipper;

///////////////////////////////////////////////////////////////////
// Important: UP and DOWN here are premised on Y-axis positive down
// displays, which is the orientation used in Clipper's development.
///////////////////////////////////////////////////////////////////
internal sealed class Active
{
    internal Vertex Bot { get; set; }

    internal Vertex Top { get; set; }

    internal double CurrentX { get; set; } // current (updated at every new scanline)

    internal double Dx { get; set; }

    internal int WindDelta { get; set; } // 1 or -1 depending on winding direction

    internal int WindCount { get; set; }

    internal int WindCount2 { get; set; } // winding count of the opposite polytype

    internal OutRec? OutRec { get; set; }

    // AEL: 'active edge list' (Vatti's AET - active edge table)
    //     a linked list of all edges (from left to right) that are present
    //     (or 'active') within the current scanbeam (a horizontal 'beam' that
    //     sweeps from bottom to top over the paths in the clipping operation).
    internal Active? PrevInAel { get; set; }

    internal Active? NextInAel { get; set; }

    // SEL: 'sorted edge list' (Vatti's ST - sorted table)
    //     linked list used when sorting edges into their new positions at the
    //     top of scanbeams, but also (re)used to process horizontals.
    internal Active? PrevInSel { get; set; }

    internal Active? NextInSel { get; set; }

    internal Active? Jump { get; set; }

    internal ClipVertex? VertexTop { get; set; }

    internal LocalMinima LocalMin { get; set; } // the bottom of an edge 'bound' (also Vatti)

    internal bool IsLeftBound { get; set; }

    internal JoinWith JoinWith { get; set; }

    internal bool IsOpen => this.LocalMin.IsOpen;

    internal bool IsHot => this.OutRec != null;

    internal bool IsOpenEnd => this.IsOpen && this.VertexTop != null && this.VertexTop.IsOpenEnd;

    internal bool IsHorizontal => PolygonUtilities.IsAlmostZero(this.Top.Y - this.Bot.Y);

    internal bool IsHeadingRightHorz => double.IsNegativeInfinity(this.Dx);

    internal bool IsHeadingLeftHorz => double.IsPositiveInfinity(this.Dx);

    internal bool IsMaxima => this.VertexTop != null && this.VertexTop.IsMaxima;

    internal bool IsFront => this.OutRec != null && this == this.OutRec.FrontEdge;

    internal ClipVertex NextVertex => this.WindDelta > 0 ? this.VertexTop!.Next! : this.VertexTop!.Prev!;

    internal ClipVertex PrevPrevVertex => this.WindDelta > 0 ? this.VertexTop!.Prev!.Prev! : this.VertexTop!.Next!.Next!;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal Active? GetPrevHotEdge()
    {
        Active? prev = this.PrevInAel;
        while (prev != null && (prev.IsOpen || !prev.IsHot))
        {
            prev = prev.PrevInAel;
        }

        return prev;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal double TopX(double currentY)
    {
        if (PolygonUtilities.IsAlmostZero(currentY - this.Top.Y) || PolygonUtilities.IsAlmostZero(this.Top.X - this.Bot.X))
        {
            return this.Top.X;
        }

        if (PolygonUtilities.IsAlmostZero(currentY - this.Bot.Y))
        {
            return this.Bot.X;
        }

        return this.Bot.X + (this.Dx * (currentY - this.Bot.Y));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal void UpdateDx() => this.Dx = GetDx(this.Bot, this.Top);

    private static double GetDx(Vertex pt1, Vertex pt2)
    {
        double dy = pt2.Y - pt1.Y;
        if (dy != 0)
        {
            return (pt2.X - pt1.X) / dy;
        }

        return pt2.X > pt1.X ? double.NegativeInfinity : double.PositiveInfinity;
    }
}

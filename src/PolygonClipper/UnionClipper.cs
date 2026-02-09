// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System.Runtime.CompilerServices;

namespace SixLabors.PolygonClipper;

internal sealed class UnionClipper
{
    private ClipperOperation clipType;
    private ClipperFillRule fillRule;
    private Active? activeEdges;
    private Active? sortedEdges;
    private readonly Stack<Active> freeActives;
    private readonly List<LocalMinima> minimaList;
    private readonly List<IntersectNode> intersectList;
    private readonly VertexPoolList vertexList;
    private readonly OutRecPoolList outrecPool;
    private readonly List<double> scanlineList;
    private readonly List<HorzSegment> horizontalSegments;
    private readonly HorzJoinPoolList horizontalJoins;
    private readonly OutPtPoolList outPointPool;
    private readonly List<Contour> pathPool;
    private int currentMinimaIndex;
    private double currentBottomY;
    private bool isSortedMinimaList;
    private bool hasOpenPaths;
    private bool usingPolyTree;
    private bool succeeded;
    private int pathPoolIndex;

    internal UnionClipper()
    {
        this.minimaList = [];
        this.intersectList = [];
        this.vertexList = [];
        this.outrecPool = new OutRecPoolList();
        this.scanlineList = [];
        this.horizontalSegments = [];
        this.horizontalJoins = [];
        this.outPointPool = [];
        this.pathPool = [];
        this.freeActives = new Stack<Active>();
        this.PreserveCollinear = true;
    }

    internal bool PreserveCollinear { get; set; }

    internal bool ReverseSolution { get; set; }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsOdd(int val) => (val & 1) != 0;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void SwapActives(ref Active ae1, ref Active ae2) => (ae2, ae1) = (ae1, ae2);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static ClipperPathType GetPolyType(Active ae) => ae.LocalMin.PathType;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsSamePolyType(Active ae1, Active ae2) => ae1.LocalMin.PathType == ae2.LocalMin.PathType;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Active? GetMaximaPair(Active ae)
    {
        Active? ae2 = ae.NextInAel;
        while (ae2 != null)
        {
            if (ae2.VertexTop == ae.VertexTop)
            {
                // Found a matching maxima pair.
                return ae2;
            }

            ae2 = ae2.NextInAel;
        }

        return null;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static ClipVertex? GetCurrYMaximaVertex_Open(Active ae)
    {
        ClipVertex? result = ae.VertexTop;
        if (ae.WindDelta > 0)
        {
            while (ClipGeometry.IsAlmostZero(result!.Next!.Point.Y - result.Point.Y) &&
                ((result.Flags & (VertexFlags.OpenEnd |
                VertexFlags.LocalMax)) == VertexFlags.None))
            {
                result = result.Next;
            }
        }
        else
        {
            while (ClipGeometry.IsAlmostZero(result!.Prev!.Point.Y - result.Point.Y) &&
                ((result.Flags & (VertexFlags.OpenEnd |
                VertexFlags.LocalMax)) == VertexFlags.None))
            {
                result = result.Prev;
            }
        }

        if (!result.IsMaxima)
        {
            // Not a maxima.
            result = null;
        }

        return result;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static ClipVertex? GetCurrYMaximaVertex(Active ae)
    {
        ClipVertex? result = ae.VertexTop;
        if (ae.WindDelta > 0)
        {
            while (ClipGeometry.IsAlmostZero(result!.Next!.Point.Y - result.Point.Y))
            {
                result = result.Next;
            }
        }
        else
        {
            while (ClipGeometry.IsAlmostZero(result!.Prev!.Point.Y - result.Point.Y))
            {
                result = result.Prev;
            }
        }

        if (!result.IsMaxima)
        {
            // Not a maxima.
            result = null;
        }

        return result;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void SetSides(OutRec outrec, Active startEdge, Active endEdge)
    {
        outrec.FrontEdge = startEdge;
        outrec.BackEdge = endEdge;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void SwapOutrecs(Active ae1, Active ae2)
    {
        // At least one edge has an assigned outrec.
        OutRec? or1 = ae1.OutRec;
        OutRec? or2 = ae2.OutRec;
        if (or1 == or2)
        {
            Active? ae = or1!.FrontEdge;
            or1.FrontEdge = or1.BackEdge;
            or1.BackEdge = ae;
            return;
        }

        if (or1 != null)
        {
            if (ae1 == or1.FrontEdge)
            {
                or1.FrontEdge = ae2;
            }
            else
            {
                or1.BackEdge = ae2;
            }
        }

        if (or2 != null)
        {
            if (ae2 == or2.FrontEdge)
            {
                or2.FrontEdge = ae1;
            }
            else
            {
                or2.BackEdge = ae1;
            }
        }

        ae1.OutRec = or2;
        ae2.OutRec = or1;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void SetOwner(OutRec outrec, OutRec newOwner)
    {
        // precondition1: new_owner is never null
        while (newOwner.Owner != null && newOwner.Owner.Points == null)
        {
            newOwner.Owner = newOwner.Owner.Owner;
        }

        // make sure that outrec isn't an owner of newOwner
        OutRec? tmp = newOwner;
        while (tmp != null && tmp != outrec)
        {
            tmp = tmp.Owner;
        }

        if (tmp != null)
        {
            newOwner.Owner = outrec.Owner;
        }

        outrec.Owner = newOwner;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static double Area(OutPt op)
    {
        // https://en.wikipedia.org/wiki/Shoelace_formula
        double area = 0.0;
        OutPt op2 = op;
        do
        {
            area += (double)(op2.Prev.Point.Y + op2.Point.Y) *
                (op2.Prev.Point.X - op2.Point.X);
            op2 = op2.Next!;
        }
        while (op2 != op);
        return area * 0.5;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static double AreaTriangle(Vertex pt1, Vertex pt2, Vertex pt3) => ((double)(pt3.Y + pt1.Y) * (pt3.X - pt1.X)) +
            ((double)(pt1.Y + pt2.Y) * (pt1.X - pt2.X)) +
            ((double)(pt2.Y + pt3.Y) * (pt2.X - pt3.X));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static OutRec? GetRealOutRec(OutRec? outRec)
    {
        while ((outRec != null) && (outRec.Points == null))
        {
            outRec = outRec.Owner;
        }

        return outRec;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsValidOwner(OutRec? outRec, OutRec? testOwner)
    {
        while ((testOwner != null) && (testOwner != outRec))
        {
            testOwner = testOwner.Owner;
        }

        return testOwner == null;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void UncoupleOutRec(Active ae)
    {
        OutRec? outrec = ae.OutRec;
        if (outrec == null)
        {
            return;
        }

        outrec.FrontEdge!.OutRec = null;
        outrec.BackEdge!.OutRec = null;
        outrec.FrontEdge = null;
        outrec.BackEdge = null;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool OutrecIsAscending(Active hotEdge) => hotEdge == hotEdge.OutRec!.FrontEdge;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void SwapFrontBackSides(OutRec outrec)
    {
        // while this proc. is needed for open paths
        // it's almost never needed for closed paths
        Active ae2 = outrec.FrontEdge!;
        outrec.FrontEdge = outrec.BackEdge;
        outrec.BackEdge = ae2;
        outrec.Points = outrec.Points!.Next;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool EdgesAdjacentInAEL(IntersectNode inode) => (inode.Edge1.NextInAel == inode.Edge2) || (inode.Edge1.PrevInAel == inode.Edge2);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void ClearSolutionOnly()
    {
        while (this.activeEdges != null)
        {
            this.DeleteFromAEL(this.activeEdges);
        }

        this.scanlineList.Clear();
        this.DisposeIntersectNodes();
        this.outrecPool.Clear();
        this.horizontalSegments.Clear();
        this.horizontalJoins.Clear();
        this.outPointPool.Clear();

        // Keep pooled actives between runs to reduce allocations when the clipper is reused.
        this.pathPoolIndex = 0;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal void Clear()
    {
        this.ClearSolutionOnly();
        this.minimaList.Clear();
        this.vertexList.Clear();
        this.currentMinimaIndex = 0;
        this.isSortedMinimaList = false;
        this.hasOpenPaths = false;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void Reset()
    {
        if (!this.isSortedMinimaList)
        {
            this.minimaList.Sort(default(LocalMinimaComparer));
            this.isSortedMinimaList = true;
        }

        this.scanlineList.EnsureCapacity(this.minimaList.Count);
        for (int i = this.minimaList.Count - 1; i >= 0; i--)
        {
            this.scanlineList.Add(this.minimaList[i].Vertex.Point.Y);
        }

        this.currentBottomY = 0;
        this.currentMinimaIndex = 0;
        this.activeEdges = null;
        this.sortedEdges = null;
        this.succeeded = true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void InsertScanline(double y)
    {
        int index = this.scanlineList.BinarySearch(y);
        if (index >= 0)
        {
            return;
        }

        index = ~index;
        this.scanlineList.Insert(index, y);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private bool PopScanline(out double y)
    {
        int cnt = this.scanlineList.Count - 1;
        if (cnt < 0)
        {
            y = 0;
            return false;
        }

        y = this.scanlineList[cnt];
        this.scanlineList.RemoveAt(cnt--);
        while (cnt >= 0 && y == this.scanlineList[cnt])
        {
            this.scanlineList.RemoveAt(cnt--);
        }

        return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private bool HasLocMinAtY(double y) => this.currentMinimaIndex < this.minimaList.Count && this.minimaList[this.currentMinimaIndex].Vertex.Point.Y == y;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private LocalMinima PopLocalMinima() => this.minimaList[this.currentMinimaIndex++];

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal void AddSubject(Contour path) => this.AddPath(path, ClipperPathType.Subject);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal void AddSubject(List<Contour> paths) => this.AddPaths(paths, ClipperPathType.Subject);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal void AddOpenSubject(Contour path) => this.AddPath(path, ClipperPathType.Subject, true);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal void AddOpenSubject(List<Contour> paths) => this.AddPaths(paths, ClipperPathType.Subject, true);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal void AddClip(Contour path) => this.AddPath(path, ClipperPathType.Clip);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal void AddClip(List<Contour> paths) => this.AddPaths(paths, ClipperPathType.Clip);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void AddPath(Contour path, ClipperPathType polytype, bool isOpen = false)
    {
        List<Contour> tmp = [path];
        this.AddPaths(tmp, polytype, isOpen);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void AddPaths(List<Contour> paths, ClipperPathType polytype, bool isOpen = false)
    {
        if (isOpen)
        {
            this.hasOpenPaths = true;
        }

        this.isSortedMinimaList = false;
        ClipperInputBuilder.AddPathsToVertexList(paths, polytype, isOpen, this.minimaList, this.vertexList);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private bool IsContributingClosed(Active ae)
    {
        switch (this.fillRule)
        {
            case ClipperFillRule.Positive:
                if (ae.WindCount != 1)
                {
                    return false;
                }

                break;
            case ClipperFillRule.Negative:
                if (ae.WindCount != -1)
                {
                    return false;
                }

                break;
            case ClipperFillRule.NonZero:
                if (Math.Abs(ae.WindCount) != 1)
                {
                    return false;
                }

                break;
        }

        switch (this.clipType)
        {
            case ClipperOperation.Intersection:
                return this.fillRule switch
                {
                    ClipperFillRule.Positive => ae.WindCount2 > 0,
                    ClipperFillRule.Negative => ae.WindCount2 < 0,
                    _ => ae.WindCount2 != 0
                };

            case ClipperOperation.Union:
                return this.fillRule switch
                {
                    ClipperFillRule.Positive => ae.WindCount2 <= 0,
                    ClipperFillRule.Negative => ae.WindCount2 >= 0,
                    _ => ae.WindCount2 == 0
                };

            case ClipperOperation.Difference:
                bool result = this.fillRule switch
                {
                    ClipperFillRule.Positive => ae.WindCount2 <= 0,
                    ClipperFillRule.Negative => ae.WindCount2 >= 0,
                    _ => ae.WindCount2 == 0
                };
                return (GetPolyType(ae) == ClipperPathType.Subject) ? result : !result;

            case ClipperOperation.Xor:
                // XOr is always contributing unless open.
                return true;

            default:
                return false;
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private bool IsContributingOpen(Active ae)
    {
        bool isInClip, isInSubj;
        switch (this.fillRule)
        {
            case ClipperFillRule.Positive:
                isInSubj = ae.WindCount > 0;
                isInClip = ae.WindCount2 > 0;
                break;
            case ClipperFillRule.Negative:
                isInSubj = ae.WindCount < 0;
                isInClip = ae.WindCount2 < 0;
                break;
            default:
                isInSubj = ae.WindCount != 0;
                isInClip = ae.WindCount2 != 0;
                break;
        }

        bool result = this.clipType switch
        {
            ClipperOperation.Intersection => isInClip,
            ClipperOperation.Union => !isInSubj && !isInClip,
            _ => !isInClip
        };
        return result;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void SetWindCountForClosedPathEdge(Active ae)
    {
        // Wind counts refer to polygon regions not edges, so here an edge's WindCnt
        // indicates the higher of the wind counts for the two regions touching the
        // edge. (nb: Adjacent regions can only ever have their wind counts differ by
        // one. Also, open paths have no meaningful wind directions or counts.)
        Active? ae2 = ae.PrevInAel;

        // find the nearest closed path edge of the same PolyType in AEL (heading left)
        ClipperPathType pt = GetPolyType(ae);
        while (ae2 != null && (GetPolyType(ae2) != pt || ae2.IsOpen))
        {
            ae2 = ae2.PrevInAel;
        }

        if (ae2 == null)
        {
            ae.WindCount = ae.WindDelta;
            ae2 = this.activeEdges;
        }
        else if (this.fillRule == ClipperFillRule.EvenOdd)
        {
            ae.WindCount = ae.WindDelta;
            ae.WindCount2 = ae2.WindCount2;
            ae2 = ae2.NextInAel;
        }
        else
        {
            // NonZero, positive, or negative filling here ...
            // when e2's WindCnt is in the SAME direction as its WindDx,
            // then polygon will fill on the right of 'e2' (and 'e' will be inside)
            // nb: neither e2.WindCnt nor e2.WindDx should ever be 0.
            if (ae2.WindCount * ae2.WindDelta < 0)
            {
                // opposite directions so 'ae' is outside 'ae2' ...
                if (Math.Abs(ae2.WindCount) > 1)
                {
                    // outside prev poly but still inside another.
                    if (ae2.WindDelta * ae.WindDelta < 0)
                    {
                        // reversing direction so use the same WC
                        ae.WindCount = ae2.WindCount;
                    }
                    else
                    {
                        // otherwise keep 'reducing' the WC by 1 (i.e. towards 0) ...
                        ae.WindCount = ae2.WindCount + ae.WindDelta;
                    }
                }
                else
                {
                    // now outside all polys of same polytype so set own WC ...
                    ae.WindCount = ae.IsOpen ? 1 : ae.WindDelta;
                }
            }
            else
            {
                // 'ae' must be inside 'ae2'
                if (ae2.WindDelta * ae.WindDelta < 0)
                {
                    // reversing direction so use the same WC
                    ae.WindCount = ae2.WindCount;
                }
                else
                {
                    // otherwise keep 'increasing' the WC by 1 (i.e. away from 0) ...
                    ae.WindCount = ae2.WindCount + ae.WindDelta;
                }
            }

            ae.WindCount2 = ae2.WindCount2;

            // Get ready to calculate WindCnt2.
            ae2 = ae2.NextInAel;
        }

        // update windCount2 ...
        if (this.fillRule == ClipperFillRule.EvenOdd)
        {
            while (ae2 != ae)
            {
                if (GetPolyType(ae2!) != pt && !ae2!.IsOpen)
                {
                    ae.WindCount2 = ae.WindCount2 == 0 ? 1 : 0;
                }

                ae2 = ae2!.NextInAel;
            }
        }
        else
        {
            while (ae2 != ae)
            {
                if (GetPolyType(ae2!) != pt && !ae2!.IsOpen)
                {
                    ae.WindCount2 += ae2!.WindDelta;
                }

                ae2 = ae2!.NextInAel;
            }
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void SetWindCountForOpenPathEdge(Active ae)
    {
        Active? ae2 = this.activeEdges;
        if (this.fillRule == ClipperFillRule.EvenOdd)
        {
            int cnt1 = 0, cnt2 = 0;
            while (ae2 != ae)
            {
                if (GetPolyType(ae2!) == ClipperPathType.Clip)
                {
                    cnt2++;
                }
                else if (!ae2!.IsOpen)
                {
                    cnt1++;
                }

                ae2 = ae2!.NextInAel;
            }

            ae.WindCount = IsOdd(cnt1) ? 1 : 0;
            ae.WindCount2 = IsOdd(cnt2) ? 1 : 0;
        }
        else
        {
            while (ae2 != ae)
            {
                if (GetPolyType(ae2!) == ClipperPathType.Clip)
                {
                    ae.WindCount2 += ae2!.WindDelta;
                }
                else if (!ae2!.IsOpen)
                {
                    ae.WindCount += ae2!.WindDelta;
                }

                ae2 = ae2!.NextInAel;
            }
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsValidAelOrder(Active resident, Active newcomer)
    {
        if (newcomer.CurrentX != resident.CurrentX)
        {
            return newcomer.CurrentX > resident.CurrentX;
        }

        // get the turning direction  a1.Top, a2.Bot, a2.Top
        int d = ClipGeometry.CrossProductSign(resident.Top, newcomer.Bot, newcomer.Top);
        if (d != 0)
        {
            return d < 0;
        }

        // edges must be collinear to get here

        // for starting open paths, place them according to
        // the direction they're about to turn
        if (!resident.IsMaxima && (resident.Top.Y > newcomer.Top.Y))
        {
            return ClipGeometry.CrossProductSign(
                newcomer.Bot,
                resident.Top,
                resident.NextVertex.Point) <= 0;
        }

        if (!newcomer.IsMaxima && (newcomer.Top.Y > resident.Top.Y))
        {
            return ClipGeometry.CrossProductSign(
                newcomer.Bot,
                newcomer.Top,
                newcomer.NextVertex.Point) >= 0;
        }

        double y = newcomer.Bot.Y;
        bool newcomerIsLeft = newcomer.IsLeftBound;

        if (resident.Bot.Y != y || resident.LocalMin.Vertex.Point.Y != y)
        {
            return newcomer.IsLeftBound;
        }

        // resident must also have just been inserted
        if (resident.IsLeftBound != newcomerIsLeft)
        {
            return newcomerIsLeft;
        }

        if (ClipGeometry.IsCollinear(
            resident.PrevPrevVertex.Point,
            resident.Bot,
            resident.Top))
        {
            return true;
        }

        // compare turning direction of the alternate bound
        return (ClipGeometry.CrossProductSign(
            resident.PrevPrevVertex.Point,
            newcomer.Bot,
            newcomer.PrevPrevVertex.Point) > 0) == newcomerIsLeft;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void InsertLeftEdge(Active ae)
    {
        if (this.activeEdges == null)
        {
            ae.PrevInAel = null;
            ae.NextInAel = null;
            this.activeEdges = ae;
        }
        else if (!IsValidAelOrder(this.activeEdges, ae))
        {
            ae.PrevInAel = null;
            ae.NextInAel = this.activeEdges;
            this.activeEdges.PrevInAel = ae;
            this.activeEdges = ae;
        }
        else
        {
            Active ae2 = this.activeEdges;
            while (ae2.NextInAel != null && IsValidAelOrder(ae2.NextInAel, ae))
            {
                ae2 = ae2.NextInAel;
            }

            // don't separate joined edges
            if (ae2.JoinWith == JoinWith.Right)
            {
                ae2 = ae2.NextInAel!;
            }

            ae.NextInAel = ae2.NextInAel;
            if (ae2.NextInAel != null)
            {
                ae2.NextInAel.PrevInAel = ae;
            }

            ae.PrevInAel = ae2;
            ae2.NextInAel = ae;
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void InsertRightEdge(Active ae, Active ae2)
    {
        ae2.NextInAel = ae.NextInAel;
        if (ae.NextInAel != null)
        {
            ae.NextInAel.PrevInAel = ae2;
        }

        ae2.PrevInAel = ae;
        ae.NextInAel = ae2;
    }

    private void InsertLocalMinimaIntoAEL(double botY)
    {
        // Add any local minima (if any) at BotY ...
        // NB horizontal local minima edges should contain locMin.Vertex.Prev
        while (this.HasLocMinAtY(botY))
        {
            LocalMinima localMinima = this.PopLocalMinima();
            Active? leftBound;
            if ((localMinima.Vertex.Flags & VertexFlags.OpenStart) != VertexFlags.None)
            {
                leftBound = null;
            }
            else
            {
                leftBound = this.NewActive();
                leftBound.Bot = localMinima.Vertex.Point;
                leftBound.CurrentX = localMinima.Vertex.Point.X;
                leftBound.WindDelta = -1;
                leftBound.VertexTop = localMinima.Vertex.Prev;
                leftBound.Top = localMinima.Vertex.Prev!.Point;
                leftBound.OutRec = null;
                leftBound.LocalMin = localMinima;
                leftBound.UpdateDx();
            }

            Active? rightBound;
            if ((localMinima.Vertex.Flags & VertexFlags.OpenEnd) != VertexFlags.None)
            {
                rightBound = null;
            }
            else
            {
                rightBound = this.NewActive();
                rightBound.Bot = localMinima.Vertex.Point;
                rightBound.CurrentX = localMinima.Vertex.Point.X;
                rightBound.WindDelta = 1;

                // Ascending.
                rightBound.VertexTop = localMinima.Vertex.Next;
                rightBound.Top = localMinima.Vertex.Next!.Point;
                rightBound.OutRec = null;
                rightBound.LocalMin = localMinima;
                rightBound.UpdateDx();
            }

            // Currently LeftB is just the descending bound and RightB is the ascending.
            // Now if the LeftB isn't on the left of RightB then we need swap them.
            if (leftBound != null && rightBound != null)
            {
                if (leftBound.IsHorizontal)
                {
                    if (leftBound.IsHeadingRightHorz)
                    {
                        SwapActives(ref leftBound, ref rightBound);
                    }
                }
                else if (rightBound.IsHorizontal)
                {
                    if (rightBound.IsHeadingLeftHorz)
                    {
                        SwapActives(ref leftBound, ref rightBound);
                    }
                }
                else if (leftBound.Dx < rightBound.Dx)
                {
                    SwapActives(ref leftBound, ref rightBound);
                }

                // so when leftBound has windDx == 1, the polygon will be oriented
                // counter-clockwise in Cartesian coords (clockwise with inverted Y).
            }
            else if (leftBound == null)
            {
                leftBound = rightBound;
                rightBound = null;
            }

            bool contributing;
            leftBound!.IsLeftBound = true;
            this.InsertLeftEdge(leftBound);

            if (leftBound.IsOpen)
            {
                this.SetWindCountForOpenPathEdge(leftBound);
                contributing = this.IsContributingOpen(leftBound);
            }
            else
            {
                this.SetWindCountForClosedPathEdge(leftBound);
                contributing = this.IsContributingClosed(leftBound);
            }

            if (rightBound != null)
            {
                rightBound.WindCount = leftBound.WindCount;
                rightBound.WindCount2 = leftBound.WindCount2;
                InsertRightEdge(leftBound, rightBound);

                if (contributing)
                {
                    this.AddLocalMinPoly(leftBound, rightBound, leftBound.Bot, true);
                    if (!leftBound.IsHorizontal)
                    {
                        this.CheckJoinLeft(leftBound, leftBound.Bot);
                    }
                }

                while (rightBound.NextInAel != null &&
                              IsValidAelOrder(rightBound.NextInAel, rightBound))
                {
                    this.IntersectEdges(rightBound, rightBound.NextInAel, rightBound.Bot);
                    this.SwapPositionsInAEL(rightBound, rightBound.NextInAel);
                }

                if (rightBound.IsHorizontal)
                {
                    this.PushHorz(rightBound);
                }
                else
                {
                    this.CheckJoinRight(rightBound, rightBound.Bot);
                    this.InsertScanline(rightBound.Top.Y);
                }
            }
            else if (contributing)
            {
                this.StartOpenPath(leftBound, leftBound.Bot);
            }

            if (leftBound.IsHorizontal)
            {
                this.PushHorz(leftBound);
            }
            else
            {
                this.InsertScanline(leftBound.Top.Y);
            }
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void PushHorz(Active ae)
    {
        ae.NextInSel = this.sortedEdges;
        this.sortedEdges = ae;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private bool PopHorz(out Active? ae)
    {
        ae = this.sortedEdges;
        if (this.sortedEdges == null)
        {
            return false;
        }

        this.sortedEdges = this.sortedEdges.NextInSel;
        return true;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private OutPt AddLocalMinPoly(Active ae1, Active ae2, Vertex pt, bool isNew = false)
    {
        OutRec outrec = this.NewOutRec();
        ae1.OutRec = outrec;
        ae2.OutRec = outrec;

        if (ae1.IsOpen)
        {
            outrec.Owner = null;
            outrec.IsOpen = true;
            if (ae1.WindDelta > 0)
            {
                SetSides(outrec, ae1, ae2);
            }
            else
            {
                SetSides(outrec, ae2, ae1);
            }
        }
        else
        {
            outrec.IsOpen = false;
            Active? prevHotEdge = ae1.GetPrevHotEdge();

            // e.WindDelta is the winding direction of the **input** paths
            // and unrelated to the winding direction of output polygons.
            // Output orientation is determined by e.OutRec.frontE which is
            // the ascending edge (see AddLocalMinPoly).
            if (prevHotEdge != null)
            {
                if (this.usingPolyTree)
                {
                    SetOwner(outrec, prevHotEdge.OutRec!);
                }

                outrec.Owner = prevHotEdge.OutRec;
                if (OutrecIsAscending(prevHotEdge) == isNew)
                {
                    SetSides(outrec, ae2, ae1);
                }
                else
                {
                    SetSides(outrec, ae1, ae2);
                }
            }
            else
            {
                outrec.Owner = null;
                if (isNew)
                {
                    SetSides(outrec, ae1, ae2);
                }
                else
                {
                    SetSides(outrec, ae2, ae1);
                }
            }
        }

        OutPt op = this.outPointPool.Add(pt, outrec);
        outrec.Points = op;
        return op;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private OutPt? AddLocalMaxPoly(Active ae1, Active ae2, Vertex pt)
    {
        if (IsJoined(ae1))
        {
            this.Split(ae1, pt);
        }

        if (IsJoined(ae2))
        {
            this.Split(ae2, pt);
        }

        if (ae1.IsFront == ae2.IsFront)
        {
            if (ae1.IsOpenEnd)
            {
                SwapFrontBackSides(ae1.OutRec!);
            }
            else if (ae2.IsOpenEnd)
            {
                SwapFrontBackSides(ae2.OutRec!);
            }
            else
            {
                this.succeeded = false;
                return null;
            }
        }

        OutPt result = this.AddOutPt(ae1, pt);
        if (ae1.OutRec == ae2.OutRec)
        {
            OutRec outrec = ae1.OutRec!;
            outrec.Points = result;

            if (this.usingPolyTree)
            {
                Active? e = ae1.GetPrevHotEdge();
                if (e == null)
                {
                    outrec.Owner = null;
                }
                else
                {
                    SetOwner(outrec, e.OutRec!);
                }

                // nb: outRec.Owner here is likely NOT the real
                // owner but this will be fixed in DeepCheckOwner()
            }

            UncoupleOutRec(ae1);
        }

        // and to preserve the winding orientation of outrec ...
        else if (ae1.IsOpen)
        {
            if (ae1.WindDelta < 0)
            {
                JoinOutrecPaths(ae1, ae2);
            }
            else
            {
                JoinOutrecPaths(ae2, ae1);
            }
        }
        else if (ae1.OutRec!.Index < ae2.OutRec!.Index)
        {
            JoinOutrecPaths(ae1, ae2);
        }
        else
        {
            JoinOutrecPaths(ae2, ae1);
        }

        return result;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void JoinOutrecPaths(Active ae1, Active ae2)
    {
        // join ae2 outrec path onto ae1 outrec path and then delete ae2 outrec path
        // pointers. (NB Only very rarely do the joining ends share the same coords.)
        OutPt p1Start = ae1.OutRec!.Points!;
        OutPt p2Start = ae2.OutRec!.Points!;
        OutPt p1End = p1Start.Next!;
        OutPt p2End = p2Start.Next!;
        if (ae1.IsFront)
        {
            p2End.Prev = p1Start;
            p1Start.Next = p2End;
            p2Start.Next = p1End;
            p1End.Prev = p2Start;
            ae1.OutRec.Points = p2Start;

            // nb: if e1.IsOpen then e1 & e2 must be a 'maximaPair'
            ae1.OutRec.FrontEdge = ae2.OutRec.FrontEdge;
            if (ae1.OutRec.FrontEdge != null)
            {
                ae1.OutRec.FrontEdge!.OutRec = ae1.OutRec;
            }
        }
        else
        {
            p1End.Prev = p2Start;
            p2Start.Next = p1End;
            p1Start.Next = p2End;
            p2End.Prev = p1Start;

            ae1.OutRec.BackEdge = ae2.OutRec.BackEdge;
            if (ae1.OutRec.BackEdge != null)
            {
                ae1.OutRec.BackEdge!.OutRec = ae1.OutRec;
            }
        }

        // after joining, the ae2.OutRec must contains no vertices ...
        ae2.OutRec.FrontEdge = null;
        ae2.OutRec.BackEdge = null;
        ae2.OutRec.Points = null;
        ae1.OutRec.OutPointCount += ae2.OutRec.OutPointCount;
        SetOwner(ae2.OutRec, ae1.OutRec);

        if (ae1.IsOpenEnd)
        {
            ae2.OutRec.Points = ae1.OutRec.Points;
            ae1.OutRec.Points = null;
        }

        // and ae1 and ae2 are maxima and are about to be dropped from the Actives list.
        ae1.OutRec = null;
        ae2.OutRec = null;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private OutPt AddOutPt(Active ae, Vertex pt)
    {
        // Outrec.OutPts: a circular doubly-linked-list of POutPt where ...
        // opFront[.Prev]* ~~~> opBack & opBack == opFront.Next
        OutRec outrec = ae.OutRec!;
        bool toFront = ae.IsFront;
        OutPt opFront = outrec.Points!;
        OutPt opBack = opFront.Next!;

        switch (toFront)
        {
            case true when ClipGeometry.PointEquals(pt, opFront.Point):
                return opFront;
            case false when ClipGeometry.PointEquals(pt, opBack.Point):
                return opBack;
        }

        OutPt newOp = this.outPointPool.Add(pt, outrec);
        opBack.Prev = newOp;
        newOp.Prev = opFront;
        newOp.Next = opBack;
        opFront.Next = newOp;
        if (toFront)
        {
            outrec.Points = newOp;
        }

        return newOp;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private OutRec NewOutRec()
    {
        int idx = this.outrecPool.Count;
        OutRec result = this.outrecPool.Add();
        result.Index = idx;
        return result;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private OutPt StartOpenPath(Active ae, Vertex pt)
    {
        OutRec outrec = this.NewOutRec();
        outrec.IsOpen = true;
        if (ae.WindDelta > 0)
        {
            outrec.FrontEdge = ae;
            outrec.BackEdge = null;
        }
        else
        {
            outrec.FrontEdge = null;
            outrec.BackEdge = ae;
        }

        ae.OutRec = outrec;
        OutPt op = this.outPointPool.Add(pt, outrec);
        outrec.Points = op;
        return op;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void UpdateEdgeIntoAEL(Active ae)
    {
        ae.Bot = ae.Top;
        ae.VertexTop = ae.NextVertex;
        ae.Top = ae.VertexTop!.Point;
        ae.CurrentX = ae.Bot.X;
        ae.UpdateDx();

        if (IsJoined(ae))
        {
            this.Split(ae, ae.Bot);
        }

        if (ae.IsHorizontal)
        {
            if (!ae.IsOpen)
            {
                TrimHorz(ae, this.PreserveCollinear);
            }

            return;
        }

        this.InsertScanline(ae.Top.Y);

        this.CheckJoinLeft(ae, ae.Bot);

        // (#500)
        this.CheckJoinRight(ae, ae.Bot, true);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Active? FindEdgeWithMatchingLocMin(Active e)
    {
        Active? result = e.NextInAel;
        while (result != null)
        {
            if (result.LocalMin == e.LocalMin)
            {
                return result;
            }

            if (!result.IsHorizontal && e.Bot != result.Bot)
            {
                result = null;
            }
            else
            {
                result = result.NextInAel;
            }
        }

        result = e.PrevInAel;
        while (result != null)
        {
            if (result.LocalMin == e.LocalMin)
            {
                return result;
            }

            if (!result.IsHorizontal && e.Bot != result.Bot)
            {
                return null;
            }

            result = result.PrevInAel;
        }

        return result;
    }

    private void IntersectEdges(Active ae1, Active ae2, Vertex pt)
    {
        OutPt? resultOp = null;

        // MANAGE OPEN PATH INTERSECTIONS SEPARATELY ...
        if (this.hasOpenPaths && (ae1.IsOpen || ae2.IsOpen))
        {
            if (ae1.IsOpen && ae2.IsOpen)
            {
                return;
            }

            // the following line avoids duplicating quite a bit of code
            if (ae2.IsOpen)
            {
                SwapActives(ref ae1, ref ae2);
            }

            if (IsJoined(ae2))
            {
                // Needed for safety.
                this.Split(ae2, pt);
            }

            if (this.clipType == ClipperOperation.Union)
            {
                if (!ae2.IsHot)
                {
                    return;
                }
            }
            else if (ae2.LocalMin.PathType == ClipperPathType.Subject)
            {
                return;
            }

            switch (this.fillRule)
            {
                case ClipperFillRule.Positive:
                    if (ae2.WindCount != 1)
                    {
                        return;
                    }

                    break;
                case ClipperFillRule.Negative:
                    if (ae2.WindCount != -1)
                    {
                        return;
                    }

                    break;
                default:
                    if (Math.Abs(ae2.WindCount) != 1)
                    {
                        return;
                    }

                    break;
            }

            // toggle contribution ...
            if (ae1.IsHot)
            {
                resultOp = this.AddOutPt(ae1, pt);
                if (ae1.IsFront)
                {
                    ae1.OutRec!.FrontEdge = null;
                }
                else
                {
                    ae1.OutRec!.BackEdge = null;
                }

                ae1.OutRec = null;
            }

            // horizontal edges can pass under open paths at a LocMins
            else if (pt == ae1.LocalMin.Vertex.Point &&
                !ae1.LocalMin.Vertex.IsOpenEnd)
            {
                // find the other side of the LocMin and
                // if it's 'hot' join up with it ...
                Active? ae3 = FindEdgeWithMatchingLocMin(ae1);
                if (ae3 != null && ae3.IsHot)
                {
                    ae1.OutRec = ae3.OutRec;
                    if (ae1.WindDelta > 0)
                    {
                        SetSides(ae3.OutRec!, ae1, ae3);
                    }
                    else
                    {
                        SetSides(ae3.OutRec!, ae3, ae1);
                    }

                    return;
                }

                resultOp = this.StartOpenPath(ae1, pt);
            }
            else
            {
                resultOp = this.StartOpenPath(ae1, pt);
            }

            return;
        }

        // MANAGING CLOSED PATHS FROM HERE ON
        if (IsJoined(ae1))
        {
            this.Split(ae1, pt);
        }

        if (IsJoined(ae2))
        {
            this.Split(ae2, pt);
        }

        // UPDATE WINDING COUNTS...
        int oldE1WindCount, oldE2WindCount;
        if (ae1.LocalMin.PathType == ae2.LocalMin.PathType)
        {
            if (this.fillRule == ClipperFillRule.EvenOdd)
            {
                oldE1WindCount = ae1.WindCount;
                ae1.WindCount = ae2.WindCount;
                ae2.WindCount = oldE1WindCount;
            }
            else
            {
                if (ae1.WindCount + ae2.WindDelta == 0)
                {
                    ae1.WindCount = -ae1.WindCount;
                }
                else
                {
                    ae1.WindCount += ae2.WindDelta;
                }

                if (ae2.WindCount - ae1.WindDelta == 0)
                {
                    ae2.WindCount = -ae2.WindCount;
                }
                else
                {
                    ae2.WindCount -= ae1.WindDelta;
                }
            }
        }
        else
        {
            if (this.fillRule != ClipperFillRule.EvenOdd)
            {
                ae1.WindCount2 += ae2.WindDelta;
            }
            else
            {
                ae1.WindCount2 = ae1.WindCount2 == 0 ? 1 : 0;
            }

            if (this.fillRule != ClipperFillRule.EvenOdd)
            {
                ae2.WindCount2 -= ae1.WindDelta;
            }
            else
            {
                ae2.WindCount2 = ae2.WindCount2 == 0 ? 1 : 0;
            }
        }

        switch (this.fillRule)
        {
            case ClipperFillRule.Positive:
                oldE1WindCount = ae1.WindCount;
                oldE2WindCount = ae2.WindCount;
                break;
            case ClipperFillRule.Negative:
                oldE1WindCount = -ae1.WindCount;
                oldE2WindCount = -ae2.WindCount;
                break;
            default:
                oldE1WindCount = Math.Abs(ae1.WindCount);
                oldE2WindCount = Math.Abs(ae2.WindCount);
                break;
        }

        bool e1WindCountIs0or1 = oldE1WindCount is 0 or 1;
        bool e2WindCountIs0or1 = oldE2WindCount is 0 or 1;

        if ((!ae1.IsHot && !e1WindCountIs0or1) ||
            (!ae2.IsHot && !e2WindCountIs0or1))
        {
            return;
        }

        // NOW PROCESS THE INTERSECTION ...

        // if both edges are 'hot' ...
        if (ae1.IsHot && ae2.IsHot)
        {
            if ((oldE1WindCount != 0 && oldE1WindCount != 1) || (oldE2WindCount != 0 && oldE2WindCount != 1) ||
                    (ae1.LocalMin.PathType != ae2.LocalMin.PathType && this.clipType != ClipperOperation.Xor))
            {
                resultOp = this.AddLocalMaxPoly(ae1, ae2, pt);
            }
            else if (ae1.IsFront || (ae1.OutRec == ae2.OutRec))
            {
                // this 'else if' condition isn't strictly needed but
                // it's sensible to split polygons that only touch at
                // a common ClipVertex (not at common edges).
                resultOp = this.AddLocalMaxPoly(ae1, ae2, pt);
            }
            else
            {
                // can't treat as maxima & minima
                resultOp = this.AddOutPt(ae1, pt);
                SwapOutrecs(ae1, ae2);
            }
        }

        // if one or other edge is 'hot' ...
        else if (ae1.IsHot)
        {
            resultOp = this.AddOutPt(ae1, pt);
            SwapOutrecs(ae1, ae2);
        }
        else if (ae2.IsHot)
        {
            resultOp = this.AddOutPt(ae2, pt);
            SwapOutrecs(ae1, ae2);
        }

        // neither edge is 'hot'
        else
        {
            double e1Wc2, e2Wc2;
            switch (this.fillRule)
            {
                case ClipperFillRule.Positive:
                    e1Wc2 = ae1.WindCount2;
                    e2Wc2 = ae2.WindCount2;
                    break;
                case ClipperFillRule.Negative:
                    e1Wc2 = -ae1.WindCount2;
                    e2Wc2 = -ae2.WindCount2;
                    break;
                default:
                    e1Wc2 = Math.Abs(ae1.WindCount2);
                    e2Wc2 = Math.Abs(ae2.WindCount2);
                    break;
            }

            if (!IsSamePolyType(ae1, ae2))
            {
                resultOp = this.AddLocalMinPoly(ae1, ae2, pt);
            }
            else if (oldE1WindCount == 1 && oldE2WindCount == 1)
            {
                resultOp = null;
                switch (this.clipType)
                {
                    case ClipperOperation.Union:
                        if (e1Wc2 > 0 && e2Wc2 > 0)
                        {
                            return;
                        }

                        resultOp = this.AddLocalMinPoly(ae1, ae2, pt);
                        break;

                    case ClipperOperation.Difference:
                        if (((GetPolyType(ae1) == ClipperPathType.Clip) && (e1Wc2 > 0) && (e2Wc2 > 0)) ||
                                ((GetPolyType(ae1) == ClipperPathType.Subject) && (e1Wc2 <= 0) && (e2Wc2 <= 0)))
                        {
                            resultOp = this.AddLocalMinPoly(ae1, ae2, pt);
                        }

                        break;

                    case ClipperOperation.Xor:
                        resultOp = this.AddLocalMinPoly(ae1, ae2, pt);
                        break;

                    // ClipperOperation.Intersection:
                    default:
                        if (e1Wc2 <= 0 || e2Wc2 <= 0)
                        {
                            return;
                        }

                        resultOp = this.AddLocalMinPoly(ae1, ae2, pt);
                        break;
                }
            }
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void DeleteFromAEL(Active ae)
    {
        Active? prev = ae.PrevInAel;
        Active? next = ae.NextInAel;
        if (prev == null && next == null && (ae != this.activeEdges))
        {
            // Already deleted.
            return;
        }

        if (prev != null)
        {
            prev.NextInAel = next;
        }
        else
        {
            this.activeEdges = next;
        }

        if (next != null)
        {
            next.PrevInAel = prev;
        }

        // delete &ae;
        this.PoolDeletedActive(ae);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void PoolDeletedActive(Active ae)
    {
        // clear refs to allow GC
        ae.Bot = default;
        ae.Top = default;
        ae.Dx = 0.0;
        ae.WindCount = 0;
        ae.WindCount2 = 0;
        ae.OutRec = null;
        ae.PrevInAel = null;
        ae.NextInAel = null;
        ae.PrevInSel = null;
        ae.NextInSel = null;
        ae.Jump = null;
        ae.VertexTop = null;
        ae.LocalMin = default;
        ae.IsLeftBound = false;
        ae.JoinWith = JoinWith.None;
        this.freeActives.Push(ae);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private Active NewActive()
    {
        Active ae;
        if (this.freeActives.Count == 0)
        {
            ae = new Active();
        }
        else
        {
            // recycle active from free list
            ae = this.freeActives.Pop();
        }

        return ae;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void AdjustCurrXAndCopyToSEL(double topY)
    {
        Active? ae = this.activeEdges;
        this.sortedEdges = ae;
        while (ae != null)
        {
            ae.PrevInSel = ae.PrevInAel;
            ae.NextInSel = ae.NextInAel;
            ae.Jump = ae.NextInSel;

            // it is safe to ignore 'joined' edges here because
            // if necessary they will be split in IntersectEdges()
            ae.CurrentX = ae.TopX(topY);

            // NB don't update ae.curr.Y yet (see AddNewIntersectNode)
            ae = ae.NextInAel;
        }
    }

    private void ExecuteInternal(ClipperOperation ct, ClipperFillRule clipperFillRule)
    {
        if (ct == ClipperOperation.NoClip)
        {
            return;
        }

        this.fillRule = clipperFillRule;
        this.clipType = ct;
        this.Reset();
        if (!this.PopScanline(out double y))
        {
            return;
        }

        while (this.succeeded)
        {
            this.InsertLocalMinimaIntoAEL(y);
            Active? ae;
            while (this.PopHorz(out ae))
            {
                this.DoHorizontal(ae!);
            }

            if (this.horizontalSegments.Count > 0)
            {
                this.ConvertHorzSegsToJoins();
                this.horizontalSegments.Clear();
            }

            // Bottom of scanbeam.
            this.currentBottomY = y;
            if (!this.PopScanline(out y))
            {
                // y new top of scanbeam.
                break;
            }

            this.DoIntersections(y);
            this.DoTopOfScanbeam(y);
            while (this.PopHorz(out ae))
            {
                this.DoHorizontal(ae!);
            }
        }

        if (this.succeeded)
        {
            this.ProcessHorzJoins();
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void DoIntersections(double topY)
    {
        if (!this.BuildIntersectList(topY))
        {
            return;
        }

        this.ProcessIntersectList();
        this.DisposeIntersectNodes();
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void DisposeIntersectNodes() => this.intersectList.Clear();

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void AddNewIntersectNode(Active ae1, Active ae2, double topY)
    {
        if (!ClipGeometry.TryGetLineIntersection(
            ae1.Bot, ae1.Top, ae2.Bot, ae2.Top, out Vertex ip))
        {
            ip = new Vertex(ae1.CurrentX, topY);
        }

        if (ip.Y > this.currentBottomY || ip.Y < topY)
        {
            double absDx1 = Math.Abs(ae1.Dx);
            double absDx2 = Math.Abs(ae2.Dx);
            switch (absDx1 > 100)
            {
                case true when absDx2 > 100:
                {
                    if (absDx1 > absDx2)
                    {
                        ip = ClipGeometry.ClosestPointOnSegment(ip, ae1.Bot, ae1.Top);
                    }
                    else
                    {
                        ip = ClipGeometry.ClosestPointOnSegment(ip, ae2.Bot, ae2.Top);
                    }

                    break;
                }

                case true:
                    ip = ClipGeometry.ClosestPointOnSegment(ip, ae1.Bot, ae1.Top);
                    break;
                default:
                {
                    if (absDx2 > 100)
                    {
                        ip = ClipGeometry.ClosestPointOnSegment(ip, ae2.Bot, ae2.Top);
                    }
                    else
                    {
                        double targetY = ip.Y < topY ? topY : this.currentBottomY;
                        double targetX = absDx1 < absDx2 ? ae1.TopX(targetY) : ae2.TopX(targetY);
                        ip = new Vertex(targetX, targetY);
                    }

                    break;
                }
            }
        }

        IntersectNode node = new(ip, ae1, ae2);
        this.intersectList.Add(node);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Active? ExtractFromSEL(Active ae)
    {
        Active? res = ae.NextInSel;
        if (res != null)
        {
            res.PrevInSel = ae.PrevInSel;
        }

        ae.PrevInSel!.NextInSel = res;
        return res;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void Insert1Before2InSEL(Active ae1, Active ae2)
    {
        ae1.PrevInSel = ae2.PrevInSel;
        if (ae1.PrevInSel != null)
        {
            ae1.PrevInSel.NextInSel = ae1;
        }

        ae1.NextInSel = ae2;
        ae2.PrevInSel = ae1;
    }

    private bool BuildIntersectList(double topY)
    {
        if (this.activeEdges?.NextInAel == null)
        {
            return false;
        }

        // Calculate edge positions at the top of the current scanbeam, and from this
        // we will determine the intersections required to reach these new positions.
        this.AdjustCurrXAndCopyToSEL(topY);

        // Find all edge intersections in the current scanbeam using a stable merge
        // sort that ensures only adjacent edges are intersecting. Intersect info is
        // stored in FIntersectList ready to be processed in ProcessIntersectList.
        // Re merge sorts see https://stackoverflow.com/a/46319131/359538
        Active? left = this.sortedEdges;

        while (left!.Jump != null)
        {
            Active? prevBase = null;
            while (left?.Jump != null)
            {
                Active? currBase = left;
                Active? right = left.Jump;
                Active? lEnd = right;
                Active? rEnd = right.Jump;
                left.Jump = rEnd;
                while (left != lEnd && right != rEnd)
                {
                    if (right!.CurrentX < left!.CurrentX)
                    {
                        Active? tmp = right.PrevInSel!;
                        for (; ;)
                        {
                            this.AddNewIntersectNode(tmp, right, topY);
                            if (tmp == left)
                            {
                                break;
                            }

                            tmp = tmp.PrevInSel!;
                        }

                        tmp = right;
                        right = ExtractFromSEL(tmp);
                        lEnd = right;
                        Insert1Before2InSEL(tmp, left);
                        if (left != currBase)
                        {
                            continue;
                        }

                        currBase = tmp;
                        currBase.Jump = rEnd;
                        if (prevBase == null)
                        {
                            this.sortedEdges = currBase;
                        }
                        else
                        {
                            prevBase.Jump = currBase;
                        }
                    }
                    else
                    {
                        left = left.NextInSel;
                    }
                }

                prevBase = currBase;
                left = rEnd;
            }

            left = this.sortedEdges;
        }

        return this.intersectList.Count > 0;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void ProcessIntersectList()
    {
        // We now have a list of intersections required so that edges will be
        // correctly positioned at the top of the scanbeam. However, it's important
        // that edge intersections are processed from the bottom up, but it's also
        // crucial that intersections only occur between adjacent edges.

        // First we do a quicksort so intersections proceed in a bottom up order ...
        this.intersectList.Sort(default(IntersectListSort));

        // Now as we process these intersections, we must sometimes adjust the order
        // to ensure that intersecting edges are always adjacent ...
        for (int i = 0; i < this.intersectList.Count; ++i)
        {
            if (!EdgesAdjacentInAEL(this.intersectList[i]))
            {
                int j = i + 1;
                while (!EdgesAdjacentInAEL(this.intersectList[j]))
                {
                    j++;
                }

                // swap
                (this.intersectList[j], this.intersectList[i]) =
                    (this.intersectList[i], this.intersectList[j]);
            }

            IntersectNode node = this.intersectList[i];
            this.IntersectEdges(node.Edge1, node.Edge2, node.Point);
            this.SwapPositionsInAEL(node.Edge1, node.Edge2);

            node.Edge1.CurrentX = node.Point.X;
            node.Edge2.CurrentX = node.Point.X;
            this.CheckJoinLeft(node.Edge2, node.Point, true);
            this.CheckJoinRight(node.Edge1, node.Point, true);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void SwapPositionsInAEL(Active ae1, Active ae2)
    {
        // preconditon: ae1 must be immediately to the left of ae2
        Active? next = ae2.NextInAel;
        if (next != null)
        {
            next.PrevInAel = ae1;
        }

        Active? prev = ae1.PrevInAel;
        if (prev != null)
        {
            prev.NextInAel = ae2;
        }

        ae2.PrevInAel = prev;
        ae2.NextInAel = ae1;
        ae1.PrevInAel = ae2;
        ae1.NextInAel = next;
        if (ae2.PrevInAel == null)
        {
            this.activeEdges = ae2;
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool ResetHorzDirection(
        Active horz,
        ClipVertex? vertexMax,
        out double leftX,
        out double rightX)
    {
        if (horz.Bot.X == horz.Top.X)
        {
            // the horizontal edge is going nowhere ...
            leftX = horz.CurrentX;
            rightX = horz.CurrentX;
            Active? ae = horz.NextInAel;
            while (ae != null && ae.VertexTop != vertexMax)
            {
                ae = ae.NextInAel;
            }

            return ae != null;
        }

        if (horz.CurrentX < horz.Top.X)
        {
            leftX = horz.CurrentX;
            rightX = horz.Top.X;
            return true;
        }

        // Right to left.
        leftX = horz.Top.X;
        rightX = horz.CurrentX;
        return false;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void TrimHorz(Active horzEdge, bool preserveCollinear)
    {
        bool wasTrimmed = false;
        Vertex pt = horzEdge.NextVertex.Point;

        while (pt.Y == horzEdge.Top.Y)
        {
            // always trim 180 deg. spikes (in closed paths)
            // but otherwise break if preserveCollinear = true
            if (preserveCollinear &&
                (pt.X < horzEdge.Top.X) != (horzEdge.Bot.X < horzEdge.Top.X))
            {
                break;
            }

            horzEdge.VertexTop = horzEdge.NextVertex;
            horzEdge.Top = pt;
            wasTrimmed = true;
            if (horzEdge.IsMaxima)
            {
                break;
            }

            pt = horzEdge.NextVertex.Point;
        }

        if (wasTrimmed)
        {
            // +/-infinity
            horzEdge.UpdateDx();
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void AddToHorzSegList(OutPt op)
    {
        if (op.OutRec.IsOpen)
        {
            return;
        }

        this.horizontalSegments.Add(new HorzSegment(op));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static OutPt GetLastOp(Active hotEdge)
    {
        OutRec outrec = hotEdge.OutRec!;
        return (hotEdge == outrec.FrontEdge) ?
            outrec.Points! : outrec.Points!.Next!;
    }

    private void DoHorizontal(Active horz)
    /*******************************************************************************
      * Notes: Horizontal edges (HEs) at scanline intersections (i.e. at the top or    *
      * bottom of a scanbeam) are processed as if layered.The order in which HEs     *
      * are processed doesn't matter. HEs intersect with the bottom vertices of      *
      * other HEs[#] and with non-horizontal edges [*]. Once these intersections     *
      * are completed, intermediate HEs are 'promoted' to the next edge in their     *
      * bounds, and they in turn may be intersected[%] by other HEs.                 *
      *                                                                              *
      * eg: 3 horizontals at a scanline:    /   |                     /           /  *
      *              |                     /    |     (HE3)o ========%========== o   *
      *              o ======= o(HE2)     /     |         /         /                *
      *          o ============#=========*======*========#=========o (HE1)           *
      *         /              |        /       |       /                            *
      *******************************************************************************/
    {
        bool horzIsOpen = horz.IsOpen;
        double y = horz.Bot.Y;

        ClipVertex? vertex_max = horzIsOpen ?
            GetCurrYMaximaVertex_Open(horz) :
            GetCurrYMaximaVertex(horz);

        bool isLeftToRight =
            ResetHorzDirection(horz, vertex_max, out double leftX, out double rightX);

        if (horz.IsHot)
        {
            OutPt op = this.AddOutPt(horz, new Vertex(horz.CurrentX, y));
            this.AddToHorzSegList(op);
        }

        for (; ;)
        {
            // loops through consec. horizontal edges (if open)
            Active? ae = isLeftToRight ? horz.NextInAel : horz.PrevInAel;

            while (ae != null)
            {
                if (ae.VertexTop == vertex_max)
                {
                    // do this first!!
                    if (horz.IsHot && IsJoined(ae))
                    {
                        this.Split(ae, ae.Top);
                    }

                    if (horz.IsHot)
                    {
                        while (horz.VertexTop != vertex_max)
                        {
                            this.AddOutPt(horz, horz.Top);
                            this.UpdateEdgeIntoAEL(horz);
                        }

                        if (isLeftToRight)
                        {
                            this.AddLocalMaxPoly(horz, ae, horz.Top);
                        }
                        else
                        {
                            this.AddLocalMaxPoly(ae, horz, horz.Top);
                        }
                    }

                    this.DeleteFromAEL(ae);
                    this.DeleteFromAEL(horz);
                    return;
                }

                // if horzEdge is a maxima, keep going until we reach
                // its maxima pair, otherwise check for break conditions
                Vertex pt;
                if (vertex_max != horz.VertexTop || horz.IsOpenEnd)
                {
                    // otherwise stop when 'ae' is beyond the end of the horizontal line
                    if ((isLeftToRight && ae.CurrentX > rightX) ||
                            (!isLeftToRight && ae.CurrentX < leftX))
                    {
                        break;
                    }

                    if (ae.CurrentX == horz.Top.X && !ae.IsHorizontal)
                    {
                        pt = horz.NextVertex.Point;

                        // to maximize the possibility of putting open edges into
                        // solutions, we'll only break if it's past HorzEdge's end
                        if (ae.IsOpen && !IsSamePolyType(ae, horz) && !ae.IsHot)
                        {
                            if ((isLeftToRight && (ae.TopX(pt.Y) > pt.X)) ||
                                (!isLeftToRight && (ae.TopX(pt.Y) < pt.X)))
                            {
                                break;
                            }
                        }

                        // otherwise for edges at horzEdge's end, only stop when horzEdge's
                        // outslope is greater than e's slope when heading right or when
                        // horzEdge's outslope is less than e's slope when heading left.
                        else if ((isLeftToRight && (ae.TopX(pt.Y) >= pt.X)) ||
                                (!isLeftToRight && (ae.TopX(pt.Y) <= pt.X)))
                        {
                            break;
                        }
                    }
                }

                pt = new Vertex(ae.CurrentX, y);

                if (isLeftToRight)
                {
                    this.IntersectEdges(horz, ae, pt);
                    this.SwapPositionsInAEL(horz, ae);
                    this.CheckJoinLeft(ae, pt);
                    horz.CurrentX = ae.CurrentX;
                    ae = horz.NextInAel;
                }
                else
                {
                    this.IntersectEdges(ae, horz, pt);
                    this.SwapPositionsInAEL(ae, horz);
                    this.CheckJoinRight(ae, pt);
                    horz.CurrentX = ae.CurrentX;
                    ae = horz.PrevInAel;
                }

                if (horz.IsHot)
                {
                    this.AddToHorzSegList(GetLastOp(horz));
                }
            }

            // check if we've finished looping
            // through consecutive horizontals
            // We've reached the end of this horizontal.
            // Open at top.
            if (horzIsOpen && horz.IsOpenEnd)
            {
                if (horz.IsHot)
                {
                    this.AddOutPt(horz, horz.Top);
                    if (horz.IsFront)
                    {
                        horz.OutRec!.FrontEdge = null;
                    }
                    else
                    {
                        horz.OutRec!.BackEdge = null;
                    }

                    horz.OutRec = null;
                }

                this.DeleteFromAEL(horz);
                return;
            }

            if (horz.NextVertex.Point.Y != horz.Top.Y)
            {
                break;
            }

            // still more horizontals in bound to process ...
            if (horz.IsHot)
            {
                this.AddOutPt(horz, horz.Top);
            }

            this.UpdateEdgeIntoAEL(horz);

            isLeftToRight = ResetHorzDirection(
                horz,
                vertex_max,
                out leftX,
                out rightX);
        }

        // End of (possible consecutive) horizontals.
        if (horz.IsHot)
        {
            OutPt op = this.AddOutPt(horz, horz.Top);
            this.AddToHorzSegList(op);
        }

        // This is the end of an intermediate horizontal.
        this.UpdateEdgeIntoAEL(horz);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void DoTopOfScanbeam(double y)
    {
        // Sorted edges are reused to flag horizontals (see PushHorz below).
        this.sortedEdges = null;
        Active? ae = this.activeEdges;
        while (ae != null)
        {
            // NB 'ae' will never be horizontal here
            if (ae.Top.Y == y)
            {
                ae.CurrentX = ae.Top.X;
                if (ae.IsMaxima)
                {
                    // TOP OF BOUND (MAXIMA)
                    ae = this.DoMaxima(ae);
                    continue;
                }

                // INTERMEDIATE ClipVertex ...
                if (ae.IsHot)
                {
                    this.AddOutPt(ae, ae.Top);
                }

                this.UpdateEdgeIntoAEL(ae);

                // Horizontals are processed later.
                if (ae.IsHorizontal)
                {
                    this.PushHorz(ae);
                }
            }

            // Not the top of the edge.
            else
            {
                ae.CurrentX = ae.TopX(y);
            }

            ae = ae.NextInAel;
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private Active? DoMaxima(Active ae)
    {
        Active? prevE = ae.PrevInAel;
        Active? nextE = ae.NextInAel;

        if (ae.IsOpenEnd)
        {
            if (ae.IsHot)
            {
                this.AddOutPt(ae, ae.Top);
            }

            if (ae.IsHorizontal)
            {
                return nextE;
            }

            if (ae.IsHot)
            {
                if (ae.IsFront)
                {
                    ae.OutRec!.FrontEdge = null;
                }
                else
                {
                    ae.OutRec!.BackEdge = null;
                }

                ae.OutRec = null;
            }

            this.DeleteFromAEL(ae);
            return nextE;
        }

        Active? maxPair = GetMaximaPair(ae);
        if (maxPair == null)
        {
            // eMaxPair is horizontal.
            return nextE;
        }

        if (IsJoined(ae))
        {
            this.Split(ae, ae.Top);
        }

        if (IsJoined(maxPair))
        {
            this.Split(maxPair, maxPair.Top);
        }

        // only non-horizontal maxima here.
        // process any edges between maxima pair ...
        while (nextE != maxPair)
        {
            this.IntersectEdges(ae, nextE!, ae.Top);
            this.SwapPositionsInAEL(ae, nextE!);
            nextE = ae.NextInAel;
        }

        if (ae.IsOpen)
        {
            if (ae.IsHot)
            {
                this.AddLocalMaxPoly(ae, maxPair, ae.Top);
            }

            this.DeleteFromAEL(maxPair);
            this.DeleteFromAEL(ae);
            return prevE != null ? prevE.NextInAel : this.activeEdges;
        }

        // here ae.nextInAel == ENext == EMaxPair ...
        if (ae.IsHot)
        {
            this.AddLocalMaxPoly(ae, maxPair, ae.Top);
        }

        this.DeleteFromAEL(ae);
        this.DeleteFromAEL(maxPair);
        return prevE != null ? prevE.NextInAel : this.activeEdges;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsJoined(Active e) => e.JoinWith != JoinWith.None;

    private void Split(Active e, Vertex currPt)
    {
        if (e.JoinWith == JoinWith.Right)
        {
            e.JoinWith = JoinWith.None;
            e.NextInAel!.JoinWith = JoinWith.None;
            this.AddLocalMinPoly(e, e.NextInAel, currPt, true);
        }
        else
        {
            e.JoinWith = JoinWith.None;
            e.PrevInAel!.JoinWith = JoinWith.None;
            this.AddLocalMinPoly(e.PrevInAel, e, currPt, true);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void CheckJoinLeft(
        Active e,
        Vertex pt,
        bool checkCurrX = false)
    {
        Active? prev = e.PrevInAel;
        if (prev == null ||
            !e.IsHot || !prev.IsHot ||
            e.IsHorizontal || prev.IsHorizontal ||
            e.IsOpen || prev.IsOpen)
        {
            return;
        }

        // Avoid trivial joins.
        if ((pt.Y < e.Top.Y + ClipGeometry.ClosePointTolerance || pt.Y < prev.Top.Y + ClipGeometry.ClosePointTolerance) &&
            ((e.Bot.Y > pt.Y) || (prev.Bot.Y > pt.Y)))
        {
            // (#490)
            return;
        }

        if (checkCurrX)
        {
            if (ClipMath.PerpendicularDistanceSquared(pt, prev.Bot, prev.Top) > ClipGeometry.JoinDistanceSquared)
            {
                return;
            }
        }
        else if (!ClipGeometry.IsAlmostZero(e.CurrentX - prev.CurrentX))
        {
            return;
        }

        if (!ClipGeometry.IsCollinear(e.Top, pt, prev.Top))
        {
            return;
        }

        if (e.OutRec!.Index == prev.OutRec!.Index)
        {
            this.AddLocalMaxPoly(prev, e, pt);
        }
        else if (e.OutRec.Index < prev.OutRec.Index)
        {
            JoinOutrecPaths(e, prev);
        }
        else
        {
            JoinOutrecPaths(prev, e);
        }

        prev.JoinWith = JoinWith.Right;
        e.JoinWith = JoinWith.Left;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void CheckJoinRight(
        Active e,
        Vertex pt,
        bool checkCurrX = false)
    {
        Active? next = e.NextInAel;
        if (next == null ||
            !e.IsHot || !next.IsHot ||
            e.IsHorizontal || next.IsHorizontal ||
            e.IsOpen || next.IsOpen)
        {
            return;
        }

        // Avoid trivial joins.
        if ((pt.Y < e.Top.Y + ClipGeometry.ClosePointTolerance || pt.Y < next.Top.Y + ClipGeometry.ClosePointTolerance) &&
            ((e.Bot.Y > pt.Y) || (next.Bot.Y > pt.Y)))
        {
            // (#490)
            return;
        }

        if (checkCurrX)
        {
            if (ClipMath.PerpendicularDistanceSquared(pt, next.Bot, next.Top) > ClipGeometry.JoinDistanceSquared)
            {
                return;
            }
        }
        else if (!ClipGeometry.IsAlmostZero(e.CurrentX - next.CurrentX))
        {
            return;
        }

        if (!ClipGeometry.IsCollinear(e.Top, pt, next.Top))
        {
            return;
        }

        if (e.OutRec!.Index == next.OutRec!.Index)
        {
            this.AddLocalMaxPoly(e, next, pt);
        }
        else if (e.OutRec.Index < next.OutRec.Index)
        {
            JoinOutrecPaths(e, next);
        }
        else
        {
            JoinOutrecPaths(next, e);
        }

        e.JoinWith = JoinWith.Right;
        next.JoinWith = JoinWith.Left;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void FixOutRecPts(OutRec outrec)
    {
        OutPt op = outrec.Points!;
        do
        {
            op.OutRec = outrec;
            op = op.Next!;
        }
        while (op != outrec.Points);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool SetHorzSegHeadingForward(HorzSegment hs, OutPt opP, OutPt opN)
    {
        if (ClipGeometry.IsAlmostZero(opP.Point.X - opN.Point.X))
        {
            return false;
        }

        if (opP.Point.X < opN.Point.X)
        {
            hs.LeftOp = opP;
            hs.RightOp = opN;
            hs.LeftToRight = true;
        }
        else
        {
            hs.LeftOp = opN;
            hs.RightOp = opP;
            hs.LeftToRight = false;
        }

        return true;
    }

    private static bool UpdateHorzSegment(HorzSegment hs)
    {
        OutPt op = hs.LeftOp!;
        OutRec outrec = GetRealOutRec(op.OutRec)!;
        bool outrecHasEdges = outrec.FrontEdge != null;
        double curr_y = op.Point.Y;
        OutPt opP = op, opN = op;
        if (outrecHasEdges)
        {
            OutPt opA = outrec.Points!, opZ = opA.Next!;
            while (opP != opZ && opP.Prev.Point.Y == curr_y)
            {
                opP = opP.Prev;
            }

            while (opN != opA && opN.Next!.Point.Y == curr_y)
            {
                opN = opN.Next;
            }
        }
        else
        {
            while (opP.Prev != opN && opP.Prev.Point.Y == curr_y)
            {
                opP = opP.Prev;
            }

            while (opN.Next != opP && opN.Next!.Point.Y == curr_y)
            {
                opN = opN.Next;
            }
        }

        bool result =
            SetHorzSegHeadingForward(hs, opP, opN) &&
            hs.LeftOp!.Horz == null;

        if (result)
        {
            hs.LeftOp!.Horz = hs;
        }
        else
        {
            // (for sorting)
            hs.RightOp = null;
        }

        return result;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private OutPt DuplicateOp(OutPt op, bool insert_after)
    {
        OutPt result = this.outPointPool.Add(op.Point, op.OutRec);
        if (insert_after)
        {
            result.Next = op.Next;
            result.Next!.Prev = result;
            result.Prev = op;
            op.Next = result;
        }
        else
        {
            result.Prev = op.Prev;
            result.Prev.Next = result;
            result.Next = op;
            op.Prev = result;
        }

        return result;
    }

    private static int HorzSegSort(HorzSegment? hs1, HorzSegment? hs2)
    {
        if (hs1 == null || hs2 == null)
        {
            return 0;
        }

        if (hs1.RightOp == null)
        {
            return hs2.RightOp == null ? 0 : 1;
        }

        if (hs2.RightOp == null)
        {
            return -1;
        }

        return hs1.LeftOp!.Point.X.CompareTo(hs2.LeftOp!.Point.X);
    }

    private void ConvertHorzSegsToJoins()
    {
        int k = 0;
        foreach (HorzSegment hs in this.horizontalSegments)
        {
            if (UpdateHorzSegment(hs))
            {
                k++;
            }
        }

        if (k < 2)
        {
            return;
        }

        this.horizontalSegments.Sort(HorzSegSort);

        for (int i = 0; i < k - 1; i++)
        {
            HorzSegment hs1 = this.horizontalSegments[i];

            // for each HorzSegment, find others that overlap
            for (int j = i + 1; j < k; j++)
            {
                HorzSegment hs2 = this.horizontalSegments[j];
                if ((hs2.LeftOp!.Point.X >= hs1.RightOp!.Point.X) ||
                    (hs2.LeftToRight == hs1.LeftToRight) ||
                    (hs2.RightOp!.Point.X <= hs1.LeftOp!.Point.X))
                {
                    continue;
                }

                double curr_y = hs1.LeftOp.Point.Y;
                if (hs1.LeftToRight)
                {
                    while (hs1.LeftOp.Next!.Point.Y == curr_y &&
                        hs1.LeftOp.Next.Point.X <= hs2.LeftOp.Point.X)
                    {
                        hs1.LeftOp = hs1.LeftOp.Next;
                    }

                    while (hs2.LeftOp.Prev.Point.Y == curr_y &&
                        hs2.LeftOp.Prev.Point.X <= hs1.LeftOp.Point.X)
                    {
                        hs2.LeftOp = hs2.LeftOp.Prev;
                    }

                    HorzJoin join = this.horizontalJoins.Add(
                        this.DuplicateOp(hs1.LeftOp, true),
                        this.DuplicateOp(hs2.LeftOp, false));
                }
                else
                {
                    while (hs1.LeftOp.Prev.Point.Y == curr_y &&
                        hs1.LeftOp.Prev.Point.X <= hs2.LeftOp.Point.X)
                    {
                        hs1.LeftOp = hs1.LeftOp.Prev;
                    }

                    while (hs2.LeftOp.Next!.Point.Y == curr_y &&
                        hs2.LeftOp.Next.Point.X <= hs1.LeftOp.Point.X)
                    {
                        hs2.LeftOp = hs2.LeftOp.Next;
                    }

                    HorzJoin join = this.horizontalJoins.Add(
                        this.DuplicateOp(hs2.LeftOp, true),
                        this.DuplicateOp(hs1.LeftOp, false));
                }
            }
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Contour GetCleanPath(OutPt op)
    {
        Contour result = [];
        OutPt op2 = op;
        while (op2.Next != op &&
            ((ClipGeometry.IsAlmostZero(op2.Point.X - op2.Next!.Point.X) &&
              ClipGeometry.IsAlmostZero(op2.Point.X - op2.Prev.Point.X)) ||
             (ClipGeometry.IsAlmostZero(op2.Point.Y - op2.Next.Point.Y) &&
              ClipGeometry.IsAlmostZero(op2.Point.Y - op2.Prev.Point.Y))))
        {
            op2 = op2.Next;
        }

        result.Add(op2.Point);
        OutPt prevOp = op2;
        op2 = op2.Next;
        while (op2 != op)
        {
            if ((!ClipGeometry.IsAlmostZero(op2.Point.X - op2.Next!.Point.X) ||
                 !ClipGeometry.IsAlmostZero(op2.Point.X - prevOp.Point.X)) &&
                (!ClipGeometry.IsAlmostZero(op2.Point.Y - op2.Next.Point.Y) ||
                 !ClipGeometry.IsAlmostZero(op2.Point.Y - prevOp.Point.Y)))
            {
                result.Add(op2.Point);
                prevOp = op2;
            }

            op2 = op2.Next;
        }

        return result;
    }

    private static ClipperPointInPolygonResult PointInOpPolygon(Vertex pt, OutPt op)
    {
        if (op == op.Next || op.Prev == op.Next)
        {
            return ClipperPointInPolygonResult.IsOutside;
        }

        OutPt op2 = op;
        do
        {
            if (op.Point.Y != pt.Y)
            {
                break;
            }

            op = op.Next!;
        }
        while (op != op2);

        // Not a proper polygon.
        if (op.Point.Y == pt.Y)
        {
            return ClipperPointInPolygonResult.IsOutside;
        }

        // must be above or below to get here
        bool isAbove = op.Point.Y < pt.Y, startingAbove = isAbove;
        int val = 0;

        op2 = op.Next!;
        while (op2 != op)
        {
            if (isAbove)
            {
                while (op2 != op && op2.Point.Y < pt.Y)
                {
                    op2 = op2.Next!;
                }
            }
            else
            {
                while (op2 != op && op2.Point.Y > pt.Y)
                {
                    op2 = op2.Next!;
                }
            }

            if (op2 == op)
            {
                break;
            }

            // must have touched or crossed the pt.Y horizontal
            // and this must happen an even number of times
            // Touching the horizontal.
            if (op2.Point.Y == pt.Y)
            {
                if (op2.Point.X == pt.X || (op2.Point.Y == op2.Prev.Point.Y &&
                    (pt.X < op2.Prev.Point.X) != (pt.X < op2.Point.X)))
                {
                    return ClipperPointInPolygonResult.IsOn;
                }

                op2 = op2.Next!;
                if (op2 == op)
                {
                    break;
                }

                continue;
            }

            if (op2.Point.X <= pt.X || op2.Prev.Point.X <= pt.X)
            {
                if (op2.Prev.Point.X < pt.X && op2.Point.X < pt.X)
                {
                    // Toggle val.
                    val = 1 - val;
                }
                else
                {
                    int d = ClipGeometry.CrossProductSign(op2.Prev.Point, op2.Point, pt);
                    if (d == 0)
                    {
                        return ClipperPointInPolygonResult.IsOn;
                    }

                    if ((d < 0) == isAbove)
                    {
                        val = 1 - val;
                    }
                }
            }

            isAbove = !isAbove;
            op2 = op2.Next!;
        }

        if (isAbove == startingAbove)
        {
            return val == 0 ? ClipperPointInPolygonResult.IsOutside : ClipperPointInPolygonResult.IsInside;
        }

        {
            int d = ClipGeometry.CrossProductSign(op2.Prev.Point, op2.Point, pt);
            if (d == 0)
            {
                return ClipperPointInPolygonResult.IsOn;
            }

            if ((d < 0) == isAbove)
            {
                val = 1 - val;
            }
        }

        return val == 0 ? ClipperPointInPolygonResult.IsOutside : ClipperPointInPolygonResult.IsInside;
    }

    private static bool Path1InsidePath2(OutPt op1, OutPt op2)
    {
        // we need to make some accommodation for rounding errors
        // so we won't jump if the first ClipVertex is found outside
        ClipperPointInPolygonResult pip = ClipperPointInPolygonResult.IsOn;
        OutPt op = op1;
        do
        {
            switch (PointInOpPolygon(op.Point, op2))
            {
                case ClipperPointInPolygonResult.IsOutside:
                    if (pip == ClipperPointInPolygonResult.IsOutside)
                    {
                        return false;
                    }

                    pip = ClipperPointInPolygonResult.IsOutside;
                    break;
                case ClipperPointInPolygonResult.IsInside:
                    if (pip == ClipperPointInPolygonResult.IsInside)
                    {
                        return true;
                    }

                    pip = ClipperPointInPolygonResult.IsInside;
                    break;
                default:
                    break;
            }

            op = op.Next!;
        }
        while (op != op1);

        // result is unclear, so try again using cleaned paths
        // (#973)
        return ClipGeometry.Path2ContainsPath1(GetCleanPath(op1), GetCleanPath(op2));
    }

    private static void MoveSplits(OutRec fromOr, OutRec toOr)
    {
        if (fromOr.Splits == null)
        {
            return;
        }

        toOr.Splits ??= [];
        foreach (int i in fromOr.Splits)
        {
            if (i != toOr.Index)
            {
                toOr.Splits.Add(i);
            }
        }

        fromOr.Splits = null;
    }

    private void ProcessHorzJoins()
    {
        foreach (HorzJoin j in this.horizontalJoins)
        {
            OutRec or1 = GetRealOutRec(j.Op1!.OutRec)!;
            OutRec or2 = GetRealOutRec(j.Op2!.OutRec)!;

            OutPt op1b = j.Op1.Next!;
            OutPt op2b = j.Op2.Prev;
            j.Op1.Next = j.Op2;
            j.Op2.Prev = j.Op1;
            op1b.Prev = op2b;
            op2b.Next = op1b;

            // 'join' is really a split
            if (or1 == or2)
            {
                or2 = this.NewOutRec();
                or2.Points = op1b;
                FixOutRecPts(or2);

                // if or1->pts has moved to or2 then update or1->pts!!
                if (or1.Points!.OutRec == or2)
                {
                    or1.Points = j.Op1;
                    or1.Points.OutRec = or1;
                }

                // #498, #520, #584, D#576, #618
                if (this.usingPolyTree)
                {
                    if (Path1InsidePath2(or1.Points, or2.Points))
                    {
                        // swap or1's & or2's pts
                        (or2.Points, or1.Points) = (or1.Points, or2.Points);
                        FixOutRecPts(or1);
                        FixOutRecPts(or2);

                        // or2 is now inside or1
                        or2.Owner = or1;
                    }
                    else if (Path1InsidePath2(or2.Points, or1.Points))
                    {
                        or2.Owner = or1;
                    }
                    else
                    {
                        or2.Owner = or1.Owner;
                    }

                    or1.Splits ??= [];
                    or1.Splits.Add(or2.Index);
                }
                else
                {
                    or2.Owner = or1;
                }
            }
            else
            {
                or2.Points = null;
                if (this.usingPolyTree)
                {
                    SetOwner(or2, or1);

                    // #618
                    MoveSplits(or2, or1);
                }
                else
                {
                    or2.Owner = or1;
                }
            }
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool PtsReallyClose(Vertex pt1, Vertex pt2)
    {
        double tolerance = ClipGeometry.ClosePointTolerance;
        return Math.Abs(pt1.X - pt2.X) < tolerance && Math.Abs(pt1.Y - pt2.Y) < tolerance;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsVerySmallTriangle(OutPt op) => op.Next!.Next == op.Prev &&
      (PtsReallyClose(op.Prev.Point, op.Next.Point) ||
        PtsReallyClose(op.Point, op.Next.Point) ||
        PtsReallyClose(op.Point, op.Prev.Point));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsValidClosedPath(OutPt? op) => op != null && op.Next != op &&
            (op.Next != op.Prev || !IsVerySmallTriangle(op));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static OutPt? DisposeOutPt(OutPt op)
    {
        OutPt? result = op.Next == op ? null : op.Next;
        op.Prev.Next = op.Next;
        op.Next!.Prev = op.Prev;

        // op == null;
        return result;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void CleanCollinear(OutRec? outrec)
    {
        outrec = GetRealOutRec(outrec);

        if (outrec == null || outrec.IsOpen)
        {
            return;
        }

        if (!IsValidClosedPath(outrec.Points))
        {
            outrec.Points = null;
            return;
        }

        OutPt startOp = outrec.Points!;
        OutPt? op2 = startOp;
        for (; ;)
        {
            // NB if preserveCollinear == true, then only remove 180 deg. spikes
            if (ClipGeometry.IsCollinear(op2!.Prev.Point, op2.Point, op2.Next!.Point) &&
                (ClipGeometry.PointEquals(op2.Point, op2.Prev.Point) || ClipGeometry.PointEquals(op2.Point, op2.Next.Point) || !this.PreserveCollinear ||
                (ClipGeometry.DotProduct(op2.Prev.Point, op2.Point, op2.Next.Point) < 0)))
            {
                if (op2 == outrec.Points)
                {
                    outrec.Points = op2.Prev;
                }

                op2 = DisposeOutPt(op2);
                if (!IsValidClosedPath(op2))
                {
                    outrec.Points = null;
                    return;
                }

                startOp = op2!;
                continue;
            }

            op2 = op2.Next;
            if (op2 == startOp)
            {
                break;
            }
        }

        this.FixSelfIntersects(outrec);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void DoSplitOp(OutRec outrec, OutPt splitOp)
    {
        // splitOp.Prev <=> splitOp &&
        // splitOp.Next <=> splitOp.Next.Next are intersecting
        OutPt prevOp = splitOp.Prev;
        OutPt nextNextOp = splitOp.Next!.Next!;
        outrec.Points = prevOp;

        ClipGeometry.TryGetLineIntersection(
            prevOp.Point, splitOp.Point, splitOp.Next.Point, nextNextOp.Point, out Vertex ip);

        double area1 = Area(prevOp);
        double absArea1 = Math.Abs(area1);

        if (absArea1 < ClipGeometry.SmallAreaTolerance2)
        {
            outrec.Points = null;
            return;
        }

        double area2 = AreaTriangle(ip, splitOp.Point, splitOp.Next.Point);
        double absArea2 = Math.Abs(area2);

        // de-link splitOp and splitOp.Next from the path
        // while inserting the intersection point
        if (ClipGeometry.PointEquals(ip, prevOp.Point) || ClipGeometry.PointEquals(ip, nextNextOp.Point))
        {
            nextNextOp.Prev = prevOp;
            prevOp.Next = nextNextOp;
        }
        else
        {
            OutPt newOp2 = this.outPointPool.Add(ip, outrec);
            newOp2.Prev = prevOp;
            newOp2.Next = nextNextOp;
            nextNextOp.Prev = newOp2;
            prevOp.Next = newOp2;
        }

        // nb: area1 is the path's area *before* splitting, whereas area2 is
        // the area of the triangle containing splitOp & splitOp.Next.
        // So the only way for these areas to have the same sign is if
        // the split triangle is larger than the path containing prevOp or
        // if there's more than one self=intersection.
        if (!(absArea2 > ClipGeometry.SmallAreaTolerance) ||
                (!(absArea2 > absArea1) &&
                  ((area2 > 0) != (area1 > 0))))
        {
            return;
        }

        OutRec newOutRec = this.NewOutRec();
        newOutRec.Owner = outrec.Owner;
        splitOp.OutRec = newOutRec;
        splitOp.Next.OutRec = newOutRec;

        OutPt newOp = this.outPointPool.Add(ip, newOutRec);
        newOp.Prev = splitOp.Next;
        newOp.Next = splitOp;
        newOutRec.Points = newOp;
        splitOp.Prev = newOp;
        splitOp.Next.Next = newOp;

        if (!this.usingPolyTree)
        {
            return;
        }

        if (Path1InsidePath2(prevOp, newOp))
        {
            newOutRec.Splits ??= [];
            newOutRec.Splits.Add(outrec.Index);
        }
        else
        {
            outrec.Splits ??= [];
            outrec.Splits.Add(newOutRec.Index);
        }

        // else { splitOp = null; splitOp.Next = null; }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void FixSelfIntersects(OutRec outrec)
    {
        OutPt op2 = outrec.Points!;
        if (op2.Prev == op2.Next!.Next)
        {
            // Triangles can't self-intersect.
            return;
        }

        for (; ;)
        {
            if (ClipGeometry.SegmentsIntersect(
                op2!.Prev.Point,
                op2.Point,
                op2.Next!.Point,
                op2.Next.Next!.Point))
            {
                if (ClipGeometry.SegmentsIntersect(
                    op2.Prev.Point,
                    op2.Point,
                    op2.Next.Next!.Point,
                    op2.Next.Next.Next!.Point))
                {
                    // adjacent intersections (ie a micro self-intersection)
                    op2 = this.DuplicateOp(op2, false);
                    op2.Point = op2.Next!.Next!.Next!.Point;
                    op2 = op2.Next;
                }
                else
                {
                    if (op2 == outrec.Points || op2.Next == outrec.Points)
                    {
                        outrec.Points = outrec.Points.Prev;
                    }

                    this.DoSplitOp(outrec, op2);
                    if (outrec.Points == null)
                    {
                        return;
                    }

                    op2 = outrec.Points;

                    // triangles can't self-intersect
                    if (op2.Prev == op2.Next!.Next)
                    {
                        break;
                    }

                    continue;
                }
            }

            op2 = op2.Next!;
            if (op2 == outrec.Points)
            {
                break;
            }
        }
    }

    internal static bool BuildPath(OutPt? op, bool reverse, bool isOpen, Contour path)
    {
        if (op == null || op.Next == op || (!isOpen && op.Next == op.Prev))
        {
            return false;
        }

        path.Clear();

        Vertex lastPt;
        OutPt op2;
        if (reverse)
        {
            lastPt = op.Point;
            op2 = op.Prev;
        }
        else
        {
            op = op.Next!;
            lastPt = op.Point;
            op2 = op.Next!;
        }

        path.Add(lastPt);

        while (op2 != op)
        {
            if (!ClipGeometry.PointEquals(op2.Point, lastPt))
            {
                lastPt = op2.Point;
                path.Add(lastPt);
            }

            if (reverse)
            {
                op2 = op2.Prev;
            }
            else
            {
                op2 = op2.Next!;
            }
        }

        return path.Count != 3 || isOpen || !IsVerySmallTriangle(op2);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vertex ToVertex(Vertex point) => new(point.X, -point.Y);

    internal static bool BuildContour(OutPt? op, bool reverse, bool isOpen, Contour contour)
    {
        if (op == null || op.Next == op || (!isOpen && op.Next == op.Prev))
        {
            return false;
        }

        contour.Clear();

        Vertex lastPt;
        OutPt op2;
        if (reverse)
        {
            lastPt = ToVertex(op.Point);
            op2 = op.Prev;
        }
        else
        {
            op = op.Next!;
            lastPt = ToVertex(op.Point);
            op2 = op.Next!;
        }

        contour.Add(lastPt);

        while (op2 != op)
        {
            Vertex current = ToVertex(op2.Point);
            if (current != lastPt)
            {
                lastPt = current;
                contour.Add(lastPt);
            }

            op2 = reverse ? op2.Prev : op2.Next!;
        }

        if (contour.Count == 3 && !isOpen && IsVerySmallTriangle(op2))
        {
            contour.Clear();
            return false;
        }

        if (!isOpen && contour.Count > 0)
        {
            contour.Add(contour[0]);
        }

        return true;
    }

    private bool BuildPaths(List<Contour> solutionClosed, List<Contour> solutionOpen)
    {
        solutionClosed.Clear();
        solutionOpen.Clear();
        solutionClosed.EnsureCapacity(this.outrecPool.Count);
        solutionOpen.EnsureCapacity(this.outrecPool.Count);
        this.pathPoolIndex = 0;

        int i = 0;

        // this.outrecPool.Count is not static here because
        // CleanCollinear can indirectly add additional OutRec
        while (i < this.outrecPool.Count)
        {
            OutRec outrec = this.outrecPool[i++];
            if (outrec.Points == null)
            {
                continue;
            }

            Contour path = this.GetPooledPath(outrec.OutPointCount);
            if (outrec.IsOpen)
            {
                if (BuildPath(outrec.Points, this.ReverseSolution, true, path))
                {
                    solutionOpen.Add(path);
                }
            }
            else
            {
                this.CleanCollinear(outrec);

                // closed paths should always return a Positive orientation
                // except when ReverseSolution == true
                if (BuildPath(outrec.Points, this.ReverseSolution, false, path))
                {
                    solutionClosed.Add(path);
                }
            }
        }

        return true;
    }

    private void BuildContours(List<Contour> solution)
    {
        solution.Clear();
        solution.EnsureCapacity(this.outrecPool.Count);

        int i = 0;
        while (i < this.outrecPool.Count)
        {
            OutRec outrec = this.outrecPool[i++];
            if (outrec.Points == null)
            {
                continue;
            }

            Contour contour = new(outrec.OutPointCount + 1);
            if (outrec.IsOpen)
            {
                if (BuildContour(outrec.Points, this.ReverseSolution, true, contour))
                {
                    solution.Add(contour);
                }
            }
            else
            {
                this.CleanCollinear(outrec);
                if (BuildContour(outrec.Points, this.ReverseSolution, false, contour))
                {
                    solution.Add(contour);
                }
            }
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private bool CheckBounds(OutRec outrec)
    {
        if (outrec.Points == null)
        {
            return false;
        }

        if (!outrec.Bounds.IsEmpty())
        {
            return true;
        }

        this.CleanCollinear(outrec);
        if (outrec.Points == null ||
            !BuildPath(outrec.Points, this.ReverseSolution, false, outrec.Path))
        {
            return false;
        }

        outrec.Bounds = ClipGeometry.GetBounds(outrec.Path);
        return true;
    }

    private bool CheckSplitOwner(OutRec outrec, List<int>? splits)
    {
        // nb: use indexing (not an iterator) in case 'splits' is modified inside this loop (#1029)
        for (int i = 0; i < splits!.Count; i++)
        {
            OutRec? split = this.outrecPool[splits[i]];
            if (split.Points == null && split.Splits != null &&
                this.CheckSplitOwner(outrec, split.Splits))
            {
                // #942
                return true;
            }

            split = GetRealOutRec(split);
            if (split == null || split == outrec || split.RecursiveSplit == outrec)
            {
                continue;
            }

            // #599
            split.RecursiveSplit = outrec;

            if (split.Splits != null && this.CheckSplitOwner(outrec, split.Splits))
            {
                return true;
            }

            if (!this.CheckBounds(split) ||
                    !split.Bounds.Contains(outrec.Bounds) ||
                    !Path1InsidePath2(outrec.Points!, split.Points!))
            {
                continue;
            }

            // split is owned by outrec (#957)
            if (!IsValidOwner(outrec, split))
            {
                split.Owner = outrec.Owner;
            }

            // Found in split.
            outrec.Owner = split;
            return true;
        }

        return false;
    }

    private void RecursiveCheckOwners(OutRec outrec, PolyPathBase polypath)
    {
        // pre-condition: outrec will have valid bounds
        // post-condition: if a valid path, outrec will have a polypath
        if (outrec.PolyPath != null || outrec.Bounds.IsEmpty())
        {
            return;
        }

        while (outrec.Owner != null)
        {
            if (outrec.Owner.Splits != null &&
                this.CheckSplitOwner(outrec, outrec.Owner.Splits))
            {
                break;
            }

            if (outrec.Owner.Points != null && this.CheckBounds(outrec.Owner) &&
                Path1InsidePath2(outrec.Points!, outrec.Owner.Points!))
            {
                break;
            }

            outrec.Owner = outrec.Owner.Owner;
        }

        if (outrec.Owner != null)
        {
            if (outrec.Owner.PolyPath == null)
            {
                this.RecursiveCheckOwners(outrec.Owner, polypath);
            }

            outrec.PolyPath = outrec.Owner.PolyPath!.AddChild(outrec.Path);
        }
        else
        {
            outrec.PolyPath = polypath.AddChild(outrec.Path);
        }
    }

    private void BuildTree(PolyPathBase polytree, List<Contour> solutionOpen)
    {
        polytree.Clear();
        solutionOpen.Clear();
        if (this.hasOpenPaths)
        {
            solutionOpen.EnsureCapacity(this.outrecPool.Count);
        }

        this.pathPoolIndex = 0;

        int i = 0;

        // this.outrecPool.Count is not static here because
        // CheckBounds below can indirectly add additional
        // OutRec (via FixOutRecPts & CleanCollinear)
        while (i < this.outrecPool.Count)
        {
            OutRec outrec = this.outrecPool[i++];
            if (outrec.Points == null)
            {
                continue;
            }

            if (outrec.IsOpen)
            {
                Contour open_path = this.GetPooledPath(outrec.OutPointCount);
                if (BuildPath(outrec.Points, this.ReverseSolution, true, open_path))
                {
                    solutionOpen.Add(open_path);
                }

                continue;
            }

            if (this.CheckBounds(outrec))
            {
                this.RecursiveCheckOwners(outrec, polytree);
            }
        }
    }

    /// <summary>
    /// Retrieves a reusable <see cref="Contour"/> with at least the specified capacity.
    /// </summary>
    /// <param name="capacity">The minimum capacity required for the path.</param>
    /// <returns>A cleared path ready for reuse.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private Contour GetPooledPath(int capacity)
    {
        if (this.pathPoolIndex < this.pathPool.Count)
        {
            Contour path = this.pathPool[this.pathPoolIndex++];
            path.Clear();
            path.EnsureVertexCapacity(capacity);

            return path;
        }

        Contour newPath = new(capacity);
        this.pathPool.Add(newPath);
        this.pathPoolIndex++;
        return newPath;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal ClipBounds GetBounds()
    {
        ClipBounds bounds = ClipMath.InvalidBounds;
        foreach (ClipVertex t in this.vertexList)
        {
            ClipVertex v = t;
            do
            {
                if (v.Point.X < bounds.Left)
                {
                    bounds.Left = v.Point.X;
                }

                if (v.Point.X > bounds.Right)
                {
                    bounds.Right = v.Point.X;
                }

                if (v.Point.Y < bounds.Top)
                {
                    bounds.Top = v.Point.Y;
                }

                if (v.Point.Y > bounds.Bottom)
                {
                    bounds.Bottom = v.Point.Y;
                }

                v = v.Next!;
            }
            while (v != t);
        }

        return bounds.IsEmpty() ? new ClipBounds(0, 0, 0, 0) : bounds;
    }

    internal bool Execute(
        ClipperOperation clipperOperation,
        ClipperFillRule clipperFillRule,
        List<Contour> solutionClosed,
        List<Contour> solutionOpen)
    {
        solutionClosed.Clear();
        solutionOpen.Clear();
        try
        {
            this.ExecuteInternal(clipperOperation, clipperFillRule);
            this.BuildPaths(solutionClosed, solutionOpen);
        }
        catch
        {
            this.succeeded = false;
        }

        this.ClearSolutionOnly();
        return this.succeeded;
    }

    internal bool Execute(
        ClipperOperation clipperOperation,
        ClipperFillRule clipperFillRule,
        PolyTree polytree,
        List<Contour> openPaths)
    {
        polytree.Clear();
        openPaths.Clear();
        this.usingPolyTree = true;
        try
        {
            this.ExecuteInternal(clipperOperation, clipperFillRule);
            this.BuildTree(polytree, openPaths);
        }
        catch
        {
            this.succeeded = false;
        }

        this.ClearSolutionOnly();
        return this.succeeded;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal bool Execute(
        ClipperOperation clipperOperation,
        ClipperFillRule clipperFillRule,
        PolyTree polytree)
        => this.Execute(clipperOperation, clipperFillRule, polytree, []);

    internal bool Execute(
        ClipperOperation clipperOperation,
        ClipperFillRule clipperFillRule,
        List<Contour> solution)
    {
        solution.Clear();
        try
        {
            this.ExecuteInternal(clipperOperation, clipperFillRule);
            this.BuildContours(solution);
        }
        catch
        {
            this.succeeded = false;
        }

        this.ClearSolutionOnly();
        return this.succeeded;
    }

    private struct IntersectListSort : IComparer<IntersectNode>
    {
        public readonly int Compare(IntersectNode a, IntersectNode b)
        {
            if (!ClipGeometry.IsAlmostZero(a.Point.Y - b.Point.Y))
            {
                return (a.Point.Y > b.Point.Y) ? -1 : 1;
            }

            if (ClipGeometry.IsAlmostZero(a.Point.X - b.Point.X))
            {
                return 0;
            }

            return (a.Point.X < b.Point.X) ? -1 : 1;
        }
    }
}

// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System.Runtime.CompilerServices;

namespace SixLabors.PolygonClipper;

/// <summary>
/// Sweep-line union clipper specialized for self-intersection removal.
/// </summary>
/// <remarks>
/// This clipper consumes subject-only paths and applies the selected fill rule
/// to compute the union. It reuses pooled data structures to keep allocations low.
/// </remarks>
internal sealed class SelfIntersectionUnionClipper
{
    private FillRule fillRule;
    private ActiveEdge? activeEdges;
    private ActiveEdge? sortedEdges;
    private readonly Stack<ActiveEdge> freeActives;
    private readonly List<LocalMinima> minimaList;
    private readonly List<IntersectNode> intersectList;
    private readonly VertexPoolList vertexList;
    private readonly OutputRecordPoolList outputRecordPool;
    private readonly List<double> scanlineList;
    private readonly List<HorizontalSegment> horizontalSegments;
    private readonly HorizontalJoinPoolList horizontalJoins;
    private readonly OutputPointPoolList outputPointPool;
    private int currentMinimaIndex;
    private double currentBottomY;
    private bool isSortedMinimaList;
    private bool buildHierarchy;
    private bool succeeded;

    internal SelfIntersectionUnionClipper()
    {
        this.minimaList = [];
        this.intersectList = [];
        this.vertexList = [];
        this.outputRecordPool = new OutputRecordPoolList();
        this.scanlineList = [];
        this.horizontalSegments = [];
        this.horizontalJoins = [];
        this.outputPointPool = [];
        this.freeActives = new Stack<ActiveEdge>();
        this.PreserveCollinear = true;
    }

    internal bool PreserveCollinear { get; set; }

    internal bool ReverseSolution { get; set; }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void SwapActives(ref ActiveEdge ae1, ref ActiveEdge ae2) => (ae2, ae1) = (ae1, ae2);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static ActiveEdge? GetMaximaPair(ActiveEdge ae)
    {
        ActiveEdge? ae2 = ae.NextInAel;
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
    private static ClipVertex? GetCurrentYMaximaVertex(ActiveEdge ae)
    {
        ClipVertex? result = ae.VertexTop;
        if (ae.WindDelta > 0)
        {
            while (PolygonUtilities.IsAlmostZero(result!.Next!.Point.Y - result.Point.Y))
            {
                result = result.Next;
            }
        }
        else
        {
            while (PolygonUtilities.IsAlmostZero(result!.Prev!.Point.Y - result.Point.Y))
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
    private static void SetSides(OutputRecord outputRecord, ActiveEdge startEdge, ActiveEdge endEdge)
    {
        outputRecord.FrontEdge = startEdge;
        outputRecord.BackEdge = endEdge;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void SwapOutputRecords(ActiveEdge ae1, ActiveEdge ae2)
    {
        // At least one edge has an assigned outputRecord.
        OutputRecord? or1 = ae1.OutputRecord;
        OutputRecord? or2 = ae2.OutputRecord;
        if (or1 == or2)
        {
            ActiveEdge? ae = or1!.FrontEdge;
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

        ae1.OutputRecord = or2;
        ae2.OutputRecord = or1;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void SetOwner(OutputRecord outputRecord, OutputRecord newOwner)
    {
        // precondition1: new_owner is never null
        while (newOwner.Owner != null && newOwner.Owner.Points == null)
        {
            newOwner.Owner = newOwner.Owner.Owner;
        }

        // make sure that outputRecord isn't an owner of newOwner
        OutputRecord? tmp = newOwner;
        while (tmp != null && tmp != outputRecord)
        {
            tmp = tmp.Owner;
        }

        if (tmp != null)
        {
            newOwner.Owner = outputRecord.Owner;
        }

        outputRecord.Owner = newOwner;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static double Area(OutputPoint op)
    {
        // https://en.wikipedia.org/wiki/Shoelace_formula
        double area = 0.0;
        OutputPoint op2 = op;
        do
        {
            area += Vertex.Cross(op2.Prev.Point, op2.Point);
            op2 = op2.Next!;
        }
        while (op2 != op);
        return area * 0.5;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static OutputRecord? GetRealOutputRecord(OutputRecord? outputRecord)
    {
        while (outputRecord != null && outputRecord.Points == null)
        {
            outputRecord = outputRecord.Owner;
        }

        return outputRecord;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsValidOwner(OutputRecord? outputRecord, OutputRecord? testOwner)
    {
        while (testOwner != null && testOwner != outputRecord)
        {
            testOwner = testOwner.Owner;
        }

        return testOwner == null;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void UncoupleOutputRecord(ActiveEdge ae)
    {
        OutputRecord? outputRecord = ae.OutputRecord;
        if (outputRecord == null)
        {
            return;
        }

        outputRecord.FrontEdge!.OutputRecord = null;
        outputRecord.BackEdge!.OutputRecord = null;
        outputRecord.FrontEdge = null;
        outputRecord.BackEdge = null;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool OutputRecordIsAscending(ActiveEdge hotEdge) => hotEdge == hotEdge.OutputRecord!.FrontEdge;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool EdgesAdjacentInAEL(in IntersectNode inode)
        => (inode.Edge1.NextInAel == inode.Edge2) || (inode.Edge1.PrevInAel == inode.Edge2);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void ClearSolutionOnly()
    {
        while (this.activeEdges != null)
        {
            this.DeleteFromAEL(this.activeEdges);
        }

        this.scanlineList.Clear();
        this.DisposeIntersectNodes();
        this.outputRecordPool.Clear();
        this.horizontalSegments.Clear();
        this.horizontalJoins.Clear();
        this.outputPointPool.Clear();
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal void Clear()
    {
        this.ClearSolutionOnly();
        this.minimaList.Clear();
        this.vertexList.Clear();
        this.currentMinimaIndex = 0;
        this.isSortedMinimaList = false;
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
    private bool HasLocMinAtY(double y)
        => this.currentMinimaIndex < this.minimaList.Count &&
           this.minimaList[this.currentMinimaIndex].Vertex.Point.Y == y;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private LocalMinima PopLocalMinima() => this.minimaList[this.currentMinimaIndex++];

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal void AddSubject(List<Contour> paths)
    {
        this.isSortedMinimaList = false;
        this.AddPathsToVertexList(paths);
    }

    /// <summary>
    /// Registers a local minima vertex once for the sweep.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void AddLocalMinima(ClipVertex vertex, List<LocalMinima> minimaList)
    {
        // Make sure the ClipVertex is added only once.
        if ((vertex.Flags & VertexFlags.LocalMin) != VertexFlags.None)
        {
            return;
        }

        vertex.Flags |= VertexFlags.LocalMin;
        minimaList.Add(new LocalMinima(vertex));
    }

    /// <summary>
    /// Builds circular vertex lists and captures local minima/maxima for the sweep.
    /// </summary>
    private void AddPathsToVertexList(List<Contour> paths)
    {
        int totalVertCnt = 0;
        foreach (Contour path in paths)
        {
            totalVertCnt += path.Count;
        }

        // Pre-size the pool so vertex allocation is deterministic and cheap.
        this.vertexList.EnsureCapacity(this.vertexList.Count + totalVertCnt);

        foreach (Contour path in paths)
        {
            ClipVertex? v0 = null;
            ClipVertex? prevVertex = null;
            ClipVertex? currVertex;
            foreach (Vertex pt in path)
            {
                if (v0 == null)
                {
                    v0 = this.vertexList.Add(pt, VertexFlags.None, null);
                    prevVertex = v0;
                    continue;
                }

                if (!PolygonUtilities.PointEquals(prevVertex!.Point, pt))
                {
                    currVertex = this.vertexList.Add(pt, VertexFlags.None, prevVertex);
                    prevVertex.Next = currVertex;
                    prevVertex = currVertex;
                }
            }

            if (v0 == null || prevVertex?.Prev == null)
            {
                continue;
            }

            if (PolygonUtilities.PointEquals(prevVertex.Point, v0.Point))
            {
                prevVertex = prevVertex.Prev;
            }

            prevVertex.Next = v0;
            v0.Prev = prevVertex;
            if (prevVertex.Next == prevVertex)
            {
                continue;
            }

            // OK, we have a valid path
            prevVertex = v0.Prev;
            while (prevVertex != v0 && PolygonUtilities.IsAlmostZero(prevVertex!.Point.Y - v0.Point.Y))
            {
                prevVertex = prevVertex.Prev;
            }

            if (prevVertex == v0)
            {
                // Closed paths cannot be completely flat.
                continue;
            }

            bool goingUp = prevVertex.Point.Y > v0.Point.Y;

            bool goingUp0 = goingUp;
            prevVertex = v0;
            currVertex = v0.Next;
            while (currVertex != v0)
            {
                if (currVertex!.Point.Y > prevVertex.Point.Y && goingUp)
                {
                    prevVertex.Flags |= VertexFlags.LocalMax;
                    goingUp = false;
                }
                else if (currVertex.Point.Y < prevVertex.Point.Y && !goingUp)
                {
                    goingUp = true;
                    AddLocalMinima(prevVertex, this.minimaList);
                }

                prevVertex = currVertex;
                currVertex = currVertex.Next;
            }

            if (goingUp != goingUp0)
            {
                if (goingUp0)
                {
                    AddLocalMinima(prevVertex, this.minimaList);
                }
                else
                {
                    prevVertex.Flags |= VertexFlags.LocalMax;
                }
            }
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private bool IsContributingClosed(ActiveEdge ae)
    {
        if (this.fillRule == FillRule.Positive)
        {
            return ae.WindCount == 1;
        }

        return ae.WindCount == -1;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void SetWindCountForClosedPathEdge(ActiveEdge ae)
    {
        // Wind counts refer to polygon regions not edges, so here an edge's WindCnt
        // indicates the higher of the wind counts for the two regions touching the
        // edge. (nb: Adjacent regions can only ever have their wind counts differ by one.)
        ActiveEdge? ae2 = ae.PrevInAel;

        if (ae2 == null)
        {
            ae.WindCount = ae.WindDelta;
        }
        else
        {
            // Positive or negative filling here ...
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
                    // now outside all polygons so set own WC ...
                    ae.WindCount = ae.WindDelta;
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
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsValidAelOrder(ActiveEdge resident, ActiveEdge newcomer)
    {
        if (newcomer.CurrentX != resident.CurrentX)
        {
            return newcomer.CurrentX > resident.CurrentX;
        }

        // get the turning direction  a1.Top, a2.Bottom, a2.Top
        int d = PolygonUtilities.CrossSign(resident.Top, newcomer.Bottom, newcomer.Top);
        if (d != 0)
        {
            return d < 0;
        }

        // edges must be collinear to get here

        // for collinear bounds, place them according to the next turn
        if (!resident.IsMaxima && (resident.Top.Y > newcomer.Top.Y))
        {
            return PolygonUtilities.CrossSign(
                newcomer.Bottom,
                resident.Top,
                resident.NextVertex.Point) <= 0;
        }

        if (!newcomer.IsMaxima && (newcomer.Top.Y > resident.Top.Y))
        {
            return PolygonUtilities.CrossSign(
                newcomer.Bottom,
                newcomer.Top,
                newcomer.NextVertex.Point) >= 0;
        }

        double y = newcomer.Bottom.Y;
        bool newcomerIsLeft = newcomer.IsLeftBound;

        if (resident.Bottom.Y != y || resident.LocalMin.Vertex.Point.Y != y)
        {
            return newcomer.IsLeftBound;
        }

        // resident must also have just been inserted
        if (resident.IsLeftBound != newcomerIsLeft)
        {
            return newcomerIsLeft;
        }

        if (PolygonUtilities.IsCollinear(
            resident.PrevPrevVertex.Point,
            resident.Bottom,
            resident.Top))
        {
            return true;
        }

        // compare turning direction of the alternate bound
        return (PolygonUtilities.CrossSign(
            resident.PrevPrevVertex.Point,
            newcomer.Bottom,
            newcomer.PrevPrevVertex.Point) > 0) == newcomerIsLeft;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void InsertLeftEdge(ActiveEdge ae)
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
            ActiveEdge ae2 = this.activeEdges;
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
    private static void InsertRightEdge(ActiveEdge ae, ActiveEdge ae2)
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
            ActiveEdge leftBound = this.NewActive();
            leftBound.Bottom = localMinima.Vertex.Point;
            leftBound.CurrentX = localMinima.Vertex.Point.X;
            leftBound.WindDelta = -1;
            leftBound.VertexTop = localMinima.Vertex.Prev;
            leftBound.Top = localMinima.Vertex.Prev!.Point;
            leftBound.OutputRecord = null;
            leftBound.LocalMin = localMinima;
            leftBound.UpdateDx();

            ActiveEdge rightBound = this.NewActive();
            rightBound.Bottom = localMinima.Vertex.Point;
            rightBound.CurrentX = localMinima.Vertex.Point.X;
            rightBound.WindDelta = 1;

            // Ascending.
            rightBound.VertexTop = localMinima.Vertex.Next;
            rightBound.Top = localMinima.Vertex.Next!.Point;
            rightBound.OutputRecord = null;
            rightBound.LocalMin = localMinima;
            rightBound.UpdateDx();

            // Currently LeftB is just the descending bound and RightB is the ascending.
            // Now if the LeftB isn't on the left of RightB then we need swap them.
            if (leftBound.IsHorizontal)
            {
                if (leftBound.IsHeadingRightHorizontal)
                {
                    SwapActives(ref leftBound, ref rightBound);
                }
            }
            else if (rightBound.IsHorizontal)
            {
                if (rightBound.IsHeadingLeftHorizontal)
                {
                    SwapActives(ref leftBound, ref rightBound);
                }
            }
            else if (leftBound.Dx < rightBound.Dx)
            {
                SwapActives(ref leftBound, ref rightBound);
            }

            bool contributing;
            leftBound.IsLeftBound = true;
            this.InsertLeftEdge(leftBound);

            SetWindCountForClosedPathEdge(leftBound);
            contributing = this.IsContributingClosed(leftBound);

            rightBound.WindCount = leftBound.WindCount;
            InsertRightEdge(leftBound, rightBound);

            if (contributing)
            {
                this.AddLocalMinPoly(leftBound, rightBound, leftBound.Bottom, true);
                if (!leftBound.IsHorizontal)
                {
                    this.CheckJoinLeft(leftBound, leftBound.Bottom);
                }
            }

            while (rightBound.NextInAel != null &&
                          IsValidAelOrder(rightBound.NextInAel, rightBound))
            {
                this.IntersectEdges(rightBound, rightBound.NextInAel, rightBound.Bottom);
                this.SwapPositionsInAEL(rightBound, rightBound.NextInAel);
            }

            if (rightBound.IsHorizontal)
            {
                this.PushHorizontal(rightBound);
            }
            else
            {
                this.CheckJoinRight(rightBound, rightBound.Bottom);
                this.InsertScanline(rightBound.Top.Y);
            }

            if (leftBound.IsHorizontal)
            {
                this.PushHorizontal(leftBound);
            }
            else
            {
                this.InsertScanline(leftBound.Top.Y);
            }
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void PushHorizontal(ActiveEdge ae)
    {
        ae.NextInSel = this.sortedEdges;
        this.sortedEdges = ae;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private bool PopHorizontal(out ActiveEdge? ae)
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
    private OutputPoint AddLocalMinPoly(ActiveEdge ae1, ActiveEdge ae2, Vertex pt, bool isNew = false)
    {
        OutputRecord outputRecord = this.NewOutputRecord();
        ae1.OutputRecord = outputRecord;
        ae2.OutputRecord = outputRecord;

        ActiveEdge? prevHotEdge = ae1.GetPrevHotEdge();

        // e.WindDelta is the winding direction of the **input** paths
        // and unrelated to the winding direction of output polygons.
        // Output orientation is determined by the edge's OutputRecord.FrontEdge which is
        // the ascending edge (see AddLocalMinPoly).
        if (prevHotEdge != null)
        {
            if (this.buildHierarchy)
            {
                SetOwner(outputRecord, prevHotEdge.OutputRecord!);
            }

            outputRecord.Owner = prevHotEdge.OutputRecord;
            if (OutputRecordIsAscending(prevHotEdge) == isNew)
            {
                SetSides(outputRecord, ae2, ae1);
            }
            else
            {
                SetSides(outputRecord, ae1, ae2);
            }
        }
        else
        {
            outputRecord.Owner = null;
            if (isNew)
            {
                SetSides(outputRecord, ae1, ae2);
            }
            else
            {
                SetSides(outputRecord, ae2, ae1);
            }
        }

        OutputPoint op = this.outputPointPool.Add(pt, outputRecord);
        outputRecord.Points = op;
        return op;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private OutputPoint? AddLocalMaxPoly(ActiveEdge ae1, ActiveEdge ae2, Vertex pt)
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
            this.succeeded = false;
            return null;
        }

        OutputPoint result = this.AddOutputPoint(ae1, pt);
        if (ae1.OutputRecord == ae2.OutputRecord)
        {
            OutputRecord outputRecord = ae1.OutputRecord!;
            outputRecord.Points = result;

            if (this.buildHierarchy)
            {
                ActiveEdge? e = ae1.GetPrevHotEdge();
                if (e == null)
                {
                    outputRecord.Owner = null;
                }
                else
                {
                    SetOwner(outputRecord, e.OutputRecord!);
                }

                // nb: outputRecord.Owner here is likely NOT the real
                // owner but this will be fixed in DeepCheckOwner()
            }

            UncoupleOutputRecord(ae1);
        }

        // and to preserve the winding orientation of OutputRecord ...
        else if (ae1.OutputRecord!.Index < ae2.OutputRecord!.Index)
        {
            JoinOutputRecordPaths(ae1, ae2);
        }
        else
        {
            JoinOutputRecordPaths(ae2, ae1);
        }

        return result;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void JoinOutputRecordPaths(ActiveEdge ae1, ActiveEdge ae2)
    {
        // join ae2 OutputRecord path onto ae1 OutputRecord path and then delete ae2 OutputRecord path
        // pointers. (NB Only very rarely do the joining ends share the same coords.)
        OutputPoint p1Start = ae1.OutputRecord!.Points!;
        OutputPoint p2Start = ae2.OutputRecord!.Points!;
        OutputPoint p1End = p1Start.Next!;
        OutputPoint p2End = p2Start.Next!;
        if (ae1.IsFront)
        {
            p2End.Prev = p1Start;
            p1Start.Next = p2End;
            p2Start.Next = p1End;
            p1End.Prev = p2Start;
            ae1.OutputRecord!.Points = p2Start;

            ae1.OutputRecord!.FrontEdge = ae2.OutputRecord!.FrontEdge;
            if (ae1.OutputRecord!.FrontEdge != null)
            {
                ae1.OutputRecord!.FrontEdge!.OutputRecord = ae1.OutputRecord;
            }
        }
        else
        {
            p1End.Prev = p2Start;
            p2Start.Next = p1End;
            p1Start.Next = p2End;
            p2End.Prev = p1Start;

            ae1.OutputRecord!.BackEdge = ae2.OutputRecord!.BackEdge;
            if (ae1.OutputRecord!.BackEdge != null)
            {
                ae1.OutputRecord!.BackEdge!.OutputRecord = ae1.OutputRecord;
            }
        }

        // after joining, the ae2.OutputRecord must contains no vertices ...
        ae2.OutputRecord!.FrontEdge = null;
        ae2.OutputRecord!.BackEdge = null;
        ae2.OutputRecord!.Points = null;
        ae1.OutputRecord!.OutputPointCount += ae2.OutputRecord!.OutputPointCount;
        SetOwner(ae2.OutputRecord, ae1.OutputRecord);

        // and ae1 and ae2 are maxima and are about to be dropped from the Actives list.
        ae1.OutputRecord = null;
        ae2.OutputRecord = null;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private OutputPoint AddOutputPoint(ActiveEdge ae, Vertex pt)
    {
        // outputRecord.Points: a circular doubly-linked list of OutputPoint where ...
        // opFront[.Prev]* ~~~> opBack & opBack == opFront.Next
        OutputRecord outputRecord = ae.OutputRecord!;
        bool toFront = ae.IsFront;
        OutputPoint opFront = outputRecord.Points!;
        OutputPoint opBack = opFront.Next!;

        switch (toFront)
        {
            case true when PolygonUtilities.PointEquals(pt, opFront.Point):
                return opFront;
            case false when PolygonUtilities.PointEquals(pt, opBack.Point):
                return opBack;
        }

        OutputPoint newOp = this.outputPointPool.Add(pt, outputRecord);
        opBack.Prev = newOp;
        newOp.Prev = opFront;
        newOp.Next = opBack;
        opFront.Next = newOp;
        if (toFront)
        {
            outputRecord.Points = newOp;
        }

        return newOp;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private OutputRecord NewOutputRecord()
    {
        int idx = this.outputRecordPool.Count;
        OutputRecord result = this.outputRecordPool.Add();
        result.Index = idx;
        return result;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void UpdateEdgeIntoAEL(ActiveEdge ae)
    {
        ae.Bottom = ae.Top;
        ae.VertexTop = ae.NextVertex;
        ae.Top = ae.VertexTop!.Point;
        ae.CurrentX = ae.Bottom.X;
        ae.UpdateDx();

        if (IsJoined(ae))
        {
            this.Split(ae, ae.Bottom);
        }

        if (ae.IsHorizontal)
        {
            TrimHorizontal(ae, this.PreserveCollinear);

            return;
        }

        this.InsertScanline(ae.Top.Y);

        this.CheckJoinLeft(ae, ae.Bottom);

        // (#500)
        this.CheckJoinRight(ae, ae.Bottom, true);
    }

    private void IntersectEdges(ActiveEdge ae1, ActiveEdge ae2, Vertex pt)
    {
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

        if (this.fillRule == FillRule.Positive)
        {
            oldE1WindCount = ae1.WindCount;
            oldE2WindCount = ae2.WindCount;
        }
        else
        {
            oldE1WindCount = -ae1.WindCount;
            oldE2WindCount = -ae2.WindCount;
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
            if ((oldE1WindCount != 0 && oldE1WindCount != 1) || (oldE2WindCount != 0 && oldE2WindCount != 1))
            {
                this.AddLocalMaxPoly(ae1, ae2, pt);
            }
            else if (ae1.IsFront || (ae1.OutputRecord == ae2.OutputRecord))
            {
                // this 'else if' condition isn't strictly needed but
                // it's sensible to split polygons that only touch at
                // a common ClipVertex (not at common edges).
                this.AddLocalMaxPoly(ae1, ae2, pt);
            }
            else
            {
                // can't treat as maxima & minima
                this.AddOutputPoint(ae1, pt);
                SwapOutputRecords(ae1, ae2);
            }
        }

        // if one or other edge is 'hot' ...
        else if (ae1.IsHot)
        {
            this.AddOutputPoint(ae1, pt);
            SwapOutputRecords(ae1, ae2);
        }
        else if (ae2.IsHot)
        {
            this.AddOutputPoint(ae2, pt);
            SwapOutputRecords(ae1, ae2);
        }

        // neither edge is 'hot'
        else
        {
            if (oldE1WindCount == 1 && oldE2WindCount == 1)
            {
                this.AddLocalMinPoly(ae1, ae2, pt);
            }
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void DeleteFromAEL(ActiveEdge ae)
    {
        ActiveEdge? prev = ae.PrevInAel;
        ActiveEdge? next = ae.NextInAel;
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
    private void PoolDeletedActive(ActiveEdge ae)
    {
        // clear refs to allow GC
        ae.Bottom = default;
        ae.Top = default;
        ae.Dx = 0.0;
        ae.WindCount = 0;
        ae.OutputRecord = null;
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
    private ActiveEdge NewActive()
    {
        ActiveEdge ae;
        if (this.freeActives.Count == 0)
        {
            ae = new ActiveEdge();
        }
        else
        {
            // recycle ActiveEdge from free list
            ae = this.freeActives.Pop();
        }

        return ae;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void AdjustCurrXAndCopyToSEL(double topY)
    {
        ActiveEdge? ae = this.activeEdges;
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

    private void ExecuteInternal(FillRule fillRule)
    {
        this.fillRule = fillRule;
        this.Reset();
        if (!this.PopScanline(out double y))
        {
            return;
        }

        // Process each scanbeam: insert local minima, handle horizontals, resolve intersections,
        // then advance to the next scanline.
        while (this.succeeded)
        {
            this.InsertLocalMinimaIntoAEL(y);
            ActiveEdge? ae;
            while (this.PopHorizontal(out ae))
            {
                this.DoHorizontal(ae!);
            }

            if (this.horizontalSegments.Count > 0)
            {
                this.ConvertHorizontalSegmentsToJoins();
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
            while (this.PopHorizontal(out ae))
            {
                this.DoHorizontal(ae!);
            }
        }

        if (this.succeeded)
        {
            this.ProcessHorizontalJoins();
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
    private void AddNewIntersectNode(ActiveEdge ae1, ActiveEdge ae2, double topY)
    {
        if (!PolygonUtilities.TryGetLineIntersection(
            ae1.Bottom, ae1.Top, ae2.Bottom, ae2.Top, out Vertex ip))
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
                        ip = PolygonUtilities.ClosestPointOnSegment(ip, ae1.Bottom, ae1.Top);
                    }
                    else
                    {
                        ip = PolygonUtilities.ClosestPointOnSegment(ip, ae2.Bottom, ae2.Top);
                    }

                    break;
                }

                case true:
                    ip = PolygonUtilities.ClosestPointOnSegment(ip, ae1.Bottom, ae1.Top);
                    break;
                default:
                {
                    if (absDx2 > 100)
                    {
                        ip = PolygonUtilities.ClosestPointOnSegment(ip, ae2.Bottom, ae2.Top);
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
    private static ActiveEdge? ExtractFromSEL(ActiveEdge ae)
    {
        ActiveEdge? res = ae.NextInSel;
        if (res != null)
        {
            res.PrevInSel = ae.PrevInSel;
        }

        ae.PrevInSel!.NextInSel = res;
        return res;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void Insert1Before2InSEL(ActiveEdge ae1, ActiveEdge ae2)
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
        ActiveEdge? left = this.sortedEdges;

        while (left!.Jump != null)
        {
            ActiveEdge? prevBase = null;
            while (left?.Jump != null)
            {
                ActiveEdge? currBase = left;
                ActiveEdge? right = left.Jump;
                ActiveEdge? lEnd = right;
                ActiveEdge? rEnd = right.Jump;
                left.Jump = rEnd;
                while (left != lEnd && right != rEnd)
                {
                    if (right!.CurrentX < left!.CurrentX)
                    {
                        ActiveEdge? tmp = right.PrevInSel!;
                        while (true)
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
    private void SwapPositionsInAEL(ActiveEdge ae1, ActiveEdge ae2)
    {
        // preconditon: ae1 must be immediately to the left of ae2
        ActiveEdge? next = ae2.NextInAel;
        if (next != null)
        {
            next.PrevInAel = ae1;
        }

        ActiveEdge? prev = ae1.PrevInAel;
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
    private static bool ResetHorizontalDirection(
        ActiveEdge horizontalEdge,
        ClipVertex? vertexMax,
        out double leftX,
        out double rightX)
    {
        if (horizontalEdge.Bottom.X == horizontalEdge.Top.X)
        {
            // the horizontal edge is going nowhere ...
            leftX = horizontalEdge.CurrentX;
            rightX = horizontalEdge.CurrentX;
            ActiveEdge? ae = horizontalEdge.NextInAel;
            while (ae != null && ae.VertexTop != vertexMax)
            {
                ae = ae.NextInAel;
            }

            return ae != null;
        }

        if (horizontalEdge.CurrentX < horizontalEdge.Top.X)
        {
            leftX = horizontalEdge.CurrentX;
            rightX = horizontalEdge.Top.X;
            return true;
        }

        // Right to left.
        leftX = horizontalEdge.Top.X;
        rightX = horizontalEdge.CurrentX;
        return false;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void TrimHorizontal(ActiveEdge horizontalEdge, bool preserveCollinear)
    {
        bool wasTrimmed = false;
        Vertex pt = horizontalEdge.NextVertex.Point;

        while (pt.Y == horizontalEdge.Top.Y)
        {
            // always trim 180 deg. spikes (in closed paths)
            // but otherwise break if preserveCollinear = true
            if (preserveCollinear &&
                (pt.X < horizontalEdge.Top.X) != (horizontalEdge.Bottom.X < horizontalEdge.Top.X))
            {
                break;
            }

            horizontalEdge.VertexTop = horizontalEdge.NextVertex;
            horizontalEdge.Top = pt;
            wasTrimmed = true;
            if (horizontalEdge.IsMaxima)
            {
                break;
            }

            pt = horizontalEdge.NextVertex.Point;
        }

        if (wasTrimmed)
        {
            // +/-infinity
            horizontalEdge.UpdateDx();
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void AddToHorizontalSegmentList(OutputPoint op)
        => this.horizontalSegments.Add(new HorizontalSegment(op));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static OutputPoint GetLastOp(ActiveEdge hotEdge)
    {
        OutputRecord outputRecord = hotEdge.OutputRecord!;
        return (hotEdge == outputRecord.FrontEdge) ?
            outputRecord.Points! : outputRecord.Points!.Next!;
    }

    private void DoHorizontal(ActiveEdge horizontalEdge)
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
        double y = horizontalEdge.Bottom.Y;

        ClipVertex? vertex_max = GetCurrentYMaximaVertex(horizontalEdge);

        bool isLeftToRight =
            ResetHorizontalDirection(horizontalEdge, vertex_max, out double leftX, out double rightX);

        if (horizontalEdge.IsHot)
        {
            OutputPoint op = this.AddOutputPoint(horizontalEdge, new Vertex(horizontalEdge.CurrentX, y));
            this.AddToHorizontalSegmentList(op);
        }

        while (true)
        {
            // loops through consecutive horizontal edges
            ActiveEdge? ae = isLeftToRight ? horizontalEdge.NextInAel : horizontalEdge.PrevInAel;

            while (ae != null)
            {
                if (ae.VertexTop == vertex_max)
                {
                    // do this first!!
                    if (horizontalEdge.IsHot && IsJoined(ae))
                    {
                        this.Split(ae, ae.Top);
                    }

                    if (horizontalEdge.IsHot)
                    {
                        while (horizontalEdge.VertexTop != vertex_max)
                        {
                            this.AddOutputPoint(horizontalEdge, horizontalEdge.Top);
                            this.UpdateEdgeIntoAEL(horizontalEdge);
                        }

                        if (isLeftToRight)
                        {
                            this.AddLocalMaxPoly(horizontalEdge, ae, horizontalEdge.Top);
                        }
                        else
                        {
                            this.AddLocalMaxPoly(ae, horizontalEdge, horizontalEdge.Top);
                        }
                    }

                    this.DeleteFromAEL(ae);
                    this.DeleteFromAEL(horizontalEdge);
                    return;
                }

                // if the horizontal edge is a maxima, keep going until we reach
                // its maxima pair, otherwise check for break conditions
                Vertex pt;
                if (vertex_max != horizontalEdge.VertexTop)
                {
                    // otherwise stop when 'ae' is beyond the end of the horizontal line
                    if ((isLeftToRight && ae.CurrentX > rightX) ||
                            (!isLeftToRight && ae.CurrentX < leftX))
                    {
                        break;
                    }

                    if (ae.CurrentX == horizontalEdge.Top.X && !ae.IsHorizontal)
                    {
                        pt = horizontalEdge.NextVertex.Point;

                        // For edges at the horizontal edge's end, only stop when the horizontal
                        // edge's outslope is greater than e's slope when heading right or when the horizontal
                        // edge's outslope is less than e's slope when heading left.
                        if ((isLeftToRight && (ae.TopX(pt.Y) >= pt.X)) ||
                                (!isLeftToRight && (ae.TopX(pt.Y) <= pt.X)))
                        {
                            break;
                        }
                    }
                }

                pt = new Vertex(ae.CurrentX, y);

                if (isLeftToRight)
                {
                    this.IntersectEdges(horizontalEdge, ae, pt);
                    this.SwapPositionsInAEL(horizontalEdge, ae);
                    this.CheckJoinLeft(ae, pt);
                    horizontalEdge.CurrentX = ae.CurrentX;
                    ae = horizontalEdge.NextInAel;
                }
                else
                {
                    this.IntersectEdges(ae, horizontalEdge, pt);
                    this.SwapPositionsInAEL(ae, horizontalEdge);
                    this.CheckJoinRight(ae, pt);
                    horizontalEdge.CurrentX = ae.CurrentX;
                    ae = horizontalEdge.PrevInAel;
                }

                if (horizontalEdge.IsHot)
                {
                    this.AddToHorizontalSegmentList(GetLastOp(horizontalEdge));
                }
            }

            // check if we've finished looping
            // through consecutive horizontals
            if (horizontalEdge.NextVertex.Point.Y != horizontalEdge.Top.Y)
            {
                break;
            }

            // still more horizontals in bound to process ...
            if (horizontalEdge.IsHot)
            {
                this.AddOutputPoint(horizontalEdge, horizontalEdge.Top);
            }

            this.UpdateEdgeIntoAEL(horizontalEdge);

            isLeftToRight = ResetHorizontalDirection(
                horizontalEdge,
                vertex_max,
                out leftX,
                out rightX);
        }

        // End of (possible consecutive) horizontals.
        if (horizontalEdge.IsHot)
        {
            OutputPoint op = this.AddOutputPoint(horizontalEdge, horizontalEdge.Top);
            this.AddToHorizontalSegmentList(op);
        }

        // This is the end of an intermediate horizontal.
        this.UpdateEdgeIntoAEL(horizontalEdge);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void DoTopOfScanbeam(double y)
    {
        // Sorted edges are reused to flag horizontals (see PushHorizontal below).
        this.sortedEdges = null;
        ActiveEdge? ae = this.activeEdges;
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
                    this.AddOutputPoint(ae, ae.Top);
                }

                this.UpdateEdgeIntoAEL(ae);

                // Horizontals are processed later.
                if (ae.IsHorizontal)
                {
                    this.PushHorizontal(ae);
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
    private ActiveEdge? DoMaxima(ActiveEdge ae)
    {
        ActiveEdge? prevE = ae.PrevInAel;
        ActiveEdge? nextE = ae.NextInAel;

        ActiveEdge? maxPair = GetMaximaPair(ae);
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
    private static bool IsJoined(ActiveEdge e) => e.JoinWith != JoinWith.None;

    private void Split(ActiveEdge e, Vertex currPt)
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
        ActiveEdge e,
        Vertex pt,
        bool checkCurrX = false)
    {
        ActiveEdge? prev = e.PrevInAel;
        if (prev == null ||
            !e.IsHot || !prev.IsHot ||
            e.IsHorizontal || prev.IsHorizontal)
        {
            return;
        }

        // Avoid trivial joins.
        if ((pt.Y < e.Top.Y + PolygonUtilities.ClosePointTolerance || pt.Y < prev.Top.Y + PolygonUtilities.ClosePointTolerance) &&
            ((e.Bottom.Y > pt.Y) || (prev.Bottom.Y > pt.Y)))
        {
            // (#490)
            return;
        }

        if (checkCurrX)
        {
            if (PolygonUtilities.PerpendicularDistanceSquared(pt, prev.Bottom, prev.Top) > PolygonUtilities.JoinDistanceSquared)
            {
                return;
            }
        }
        else if (!PolygonUtilities.IsAlmostZero(e.CurrentX - prev.CurrentX))
        {
            return;
        }

        if (!PolygonUtilities.IsCollinear(e.Top, pt, prev.Top))
        {
            return;
        }

        if (e.OutputRecord!.Index == prev.OutputRecord!.Index)
        {
            this.AddLocalMaxPoly(prev, e, pt);
        }
        else if (e.OutputRecord!.Index < prev.OutputRecord!.Index)
        {
            JoinOutputRecordPaths(e, prev);
        }
        else
        {
            JoinOutputRecordPaths(prev, e);
        }

        prev.JoinWith = JoinWith.Right;
        e.JoinWith = JoinWith.Left;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void CheckJoinRight(
        ActiveEdge e,
        Vertex pt,
        bool checkCurrX = false)
    {
        ActiveEdge? next = e.NextInAel;
        if (next == null ||
            !e.IsHot || !next.IsHot ||
            e.IsHorizontal || next.IsHorizontal)
        {
            return;
        }

        // Avoid trivial joins.
        if ((pt.Y < e.Top.Y + PolygonUtilities.ClosePointTolerance || pt.Y < next.Top.Y + PolygonUtilities.ClosePointTolerance) &&
            ((e.Bottom.Y > pt.Y) || (next.Bottom.Y > pt.Y)))
        {
            // (#490)
            return;
        }

        if (checkCurrX)
        {
            if (PolygonUtilities.PerpendicularDistanceSquared(pt, next.Bottom, next.Top) > PolygonUtilities.JoinDistanceSquared)
            {
                return;
            }
        }
        else if (!PolygonUtilities.IsAlmostZero(e.CurrentX - next.CurrentX))
        {
            return;
        }

        if (!PolygonUtilities.IsCollinear(e.Top, pt, next.Top))
        {
            return;
        }

        if (e.OutputRecord!.Index == next.OutputRecord!.Index)
        {
            this.AddLocalMaxPoly(e, next, pt);
        }
        else if (e.OutputRecord!.Index < next.OutputRecord!.Index)
        {
            JoinOutputRecordPaths(e, next);
        }
        else
        {
            JoinOutputRecordPaths(next, e);
        }

        e.JoinWith = JoinWith.Right;
        next.JoinWith = JoinWith.Left;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void FixOutputRecordPoints(OutputRecord outputRecord)
    {
        OutputPoint op = outputRecord.Points!;
        do
        {
            op.OutputRecord = outputRecord;
            op = op.Next!;
        }
        while (op != outputRecord.Points);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool SetHorizontalSegmentHeadingForward(HorizontalSegment hs, OutputPoint prevPoint, OutputPoint nextPoint)
    {
        if (PolygonUtilities.IsAlmostZero(prevPoint.Point.X - nextPoint.Point.X))
        {
            return false;
        }

        if (prevPoint.Point.X < nextPoint.Point.X)
        {
            hs.LeftPoint = prevPoint;
            hs.RightPoint = nextPoint;
            hs.LeftToRight = true;
        }
        else
        {
            hs.LeftPoint = nextPoint;
            hs.RightPoint = prevPoint;
            hs.LeftToRight = false;
        }

        return true;
    }

    private static bool UpdateHorizontalSegment(HorizontalSegment hs)
    {
        OutputPoint op = hs.LeftPoint!;
        OutputRecord outputRecord = GetRealOutputRecord(op.OutputRecord)!;
        bool outputRecordHasEdges = outputRecord.FrontEdge != null;
        double currentY = op.Point.Y;
        OutputPoint prevPoint = op, nextPoint = op;
        if (outputRecordHasEdges)
        {
            OutputPoint opA = outputRecord.Points!, opZ = opA.Next!;
            while (prevPoint != opZ && prevPoint.Prev.Point.Y == currentY)
            {
                prevPoint = prevPoint.Prev;
            }

            while (nextPoint != opA && nextPoint.Next!.Point.Y == currentY)
            {
                nextPoint = nextPoint.Next;
            }
        }
        else
        {
            while (prevPoint.Prev != nextPoint && prevPoint.Prev.Point.Y == currentY)
            {
                prevPoint = prevPoint.Prev;
            }

            while (nextPoint.Next != prevPoint && nextPoint.Next!.Point.Y == currentY)
            {
                nextPoint = nextPoint.Next;
            }
        }

        bool result =
            SetHorizontalSegmentHeadingForward(hs, prevPoint, nextPoint) &&
            hs.LeftPoint!.HorizontalSegment == null;

        if (result)
        {
            hs.LeftPoint!.HorizontalSegment = hs;
        }
        else
        {
            // (for sorting)
            hs.RightPoint = null;
        }

        return result;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private OutputPoint DuplicateOp(OutputPoint op, bool insertAfter)
    {
        OutputPoint result = this.outputPointPool.Add(op.Point, op.OutputRecord);
        if (insertAfter)
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

    private static int HorizontalSegmentSort(HorizontalSegment? hs1, HorizontalSegment? hs2)
    {
        if (hs1 == null || hs2 == null)
        {
            return 0;
        }

        if (hs1.RightPoint == null)
        {
            return hs2.RightPoint == null ? 0 : 1;
        }

        if (hs2.RightPoint == null)
        {
            return -1;
        }

        return hs1.LeftPoint!.Point.X.CompareTo(hs2.LeftPoint!.Point.X);
    }

    private void ConvertHorizontalSegmentsToJoins()
    {
        int k = 0;
        foreach (HorizontalSegment hs in this.horizontalSegments)
        {
            if (UpdateHorizontalSegment(hs))
            {
                k++;
            }
        }

        if (k < 2)
        {
            return;
        }

        this.horizontalSegments.Sort(HorizontalSegmentSort);

        for (int i = 0; i < k - 1; i++)
        {
            HorizontalSegment hs1 = this.horizontalSegments[i];

            // for each HorizontalSegment, find others that overlap
            for (int j = i + 1; j < k; j++)
            {
                HorizontalSegment hs2 = this.horizontalSegments[j];
                if ((hs2.LeftPoint!.Point.X >= hs1.RightPoint!.Point.X) ||
                    (hs2.LeftToRight == hs1.LeftToRight) ||
                    (hs2.RightPoint!.Point.X <= hs1.LeftPoint!.Point.X))
                {
                    continue;
                }

                double curr_y = hs1.LeftPoint.Point.Y;
                if (hs1.LeftToRight)
                {
                    while (hs1.LeftPoint.Next!.Point.Y == curr_y &&
                        hs1.LeftPoint.Next.Point.X <= hs2.LeftPoint.Point.X)
                    {
                        hs1.LeftPoint = hs1.LeftPoint.Next;
                    }

                    while (hs2.LeftPoint.Prev.Point.Y == curr_y &&
                        hs2.LeftPoint.Prev.Point.X <= hs1.LeftPoint.Point.X)
                    {
                        hs2.LeftPoint = hs2.LeftPoint.Prev;
                    }

                    HorizontalJoin join = this.horizontalJoins.Add(
                        this.DuplicateOp(hs1.LeftPoint, true),
                        this.DuplicateOp(hs2.LeftPoint, false));
                }
                else
                {
                    while (hs1.LeftPoint.Prev.Point.Y == curr_y &&
                        hs1.LeftPoint.Prev.Point.X <= hs2.LeftPoint.Point.X)
                    {
                        hs1.LeftPoint = hs1.LeftPoint.Prev;
                    }

                    while (hs2.LeftPoint.Next!.Point.Y == curr_y &&
                        hs2.LeftPoint.Next.Point.X <= hs1.LeftPoint.Point.X)
                    {
                        hs2.LeftPoint = hs2.LeftPoint.Next;
                    }

                    HorizontalJoin join = this.horizontalJoins.Add(
                        this.DuplicateOp(hs2.LeftPoint, true),
                        this.DuplicateOp(hs1.LeftPoint, false));
                }
            }
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Contour GetCleanPath(OutputPoint op)
    {
        Contour result = [];
        OutputPoint op2 = op;
        while (op2.Next != op &&
            ((PolygonUtilities.IsAlmostZero(op2.Point.X - op2.Next!.Point.X) &&
              PolygonUtilities.IsAlmostZero(op2.Point.X - op2.Prev.Point.X)) ||
             (PolygonUtilities.IsAlmostZero(op2.Point.Y - op2.Next.Point.Y) &&
              PolygonUtilities.IsAlmostZero(op2.Point.Y - op2.Prev.Point.Y))))
        {
            op2 = op2.Next;
        }

        result.Add(op2.Point);
        OutputPoint prevOp = op2;
        op2 = op2.Next;
        while (op2 != op)
        {
            if ((!PolygonUtilities.IsAlmostZero(op2.Point.X - op2.Next!.Point.X) ||
                 !PolygonUtilities.IsAlmostZero(op2.Point.X - prevOp.Point.X)) &&
                (!PolygonUtilities.IsAlmostZero(op2.Point.Y - op2.Next.Point.Y) ||
                 !PolygonUtilities.IsAlmostZero(op2.Point.Y - prevOp.Point.Y)))
            {
                result.Add(op2.Point);
                prevOp = op2;
            }

            op2 = op2.Next;
        }

        return result;
    }

    private static PointInPolygonResult PointInOpPolygon(Vertex pt, OutputPoint op)
    {
        if (op == op.Next || op.Prev == op.Next)
        {
            return PointInPolygonResult.IsOutside;
        }

        OutputPoint op2 = op;
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
            return PointInPolygonResult.IsOutside;
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
                    return PointInPolygonResult.IsOn;
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
                    int d = PolygonUtilities.CrossSign(op2.Prev.Point, op2.Point, pt);
                    if (d == 0)
                    {
                        return PointInPolygonResult.IsOn;
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
            return val == 0 ? PointInPolygonResult.IsOutside : PointInPolygonResult.IsInside;
        }

        {
            int d = PolygonUtilities.CrossSign(op2.Prev.Point, op2.Point, pt);
            if (d == 0)
            {
                return PointInPolygonResult.IsOn;
            }

            if ((d < 0) == isAbove)
            {
                val = 1 - val;
            }
        }

        return val == 0 ? PointInPolygonResult.IsOutside : PointInPolygonResult.IsInside;
    }

    private static bool Path1InsidePath2(OutputPoint op1, OutputPoint op2)
    {
        // we need to make some accommodation for rounding errors
        // so we won't jump if the first ClipVertex is found outside
        PointInPolygonResult pip = PointInPolygonResult.IsOn;
        OutputPoint op = op1;
        do
        {
            switch (PointInOpPolygon(op.Point, op2))
            {
                case PointInPolygonResult.IsOutside:
                    if (pip == PointInPolygonResult.IsOutside)
                    {
                        return false;
                    }

                    pip = PointInPolygonResult.IsOutside;
                    break;
                case PointInPolygonResult.IsInside:
                    if (pip == PointInPolygonResult.IsInside)
                    {
                        return true;
                    }

                    pip = PointInPolygonResult.IsInside;
                    break;
                default:
                    break;
            }

            op = op.Next!;
        }
        while (op != op1);

        // result is unclear, so try again using cleaned paths
        // (#973)
        return PolygonUtilities.Path2ContainsPath1(GetCleanPath(op1), GetCleanPath(op2));
    }

    private static void MoveSplits(OutputRecord fromOr, OutputRecord toOr)
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

    private void ProcessHorizontalJoins()
    {
        foreach (HorizontalJoin j in this.horizontalJoins)
        {
            OutputRecord or1 = GetRealOutputRecord(j.LeftToRight!.OutputRecord)!;
            OutputRecord or2 = GetRealOutputRecord(j.RightToLeft!.OutputRecord)!;

            OutputPoint op1b = j.LeftToRight.Next!;
            OutputPoint op2b = j.RightToLeft.Prev;
            j.LeftToRight.Next = j.RightToLeft;
            j.RightToLeft.Prev = j.LeftToRight;
            op1b.Prev = op2b;
            op2b.Next = op1b;

            // 'join' is really a split
            if (or1 == or2)
            {
                or2 = this.NewOutputRecord();
                or2.Points = op1b;
                FixOutputRecordPoints(or2);

                // if or1.Points has moved to or2 then update or1.Points.
                if (or1.Points!.OutputRecord == or2)
                {
                    or1.Points = j.LeftToRight;
                    or1.Points.OutputRecord = or1;
                }

                // #498, #520, #584, D#576, #618
                if (this.buildHierarchy)
                {
                    if (Path1InsidePath2(or1.Points, or2.Points))
                    {
                        // swap or1's and or2's points
                        (or2.Points, or1.Points) = (or1.Points, or2.Points);
                        FixOutputRecordPoints(or1);
                        FixOutputRecordPoints(or2);

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
                if (this.buildHierarchy)
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
    private static bool PointsReallyClose(in Vertex pt1, in Vertex pt2)
    {
        double tolerance = PolygonUtilities.ClosePointTolerance;
        return Math.Abs(pt1.X - pt2.X) < tolerance && Math.Abs(pt1.Y - pt2.Y) < tolerance;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsVerySmallTriangle(OutputPoint op) => op.Next!.Next == op.Prev &&
      (PointsReallyClose(op.Prev.Point, op.Next.Point) ||
        PointsReallyClose(op.Point, op.Next.Point) ||
        PointsReallyClose(op.Point, op.Prev.Point));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsValidClosedPath(OutputPoint? op) => op != null && op.Next != op &&
            (op.Next != op.Prev || !IsVerySmallTriangle(op));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static OutputPoint? DisposeOutputPoint(OutputPoint op)
    {
        OutputPoint? result = op.Next == op ? null : op.Next;
        op.Prev.Next = op.Next;
        op.Next!.Prev = op.Prev;

        // op == null;
        return result;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void CleanCollinear(OutputRecord? outputRecord)
    {
        outputRecord = GetRealOutputRecord(outputRecord);

        if (outputRecord == null)
        {
            return;
        }

        if (!IsValidClosedPath(outputRecord.Points))
        {
            outputRecord.Points = null;
            return;
        }

        OutputPoint startOp = outputRecord.Points!;
        OutputPoint? op2 = startOp;
        while (true)
        {
            // NB if preserveCollinear == true, then only remove 180 deg. spikes
            if (PolygonUtilities.IsCollinear(op2!.Prev.Point, op2.Point, op2.Next!.Point) &&
                (PolygonUtilities.PointEquals(op2.Point, op2.Prev.Point) || PolygonUtilities.PointEquals(op2.Point, op2.Next.Point) || !this.PreserveCollinear ||
                (PolygonUtilities.Dot(op2.Prev.Point, op2.Point, op2.Next.Point) < 0)))
            {
                if (op2 == outputRecord.Points)
                {
                    outputRecord.Points = op2.Prev;
                }

                op2 = DisposeOutputPoint(op2);
                if (!IsValidClosedPath(op2))
                {
                    outputRecord.Points = null;
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

        this.FixSelfIntersects(outputRecord);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void DoSplitOp(OutputRecord outputRecord, OutputPoint splitOp)
    {
        // splitOp.Prev <=> splitOp &&
        // splitOp.Next <=> splitOp.Next.Next are intersecting
        OutputPoint prevOp = splitOp.Prev;
        OutputPoint nextNextOp = splitOp.Next!.Next!;
        outputRecord.Points = prevOp;

        PolygonUtilities.TryGetLineIntersection(
            prevOp.Point, splitOp.Point, splitOp.Next.Point, nextNextOp.Point, out Vertex ip);

        double area1 = Area(prevOp);
        double absArea1 = Math.Abs(area1);

        if (absArea1 < PolygonUtilities.SmallAreaTolerance2)
        {
            outputRecord.Points = null;
            return;
        }

        double area2 = PolygonUtilities.SignedArea(ip, splitOp.Point, splitOp.Next.Point);
        double absArea2 = Math.Abs(area2);

        // de-link splitOp and splitOp.Next from the path
        // while inserting the intersection point
        if (PolygonUtilities.PointEquals(ip, prevOp.Point) || PolygonUtilities.PointEquals(ip, nextNextOp.Point))
        {
            nextNextOp.Prev = prevOp;
            prevOp.Next = nextNextOp;
        }
        else
        {
            OutputPoint newOp2 = this.outputPointPool.Add(ip, outputRecord);
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
        if (!(absArea2 > PolygonUtilities.SmallAreaTolerance) ||
                (!(absArea2 > absArea1) &&
                  ((area2 > 0) != (area1 > 0))))
        {
            return;
        }

        OutputRecord newOutputRecord = this.NewOutputRecord();
        newOutputRecord.Owner = outputRecord.Owner;
        splitOp.OutputRecord = newOutputRecord;
        splitOp.Next.OutputRecord = newOutputRecord;

        OutputPoint newOp = this.outputPointPool.Add(ip, newOutputRecord);
        newOp.Prev = splitOp.Next;
        newOp.Next = splitOp;
        newOutputRecord.Points = newOp;
        splitOp.Prev = newOp;
        splitOp.Next.Next = newOp;

        if (!this.buildHierarchy)
        {
            return;
        }

        if (Path1InsidePath2(prevOp, newOp))
        {
            newOutputRecord.Splits ??= [];
            newOutputRecord.Splits.Add(outputRecord.Index);
        }
        else
        {
            outputRecord.Splits ??= [];
            outputRecord.Splits.Add(newOutputRecord.Index);
        }

        // else { splitOp = null; splitOp.Next = null; }
    }

    private void FixSelfIntersects(OutputRecord outputRecord)
    {
        OutputPoint op2 = outputRecord.Points!;
        if (op2.Prev == op2.Next!.Next)
        {
            // Triangles can't self-intersect.
            return;
        }

        while (true)
        {
            if (PolygonUtilities.SegmentsIntersect(
                op2!.Prev.Point,
                op2.Point,
                op2.Next!.Point,
                op2.Next.Next!.Point))
            {
                if (PolygonUtilities.SegmentsIntersect(
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
                    if (op2 == outputRecord.Points || op2.Next == outputRecord.Points)
                    {
                        outputRecord.Points = outputRecord.Points.Prev;
                    }

                    this.DoSplitOp(outputRecord, op2);
                    if (outputRecord.Points == null)
                    {
                        return;
                    }

                    op2 = outputRecord.Points;

                    // triangles can't self-intersect
                    if (op2.Prev == op2.Next!.Next)
                    {
                        break;
                    }

                    continue;
                }
            }

            op2 = op2.Next!;
            if (op2 == outputRecord.Points)
            {
                break;
            }
        }
    }

    private static bool BuildPath(OutputPoint? op, bool reverse, Contour path)
    {
        if (op == null || op.Next == op || op.Next == op.Prev)
        {
            return false;
        }

        path.Clear();

        Vertex lastPt;
        OutputPoint op2;
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
            if (!PolygonUtilities.PointEquals(op2.Point, lastPt))
            {
                lastPt = op2.Point;
                path.Add(lastPt);
            }

            op2 = reverse ? op2.Prev : op2.Next!;
        }

        return path.Count != 3 || !IsVerySmallTriangle(op2);
    }

    private static bool BuildContour(OutputPoint? op, bool reverse, Contour contour)
    {
        if (op == null || op.Next == op || op.Next == op.Prev)
        {
            return false;
        }

        contour.Clear();

        Vertex lastPt;
        OutputPoint op2;
        if (reverse)
        {
            lastPt = new Vertex(op.Point.X, op.Point.Y);
            op2 = op.Prev;
        }
        else
        {
            op = op.Next!;
            lastPt = new Vertex(op.Point.X, op.Point.Y);
            op2 = op.Next!;
        }

        contour.Add(lastPt);

        while (op2 != op)
        {
            Vertex current = new Vertex(op2.Point.X, op2.Point.Y);
            if (current != lastPt)
            {
                lastPt = current;
                contour.Add(lastPt);
            }

            op2 = reverse ? op2.Prev : op2.Next!;
        }

        if (contour.Count == 3 && IsVerySmallTriangle(op2))
        {
            contour.Clear();
            return false;
        }

        if (contour.Count > 0)
        {
            contour.Add(contour[0]);
        }

        return true;
    }

    private void BuildContours(List<Contour> solution)
    {
        solution.Clear();
        solution.EnsureCapacity(this.outputRecordPool.Count);

        int i = 0;
        while (i < this.outputRecordPool.Count)
        {
            OutputRecord outputRecord = this.outputRecordPool[i++];
            if (outputRecord.Points == null)
            {
                continue;
            }

            Contour contour = new(outputRecord.OutputPointCount + 1);
            this.CleanCollinear(outputRecord);
            if (BuildContour(outputRecord.Points, this.ReverseSolution, contour))
            {
                solution.Add(contour);
            }
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private bool CheckBounds(OutputRecord outputRecord)
    {
        if (outputRecord.Points == null)
        {
            return false;
        }

        if (!outputRecord.Bounds.IsEmpty())
        {
            return true;
        }

        this.CleanCollinear(outputRecord);
        if (outputRecord.Points == null ||
            !BuildPath(outputRecord.Points, this.ReverseSolution, outputRecord.Path))
        {
            return false;
        }

        outputRecord.Bounds = PolygonUtilities.GetBounds(outputRecord.Path);
        return true;
    }

    private bool CheckSplitOwner(OutputRecord outputRecord, List<int>? splits)
    {
        // nb: use indexing (not an iterator) in case 'splits' is modified inside this loop (#1029)
        for (int i = 0; i < splits!.Count; i++)
        {
            OutputRecord? split = this.outputRecordPool[splits[i]];
            if (split.Points == null && split.Splits != null &&
                this.CheckSplitOwner(outputRecord, split.Splits))
            {
                // #942
                return true;
            }

            split = GetRealOutputRecord(split);
            if (split == null || split == outputRecord || split.RecursiveSplit == outputRecord)
            {
                continue;
            }

            // #599
            split.RecursiveSplit = outputRecord;

            if (split.Splits != null && this.CheckSplitOwner(outputRecord, split.Splits))
            {
                return true;
            }

            if (!this.CheckBounds(split) ||
                    !split.Bounds.Contains(outputRecord.Bounds) ||
                    !Path1InsidePath2(outputRecord.Points!, split.Points!))
            {
                continue;
            }

            // split is owned by outputRecord (#957)
            if (!IsValidOwner(outputRecord, split))
            {
                split.Owner = outputRecord.Owner;
            }

            // Found in split.
            outputRecord.Owner = split;
            return true;
        }

        return false;
    }

    private void ResolveOwner(OutputRecord outputRecord)
    {
        if (outputRecord.Bounds.IsEmpty())
        {
            return;
        }

        while (outputRecord.Owner != null)
        {
            if (outputRecord.Owner.Splits != null &&
                this.CheckSplitOwner(outputRecord, outputRecord.Owner.Splits))
            {
                break;
            }

            if (outputRecord.Owner.Points != null && this.CheckBounds(outputRecord.Owner) &&
                Path1InsidePath2(outputRecord.Points!, outputRecord.Owner.Points!))
            {
                break;
            }

            outputRecord.Owner = outputRecord.Owner.Owner;
        }
    }

    private void BuildPolygon(Polygon polygon)
    {
        polygon.Clear();
        List<OutputRecord> closedOutputRecords = new(this.outputRecordPool.Count);

        int i = 0;

        // this.outputRecordPool.Count is not static here because
        // CheckBounds below can indirectly add additional
        // OutputRecord (via FixOutputRecordPoints & CleanCollinear)
        while (i < this.outputRecordPool.Count)
        {
            OutputRecord outputRecord = this.outputRecordPool[i++];
            if (outputRecord.Points == null)
            {
                continue;
            }

            if (this.CheckBounds(outputRecord))
            {
                this.ResolveOwner(outputRecord);
                closedOutputRecords.Add(outputRecord);
            }
        }

        if (closedOutputRecords.Count == 0)
        {
            return;
        }

        Dictionary<OutputRecord, int> outputRecordIndices = new(closedOutputRecords.Count);
        for (int index = 0; index < closedOutputRecords.Count; index++)
        {
            OutputRecord outputRecord = closedOutputRecords[index];
            Contour contour = new(outputRecord.OutputPointCount + 1);
            if (!BuildContour(outputRecord.Points, this.ReverseSolution, contour))
            {
                continue;
            }

            int contourIndex = polygon.Count;
            polygon.Add(contour);
            outputRecordIndices[outputRecord] = contourIndex;
        }

        if (polygon.Count == 0)
        {
            return;
        }

        for (int index = 0; index < polygon.Count; index++)
        {
            Contour contour = polygon[index];
            contour.ParentIndex = null;
            contour.Depth = 0;
            contour.ClearHoles();
        }

        for (int index = 0; index < closedOutputRecords.Count; index++)
        {
            OutputRecord outputRecord = closedOutputRecords[index];
            if (!outputRecordIndices.TryGetValue(outputRecord, out int contourIndex))
            {
                continue;
            }

            OutputRecord? owner = outputRecord.Owner;
            if (owner != null && outputRecordIndices.TryGetValue(owner, out int parentIndex))
            {
                polygon[contourIndex].ParentIndex = parentIndex;
            }
        }

        for (int index = 0; index < closedOutputRecords.Count; index++)
        {
            OutputRecord outputRecord = closedOutputRecords[index];
            if (!outputRecordIndices.TryGetValue(outputRecord, out int contourIndex))
            {
                continue;
            }

            int depth = 0;
            OutputRecord? owner = outputRecord.Owner;
            while (owner != null && outputRecordIndices.ContainsKey(owner))
            {
                depth++;
                owner = owner.Owner;
            }

            polygon[contourIndex].Depth = depth;
        }

        for (int index = 0; index < polygon.Count; index++)
        {
            Contour contour = polygon[index];
            if (contour.ParentIndex != null)
            {
                polygon[contour.ParentIndex.Value].AddHoleIndex(index);
            }
        }
    }

    internal bool Execute(
        FillRule fillRule,
        Polygon polygon)
    {
        polygon.Clear();
        this.buildHierarchy = true;
        try
        {
            this.ExecuteInternal(fillRule);
            this.BuildPolygon(polygon);
        }
        catch
        {
            this.succeeded = false;
        }

        this.ClearSolutionOnly();
        return this.succeeded;
    }

    internal bool Execute(
        FillRule fillRule,
        List<Contour> solution)
    {
        solution.Clear();
        this.buildHierarchy = false;
        try
        {
            this.ExecuteInternal(fillRule);
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
            Vertex delta = a.Point - b.Point;
            if (!PolygonUtilities.IsAlmostZero(delta.Y))
            {
                return delta.Y > 0 ? -1 : 1;
            }

            if (PolygonUtilities.IsAlmostZero(delta.X))
            {
                return 0;
            }

            return delta.X < 0 ? -1 : 1;
        }
    }
}

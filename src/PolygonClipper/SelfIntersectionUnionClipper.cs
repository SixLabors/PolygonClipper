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
    private ActiveEdge? activeEdgeHead;
    private ActiveEdge? sortedEdgeHead;
    private readonly Stack<ActiveEdge> freeActiveEdges;
    private readonly List<LocalMinima> localMinimaList;
    private readonly List<IntersectNode> intersectionList;
    private readonly VertexPoolList vertexList;
    private readonly OutputRecordPoolList outputRecordPool;
    private readonly List<double> scanlines;
    private readonly List<HorizontalSegment> horizontalSegments;
    private readonly HorizontalJoinPoolList horizontalJoins;
    private readonly OutputPointPoolList outputPointPool;
    private int localMinimaIndex;
    private double currentScanlineBottomY;
    private bool isLocalMinimaSorted;
    private bool buildHierarchy;
    private bool succeeded;

    /// <summary>
    /// Initializes a new instance of the <see cref="SelfIntersectionUnionClipper"/> class.
    /// </summary>
    internal SelfIntersectionUnionClipper()
    {
        this.localMinimaList = [];
        this.intersectionList = [];
        this.vertexList = [];
        this.outputRecordPool = new OutputRecordPoolList();
        this.scanlines = [];
        this.horizontalSegments = [];
        this.horizontalJoins = [];
        this.outputPointPool = [];
        this.freeActiveEdges = new Stack<ActiveEdge>();
        this.PreserveCollinear = true;
    }

    /// <summary>
    /// Gets or sets a value indicating whether collinear output points are preserved.
    /// </summary>
    internal bool PreserveCollinear { get; set; }

    /// <summary>
    /// Gets or sets a value indicating whether the output orientation is reversed.
    /// </summary>
    internal bool ReverseSolution { get; set; }

    /// <summary>
    /// Swaps two active edge references.
    /// </summary>
    /// <param name="edge1">The first active edge.</param>
    /// <param name="edge2">The second active edge.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void SwapActiveEdges(ref ActiveEdge edge1, ref ActiveEdge edge2) => (edge2, edge1) = (edge1, edge2);

    /// <summary>
    /// Locates the active edge that shares the same maxima vertex.
    /// </summary>
    /// <param name="edge">The active edge being matched.</param>
    /// <returns>The paired maxima edge, or <see langword="null"/> if none exists in the active list.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static ActiveEdge? FindMaximaPair(ActiveEdge edge)
    {
        ActiveEdge? edge2 = edge.NextInAel;
        while (edge2 != null)
        {
            if (edge2.VertexTop == edge.VertexTop)
            {
                // Found a matching maxima pair.
                return edge2;
            }

            edge2 = edge2.NextInAel;
        }

        return null;
    }

    /// <summary>
    /// Returns the maxima vertex on the current Y scanline for the edge.
    /// </summary>
    /// <param name="edge">The active edge to inspect.</param>
    /// <returns>The maxima vertex at the current Y, or <see langword="null"/> if not a maxima.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static SweepVertex? GetMaximaVertexAtCurrentY(ActiveEdge edge)
    {
        SweepVertex? result = edge.VertexTop;
        if (edge.WindDelta > 0)
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

    /// <summary>
    /// Assigns the output record edges that define the front and back sides.
    /// </summary>
    /// <param name="outputRecord">The output record to update.</param>
    /// <param name="startEdge">The edge used for the front side.</param>
    /// <param name="endEdge">The edge used for the back side.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void SetOutputSides(OutputRecord outputRecord, ActiveEdge startEdge, ActiveEdge endEdge)
    {
        outputRecord.FrontEdge = startEdge;
        outputRecord.BackEdge = endEdge;
    }

    /// <summary>
    /// Swaps output record ownership between two active edges.
    /// </summary>
    /// <param name="edge1">The first active edge.</param>
    /// <param name="edge2">The second active edge.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void SwapOutputRecords(ActiveEdge edge1, ActiveEdge edge2)
    {
        // At least one edge has an assigned outputRecord.
        OutputRecord? outputRecord1 = edge1.OutputRecord;
        OutputRecord? outputRecord2 = edge2.OutputRecord;
        if (outputRecord1 == outputRecord2)
        {
            ActiveEdge? edge = outputRecord1!.FrontEdge;
            outputRecord1.FrontEdge = outputRecord1.BackEdge;
            outputRecord1.BackEdge = edge;
            return;
        }

        if (outputRecord1 != null)
        {
            if (edge1 == outputRecord1.FrontEdge)
            {
                outputRecord1.FrontEdge = edge2;
            }
            else
            {
                outputRecord1.BackEdge = edge2;
            }
        }

        if (outputRecord2 != null)
        {
            if (edge2 == outputRecord2.FrontEdge)
            {
                outputRecord2.FrontEdge = edge1;
            }
            else
            {
                outputRecord2.BackEdge = edge1;
            }
        }

        edge1.OutputRecord = outputRecord2;
        edge2.OutputRecord = outputRecord1;
    }

    /// <summary>
    /// Assigns an output record's owner while preventing cyclic ownership.
    /// </summary>
    /// <param name="outputRecord">The output record to update.</param>
    /// <param name="newOwner">The candidate owner.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void SetOutputOwner(OutputRecord outputRecord, OutputRecord newOwner)
    {
        // Precondition: newOwner is never null.
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

    /// <summary>
    /// Computes the signed area of a closed output ring.
    /// </summary>
    /// <param name="outputPoint">A point on the output ring.</param>
    /// <returns>The signed area of the ring.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static double ComputeSignedArea(OutputPoint outputPoint)
    {
        // https://en.wikipedia.org/wiki/Shoelace_formula
        double signedArea = 0.0;
        OutputPoint outputPoint2 = outputPoint;
        do
        {
            signedArea += Vertex.Cross(outputPoint2.Prev.Point, outputPoint2.Point);
            outputPoint2 = outputPoint2.Next!;
        }
        while (outputPoint2 != outputPoint);
        return signedArea * 0.5;
    }

    /// <summary>
    /// Resolves a non-null output record that still owns geometry.
    /// </summary>
    /// <param name="outputRecord">The candidate output record.</param>
    /// <returns>The resolved output record, or <see langword="null"/> if none remains.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static OutputRecord? ResolveOutputRecord(OutputRecord? outputRecord)
    {
        while (outputRecord != null && outputRecord.Points == null)
        {
            outputRecord = outputRecord.Owner;
        }

        return outputRecord;
    }

    /// <summary>
    /// Validates that an output record is not owned by a descendant.
    /// </summary>
    /// <param name="outputRecord">The output record to validate.</param>
    /// <param name="testOwner">The owner candidate.</param>
    /// <returns><see langword="true"/> when the ownership chain is valid.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsOwnerValid(OutputRecord? outputRecord, OutputRecord? testOwner)
    {
        while (testOwner != null && testOwner != outputRecord)
        {
            testOwner = testOwner.Owner;
        }

        return testOwner == null;
    }

    /// <summary>
    /// Clears output record links from a hot edge.
    /// </summary>
    /// <param name="edge">The active edge to detach.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void DetachOutputRecord(ActiveEdge edge)
    {
        OutputRecord? outputRecord = edge.OutputRecord;
        if (outputRecord == null)
        {
            return;
        }

        outputRecord.FrontEdge!.OutputRecord = null;
        outputRecord.BackEdge!.OutputRecord = null;
        outputRecord.FrontEdge = null;
        outputRecord.BackEdge = null;
    }

    /// <summary>
    /// Determines whether an edge is the front edge of its output record.
    /// </summary>
    /// <param name="hotEdge">The active edge to query.</param>
    /// <returns><see langword="true"/> when the edge is the front edge.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsOutputRecordAscending(ActiveEdge hotEdge) => hotEdge == hotEdge.OutputRecord!.FrontEdge;

    /// <summary>
    /// Checks whether the two edges in an intersection node are adjacent in the active list.
    /// </summary>
    /// <param name="intersectionNode">The intersection node to inspect.</param>
    /// <returns><see langword="true"/> if the edges are adjacent in the active list.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool AreEdgesAdjacentInActiveList(in IntersectNode intersectionNode)
        => (intersectionNode.Edge1.NextInAel == intersectionNode.Edge2) || (intersectionNode.Edge1.PrevInAel == intersectionNode.Edge2);

    /// <summary>
    /// Clears solution-only data while preserving the input vertices and minima list.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void ClearSolutionData()
    {
        while (this.activeEdgeHead != null)
        {
            this.DeleteFromActiveList(this.activeEdgeHead);
        }

        this.scanlines.Clear();
        this.ClearIntersectionNodes();
        this.outputRecordPool.Clear();
        this.horizontalSegments.Clear();
        this.horizontalJoins.Clear();
        this.outputPointPool.Clear();
    }

    /// <summary>
    /// Clears all clipper state, including cached input data.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal void Clear()
    {
        this.ClearSolutionData();
        this.localMinimaList.Clear();
        this.vertexList.Clear();
        this.localMinimaIndex = 0;
        this.isLocalMinimaSorted = false;
    }

    /// <summary>
    /// Resets scanline state and sorts local minima before an execution pass.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void ResetState()
    {
        if (!this.isLocalMinimaSorted)
        {
            this.localMinimaList.Sort(new LocalMinimaComparer());
            this.isLocalMinimaSorted = true;
        }

        this.scanlines.EnsureCapacity(this.localMinimaList.Count);
        for (int i = this.localMinimaList.Count - 1; i >= 0; i--)
        {
            this.scanlines.Add(this.localMinimaList[i].Vertex.Point.Y);
        }

        this.currentScanlineBottomY = 0;
        this.localMinimaIndex = 0;
        this.activeEdgeHead = null;
        this.sortedEdgeHead = null;
        this.succeeded = true;
    }

    /// <summary>
    /// Inserts a scanline Y coordinate into the ordered scanline list.
    /// </summary>
    /// <param name="y">The Y coordinate to insert.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void InsertScanline(double y)
    {
        int index = this.scanlines.BinarySearch(y);
        if (index >= 0)
        {
            return;
        }

        index = ~index;
        this.scanlines.Insert(index, y);
    }

    /// <summary>
    /// Pops the next scanline from the list.
    /// </summary>
    /// <param name="y">The popped scanline value.</param>
    /// <returns><see langword="true"/> when a scanline was available.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private bool TryPopScanline(out double y)
    {
        int cnt = this.scanlines.Count - 1;
        if (cnt < 0)
        {
            y = 0;
            return false;
        }

        y = this.scanlines[cnt];
        this.scanlines.RemoveAt(cnt--);
        while (cnt >= 0 && y == this.scanlines[cnt])
        {
            this.scanlines.RemoveAt(cnt--);
        }

        return true;
    }

    /// <summary>
    /// Tests whether the next local minima is at the given Y coordinate.
    /// </summary>
    /// <param name="y">The scanline Y coordinate.</param>
    /// <returns><see langword="true"/> when a minima exists at this Y.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private bool HasLocalMinimaAtY(double y)
        => this.localMinimaIndex < this.localMinimaList.Count &&
           this.localMinimaList[this.localMinimaIndex].Vertex.Point.Y == y;

    /// <summary>
    /// Returns the next local minima in sorted order.
    /// </summary>
    /// <returns>The next local minima.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private LocalMinima PopLocalMinima() => this.localMinimaList[this.localMinimaIndex++];

    /// <summary>
    /// Adds subject contours for the union operation.
    /// </summary>
    /// <param name="paths">The subject contours to add.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal void AddSubject(List<Contour> paths)
    {
        this.isLocalMinimaSorted = false;
        this.AddPathsToVertexList(paths);
    }

    /// <summary>
    /// Registers a local minima vertex once for the sweep.
    /// </summary>
    /// <param name="vertex">The vertex that marks a local minima.</param>
    /// <param name="localMinimaList">The list collecting minima.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void RegisterLocalMinima(SweepVertex vertex, List<LocalMinima> localMinimaList)
    {
        // Make sure the sweep vertex is added only once.
        if ((vertex.Flags & VertexFlags.LocalMin) != VertexFlags.None)
        {
            return;
        }

        vertex.Flags |= VertexFlags.LocalMin;
        localMinimaList.Add(new LocalMinima(vertex));
    }

    /// <summary>
    /// Builds circular vertex lists and captures local minima/maxima for the sweep.
    /// </summary>
    /// <param name="paths">The subject contours to process.</param>
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
            SweepVertex? v0 = null;
            SweepVertex? prevVertex = null;
            SweepVertex? currVertex;
            foreach (Vertex point in path)
            {
                if (v0 == null)
                {
                    v0 = this.vertexList.Add(point, VertexFlags.None, null);
                    prevVertex = v0;
                    continue;
                }

                if (!PolygonUtilities.PointEquals(prevVertex!.Point, point))
                {
                    currVertex = this.vertexList.Add(point, VertexFlags.None, prevVertex);
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
                    RegisterLocalMinima(prevVertex, this.localMinimaList);
                }

                prevVertex = currVertex;
                currVertex = currVertex.Next;
            }

            if (goingUp != goingUp0)
            {
                if (goingUp0)
                {
                    RegisterLocalMinima(prevVertex, this.localMinimaList);
                }
                else
                {
                    prevVertex.Flags |= VertexFlags.LocalMax;
                }
            }
        }
    }

    /// <summary>
    /// Determines whether a closed edge contributes to the union result.
    /// </summary>
    /// <param name="edge">The edge to test.</param>
    /// <returns><see langword="true"/> if the edge contributes.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private bool IsContributingClosedEdge(ActiveEdge edge)
    {
        if (this.fillRule == FillRule.Positive)
        {
            return edge.WindCount == 1;
        }

        return edge.WindCount == -1;
    }

    /// <summary>
    /// Updates the winding count for a closed path edge.
    /// </summary>
    /// <param name="edge">The active edge to update.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void SetWindingCountForClosedEdge(ActiveEdge edge)
    {
        // Wind counts refer to polygon regions not edges, so here an edge's WindCnt
        // indicates the higher of the wind counts for the two regions touching the
        // edge. (nb: Adjacent regions can only ever have their wind counts differ by one.)
        ActiveEdge? edge2 = edge.PrevInAel;

        if (edge2 == null)
        {
            edge.WindCount = edge.WindDelta;
        }
        else
        {
            // Positive or negative filling here ...
            // when edge2's wind count is in the SAME direction as its wind delta,
            // then polygon will fill on the right of 'edge2' (and 'edge' will be inside).
            // nb: neither edge2.WindCount nor edge2.WindDelta should ever be 0.
            if (edge2.WindCount * edge2.WindDelta < 0)
            {
                // opposite directions so 'edge' is outside 'edge2' ...
                if (Math.Abs(edge2.WindCount) > 1)
                {
                    // outside prev poly but still inside another.
                    if (edge2.WindDelta * edge.WindDelta < 0)
                    {
                        // reversing direction so use the same WC
                        edge.WindCount = edge2.WindCount;
                    }
                    else
                    {
                        // otherwise keep 'reducing' the WC by 1 (i.e. towards 0) ...
                        edge.WindCount = edge2.WindCount + edge.WindDelta;
                    }
                }
                else
                {
                    // now outside all polygons so set own WC ...
                    edge.WindCount = edge.WindDelta;
                }
            }
            else
            {
                // 'edge' must be inside 'edge2'
                if (edge2.WindDelta * edge.WindDelta < 0)
                {
                    // reversing direction so use the same WC
                    edge.WindCount = edge2.WindCount;
                }
                else
                {
                    // otherwise keep 'increasing' the WC by 1 (i.e. away from 0) ...
                    edge.WindCount = edge2.WindCount + edge.WindDelta;
                }
            }
        }
    }

    /// <summary>
    /// Determines whether the newcomer should be inserted after the resident in the active list.
    /// </summary>
    /// <param name="resident">The current resident edge.</param>
    /// <param name="newcomer">The incoming edge to compare.</param>
    /// <returns><see langword="true"/> if the newcomer belongs after the resident.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsValidActiveEdgeOrder(ActiveEdge resident, ActiveEdge newcomer)
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

    /// <summary>
    /// Inserts an edge to the left of its current neighbors in the active list.
    /// </summary>
    /// <param name="edge">The edge to insert.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void InsertLeftEdge(ActiveEdge edge)
    {
        if (this.activeEdgeHead == null)
        {
            edge.PrevInAel = null;
            edge.NextInAel = null;
            this.activeEdgeHead = edge;
        }
        else if (!IsValidActiveEdgeOrder(this.activeEdgeHead, edge))
        {
            edge.PrevInAel = null;
            edge.NextInAel = this.activeEdgeHead;
            this.activeEdgeHead.PrevInAel = edge;
            this.activeEdgeHead = edge;
        }
        else
        {
            ActiveEdge edge2 = this.activeEdgeHead;
            while (edge2.NextInAel != null && IsValidActiveEdgeOrder(edge2.NextInAel, edge))
            {
                edge2 = edge2.NextInAel;
            }

            // don't separate joined edges
            if (edge2.JoinWith == JoinWith.Right)
            {
                edge2 = edge2.NextInAel!;
            }

            edge.NextInAel = edge2.NextInAel;
            if (edge2.NextInAel != null)
            {
                edge2.NextInAel.PrevInAel = edge;
            }

            edge.PrevInAel = edge2;
            edge2.NextInAel = edge;
        }
    }

    /// <summary>
    /// Inserts a right bound edge immediately after another edge in the active list.
    /// </summary>
    /// <param name="edge">The anchor edge.</param>
    /// <param name="edge2">The edge to insert.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void InsertRightEdge(ActiveEdge edge, ActiveEdge edge2)
    {
        edge2.NextInAel = edge.NextInAel;
        if (edge.NextInAel != null)
        {
            edge.NextInAel.PrevInAel = edge2;
        }

        edge2.PrevInAel = edge;
        edge.NextInAel = edge2;
    }

    /// <summary>
    /// Inserts any local minima that occur at the current scanline into the active list.
    /// </summary>
    /// <param name="botY">The current scanline Y coordinate.</param>
    private void InsertLocalMinimaIntoActiveList(double botY)
    {
        // Add any local minima (if any) at BotY ...
        // NB horizontal local minima edges should contain locMin.Vertex.Prev
        while (this.HasLocalMinimaAtY(botY))
        {
            LocalMinima localMinima = this.PopLocalMinima();
            ActiveEdge leftBound = this.AcquireActiveEdge();
            leftBound.Bottom = localMinima.Vertex.Point;
            leftBound.CurrentX = localMinima.Vertex.Point.X;
            leftBound.WindDelta = -1;
            leftBound.VertexTop = localMinima.Vertex.Prev;
            leftBound.Top = localMinima.Vertex.Prev!.Point;
            leftBound.OutputRecord = null;
            leftBound.LocalMin = localMinima;
            leftBound.UpdateDx();

            ActiveEdge rightBound = this.AcquireActiveEdge();
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
                    SwapActiveEdges(ref leftBound, ref rightBound);
                }
            }
            else if (rightBound.IsHorizontal)
            {
                if (rightBound.IsHeadingLeftHorizontal)
                {
                    SwapActiveEdges(ref leftBound, ref rightBound);
                }
            }
            else if (leftBound.Dx < rightBound.Dx)
            {
                SwapActiveEdges(ref leftBound, ref rightBound);
            }

            bool contributing;
            leftBound.IsLeftBound = true;
            this.InsertLeftEdge(leftBound);

            SetWindingCountForClosedEdge(leftBound);
            contributing = this.IsContributingClosedEdge(leftBound);

            rightBound.WindCount = leftBound.WindCount;
            InsertRightEdge(leftBound, rightBound);

            if (contributing)
            {
                this.AddLocalMinimumOutput(leftBound, rightBound, leftBound.Bottom, true);
                if (!leftBound.IsHorizontal)
                {
                    this.CheckJoinLeft(leftBound, leftBound.Bottom);
                }
            }

            while (rightBound.NextInAel != null &&
                          IsValidActiveEdgeOrder(rightBound.NextInAel, rightBound))
            {
                this.IntersectActiveEdges(rightBound, rightBound.NextInAel, rightBound.Bottom);
                this.SwapPositionsInActiveList(rightBound, rightBound.NextInAel);
            }

            if (rightBound.IsHorizontal)
            {
                this.PushHorizontalEdge(rightBound);
            }
            else
            {
                this.CheckJoinRight(rightBound, rightBound.Bottom);
                this.InsertScanline(rightBound.Top.Y);
            }

            if (leftBound.IsHorizontal)
            {
                this.PushHorizontalEdge(leftBound);
            }
            else
            {
                this.InsertScanline(leftBound.Top.Y);
            }
        }
    }

    /// <summary>
    /// Pushes a horizontal edge onto the horizontal processing stack.
    /// </summary>
    /// <param name="edge">The horizontal edge to push.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void PushHorizontalEdge(ActiveEdge edge)
    {
        edge.NextInSel = this.sortedEdgeHead;
        this.sortedEdgeHead = edge;
    }

    /// <summary>
    /// Pops the next horizontal edge to process.
    /// </summary>
    /// <param name="edge">The popped edge.</param>
    /// <returns><see langword="true"/> when an edge was available.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private bool TryPopHorizontalEdge(out ActiveEdge? edge)
    {
        edge = this.sortedEdgeHead;
        if (this.sortedEdgeHead == null)
        {
            return false;
        }

        this.sortedEdgeHead = this.sortedEdgeHead.NextInSel;
        return true;
    }

    /// <summary>
    /// Creates a new output record at a local minimum.
    /// </summary>
    /// <param name="edge1">The first bound edge.</param>
    /// <param name="edge2">The second bound edge.</param>
    /// <param name="point">The local minimum point.</param>
    /// <param name="isNew">Whether this output is created for a split.</param>
    /// <returns>The created output point.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private OutputPoint AddLocalMinimumOutput(ActiveEdge edge1, ActiveEdge edge2, Vertex point, bool isNew = false)
    {
        OutputRecord outputRecord = this.CreateOutputRecord();
        edge1.OutputRecord = outputRecord;
        edge2.OutputRecord = outputRecord;

        ActiveEdge? prevHotEdge = edge1.GetPrevHotEdge();

        // WindDelta is the winding direction of the **input** paths
        // and unrelated to the winding direction of output polygons.
        // Output orientation is determined by the edge's OutputRecord.FrontEdge which is
        // the ascending edge (see AddLocalMinimumOutput).
        if (prevHotEdge != null)
        {
            if (this.buildHierarchy)
            {
                SetOutputOwner(outputRecord, prevHotEdge.OutputRecord!);
            }

            outputRecord.Owner = prevHotEdge.OutputRecord;
            if (IsOutputRecordAscending(prevHotEdge) == isNew)
            {
                SetOutputSides(outputRecord, edge2, edge1);
            }
            else
            {
                SetOutputSides(outputRecord, edge1, edge2);
            }
        }
        else
        {
            outputRecord.Owner = null;
            if (isNew)
            {
                SetOutputSides(outputRecord, edge1, edge2);
            }
            else
            {
                SetOutputSides(outputRecord, edge2, edge1);
            }
        }

        OutputPoint outputPoint = this.outputPointPool.Add(point, outputRecord);
        outputRecord.Points = outputPoint;
        return outputPoint;
    }

    /// <summary>
    /// Joins two output records when a local maximum is encountered.
    /// </summary>
    /// <param name="edge1">The first active edge.</param>
    /// <param name="edge2">The second active edge.</param>
    /// <param name="point">The local maximum point.</param>
    /// <returns>The last output point, or <see langword="null"/> when no output remains.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private OutputPoint? AddLocalMaximumOutput(ActiveEdge edge1, ActiveEdge edge2, Vertex point)
    {
        if (IsJoined(edge1))
        {
            this.SplitEdge(edge1, point);
        }

        if (IsJoined(edge2))
        {
            this.SplitEdge(edge2, point);
        }

        if (edge1.IsFront == edge2.IsFront)
        {
            this.succeeded = false;
            return null;
        }

        OutputPoint result = this.AddOutputPoint(edge1, point);
        if (edge1.OutputRecord == edge2.OutputRecord)
        {
            OutputRecord outputRecord = edge1.OutputRecord!;
            outputRecord.Points = result;

            if (this.buildHierarchy)
            {
                ActiveEdge? e = edge1.GetPrevHotEdge();
                if (e == null)
                {
                    outputRecord.Owner = null;
                }
                else
                {
                    SetOutputOwner(outputRecord, e.OutputRecord!);
                }

                // nb: outputRecord.Owner here is likely NOT the real
                // owner but this will be fixed in DeepCheckOwner()
            }

            DetachOutputRecord(edge1);
        }

        // and to preserve the winding orientation of OutputRecord ...
        else if (edge1.OutputRecord!.Index < edge2.OutputRecord!.Index)
        {
            JoinOutputRecords(edge1, edge2);
        }
        else
        {
            JoinOutputRecords(edge2, edge1);
        }

        return result;
    }

    /// <summary>
    /// Merges the output paths from two active edges into a single record.
    /// </summary>
    /// <param name="edge1">The primary edge to keep.</param>
    /// <param name="edge2">The secondary edge to merge.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void JoinOutputRecords(ActiveEdge edge1, ActiveEdge edge2)
    {
        // join edge2 OutputRecord path onto edge1 OutputRecord path and then delete edge2 OutputRecord path
        // pointers. (NB Only very rarely do the joining ends share the same coords.)
        OutputPoint p1Start = edge1.OutputRecord!.Points!;
        OutputPoint p2Start = edge2.OutputRecord!.Points!;
        OutputPoint p1End = p1Start.Next!;
        OutputPoint p2End = p2Start.Next!;
        if (edge1.IsFront)
        {
            p2End.Prev = p1Start;
            p1Start.Next = p2End;
            p2Start.Next = p1End;
            p1End.Prev = p2Start;
            edge1.OutputRecord!.Points = p2Start;

            edge1.OutputRecord!.FrontEdge = edge2.OutputRecord!.FrontEdge;
            if (edge1.OutputRecord!.FrontEdge != null)
            {
                edge1.OutputRecord!.FrontEdge!.OutputRecord = edge1.OutputRecord;
            }
        }
        else
        {
            p1End.Prev = p2Start;
            p2Start.Next = p1End;
            p1Start.Next = p2End;
            p2End.Prev = p1Start;

            edge1.OutputRecord!.BackEdge = edge2.OutputRecord!.BackEdge;
            if (edge1.OutputRecord!.BackEdge != null)
            {
                edge1.OutputRecord!.BackEdge!.OutputRecord = edge1.OutputRecord;
            }
        }

        // after joining, the edge2.OutputRecord must contains no vertices ...
        edge2.OutputRecord!.FrontEdge = null;
        edge2.OutputRecord!.BackEdge = null;
        edge2.OutputRecord!.Points = null;
        edge1.OutputRecord!.OutputPointCount += edge2.OutputRecord!.OutputPointCount;
        SetOutputOwner(edge2.OutputRecord, edge1.OutputRecord);

        // and edge1 and edge2 are maxima and are about to be dropped from the Actives list.
        edge1.OutputRecord = null;
        edge2.OutputRecord = null;
    }

    /// <summary>
    /// Adds an output point to the front or back of the current output record.
    /// </summary>
    /// <param name="edge">The active edge that owns the output.</param>
    /// <param name="point">The point to add.</param>
    /// <returns>The output point that was added or reused.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private OutputPoint AddOutputPoint(ActiveEdge edge, Vertex point)
    {
        // outputRecord.Points: a circular doubly-linked list of OutputPoint where ...
        // opFront[.Prev]* ~~~> opBack & opBack == opFront.Next
        OutputRecord outputRecord = edge.OutputRecord!;
        bool toFront = edge.IsFront;
        OutputPoint opFront = outputRecord.Points!;
        OutputPoint opBack = opFront.Next!;

        switch (toFront)
        {
            case true when PolygonUtilities.PointEquals(point, opFront.Point):
                return opFront;
            case false when PolygonUtilities.PointEquals(point, opBack.Point):
                return opBack;
        }

        OutputPoint newOp = this.outputPointPool.Add(point, outputRecord);
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

    /// <summary>
    /// Creates a new output record and assigns the next index.
    /// </summary>
    /// <returns>The created output record.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private OutputRecord CreateOutputRecord()
    {
        int idx = this.outputRecordPool.Count;
        OutputRecord result = this.outputRecordPool.Add();
        result.Index = idx;
        return result;
    }

    /// <summary>
    /// Advances the active edge to the next vertex in the scanbeam.
    /// </summary>
    /// <param name="edge">The active edge to update.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void UpdateEdgeInActiveList(ActiveEdge edge)
    {
        edge.Bottom = edge.Top;
        edge.VertexTop = edge.NextVertex;
        edge.Top = edge.VertexTop!.Point;
        edge.CurrentX = edge.Bottom.X;
        edge.UpdateDx();

        if (IsJoined(edge))
        {
            this.SplitEdge(edge, edge.Bottom);
        }

        if (edge.IsHorizontal)
        {
            TrimHorizontal(edge, this.PreserveCollinear);

            return;
        }

        this.InsertScanline(edge.Top.Y);

        this.CheckJoinLeft(edge, edge.Bottom);

        // (#500)
        this.CheckJoinRight(edge, edge.Bottom, true);
    }

    /// <summary>
    /// Handles an intersection between two active edges at a given point.
    /// </summary>
    /// <param name="edge1">The first intersecting edge.</param>
    /// <param name="edge2">The second intersecting edge.</param>
    /// <param name="point">The intersection point.</param>
    private void IntersectActiveEdges(ActiveEdge edge1, ActiveEdge edge2, Vertex point)
    {
        if (IsJoined(edge1))
        {
            this.SplitEdge(edge1, point);
        }

        if (IsJoined(edge2))
        {
            this.SplitEdge(edge2, point);
        }

        // UPDATE WINDING COUNTS...
        int oldE1WindCount, oldE2WindCount;
        if (edge1.WindCount + edge2.WindDelta == 0)
        {
            edge1.WindCount = -edge1.WindCount;
        }
        else
        {
            edge1.WindCount += edge2.WindDelta;
        }

        if (edge2.WindCount - edge1.WindDelta == 0)
        {
            edge2.WindCount = -edge2.WindCount;
        }
        else
        {
            edge2.WindCount -= edge1.WindDelta;
        }

        if (this.fillRule == FillRule.Positive)
        {
            oldE1WindCount = edge1.WindCount;
            oldE2WindCount = edge2.WindCount;
        }
        else
        {
            oldE1WindCount = -edge1.WindCount;
            oldE2WindCount = -edge2.WindCount;
        }

        bool e1WindCountIs0or1 = oldE1WindCount is 0 or 1;
        bool e2WindCountIs0or1 = oldE2WindCount is 0 or 1;

        if ((!edge1.IsHot && !e1WindCountIs0or1) ||
            (!edge2.IsHot && !e2WindCountIs0or1))
        {
            return;
        }

        // NOW PROCESS THE INTERSECTION ...

        // if both edges are 'hot' ...
        if (edge1.IsHot && edge2.IsHot)
        {
            if ((oldE1WindCount != 0 && oldE1WindCount != 1) || (oldE2WindCount != 0 && oldE2WindCount != 1))
            {
                this.AddLocalMaximumOutput(edge1, edge2, point);
            }
            else if (edge1.IsFront || (edge1.OutputRecord == edge2.OutputRecord))
            {
                // this 'else if' condition isn't strictly needed but
                // it's sensible to split polygons that only touch at
                // a common vertex (not at common edges).
                this.AddLocalMaximumOutput(edge1, edge2, point);
            }
            else
            {
                // can't treat as maxima & minima
                this.AddOutputPoint(edge1, point);
                SwapOutputRecords(edge1, edge2);
            }
        }

        // if one or other edge is 'hot' ...
        else if (edge1.IsHot)
        {
            this.AddOutputPoint(edge1, point);
            SwapOutputRecords(edge1, edge2);
        }
        else if (edge2.IsHot)
        {
            this.AddOutputPoint(edge2, point);
            SwapOutputRecords(edge1, edge2);
        }

        // neither edge is 'hot'
        else
        {
            if (oldE1WindCount == 1 && oldE2WindCount == 1)
            {
                this.AddLocalMinimumOutput(edge1, edge2, point);
            }
        }
    }

    /// <summary>
    /// Removes an edge from the active list and returns it to the pool.
    /// </summary>
    /// <param name="edge">The edge to remove.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void DeleteFromActiveList(ActiveEdge edge)
    {
        ActiveEdge? prev = edge.PrevInAel;
        ActiveEdge? next = edge.NextInAel;
        if (prev == null && next == null && (edge != this.activeEdgeHead))
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
            this.activeEdgeHead = next;
        }

        if (next != null)
        {
            next.PrevInAel = prev;
        }

        this.RecycleActiveEdge(edge);
    }

    /// <summary>
    /// Resets and returns an active edge to the reuse pool.
    /// </summary>
    /// <param name="edge">The edge to recycle.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void RecycleActiveEdge(ActiveEdge edge)
    {
        // clear refs to allow GC
        edge.Bottom = default;
        edge.Top = default;
        edge.Dx = 0.0;
        edge.WindCount = 0;
        edge.OutputRecord = null;
        edge.PrevInAel = null;
        edge.NextInAel = null;
        edge.PrevInSel = null;
        edge.NextInSel = null;
        edge.Jump = null;
        edge.VertexTop = null;
        edge.LocalMin = default;
        edge.IsLeftBound = false;
        edge.JoinWith = JoinWith.None;
        this.freeActiveEdges.Push(edge);
    }

    /// <summary>
    /// Acquires a reusable active edge, allocating if the pool is empty.
    /// </summary>
    /// <returns>The acquired active edge.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private ActiveEdge AcquireActiveEdge()
    {
        ActiveEdge edge;
        if (this.freeActiveEdges.Count == 0)
        {
            edge = new ActiveEdge();
        }
        else
        {
            // recycle ActiveEdge from free list
            edge = this.freeActiveEdges.Pop();
        }

        return edge;
    }

    /// <summary>
    /// Updates current X values and copies the active list into the sorted edge list.
    /// </summary>
    /// <param name="topY">The scanline top Y coordinate.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void UpdateCurrentXAndCopyToSortedEdges(double topY)
    {
        ActiveEdge? edge = this.activeEdgeHead;
        this.sortedEdgeHead = edge;
        while (edge != null)
        {
            edge.PrevInSel = edge.PrevInAel;
            edge.NextInSel = edge.NextInAel;
            edge.Jump = edge.NextInSel;

            // it is safe to ignore 'joined' edges here because
            // if necessary they will be split in IntersectActiveEdges()
            edge.CurrentX = edge.TopX(topY);

            // NB don't update edge.curr.Y yet (see AddIntersectionNode)
            edge = edge.NextInAel;
        }
    }

    /// <summary>
    /// Executes the sweep-line union for the provided fill rule.
    /// </summary>
    /// <param name="fillRule">The fill rule used to resolve inside/outside.</param>
    private void ExecuteInternal(FillRule fillRule)
    {
        this.fillRule = fillRule;
        this.ResetState();
        if (!this.TryPopScanline(out double y))
        {
            return;
        }

        // Process each scanbeam: insert local minima, handle horizontals, resolve intersections,
        // then advance to the next scanline.
        while (this.succeeded)
        {
            this.InsertLocalMinimaIntoActiveList(y);
            ActiveEdge? edge;
            while (this.TryPopHorizontalEdge(out edge))
            {
                this.ProcessHorizontal(edge!);
            }

            if (this.horizontalSegments.Count > 0)
            {
                this.ConvertHorizontalSegmentsToJoins();
                this.horizontalSegments.Clear();
            }

            // Bottom of scanbeam.
            this.currentScanlineBottomY = y;
            if (!this.TryPopScanline(out y))
            {
                // y new top of scanbeam.
                break;
            }

            this.ProcessIntersections(y);
            this.ProcessScanbeamTop(y);
            while (this.TryPopHorizontalEdge(out edge))
            {
                this.ProcessHorizontal(edge!);
            }
        }

        if (this.succeeded)
        {
            this.ProcessHorizontalJoins();
        }
    }

    /// <summary>
    /// Builds and processes edge intersections for the current scanbeam.
    /// </summary>
    /// <param name="topY">The scanbeam top Y coordinate.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void ProcessIntersections(double topY)
    {
        if (!this.BuildIntersectionList(topY))
        {
            return;
        }

        this.ProcessIntersectionList();
        this.ClearIntersectionNodes();
    }

    /// <summary>
    /// Clears the list of pending intersection nodes.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void ClearIntersectionNodes() => this.intersectionList.Clear();

    /// <summary>
    /// Adds a new intersection node between two edges at the current scanbeam.
    /// </summary>
    /// <param name="edge1">The first edge.</param>
    /// <param name="edge2">The second edge.</param>
    /// <param name="topY">The scanbeam top Y coordinate.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void AddIntersectionNode(ActiveEdge edge1, ActiveEdge edge2, double topY)
    {
        if (!PolygonUtilities.TryGetLineIntersection(
            edge1.Bottom, edge1.Top, edge2.Bottom, edge2.Top, out Vertex intersectionPoint))
        {
            intersectionPoint = new Vertex(edge1.CurrentX, topY);
        }

        if (intersectionPoint.Y > this.currentScanlineBottomY || intersectionPoint.Y < topY)
        {
            double absDx1 = Math.Abs(edge1.Dx);
            double absDx2 = Math.Abs(edge2.Dx);
            switch (absDx1 > 100)
            {
                case true when absDx2 > 100:
                {
                    if (absDx1 > absDx2)
                    {
                        intersectionPoint = PolygonUtilities.ClosestPointOnSegment(intersectionPoint, edge1.Bottom, edge1.Top);
                    }
                    else
                    {
                        intersectionPoint = PolygonUtilities.ClosestPointOnSegment(intersectionPoint, edge2.Bottom, edge2.Top);
                    }

                    break;
                }

                case true:
                    intersectionPoint = PolygonUtilities.ClosestPointOnSegment(intersectionPoint, edge1.Bottom, edge1.Top);
                    break;
                default:
                {
                    if (absDx2 > 100)
                    {
                        intersectionPoint = PolygonUtilities.ClosestPointOnSegment(intersectionPoint, edge2.Bottom, edge2.Top);
                    }
                    else
                    {
                        double targetY = intersectionPoint.Y < topY ? topY : this.currentScanlineBottomY;
                        double targetX = absDx1 < absDx2 ? edge1.TopX(targetY) : edge2.TopX(targetY);
                        intersectionPoint = new Vertex(targetX, targetY);
                    }

                    break;
                }
            }
        }

        IntersectNode node = new(intersectionPoint, edge1, edge2);
        this.intersectionList.Add(node);
    }

    /// <summary>
    /// Extracts an edge from the sorted edge list.
    /// </summary>
    /// <param name="edge">The edge to extract.</param>
    /// <returns>The next edge after the extracted one.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static ActiveEdge? ExtractFromSortedEdges(ActiveEdge edge)
    {
        ActiveEdge? res = edge.NextInSel;
        if (res != null)
        {
            res.PrevInSel = edge.PrevInSel;
        }

        edge.PrevInSel!.NextInSel = res;
        return res;
    }

    /// <summary>
    /// Inserts an edge before another edge in the sorted edge list.
    /// </summary>
    /// <param name="edge1">The edge to insert.</param>
    /// <param name="edge2">The reference edge.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void InsertBeforeInSortedEdges(ActiveEdge edge1, ActiveEdge edge2)
    {
        edge1.PrevInSel = edge2.PrevInSel;
        if (edge1.PrevInSel != null)
        {
            edge1.PrevInSel.NextInSel = edge1;
        }

        edge1.NextInSel = edge2;
        edge2.PrevInSel = edge1;
    }

    /// <summary>
    /// Builds the list of intersections required to sort edges at the top of the scanbeam.
    /// </summary>
    /// <param name="topY">The scanbeam top Y coordinate.</param>
    /// <returns><see langword="true"/> if any intersections were found.</returns>
    private bool BuildIntersectionList(double topY)
    {
        if (this.activeEdgeHead?.NextInAel == null)
        {
            return false;
        }

        // Calculate edge positions at the top of the current scanbeam, and from this
        // we will determine the intersections required to reach these new positions.
        this.UpdateCurrentXAndCopyToSortedEdges(topY);

        // Find all edge intersections in the current scanbeam using a stable merge
        // sort that ensures only adjacent edges are intersecting. Intersect info is
        // stored in the intersection list ready to be processed in ProcessIntersectionList.
        // Re merge sorts see https://stackoverflow.com/a/46319131/359538
        ActiveEdge? left = this.sortedEdgeHead;

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
                            this.AddIntersectionNode(tmp, right, topY);
                            if (tmp == left)
                            {
                                break;
                            }

                            tmp = tmp.PrevInSel!;
                        }

                        tmp = right;
                        right = ExtractFromSortedEdges(tmp);
                        lEnd = right;
                        InsertBeforeInSortedEdges(tmp, left);
                        if (left != currBase)
                        {
                            continue;
                        }

                        currBase = tmp;
                        currBase.Jump = rEnd;
                        if (prevBase == null)
                        {
                            this.sortedEdgeHead = currBase;
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

            left = this.sortedEdgeHead;
        }

        return this.intersectionList.Count > 0;
    }

    /// <summary>
    /// Processes the intersection list in bottom-up order, swapping edges and generating output.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void ProcessIntersectionList()
    {
        // We now have a list of intersections required so that edges will be
        // correctly positioned at the top of the scanbeam. However, it's important
        // that edge intersections are processed from the bottom up, but it's also
        // crucial that intersections only occur between adjacent edges.

        // First we do a quicksort so intersections proceed in a bottom up order ...
        this.intersectionList.Sort(default(IntersectNodeComparer));

        // Now as we process these intersections, we must sometimes adjust the order
        // to ensure that intersecting edges are always adjacent ...
        for (int i = 0; i < this.intersectionList.Count; ++i)
        {
            if (!AreEdgesAdjacentInActiveList(this.intersectionList[i]))
            {
                int j = i + 1;
                while (!AreEdgesAdjacentInActiveList(this.intersectionList[j]))
                {
                    j++;
                }

                // swap
                (this.intersectionList[j], this.intersectionList[i]) =
                    (this.intersectionList[i], this.intersectionList[j]);
            }

            IntersectNode node = this.intersectionList[i];
            this.IntersectActiveEdges(node.Edge1, node.Edge2, node.Point);
            this.SwapPositionsInActiveList(node.Edge1, node.Edge2);

            node.Edge1.CurrentX = node.Point.X;
            node.Edge2.CurrentX = node.Point.X;
            this.CheckJoinLeft(node.Edge2, node.Point, true);
            this.CheckJoinRight(node.Edge1, node.Point, true);
        }
    }

    /// <summary>
    /// Swaps the positions of two adjacent edges in the active list.
    /// </summary>
    /// <param name="edge1">The left edge.</param>
    /// <param name="edge2">The right edge.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void SwapPositionsInActiveList(ActiveEdge edge1, ActiveEdge edge2)
    {
        // preconditon: edge1 must be immediately to the left of edge2
        ActiveEdge? next = edge2.NextInAel;
        if (next != null)
        {
            next.PrevInAel = edge1;
        }

        ActiveEdge? prev = edge1.PrevInAel;
        if (prev != null)
        {
            prev.NextInAel = edge2;
        }

        edge2.PrevInAel = prev;
        edge2.NextInAel = edge1;
        edge1.PrevInAel = edge2;
        edge1.NextInAel = next;
        if (edge2.PrevInAel == null)
        {
            this.activeEdgeHead = edge2;
        }
    }

    /// <summary>
    /// Resolves left-to-right direction and bounds for a horizontal edge.
    /// </summary>
    /// <param name="horizontalEdge">The horizontal edge.</param>
    /// <param name="vertexMax">The maxima vertex for the horizontal span.</param>
    /// <param name="leftX">The left bound X value.</param>
    /// <param name="rightX">The right bound X value.</param>
    /// <returns><see langword="true"/> when the edge is left-to-right.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool ResetHorizontalDirection(
        ActiveEdge horizontalEdge,
        SweepVertex? vertexMax,
        out double leftX,
        out double rightX)
    {
        if (horizontalEdge.Bottom.X == horizontalEdge.Top.X)
        {
            // the horizontal edge is going nowhere ...
            leftX = horizontalEdge.CurrentX;
            rightX = horizontalEdge.CurrentX;
            ActiveEdge? edge = horizontalEdge.NextInAel;
            while (edge != null && edge.VertexTop != vertexMax)
            {
                edge = edge.NextInAel;
            }

            return edge != null;
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

    /// <summary>
    /// Trims collinear points from a horizontal edge.
    /// </summary>
    /// <param name="horizontalEdge">The horizontal edge.</param>
    /// <param name="preserveCollinear">Whether collinear points are preserved.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void TrimHorizontal(ActiveEdge horizontalEdge, bool preserveCollinear)
    {
        bool wasTrimmed = false;
        Vertex point = horizontalEdge.NextVertex.Point;

        while (point.Y == horizontalEdge.Top.Y)
        {
            // always trim 180 deg. spikes (in closed paths)
            // but otherwise break if preserveCollinear = true
            if (preserveCollinear &&
                (point.X < horizontalEdge.Top.X) != (horizontalEdge.Bottom.X < horizontalEdge.Top.X))
            {
                break;
            }

            horizontalEdge.VertexTop = horizontalEdge.NextVertex;
            horizontalEdge.Top = point;
            wasTrimmed = true;
            if (horizontalEdge.IsMaxima)
            {
                break;
            }

            point = horizontalEdge.NextVertex.Point;
        }

        if (wasTrimmed)
        {
            // +/-infinity
            horizontalEdge.UpdateDx();
        }
    }

    /// <summary>
    /// Adds a horizontal segment for later join processing.
    /// </summary>
    /// <param name="outputPoint">The output point that anchors the segment.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void AddHorizontalSegment(OutputPoint outputPoint)
        => this.horizontalSegments.Add(new HorizontalSegment(outputPoint));

    /// <summary>
    /// Returns the last output point for a hot edge.
    /// </summary>
    /// <param name="hotEdge">The hot edge to inspect.</param>
    /// <returns>The last output point.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static OutputPoint GetLastOutputPoint(ActiveEdge hotEdge)
    {
        OutputRecord outputRecord = hotEdge.OutputRecord!;
        return (hotEdge == outputRecord.FrontEdge) ?
            outputRecord.Points! : outputRecord.Points!.Next!;
    }

    /// <summary>
    /// Processes a horizontal edge and resolves any intersections along the scanline.
    /// </summary>
    /// <param name="horizontalEdge">The horizontal edge to process.</param>
    private void ProcessHorizontal(ActiveEdge horizontalEdge)
    /*******************************************************************************
      * Notes: Horizontal edges (HEs) at scanline intersections (i.e. at the top or    *
      * bottom of a scanbeam) are processed as if layered. The order in which HEs    *
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

        SweepVertex? vertexMax = GetMaximaVertexAtCurrentY(horizontalEdge);

        bool isLeftToRight =
            ResetHorizontalDirection(horizontalEdge, vertexMax, out double leftX, out double rightX);

        if (horizontalEdge.IsHot)
        {
            OutputPoint outputPoint = this.AddOutputPoint(horizontalEdge, new Vertex(horizontalEdge.CurrentX, y));
            this.AddHorizontalSegment(outputPoint);
        }

        while (true)
        {
            // loops through consecutive horizontal edges
            ActiveEdge? edge = isLeftToRight ? horizontalEdge.NextInAel : horizontalEdge.PrevInAel;

            while (edge != null)
            {
                if (edge.VertexTop == vertexMax)
                {
                    // do this first!!
                    if (horizontalEdge.IsHot && IsJoined(edge))
                    {
                        this.SplitEdge(edge, edge.Top);
                    }

                    if (horizontalEdge.IsHot)
                    {
                        while (horizontalEdge.VertexTop != vertexMax)
                        {
                            this.AddOutputPoint(horizontalEdge, horizontalEdge.Top);
                            this.UpdateEdgeInActiveList(horizontalEdge);
                        }

                        if (isLeftToRight)
                        {
                            this.AddLocalMaximumOutput(horizontalEdge, edge, horizontalEdge.Top);
                        }
                        else
                        {
                            this.AddLocalMaximumOutput(edge, horizontalEdge, horizontalEdge.Top);
                        }
                    }

                    this.DeleteFromActiveList(edge);
                    this.DeleteFromActiveList(horizontalEdge);
                    return;
                }

                // if the horizontal edge is a maxima, keep going until we reach
                // its maxima pair, otherwise check for break conditions
                Vertex point;
                if (vertexMax != horizontalEdge.VertexTop)
                {
                    // otherwise stop when 'edge' is beyond the end of the horizontal line
                    if ((isLeftToRight && edge.CurrentX > rightX) ||
                            (!isLeftToRight && edge.CurrentX < leftX))
                    {
                        break;
                    }

                    if (edge.CurrentX == horizontalEdge.Top.X && !edge.IsHorizontal)
                    {
                        point = horizontalEdge.NextVertex.Point;

                        // For edges at the horizontal edge's end, only stop when the horizontal
                        // edge's outslope is greater than e's slope when heading right or when the horizontal
                        // edge's outslope is less than e's slope when heading left.
                        if ((isLeftToRight && (edge.TopX(point.Y) >= point.X)) ||
                                (!isLeftToRight && (edge.TopX(point.Y) <= point.X)))
                        {
                            break;
                        }
                    }
                }

                point = new Vertex(edge.CurrentX, y);

                if (isLeftToRight)
                {
                    this.IntersectActiveEdges(horizontalEdge, edge, point);
                    this.SwapPositionsInActiveList(horizontalEdge, edge);
                    this.CheckJoinLeft(edge, point);
                    horizontalEdge.CurrentX = edge.CurrentX;
                    edge = horizontalEdge.NextInAel;
                }
                else
                {
                    this.IntersectActiveEdges(edge, horizontalEdge, point);
                    this.SwapPositionsInActiveList(edge, horizontalEdge);
                    this.CheckJoinRight(edge, point);
                    horizontalEdge.CurrentX = edge.CurrentX;
                    edge = horizontalEdge.PrevInAel;
                }

                if (horizontalEdge.IsHot)
                {
                    this.AddHorizontalSegment(GetLastOutputPoint(horizontalEdge));
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

            this.UpdateEdgeInActiveList(horizontalEdge);

            isLeftToRight = ResetHorizontalDirection(
                horizontalEdge,
                vertexMax,
                out leftX,
                out rightX);
        }

        // End of (possible consecutive) horizontals.
        if (horizontalEdge.IsHot)
        {
            OutputPoint outputPoint = this.AddOutputPoint(horizontalEdge, horizontalEdge.Top);
            this.AddHorizontalSegment(outputPoint);
        }

        // This is the end of an intermediate horizontal.
        this.UpdateEdgeInActiveList(horizontalEdge);
    }

    /// <summary>
    /// Processes edges that reach the top of the scanbeam, updating or removing them.
    /// </summary>
    /// <param name="y">The scanbeam top Y coordinate.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void ProcessScanbeamTop(double y)
    {
        // Sorted edges are reused to flag horizontals (see PushHorizontalEdge below).
        this.sortedEdgeHead = null;
        ActiveEdge? edge = this.activeEdgeHead;
        while (edge != null)
        {
            // NB 'edge' will never be horizontal here
            if (edge.Top.Y == y)
            {
                edge.CurrentX = edge.Top.X;
                if (edge.IsMaxima)
                {
                    // TOP OF BOUND (MAXIMA)
                    edge = this.ProcessMaxima(edge);
                    continue;
                }

                // INTERMEDIATE vertex ...
                if (edge.IsHot)
                {
                    this.AddOutputPoint(edge, edge.Top);
                }

                this.UpdateEdgeInActiveList(edge);

                // Horizontals are processed later.
                if (edge.IsHorizontal)
                {
                    this.PushHorizontalEdge(edge);
                }
            }

            // Not the top of the edge.
            else
            {
                edge.CurrentX = edge.TopX(y);
            }

            edge = edge.NextInAel;
        }
    }

    /// <summary>
    /// Handles a maxima event for the active edge.
    /// </summary>
    /// <param name="edge">The active edge at the maxima.</param>
    /// <returns>The next edge to continue scanning from.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private ActiveEdge? ProcessMaxima(ActiveEdge edge)
    {
        ActiveEdge? prevEdge = edge.PrevInAel;
        ActiveEdge? nextEdge = edge.NextInAel;

        ActiveEdge? maxPair = FindMaximaPair(edge);
        if (maxPair == null)
        {
            // maxPair is horizontal.
            return nextEdge;
        }

        if (IsJoined(edge))
        {
            this.SplitEdge(edge, edge.Top);
        }

        if (IsJoined(maxPair))
        {
            this.SplitEdge(maxPair, maxPair.Top);
        }

        // only non-horizontal maxima here.
        // process any edges between maxima pair ...
        while (nextEdge != maxPair)
        {
            this.IntersectActiveEdges(edge, nextEdge!, edge.Top);
            this.SwapPositionsInActiveList(edge, nextEdge!);
            nextEdge = edge.NextInAel;
        }

        // here edge.nextInAel == ENext == EMaxPair ...
        if (edge.IsHot)
        {
            this.AddLocalMaximumOutput(edge, maxPair, edge.Top);
        }

        this.DeleteFromActiveList(edge);
        this.DeleteFromActiveList(maxPair);
        return prevEdge != null ? prevEdge.NextInAel : this.activeEdgeHead;
    }

    /// <summary>
    /// Tests whether an edge is currently joined to a neighbor.
    /// </summary>
    /// <param name="edge">The edge to inspect.</param>
    /// <returns><see langword="true"/> if the edge is joined.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsJoined(ActiveEdge edge) => edge.JoinWith != JoinWith.None;

    /// <summary>
    /// Splits a joined edge at the specified point.
    /// </summary>
    /// <param name="edge">The edge to split.</param>
    /// <param name="point">The split point.</param>
    private void SplitEdge(ActiveEdge edge, Vertex point)
    {
        if (edge.JoinWith == JoinWith.Right)
        {
            edge.JoinWith = JoinWith.None;
            edge.NextInAel!.JoinWith = JoinWith.None;
            this.AddLocalMinimumOutput(edge, edge.NextInAel, point, true);
        }
        else
        {
            edge.JoinWith = JoinWith.None;
            edge.PrevInAel!.JoinWith = JoinWith.None;
            this.AddLocalMinimumOutput(edge.PrevInAel, edge, point, true);
        }
    }

    /// <summary>
    /// Attempts to join the current edge with its left neighbor.
    /// </summary>
    /// <param name="edge">The active edge being evaluated.</param>
    /// <param name="point">The candidate join point.</param>
    /// <param name="checkCurrX">Whether to check the current X for proximity.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void CheckJoinLeft(
        ActiveEdge edge,
        Vertex point,
        bool checkCurrX = false)
    {
        ActiveEdge? prev = edge.PrevInAel;
        if (prev == null ||
            !edge.IsHot || !prev.IsHot ||
            edge.IsHorizontal || prev.IsHorizontal)
        {
            return;
        }

        // Avoid trivial joins.
        if ((point.Y < edge.Top.Y + PolygonUtilities.ClosePointTolerance || point.Y < prev.Top.Y + PolygonUtilities.ClosePointTolerance) &&
            ((edge.Bottom.Y > point.Y) || (prev.Bottom.Y > point.Y)))
        {
            // (#490)
            return;
        }

        if (checkCurrX)
        {
            if (PolygonUtilities.PerpendicularDistanceSquared(point, prev.Bottom, prev.Top) > PolygonUtilities.JoinDistanceSquared)
            {
                return;
            }
        }
        else if (!PolygonUtilities.IsAlmostZero(edge.CurrentX - prev.CurrentX))
        {
            return;
        }

        if (!PolygonUtilities.IsCollinear(edge.Top, point, prev.Top))
        {
            return;
        }

        if (edge.OutputRecord!.Index == prev.OutputRecord!.Index)
        {
            this.AddLocalMaximumOutput(prev, edge, point);
        }
        else if (edge.OutputRecord!.Index < prev.OutputRecord!.Index)
        {
            JoinOutputRecords(edge, prev);
        }
        else
        {
            JoinOutputRecords(prev, edge);
        }

        prev.JoinWith = JoinWith.Right;
        edge.JoinWith = JoinWith.Left;
    }

    /// <summary>
    /// Attempts to join the current edge with its right neighbor.
    /// </summary>
    /// <param name="edge">The active edge being evaluated.</param>
    /// <param name="point">The candidate join point.</param>
    /// <param name="checkCurrX">Whether to check the current X for proximity.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void CheckJoinRight(
        ActiveEdge edge,
        Vertex point,
        bool checkCurrX = false)
    {
        ActiveEdge? next = edge.NextInAel;
        if (next == null ||
            !edge.IsHot || !next.IsHot ||
            edge.IsHorizontal || next.IsHorizontal)
        {
            return;
        }

        // Avoid trivial joins.
        if ((point.Y < edge.Top.Y + PolygonUtilities.ClosePointTolerance || point.Y < next.Top.Y + PolygonUtilities.ClosePointTolerance) &&
            ((edge.Bottom.Y > point.Y) || (next.Bottom.Y > point.Y)))
        {
            // (#490)
            return;
        }

        if (checkCurrX)
        {
            if (PolygonUtilities.PerpendicularDistanceSquared(point, next.Bottom, next.Top) > PolygonUtilities.JoinDistanceSquared)
            {
                return;
            }
        }
        else if (!PolygonUtilities.IsAlmostZero(edge.CurrentX - next.CurrentX))
        {
            return;
        }

        if (!PolygonUtilities.IsCollinear(edge.Top, point, next.Top))
        {
            return;
        }

        if (edge.OutputRecord!.Index == next.OutputRecord!.Index)
        {
            this.AddLocalMaximumOutput(edge, next, point);
        }
        else if (edge.OutputRecord!.Index < next.OutputRecord!.Index)
        {
            JoinOutputRecords(edge, next);
        }
        else
        {
            JoinOutputRecords(next, edge);
        }

        edge.JoinWith = JoinWith.Right;
        next.JoinWith = JoinWith.Left;
    }

    /// <summary>
    /// Ensures all output points in a record reference the correct owner.
    /// </summary>
    /// <param name="outputRecord">The output record to normalize.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void FixOutputRecordPoints(OutputRecord outputRecord)
    {
        OutputPoint outputPoint = outputRecord.Points!;
        do
        {
            outputPoint.OutputRecord = outputRecord;
            outputPoint = outputPoint.Next!;
        }
        while (outputPoint != outputRecord.Points);
    }

    /// <summary>
    /// Determines the left/right ordering of a horizontal segment.
    /// </summary>
    /// <param name="horizontalSegment">The segment to update.</param>
    /// <param name="prevPoint">The previous output point.</param>
    /// <param name="nextPoint">The next output point.</param>
    /// <returns><see langword="true"/> if the segment has non-zero length.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool SetHorizontalSegmentHeadingForward(HorizontalSegment horizontalSegment, OutputPoint prevPoint, OutputPoint nextPoint)
    {
        if (PolygonUtilities.IsAlmostZero(prevPoint.Point.X - nextPoint.Point.X))
        {
            return false;
        }

        if (prevPoint.Point.X < nextPoint.Point.X)
        {
            horizontalSegment.LeftPoint = prevPoint;
            horizontalSegment.RightPoint = nextPoint;
            horizontalSegment.LeftToRight = true;
        }
        else
        {
            horizontalSegment.LeftPoint = nextPoint;
            horizontalSegment.RightPoint = prevPoint;
            horizontalSegment.LeftToRight = false;
        }

        return true;
    }

    /// <summary>
    /// Normalizes a horizontal segment and sets its left/right pointers.
    /// </summary>
    /// <param name="horizontalSegment">The segment to update.</param>
    /// <returns><see langword="true"/> if the segment remains valid after normalization.</returns>
    private static bool UpdateHorizontalSegment(HorizontalSegment horizontalSegment)
    {
        OutputPoint outputPoint = horizontalSegment.LeftPoint!;
        OutputRecord outputRecord = ResolveOutputRecord(outputPoint.OutputRecord)!;
        bool outputRecordHasEdges = outputRecord.FrontEdge != null;
        double currentY = outputPoint.Point.Y;
        OutputPoint prevPoint = outputPoint, nextPoint = outputPoint;
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
            SetHorizontalSegmentHeadingForward(horizontalSegment, prevPoint, nextPoint) &&
            horizontalSegment.LeftPoint!.HorizontalSegment == null;

        if (result)
        {
            horizontalSegment.LeftPoint!.HorizontalSegment = horizontalSegment;
        }
        else
        {
            // (for sorting)
            horizontalSegment.RightPoint = null;
        }

        return result;
    }

    /// <summary>
    /// Duplicates an output point and inserts it before or after the original.
    /// </summary>
    /// <param name="outputPoint">The point to duplicate.</param>
    /// <param name="insertAfter">Whether to insert after the original.</param>
    /// <returns>The newly inserted output point.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private OutputPoint DuplicateOutputPoint(OutputPoint outputPoint, bool insertAfter)
    {
        OutputPoint result = this.outputPointPool.Add(outputPoint.Point, outputPoint.OutputRecord);
        if (insertAfter)
        {
            result.Next = outputPoint.Next;
            result.Next!.Prev = result;
            result.Prev = outputPoint;
            outputPoint.Next = result;
        }
        else
        {
            result.Prev = outputPoint.Prev;
            result.Prev.Next = result;
            result.Next = outputPoint;
            outputPoint.Prev = result;
        }

        return result;
    }

    /// <summary>
    /// Sorts horizontal segments by their X extents.
    /// </summary>
    /// <param name="segment1">The first segment.</param>
    /// <param name="segment2">The second segment.</param>
    /// <returns>A comparison result for sorting.</returns>
    private static int CompareHorizontalSegments(HorizontalSegment? segment1, HorizontalSegment? segment2)
    {
        if (segment1 == null || segment2 == null)
        {
            return 0;
        }

        if (segment1.RightPoint == null)
        {
            return segment2.RightPoint == null ? 0 : 1;
        }

        if (segment2.RightPoint == null)
        {
            return -1;
        }

        return segment1.LeftPoint!.Point.X.CompareTo(segment2.LeftPoint!.Point.X);
    }

    /// <summary>
    /// Converts horizontal segments into join candidates for post-processing.
    /// </summary>
    private void ConvertHorizontalSegmentsToJoins()
    {
        int k = 0;
        foreach (HorizontalSegment horizontalSegment in this.horizontalSegments)
        {
            if (UpdateHorizontalSegment(horizontalSegment))
            {
                k++;
            }
        }

        if (k < 2)
        {
            return;
        }

        this.horizontalSegments.Sort(CompareHorizontalSegments);

        for (int i = 0; i < k - 1; i++)
        {
            HorizontalSegment segment1 = this.horizontalSegments[i];

            // for each HorizontalSegment, find others that overlap
            for (int j = i + 1; j < k; j++)
            {
                HorizontalSegment segment2 = this.horizontalSegments[j];
                if ((segment2.LeftPoint!.Point.X >= segment1.RightPoint!.Point.X) ||
                    (segment2.LeftToRight == segment1.LeftToRight) ||
                    (segment2.RightPoint!.Point.X <= segment1.LeftPoint!.Point.X))
                {
                    continue;
                }

                double currentY = segment1.LeftPoint.Point.Y;
                if (segment1.LeftToRight)
                {
                    while (segment1.LeftPoint.Next!.Point.Y == currentY &&
                        segment1.LeftPoint.Next.Point.X <= segment2.LeftPoint.Point.X)
                    {
                        segment1.LeftPoint = segment1.LeftPoint.Next;
                    }

                    while (segment2.LeftPoint.Prev.Point.Y == currentY &&
                        segment2.LeftPoint.Prev.Point.X <= segment1.LeftPoint.Point.X)
                    {
                        segment2.LeftPoint = segment2.LeftPoint.Prev;
                    }

                    _ = this.horizontalJoins.Add(
                        this.DuplicateOutputPoint(segment1.LeftPoint, true),
                        this.DuplicateOutputPoint(segment2.LeftPoint, false));
                }
                else
                {
                    while (segment1.LeftPoint.Prev.Point.Y == currentY &&
                        segment1.LeftPoint.Prev.Point.X <= segment2.LeftPoint.Point.X)
                    {
                        segment1.LeftPoint = segment1.LeftPoint.Prev;
                    }

                    while (segment2.LeftPoint.Next!.Point.Y == currentY &&
                        segment2.LeftPoint.Next.Point.X <= segment1.LeftPoint.Point.X)
                    {
                        segment2.LeftPoint = segment2.LeftPoint.Next;
                    }

                    _ = this.horizontalJoins.Add(
                        this.DuplicateOutputPoint(segment2.LeftPoint, true),
                        this.DuplicateOutputPoint(segment1.LeftPoint, false));
                }
            }
        }
    }

    /// <summary>
    /// Builds a cleaned contour by removing redundant collinear points.
    /// </summary>
    /// <param name="outputPoint">A point on the output ring.</param>
    /// <returns>A contour with redundant points removed.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Contour BuildCleanContour(OutputPoint outputPoint)
    {
        Contour result = [];
        OutputPoint outputPoint2 = outputPoint;
        while (outputPoint2.Next != outputPoint &&
            ((PolygonUtilities.IsAlmostZero(outputPoint2.Point.X - outputPoint2.Next!.Point.X) &&
              PolygonUtilities.IsAlmostZero(outputPoint2.Point.X - outputPoint2.Prev.Point.X)) ||
             (PolygonUtilities.IsAlmostZero(outputPoint2.Point.Y - outputPoint2.Next.Point.Y) &&
              PolygonUtilities.IsAlmostZero(outputPoint2.Point.Y - outputPoint2.Prev.Point.Y))))
        {
            outputPoint2 = outputPoint2.Next;
        }

        result.Add(outputPoint2.Point);
        OutputPoint prevOp = outputPoint2;
        outputPoint2 = outputPoint2.Next;
        while (outputPoint2 != outputPoint)
        {
            if ((!PolygonUtilities.IsAlmostZero(outputPoint2.Point.X - outputPoint2.Next!.Point.X) ||
                 !PolygonUtilities.IsAlmostZero(outputPoint2.Point.X - prevOp.Point.X)) &&
                (!PolygonUtilities.IsAlmostZero(outputPoint2.Point.Y - outputPoint2.Next.Point.Y) ||
                 !PolygonUtilities.IsAlmostZero(outputPoint2.Point.Y - prevOp.Point.Y)))
            {
                result.Add(outputPoint2.Point);
                prevOp = outputPoint2;
            }

            outputPoint2 = outputPoint2.Next;
        }

        return result;
    }

    /// <summary>
    /// Classifies a point against an output polygon.
    /// </summary>
    /// <param name="point">The point to test.</param>
    /// <param name="outputPoint">A point on the polygon ring.</param>
    /// <returns>The point-in-polygon classification.</returns>
    private static PointInPolygonResult PointInOutputPolygon(Vertex point, OutputPoint outputPoint)
    {
        if (outputPoint == outputPoint.Next || outputPoint.Prev == outputPoint.Next)
        {
            return PointInPolygonResult.Outside;
        }

        OutputPoint outputPoint2 = outputPoint;
        do
        {
            if (outputPoint.Point.Y != point.Y)
            {
                break;
            }

            outputPoint = outputPoint.Next!;
        }
        while (outputPoint != outputPoint2);

        // Not a proper polygon.
        if (outputPoint.Point.Y == point.Y)
        {
            return PointInPolygonResult.Outside;
        }

        // must be above or below to get here
        bool isAbove = outputPoint.Point.Y < point.Y, startingAbove = isAbove;
        int val = 0;

        outputPoint2 = outputPoint.Next!;
        while (outputPoint2 != outputPoint)
        {
            if (isAbove)
            {
                while (outputPoint2 != outputPoint && outputPoint2.Point.Y < point.Y)
                {
                    outputPoint2 = outputPoint2.Next!;
                }
            }
            else
            {
                while (outputPoint2 != outputPoint && outputPoint2.Point.Y > point.Y)
                {
                    outputPoint2 = outputPoint2.Next!;
                }
            }

            if (outputPoint2 == outputPoint)
            {
                break;
            }

            // must have touched or crossed the point.Y horizontal
            // and this must happen an even number of times
            // Touching the horizontal.
            if (outputPoint2.Point.Y == point.Y)
            {
                if (outputPoint2.Point.X == point.X || (outputPoint2.Point.Y == outputPoint2.Prev.Point.Y &&
                    (point.X < outputPoint2.Prev.Point.X) != (point.X < outputPoint2.Point.X)))
                {
                    return PointInPolygonResult.On;
                }

                outputPoint2 = outputPoint2.Next!;
                if (outputPoint2 == outputPoint)
                {
                    break;
                }

                continue;
            }

            if (outputPoint2.Point.X <= point.X || outputPoint2.Prev.Point.X <= point.X)
            {
                if (outputPoint2.Prev.Point.X < point.X && outputPoint2.Point.X < point.X)
                {
                    // Toggle val.
                    val = 1 - val;
                }
                else
                {
                    int d = PolygonUtilities.CrossSign(outputPoint2.Prev.Point, outputPoint2.Point, point);
                    if (d == 0)
                    {
                        return PointInPolygonResult.On;
                    }

                    if ((d < 0) == isAbove)
                    {
                        val = 1 - val;
                    }
                }
            }

            isAbove = !isAbove;
            outputPoint2 = outputPoint2.Next!;
        }

        if (isAbove == startingAbove)
        {
            return val == 0 ? PointInPolygonResult.Outside : PointInPolygonResult.Inside;
        }

        {
            int d = PolygonUtilities.CrossSign(outputPoint2.Prev.Point, outputPoint2.Point, point);
            if (d == 0)
            {
                return PointInPolygonResult.On;
            }

            if ((d < 0) == isAbove)
            {
                val = 1 - val;
            }
        }

        return val == 0 ? PointInPolygonResult.Outside : PointInPolygonResult.Inside;
    }

    /// <summary>
    /// Determines whether one output ring lies inside another.
    /// </summary>
    /// <param name="outputPoint1">A point on the candidate inner ring.</param>
    /// <param name="outputPoint2">A point on the candidate outer ring.</param>
    /// <returns><see langword="true"/> if the first ring is inside the second.</returns>
    private static bool IsPathInsidePath(OutputPoint outputPoint1, OutputPoint outputPoint2)
    {
        // we need to make some accommodation for rounding errors
        // so we won't jump if the first ClipVertex is found outside
        PointInPolygonResult pip = PointInPolygonResult.On;
        OutputPoint outputPoint = outputPoint1;
        do
        {
            switch (PointInOutputPolygon(outputPoint.Point, outputPoint2))
            {
                case PointInPolygonResult.Outside:
                    if (pip == PointInPolygonResult.Outside)
                    {
                        return false;
                    }

                    pip = PointInPolygonResult.Outside;
                    break;
                case PointInPolygonResult.Inside:
                    if (pip == PointInPolygonResult.Inside)
                    {
                        return true;
                    }

                    pip = PointInPolygonResult.Inside;
                    break;
                default:
                    break;
            }

            outputPoint = outputPoint.Next!;
        }
        while (outputPoint != outputPoint1);

        // result is unclear, so try again using cleaned paths
        // (#973)
        return PolygonUtilities.Path2ContainsPath1(BuildCleanContour(outputPoint1), BuildCleanContour(outputPoint2));
    }

    /// <summary>
    /// Moves split ownership from one output record to another.
    /// </summary>
    /// <param name="sourceRecord">The output record to move from.</param>
    /// <param name="targetRecord">The output record to move to.</param>
    private static void MoveOutputSplits(OutputRecord sourceRecord, OutputRecord targetRecord)
    {
        if (sourceRecord.Splits == null)
        {
            return;
        }

        targetRecord.Splits ??= [];
        foreach (int i in sourceRecord.Splits)
        {
            if (i != targetRecord.Index)
            {
                targetRecord.Splits.Add(i);
            }
        }

        sourceRecord.Splits = null;
    }

    /// <summary>
    /// Processes horizontal joins captured during the sweep.
    /// </summary>
    private void ProcessHorizontalJoins()
    {
        foreach (HorizontalJoin join in this.horizontalJoins)
        {
            OutputRecord outputRecord1 = ResolveOutputRecord(join.LeftToRight!.OutputRecord)!;
            OutputRecord outputRecord2 = ResolveOutputRecord(join.RightToLeft!.OutputRecord)!;

            OutputPoint op1b = join.LeftToRight.Next!;
            OutputPoint op2b = join.RightToLeft.Prev;
            join.LeftToRight.Next = join.RightToLeft;
            join.RightToLeft.Prev = join.LeftToRight;
            op1b.Prev = op2b;
            op2b.Next = op1b;

            // 'join' is really a split
            if (outputRecord1 == outputRecord2)
            {
                outputRecord2 = this.CreateOutputRecord();
                outputRecord2.Points = op1b;
                FixOutputRecordPoints(outputRecord2);

                // if outputRecord1.Points has moved to outputRecord2 then update outputRecord1.Points.
                if (outputRecord1.Points!.OutputRecord == outputRecord2)
                {
                    outputRecord1.Points = join.LeftToRight;
                    outputRecord1.Points.OutputRecord = outputRecord1;
                }

                // #498, #520, #584, D#576, #618
                if (this.buildHierarchy)
                {
                    if (IsPathInsidePath(outputRecord1.Points, outputRecord2.Points))
                    {
                        // swap outputRecord1's and outputRecord2's points
                        (outputRecord2.Points, outputRecord1.Points) = (outputRecord1.Points, outputRecord2.Points);
                        FixOutputRecordPoints(outputRecord1);
                        FixOutputRecordPoints(outputRecord2);

                        // outputRecord2 is now inside outputRecord1
                        outputRecord2.Owner = outputRecord1;
                    }
                    else if (IsPathInsidePath(outputRecord2.Points, outputRecord1.Points))
                    {
                        outputRecord2.Owner = outputRecord1;
                    }
                    else
                    {
                        outputRecord2.Owner = outputRecord1.Owner;
                    }

                    outputRecord1.Splits ??= [];
                    outputRecord1.Splits.Add(outputRecord2.Index);
                }
                else
                {
                    outputRecord2.Owner = outputRecord1;
                }
            }
            else
            {
                outputRecord2.Points = null;
                if (this.buildHierarchy)
                {
                    SetOutputOwner(outputRecord2, outputRecord1);

                    // #618
                    MoveOutputSplits(outputRecord2, outputRecord1);
                }
                else
                {
                    outputRecord2.Owner = outputRecord1;
                }
            }
        }
    }

    /// <summary>
    /// Determines whether two points are within a tight tolerance.
    /// </summary>
    /// <param name="firstPoint">The first point.</param>
    /// <param name="secondPoint">The second point.</param>
    /// <returns><see langword="true"/> if the points are nearly coincident.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool ArePointsVeryClose(in Vertex firstPoint, in Vertex secondPoint)
    {
        double tolerance = PolygonUtilities.ClosePointTolerance;
        return Math.Abs(firstPoint.X - secondPoint.X) < tolerance && Math.Abs(firstPoint.Y - secondPoint.Y) < tolerance;
    }

    /// <summary>
    /// Tests whether an output ring collapses to a very small triangle.
    /// </summary>
    /// <param name="outputPoint">A point on the ring.</param>
    /// <returns><see langword="true"/> if the triangle is degenerate.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsVerySmallTriangle(OutputPoint outputPoint) => outputPoint.Next!.Next == outputPoint.Prev &&
      (ArePointsVeryClose(outputPoint.Prev.Point, outputPoint.Next.Point) ||
        ArePointsVeryClose(outputPoint.Point, outputPoint.Next.Point) ||
        ArePointsVeryClose(outputPoint.Point, outputPoint.Prev.Point));

    /// <summary>
    /// Validates that an output ring is a non-degenerate closed loop.
    /// </summary>
    /// <param name="outputPoint">A point on the ring.</param>
    /// <returns><see langword="true"/> if the ring is valid.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsValidClosedPath(OutputPoint? outputPoint) => outputPoint != null && outputPoint.Next != outputPoint &&
            (outputPoint.Next != outputPoint.Prev || !IsVerySmallTriangle(outputPoint));

    /// <summary>
    /// Removes an output point from the ring and returns the next point.
    /// </summary>
    /// <param name="outputPoint">The output point to remove.</param>
    /// <returns>The next output point in the ring, or <see langword="null"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static OutputPoint? RecycleOutputPoint(OutputPoint outputPoint)
    {
        OutputPoint? result = outputPoint.Next == outputPoint ? null : outputPoint.Next;
        outputPoint.Prev.Next = outputPoint.Next;
        outputPoint.Next!.Prev = outputPoint.Prev;

        // outputPoint == null;
        return result;
    }

    /// <summary>
    /// Removes collinear points and resolves self-intersections in an output record.
    /// </summary>
    /// <param name="outputRecord">The output record to clean.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void CleanCollinearEdges(OutputRecord? outputRecord)
    {
        outputRecord = ResolveOutputRecord(outputRecord);

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
        OutputPoint? outputPoint2 = startOp;
        while (true)
        {
            // NB if preserveCollinear == true, then only remove 180 deg. spikes
            if (PolygonUtilities.IsCollinear(outputPoint2!.Prev.Point, outputPoint2.Point, outputPoint2.Next!.Point) &&
                (PolygonUtilities.PointEquals(outputPoint2.Point, outputPoint2.Prev.Point) || PolygonUtilities.PointEquals(outputPoint2.Point, outputPoint2.Next.Point) || !this.PreserveCollinear ||
                (PolygonUtilities.Dot(outputPoint2.Prev.Point, outputPoint2.Point, outputPoint2.Next.Point) < 0)))
            {
                if (outputPoint2 == outputRecord.Points)
                {
                    outputRecord.Points = outputPoint2.Prev;
                }

                outputPoint2 = RecycleOutputPoint(outputPoint2);
                if (!IsValidClosedPath(outputPoint2))
                {
                    outputRecord.Points = null;
                    return;
                }

                startOp = outputPoint2!;
                continue;
            }

            outputPoint2 = outputPoint2.Next;
            if (outputPoint2 == startOp)
            {
                break;
            }
        }

        this.FixSelfIntersections(outputRecord);
    }

    /// <summary>
    /// Splits an output record at a self-intersection.
    /// </summary>
    /// <param name="outputRecord">The record being split.</param>
    /// <param name="splitOp">The output point where the split occurs.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void SplitOutputRecord(OutputRecord outputRecord, OutputPoint splitOp)
    {
        // splitOp.Prev <=> splitOp &&
        // splitOp.Next <=> splitOp.Next.Next are intersecting
        OutputPoint prevOp = splitOp.Prev;
        OutputPoint nextNextOp = splitOp.Next!.Next!;
        outputRecord.Points = prevOp;

        PolygonUtilities.TryGetLineIntersection(
            prevOp.Point, splitOp.Point, splitOp.Next.Point, nextNextOp.Point, out Vertex intersectionPoint);

        double area1 = ComputeSignedArea(prevOp);
        double absArea1 = Math.Abs(area1);

        if (absArea1 < PolygonUtilities.SmallAreaTolerance2)
        {
            outputRecord.Points = null;
            return;
        }

        double area2 = PolygonUtilities.SignedArea(intersectionPoint, splitOp.Point, splitOp.Next.Point);
        double absArea2 = Math.Abs(area2);

        // de-link splitOp and splitOp.Next from the path
        // while inserting the intersection point
        if (PolygonUtilities.PointEquals(intersectionPoint, prevOp.Point) || PolygonUtilities.PointEquals(intersectionPoint, nextNextOp.Point))
        {
            nextNextOp.Prev = prevOp;
            prevOp.Next = nextNextOp;
        }
        else
        {
            OutputPoint newOp2 = this.outputPointPool.Add(intersectionPoint, outputRecord);
            newOp2.Prev = prevOp;
            newOp2.Next = nextNextOp;
            nextNextOp.Prev = newOp2;
            prevOp.Next = newOp2;
        }

        // nb: area1 is the path's signed area *before* splitting, whereas area2 is
        // the signed area of the triangle containing splitOp & splitOp.Next.
        // So the only way for these areas to have the same sign is if
        // the split triangle is larger than the path containing prevOp or
        // if there's more than one self-intersection.
        if (!(absArea2 > PolygonUtilities.SmallAreaTolerance) ||
                (!(absArea2 > absArea1) &&
                  ((area2 > 0) != (area1 > 0))))
        {
            return;
        }

        OutputRecord newOutputRecord = this.CreateOutputRecord();
        newOutputRecord.Owner = outputRecord.Owner;
        splitOp.OutputRecord = newOutputRecord;
        splitOp.Next.OutputRecord = newOutputRecord;

        OutputPoint newOp = this.outputPointPool.Add(intersectionPoint, newOutputRecord);
        newOp.Prev = splitOp.Next;
        newOp.Next = splitOp;
        newOutputRecord.Points = newOp;
        splitOp.Prev = newOp;
        splitOp.Next.Next = newOp;

        if (!this.buildHierarchy)
        {
            return;
        }

        if (IsPathInsidePath(prevOp, newOp))
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

    /// <summary>
    /// Resolves self-intersections within an output record.
    /// </summary>
    /// <param name="outputRecord">The output record to inspect.</param>
    private void FixSelfIntersections(OutputRecord outputRecord)
    {
        OutputPoint outputPoint2 = outputRecord.Points!;
        if (outputPoint2.Prev == outputPoint2.Next!.Next)
        {
            // Triangles can't self-intersect.
            return;
        }

        while (true)
        {
            if (PolygonUtilities.SegmentsIntersect(
                outputPoint2!.Prev.Point,
                outputPoint2.Point,
                outputPoint2.Next!.Point,
                outputPoint2.Next.Next!.Point))
            {
                if (PolygonUtilities.SegmentsIntersect(
                    outputPoint2.Prev.Point,
                    outputPoint2.Point,
                    outputPoint2.Next.Next!.Point,
                    outputPoint2.Next.Next.Next!.Point))
                {
                    // adjacent intersections (ie a micro self-intersection)
                    outputPoint2 = this.DuplicateOutputPoint(outputPoint2, false);
                    outputPoint2.Point = outputPoint2.Next!.Next!.Next!.Point;
                    outputPoint2 = outputPoint2.Next;
                }
                else
                {
                    if (outputPoint2 == outputRecord.Points || outputPoint2.Next == outputRecord.Points)
                    {
                        outputRecord.Points = outputRecord.Points.Prev;
                    }

                    this.SplitOutputRecord(outputRecord, outputPoint2);
                    if (outputRecord.Points == null)
                    {
                        return;
                    }

                    outputPoint2 = outputRecord.Points;

                    // triangles can't self-intersect
                    if (outputPoint2.Prev == outputPoint2.Next!.Next)
                    {
                        break;
                    }

                    continue;
                }
            }

            outputPoint2 = outputPoint2.Next!;
            if (outputPoint2 == outputRecord.Points)
            {
                break;
            }
        }
    }

    /// <summary>
    /// Builds a lightweight path from an output ring.
    /// </summary>
    /// <param name="outputPoint">A point on the output ring.</param>
    /// <param name="reverse">Whether to reverse point order.</param>
    /// <param name="path">The destination contour.</param>
    /// <returns><see langword="true"/> if a valid path was built.</returns>
    private static bool BuildPath(OutputPoint? outputPoint, bool reverse, Contour path)
    {
        if (outputPoint == null || outputPoint.Next == outputPoint || outputPoint.Next == outputPoint.Prev)
        {
            return false;
        }

        path.Clear();

        Vertex lastPoint;
        OutputPoint currentPoint;
        if (reverse)
        {
            lastPoint = outputPoint.Point;
            currentPoint = outputPoint.Prev;
        }
        else
        {
            outputPoint = outputPoint.Next!;
            lastPoint = outputPoint.Point;
            currentPoint = outputPoint.Next!;
        }

        path.Add(lastPoint);

        while (currentPoint != outputPoint)
        {
            if (!PolygonUtilities.PointEquals(currentPoint.Point, lastPoint))
            {
                lastPoint = currentPoint.Point;
                path.Add(lastPoint);
            }

            currentPoint = reverse ? currentPoint.Prev : currentPoint.Next!;
        }

        return path.Count != 3 || !IsVerySmallTriangle(currentPoint);
    }

    /// <summary>
    /// Builds a closed contour from an output ring and adds a closing vertex.
    /// </summary>
    /// <param name="outputPoint">A point on the output ring.</param>
    /// <param name="reverse">Whether to reverse point order.</param>
    /// <param name="contour">The destination contour.</param>
    /// <returns><see langword="true"/> if a valid contour was built.</returns>
    private static bool BuildContour(OutputPoint? outputPoint, bool reverse, Contour contour)
    {
        if (outputPoint == null || outputPoint.Next == outputPoint || outputPoint.Next == outputPoint.Prev)
        {
            return false;
        }

        contour.Clear();

        Vertex lastPoint;
        OutputPoint currentPoint;
        if (reverse)
        {
            lastPoint = new Vertex(outputPoint.Point.X, outputPoint.Point.Y);
            currentPoint = outputPoint.Prev;
        }
        else
        {
            outputPoint = outputPoint.Next!;
            lastPoint = new Vertex(outputPoint.Point.X, outputPoint.Point.Y);
            currentPoint = outputPoint.Next!;
        }

        contour.Add(lastPoint);

        while (currentPoint != outputPoint)
        {
            Vertex current = new Vertex(currentPoint.Point.X, currentPoint.Point.Y);
            if (current != lastPoint)
            {
                lastPoint = current;
                contour.Add(lastPoint);
            }

            currentPoint = reverse ? currentPoint.Prev : currentPoint.Next!;
        }

        if (contour.Count == 3 && IsVerySmallTriangle(currentPoint))
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

    /// <summary>
    /// Builds output contours from completed output records.
    /// </summary>
    /// <param name="solution">The destination contour list.</param>
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
            this.CleanCollinearEdges(outputRecord);
            if (BuildContour(outputRecord.Points, this.ReverseSolution, contour))
            {
                solution.Add(contour);
            }
        }
    }

    /// <summary>
    /// Ensures an output record has bounds populated and valid geometry.
    /// </summary>
    /// <param name="outputRecord">The output record to check.</param>
    /// <returns><see langword="true"/> if bounds are available.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private bool CheckOutputBounds(OutputRecord outputRecord)
    {
        if (outputRecord.Points == null)
        {
            return false;
        }

        if (!outputRecord.Bounds.IsEmpty())
        {
            return true;
        }

        this.CleanCollinearEdges(outputRecord);
        if (outputRecord.Points == null ||
            !BuildPath(outputRecord.Points, this.ReverseSolution, outputRecord.Path))
        {
            return false;
        }

        outputRecord.Bounds = PolygonUtilities.GetBounds(outputRecord.Path);
        return true;
    }

    /// <summary>
    /// Determines ownership for split output records.
    /// </summary>
    /// <param name="outputRecord">The output record whose owner is being resolved.</param>
    /// <param name="splits">The split indices to evaluate.</param>
    /// <returns><see langword="true"/> if ownership could be resolved.</returns>
    private bool CheckSplitOwner(OutputRecord outputRecord, List<int>? splits)
    {
        // nb: use indexing (not an iterator) in case 'splits' is modified inside this loop (#1029)
        for (int i = 0; i < splits!.Count; i++)
        {
            OutputRecord? splitRecord = this.outputRecordPool[splits[i]];
            if (splitRecord.Points == null && splitRecord.Splits != null &&
                this.CheckSplitOwner(outputRecord, splitRecord.Splits))
            {
                // #942
                return true;
            }

            splitRecord = ResolveOutputRecord(splitRecord);
            if (splitRecord == null || splitRecord == outputRecord || splitRecord.RecursiveSplit == outputRecord)
            {
                continue;
            }

            // #599
            splitRecord.RecursiveSplit = outputRecord;

            if (splitRecord.Splits != null && this.CheckSplitOwner(outputRecord, splitRecord.Splits))
            {
                return true;
            }

            if (!this.CheckOutputBounds(splitRecord) ||
                    !splitRecord.Bounds.Contains(outputRecord.Bounds) ||
                    !IsPathInsidePath(outputRecord.Points!, splitRecord.Points!))
            {
                continue;
            }

            // splitRecord is owned by outputRecord (#957)
            if (!IsOwnerValid(outputRecord, splitRecord))
            {
                splitRecord.Owner = outputRecord.Owner;
            }

            // Found in splitRecord.
            outputRecord.Owner = splitRecord;
            return true;
        }

        return false;
    }

    /// <summary>
    /// Resolves the owning output record for hierarchy construction.
    /// </summary>
    /// <param name="outputRecord">The output record to resolve.</param>
    private void ResolveOutputOwner(OutputRecord outputRecord)
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

            if (outputRecord.Owner.Points != null && this.CheckOutputBounds(outputRecord.Owner) &&
                IsPathInsidePath(outputRecord.Points!, outputRecord.Owner.Points!))
            {
                break;
            }

            outputRecord.Owner = outputRecord.Owner.Owner;
        }
    }

    /// <summary>
    /// Builds a hierarchical polygon from output records.
    /// </summary>
    /// <param name="polygon">The polygon to populate.</param>
    private void BuildPolygon(Polygon polygon)
    {
        polygon.Clear();
        List<OutputRecord> closedOutputRecords = new(this.outputRecordPool.Count);

        int i = 0;

        // this.outputRecordPool.Count is not static here because
        // CheckOutputBounds below can indirectly add additional
        // OutputRecord (via FixOutputRecordPoints & CleanCollinearEdges)
        while (i < this.outputRecordPool.Count)
        {
            OutputRecord outputRecord = this.outputRecordPool[i++];
            if (outputRecord.Points == null)
            {
                continue;
            }

            if (this.CheckOutputBounds(outputRecord))
            {
                this.ResolveOutputOwner(outputRecord);
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

    /// <summary>
    /// Executes the union and builds a hierarchical polygon.
    /// </summary>
    /// <param name="fillRule">The fill rule for the union.</param>
    /// <param name="polygon">The polygon to populate.</param>
    /// <returns><see langword="true"/> if the union completed successfully.</returns>
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

        this.ClearSolutionData();
        return this.succeeded;
    }

    /// <summary>
    /// Executes the union and builds a flat list of contours.
    /// </summary>
    /// <param name="fillRule">The fill rule for the union.</param>
    /// <param name="solution">The contour list to populate.</param>
    /// <returns><see langword="true"/> if the union completed successfully.</returns>
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

        this.ClearSolutionData();
        return this.succeeded;
    }

    /// <summary>
    /// Sorts intersection nodes from top to bottom, then left to right.
    /// </summary>
    private struct IntersectNodeComparer : IComparer<IntersectNode>
    {
        /// <summary>
        /// Compares two intersection nodes for sorting.
        /// </summary>
        /// <param name="a">The first node.</param>
        /// <param name="b">The second node.</param>
        /// <returns>A comparison result for sorting.</returns>
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

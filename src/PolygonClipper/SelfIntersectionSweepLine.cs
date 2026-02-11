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
internal sealed class SelfIntersectionSweepLine
{
    private FillRule fillRule;
    private readonly ActiveEdgeList activeEdges;
    private readonly ScanlineSchedule scanlineSchedule;
    private readonly List<IntersectNode> intersectionList;
    private readonly VertexPoolList vertexList;
    private readonly OutputRecordPoolList outputRecordPool;
    private readonly List<HorizontalSegment> horizontalSegments;
    private readonly HorizontalJoinPoolList horizontalJoins;
    private readonly OutputPointPoolList outputPointPool;
    private long currentScanlineBottomY;
    private bool buildHierarchy;
    private bool succeeded;

    /// <summary>
    /// Initializes a new instance of the <see cref="SelfIntersectionSweepLine"/> class.
    /// </summary>
    public SelfIntersectionSweepLine()
    {
        this.activeEdges = new ActiveEdgeList();
        this.scanlineSchedule = new ScanlineSchedule();
        this.intersectionList = [];
        this.vertexList = [];
        this.outputRecordPool = new OutputRecordPoolList();
        this.horizontalSegments = [];
        this.horizontalJoins = [];
        this.outputPointPool = [];
        this.PreserveCollinear = true;
    }

    /// <summary>
    /// Gets or sets a value indicating whether collinear output points are preserved.
    /// </summary>
    public bool PreserveCollinear { get; set; }

    /// <summary>
    /// Gets the pooled output records produced by the sweep.
    /// </summary>
    public OutputRecordPoolList OutputRecords => this.outputRecordPool;

    /// <summary>
    /// Gets the pooled output points produced by the sweep.
    /// </summary>
    public OutputPointPoolList OutputPoints => this.outputPointPool;

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
                // Matched the companion maxima edge.
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
            while (result!.Next!.Point.Y == result.Point.Y)
            {
                result = result.Next;
            }
        }
        else
        {
            while (result!.Prev!.Point.Y == result.Point.Y)
            {
                result = result.Prev;
            }
        }

        if (!result.IsMaxima)
        {
            // No maxima at the current scanline.
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
        // At least one edge already owns an output record.
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

        // Avoid cycles: ensure outputRecord is not already an ancestor of newOwner.
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
    public static double ComputeSignedArea(OutputPoint outputPoint)
    {
        // https://en.wikipedia.org/wiki/Shoelace_formula
        double signedArea = 0.0;
        OutputPoint outputPoint2 = outputPoint;
        do
        {
            signedArea += (double)Vertex64.Cross(outputPoint2.Prev.Point, outputPoint2.Point);
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
    public static OutputRecord? ResolveOutputRecord(OutputRecord? outputRecord)
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
    public static bool IsOwnerValid(OutputRecord? outputRecord, OutputRecord? testOwner)
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
    public void ClearSolutionData()
    {
        this.activeEdges.ClearActiveEdges();
        this.scanlineSchedule.ClearScanlines();
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
    public void Clear()
    {
        this.ClearSolutionData();
        this.scanlineSchedule.Clear();
        this.vertexList.Clear();
    }

    /// <summary>
    /// Resets scanline state and sorts local minima before an execution pass.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void ResetState()
    {
        this.scanlineSchedule.Reset();
        this.currentScanlineBottomY = 0;
        this.activeEdges.Reset();
        this.succeeded = true;
    }

    /// <summary>
    /// Adds subject contours for the union operation.
    /// </summary>
    /// <param name="paths">The subject contours to add.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void AddSubject(List<List<Vertex64>> paths)
    {
        this.scanlineSchedule.MarkDirty();
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
        // Guard against registering the same vertex twice.
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
    private void AddPathsToVertexList(List<List<Vertex64>> paths)
    {
        int totalVertCnt = 0;
        foreach (List<Vertex64> path in paths)
        {
            totalVertCnt += path.Count;
        }

        // Pre-size the pool to avoid growth during vertex creation.
        this.vertexList.EnsureCapacity(this.vertexList.Count + totalVertCnt);

        foreach (List<Vertex64> path in paths)
        {
            SweepVertex? v0 = null;
            SweepVertex? prevVertex = null;
            SweepVertex? currVertex;
            foreach (Vertex64 point in path)
            {
                if (v0 == null)
                {
                    v0 = this.vertexList.Add(point, VertexFlags.None, null);
                    prevVertex = v0;
                    continue;
                }

                if (prevVertex!.Point != point)
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

            if (prevVertex.Point == v0.Point)
            {
                prevVertex = prevVertex.Prev;
            }

            prevVertex.Next = v0;
            v0.Prev = prevVertex;
            if (prevVertex.Next == prevVertex)
            {
                continue;
            }

            // Non-degenerate closed ring.
            prevVertex = v0.Prev;
            while (prevVertex != v0 && prevVertex!.Point.Y == v0.Point.Y)
            {
                prevVertex = prevVertex.Prev;
            }

            if (prevVertex == v0)
            {
                // Closed rings cannot be completely flat.
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
                    RegisterLocalMinima(prevVertex, this.scanlineSchedule.LocalMinima);
                }

                prevVertex = currVertex;
                currVertex = currVertex.Next;
            }

            if (goingUp != goingUp0)
            {
                if (goingUp0)
                {
                    RegisterLocalMinima(prevVertex, this.scanlineSchedule.LocalMinima);
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
        // Winding counts apply to regions, not edges. The edge wind count tracks the
        // higher of the two adjacent region counts. Adjacent regions differ by one.
        ActiveEdge? edge2 = edge.PrevInAel;

        if (edge2 == null)
        {
            edge.WindCount = edge.WindDelta;
        }
        else
        {
            // For positive/negative filling, if edge2's wind count follows its wind delta,
            // the filled region is to the right of edge2 (so edge is inside). Neither value is 0.
            if (edge2.WindCount * edge2.WindDelta < 0)
            {
                // Opposite signs: edge lies outside edge2's region.
                if (Math.Abs(edge2.WindCount) > 1)
                {
                    // Outside this polygon but still inside another.
                    if (edge2.WindDelta * edge.WindDelta < 0)
                    {
                        // Reversing direction; keep the same winding count.
                        edge.WindCount = edge2.WindCount;
                    }
                    else
                    {
                        // Otherwise step the winding count toward zero.
                        edge.WindCount = edge2.WindCount + edge.WindDelta;
                    }
                }
                else
                {
                    // Outside all polygons; reset to the edge's own winding.
                    edge.WindCount = edge.WindDelta;
                }
            }
            else
            {
                // Same sign: edge lies inside edge2's region.
                if (edge2.WindDelta * edge.WindDelta < 0)
                {
                    // Reversing direction; keep the same winding count.
                    edge.WindCount = edge2.WindCount;
                }
                else
                {
                    // Otherwise step the winding count away from zero.
                    edge.WindCount = edge2.WindCount + edge.WindDelta;
                }
            }
        }
    }

    /// <summary>
    /// Inserts any local minima that occur at the current scanline into the active list.
    /// </summary>
    /// <param name="botY">The current scanline Y coordinate.</param>
    private void InsertLocalMinimaIntoActiveList(long botY)
    {
        // Insert all minima on the current scanline.
        // Horizontal minima use the previous vertex as the descending bound.
        while (this.scanlineSchedule.HasLocalMinimaAtY(botY))
        {
            LocalMinima localMinima = this.scanlineSchedule.PopLocalMinima();
            ActiveEdge leftBound = this.activeEdges.Acquire();
            leftBound.Bottom = localMinima.Vertex.Point;
            leftBound.CurrentX = localMinima.Vertex.Point.X;
            leftBound.WindDelta = -1;
            leftBound.VertexTop = localMinima.Vertex.Prev;
            leftBound.Top = localMinima.Vertex.Prev!.Point;
            leftBound.OutputRecord = null;
            leftBound.LocalMin = localMinima;
            leftBound.UpdateDx();

            ActiveEdge rightBound = this.activeEdges.Acquire();
            rightBound.Bottom = localMinima.Vertex.Point;
            rightBound.CurrentX = localMinima.Vertex.Point.X;
            rightBound.WindDelta = 1;

            // Ascending bound.
            rightBound.VertexTop = localMinima.Vertex.Next;
            rightBound.Top = localMinima.Vertex.Next!.Point;
            rightBound.OutputRecord = null;
            rightBound.LocalMin = localMinima;
            rightBound.UpdateDx();

            // leftBound starts descending and rightBound ascending.
            // Swap them if their geometric ordering is inverted.
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
            this.activeEdges.InsertLeft(leftBound);

            SetWindingCountForClosedEdge(leftBound);
            contributing = this.IsContributingClosedEdge(leftBound);

            rightBound.WindCount = leftBound.WindCount;
            ActiveEdgeList.InsertRight(leftBound, rightBound);

            if (contributing)
            {
                this.AddLocalMinimumOutput(leftBound, rightBound, leftBound.Bottom, true);
                if (!leftBound.IsHorizontal)
                {
                    this.CheckJoinLeft(leftBound, leftBound.Bottom);
                }
            }

            while (rightBound.NextInAel != null &&
                          ActiveEdgeList.IsValidActiveEdgeOrder(rightBound.NextInAel, rightBound))
            {
                this.IntersectActiveEdges(rightBound, rightBound.NextInAel, rightBound.Bottom);
                this.activeEdges.SwapPositions(rightBound, rightBound.NextInAel);
            }

            if (rightBound.IsHorizontal)
            {
                this.activeEdges.PushHorizontal(rightBound);
            }
            else
            {
                this.CheckJoinRight(rightBound, rightBound.Bottom);
                this.scanlineSchedule.InsertScanline(rightBound.Top.Y);
            }

            if (leftBound.IsHorizontal)
            {
                this.activeEdges.PushHorizontal(leftBound);
            }
            else
            {
                this.scanlineSchedule.InsertScanline(leftBound.Top.Y);
            }
        }
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
    private OutputPoint AddLocalMinimumOutput(ActiveEdge edge1, ActiveEdge edge2, Vertex64 point, bool isNew = false)
    {
        OutputRecord outputRecord = this.CreateOutputRecord();
        edge1.OutputRecord = outputRecord;
        edge2.OutputRecord = outputRecord;

        ActiveEdge? prevHotEdge = edge1.GetPrevHotEdge();

        // WindDelta reflects input winding, not output orientation.
        // Output orientation is driven by which edge is assigned as the front (ascending) edge.
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
    private OutputPoint? AddLocalMaximumOutput(ActiveEdge edge1, ActiveEdge edge2, Vertex64 point)
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

                // Owner assignment here is provisional and will be resolved later.
            }

            DetachOutputRecord(edge1);
        }

        // Join in index order to preserve output orientation.
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
        // Append edge2's path onto edge1's path, then discard edge2's path pointers.
        // The joining ends rarely share coordinates, so pointer swaps are safe.
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

        // After joining, edge2's output record contains no vertices.
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
    private OutputPoint AddOutputPoint(ActiveEdge edge, Vertex64 point)
    {
        // outputRecord.Points is a circular list; Points is the front point and
        // Points.Next is the back point for this output record.
        OutputRecord outputRecord = edge.OutputRecord!;
        bool toFront = edge.IsFront;
        OutputPoint opFront = outputRecord.Points!;
        OutputPoint opBack = opFront.Next!;

        switch (toFront)
        {
            case true when point == opFront.Point:
                return opFront;
            case false when point == opBack.Point:
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
            // Split joined edges before advancing to avoid missing intersections.
            this.SplitEdge(edge, edge.Bottom);
        }

        if (edge.IsHorizontal)
        {
            TrimHorizontal(edge, this.PreserveCollinear);

            return;
        }

        this.scanlineSchedule.InsertScanline(edge.Top.Y);

        this.CheckJoinLeft(edge, edge.Bottom);

        // Issue #500: check join on the right bound at the bottom point.
        this.CheckJoinRight(edge, edge.Bottom, true);
    }

    /// <summary>
    /// Handles an intersection between two active edges at a given point.
    /// </summary>
    /// <param name="edge1">The first intersecting edge.</param>
    /// <param name="edge2">The second intersecting edge.</param>
    /// <param name="point">The intersection point.</param>
    private void IntersectActiveEdges(ActiveEdge edge1, ActiveEdge edge2, Vertex64 point)
    {
        if (IsJoined(edge1))
        {
            this.SplitEdge(edge1, point);
        }

        if (IsJoined(edge2))
        {
            this.SplitEdge(edge2, point);
        }

        // Update winding counts for both edges.
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

        // Emit output based on hot edges and winding state.

        // If both edges are hot, treat as maxima or crossing.
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
                // Treat as a crossing; emit and swap output records.
                this.AddOutputPoint(edge1, point);
                SwapOutputRecords(edge1, edge2);
            }
        }

        // If only one edge is hot, emit and swap.
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

        // If both edges are cold, only minima with winding=1 start output.
        else
        {
            if (oldE1WindCount == 1 && oldE2WindCount == 1)
            {
                this.AddLocalMinimumOutput(edge1, edge2, point);
            }
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
        if (!this.scanlineSchedule.TryPopScanline(out long y))
        {
            return;
        }

        // Process each scanbeam: insert local minima, handle horizontals, resolve intersections,
        // then advance to the next scanline.
        while (this.succeeded)
        {
            this.InsertLocalMinimaIntoActiveList(y);
            ActiveEdge? edge;
            while (this.activeEdges.TryPopHorizontal(out edge))
            {
                this.ProcessHorizontal(edge!);
            }

            if (this.horizontalSegments.Count > 0)
            {
                this.ConvertHorizontalSegmentsToJoins();
                this.horizontalSegments.Clear();
            }

            // Advance to the next scanbeam.
            this.currentScanlineBottomY = y;
            if (!this.scanlineSchedule.TryPopScanline(out y))
            {
                // y is now the new top of the scanbeam.
                break;
            }

            this.ProcessIntersections(y);
            this.ProcessScanbeamTop(y);
            while (this.activeEdges.TryPopHorizontal(out edge))
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
    /// Executes the sweep-line union, leaving output records populated for conversion.
    /// </summary>
    /// <param name="fillRule">The fill rule used to resolve inside/outside.</param>
    /// <param name="buildHierarchy">Whether hierarchy-sensitive output ownership is required.</param>
    /// <returns><see langword="true"/> if the sweep completed successfully.</returns>
    public bool Execute(FillRule fillRule, bool buildHierarchy)
    {
        this.buildHierarchy = buildHierarchy;
        try
        {
            this.ExecuteInternal(fillRule);
        }
        catch
        {
            this.succeeded = false;
        }

        return this.succeeded;
    }

    /// <summary>
    /// Builds and processes edge intersections for the current scanbeam.
    /// </summary>
    /// <param name="topY">The scanbeam top Y coordinate.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void ProcessIntersections(long topY)
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
    private void AddIntersectionNode(ActiveEdge edge1, ActiveEdge edge2, long topY)
    {
        if (!PolygonUtilities.TryGetLineIntersection(
            edge1.Bottom, edge1.Top, edge2.Bottom, edge2.Top, out Vertex64 intersectionPoint))
        {
            intersectionPoint = new Vertex64(edge1.CurrentX, topY);
        }

        // Clamp intersections that drift outside the scanbeam due to numeric error.
        if (intersectionPoint.Y > this.currentScanlineBottomY || intersectionPoint.Y < topY)
        {
            double absDx1 = Math.Abs(edge1.Dx);
            double absDx2 = Math.Abs(edge2.Dx);

            // dx is dX/dY, so large magnitudes mean the edge is nearly horizontal (dY is tiny).
            // Using TopX with a clamped Y can amplify floating-point error in that case, so we
            // fall back to closest-point clamping when |dx| > 100 (about 0.57 degrees from horizontal).
            // This threshold keeps near-horizontal intersections stable without scaling the input.
            switch (absDx1 > 100)
            {
                case true when absDx2 > 100:
                {
                    intersectionPoint = absDx1 > absDx2
                        ? PolygonUtilities.ClosestPointOnSegment(intersectionPoint, edge1.Bottom, edge1.Top)
                        : PolygonUtilities.ClosestPointOnSegment(intersectionPoint, edge2.Bottom, edge2.Top);

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
                        long targetY = intersectionPoint.Y < topY ? topY : this.currentScanlineBottomY;
                        long targetX = absDx1 < absDx2 ? edge1.TopX(targetY) : edge2.TopX(targetY);
                        intersectionPoint = new Vertex64(targetX, targetY);
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
    private bool BuildIntersectionList(long topY)
    {
        if (this.activeEdges.Head?.NextInAel == null)
        {
            return false;
        }

        // Compute edge positions at the top of the scanbeam to derive required intersections.
        ActiveEdge? sortedHead = this.activeEdges.CopyToSorted(topY);

        // Find intersections via a stable merge sort so only adjacent edges intersect.
        // Nodes are stored for ProcessIntersectionList. See https://stackoverflow.com/a/46319131/359538.
        ActiveEdge? left = sortedHead;

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
                            sortedHead = currBase;
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

            left = sortedHead;
        }

        return this.intersectionList.Count > 0;
    }

    /// <summary>
    /// Processes the intersection list in bottom-up order, swapping edges and generating output.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void ProcessIntersectionList()
    {
        // Intersections must be processed bottom-up, and only between adjacent edges.

        // Sort so intersections proceed from bottom to top.
        this.intersectionList.Sort(default(IntersectNodeComparer));

        // Reorder as needed to ensure intersecting edges are adjacent.
        for (int i = 0; i < this.intersectionList.Count; ++i)
        {
            if (!AreEdgesAdjacentInActiveList(this.intersectionList[i]))
            {
                int j = i + 1;
                while (!AreEdgesAdjacentInActiveList(this.intersectionList[j]))
                {
                    j++;
                }

                // Swap into adjacency.
                (this.intersectionList[j], this.intersectionList[i]) =
                    (this.intersectionList[i], this.intersectionList[j]);
            }

            IntersectNode node = this.intersectionList[i];
            this.IntersectActiveEdges(node.Edge1, node.Edge2, node.Point);
            this.activeEdges.SwapPositions(node.Edge1, node.Edge2);

            node.Edge1.CurrentX = node.Point.X;
            node.Edge2.CurrentX = node.Point.X;
            this.CheckJoinLeft(node.Edge2, node.Point, true);
            this.CheckJoinRight(node.Edge1, node.Point, true);
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
        out long leftX,
        out long rightX)
    {
        if (horizontalEdge.Bottom.X == horizontalEdge.Top.X)
        {
            // Degenerate horizontal edge (zero length).
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
        Vertex64 point = horizontalEdge.NextVertex.Point;

        while (point.Y == horizontalEdge.Top.Y)
        {
            // Always trim 180-degree spikes in closed paths; otherwise stop when preserving collinear.
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
            // Recompute slope after trimming.
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
        long y = horizontalEdge.Bottom.Y;

        SweepVertex? vertexMax = GetMaximaVertexAtCurrentY(horizontalEdge);

        bool isLeftToRight =
            ResetHorizontalDirection(horizontalEdge, vertexMax, out long leftX, out long rightX);

        if (horizontalEdge.IsHot)
        {
            OutputPoint outputPoint = this.AddOutputPoint(horizontalEdge, new Vertex64(horizontalEdge.CurrentX, y));
            this.AddHorizontalSegment(outputPoint);
        }

        while (true)
        {
            // Traverse consecutive horizontal edges on this scanline.
            ActiveEdge? edge = isLeftToRight ? horizontalEdge.NextInAel : horizontalEdge.PrevInAel;

            while (edge != null)
            {
                if (edge.VertexTop == vertexMax)
                {
                    // Handle the maxima pair before processing other intersections.
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

                    this.activeEdges.Remove(edge);
                    this.activeEdges.Remove(horizontalEdge);
                    return;
                }

                // If this horizontal is a maxima, keep going until its pair is reached;
                // otherwise check for break conditions.
                Vertex64 point;
                if (vertexMax != horizontalEdge.VertexTop)
                {
                    // Stop once the edge moves beyond the horizontal span.
                    if ((isLeftToRight && edge.CurrentX > rightX) ||
                            (!isLeftToRight && edge.CurrentX < leftX))
                    {
                        break;
                    }

                    if (edge.CurrentX == horizontalEdge.Top.X && !edge.IsHorizontal)
                    {
                        point = horizontalEdge.NextVertex.Point;

                        // At the horizontal end, stop only when the outslope overtakes the edge
                        // (greater when heading right, smaller when heading left).
                        if ((isLeftToRight && (edge.TopX(point.Y) >= point.X)) ||
                                (!isLeftToRight && (edge.TopX(point.Y) <= point.X)))
                        {
                            break;
                        }
                    }
                }

                point = new Vertex64(edge.CurrentX, y);

                if (isLeftToRight)
                {
                    this.IntersectActiveEdges(horizontalEdge, edge, point);
                    this.activeEdges.SwapPositions(horizontalEdge, edge);
                    this.CheckJoinLeft(edge, point);
                    horizontalEdge.CurrentX = edge.CurrentX;
                    edge = horizontalEdge.NextInAel;
                }
                else
                {
                    this.IntersectActiveEdges(edge, horizontalEdge, point);
                    this.activeEdges.SwapPositions(edge, horizontalEdge);
                    this.CheckJoinRight(edge, point);
                    horizontalEdge.CurrentX = edge.CurrentX;
                    edge = horizontalEdge.PrevInAel;
                }

                if (horizontalEdge.IsHot)
                {
                    this.AddHorizontalSegment(GetLastOutputPoint(horizontalEdge));
                }
            }

            // Stop once no more consecutive horizontals remain.
            if (horizontalEdge.NextVertex.Point.Y != horizontalEdge.Top.Y)
            {
                break;
            }

            // Advance to the next horizontal segment in the bound.
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

        // Finished this horizontal chain.
        if (horizontalEdge.IsHot)
        {
            OutputPoint outputPoint = this.AddOutputPoint(horizontalEdge, horizontalEdge.Top);
            this.AddHorizontalSegment(outputPoint);
        }

        // Advance past the final intermediate horizontal.
        this.UpdateEdgeInActiveList(horizontalEdge);
    }

    /// <summary>
    /// Processes edges that reach the top of the scanbeam, updating or removing them.
    /// </summary>
    /// <param name="y">The scanbeam top Y coordinate.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void ProcessScanbeamTop(long y)
    {
        this.activeEdges.ClearHorizontalQueue();
        ActiveEdge? edge = this.activeEdges.Head;
        while (edge != null)
        {
            // Edge is never horizontal at this point.
            if (edge.Top.Y == y)
            {
                edge.CurrentX = edge.Top.X;
                if (edge.IsMaxima)
                {
                    // Maxima reached; finalize this bound.
                    edge = this.ProcessMaxima(edge);
                    continue;
                }

                // Intermediate vertex on the bound.
                if (edge.IsHot)
                {
                    this.AddOutputPoint(edge, edge.Top);
                }

                this.UpdateEdgeInActiveList(edge);

                // Queue horizontals for dedicated processing.
                if (edge.IsHorizontal)
                {
                    this.activeEdges.PushHorizontal(edge);
                }
            }

            // Edge continues through the scanbeam.
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
            // Horizontal maxima pair is handled in horizontal processing.
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

        // Only non-horizontal maxima reach here.
        // Process edges between the maxima pair.
        while (nextEdge != maxPair)
        {
            this.IntersectActiveEdges(edge, nextEdge!, edge.Top);
            this.activeEdges.SwapPositions(edge, nextEdge!);
            nextEdge = edge.NextInAel;
        }

        // At this point edge.NextInAel == maxPair.
        if (edge.IsHot)
        {
            this.AddLocalMaximumOutput(edge, maxPair, edge.Top);
        }

        this.activeEdges.Remove(edge);
        this.activeEdges.Remove(maxPair);
        return prevEdge != null ? prevEdge.NextInAel : this.activeEdges.Head;
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
    private void SplitEdge(ActiveEdge edge, Vertex64 point)
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
        Vertex64 point,
        bool checkCurrX = false)
    {
        ActiveEdge? prev = edge.PrevInAel;
        if (prev == null ||
            !edge.IsHot || !prev.IsHot ||
            edge.IsHorizontal || prev.IsHorizontal)
        {
            return;
        }

        // Reject joins that are too close to extrema (Issue #490).
        if ((point.Y < edge.Top.Y + 2 || point.Y < prev.Top.Y + 2) &&
            ((edge.Bottom.Y > point.Y) || (prev.Bottom.Y > point.Y)))
        {
            // Issue #490.
            return;
        }

        if (checkCurrX)
        {
            if (PolygonUtilities.PerpendicularDistanceSquared(point, prev.Bottom, prev.Top) > 0.25D)
            {
                return;
            }
        }
        else if (edge.CurrentX != prev.CurrentX)
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
        Vertex64 point,
        bool checkCurrX = false)
    {
        ActiveEdge? next = edge.NextInAel;
        if (next == null ||
            !edge.IsHot || !next.IsHot ||
            edge.IsHorizontal || next.IsHorizontal)
        {
            return;
        }

        // Reject joins that are too close to extrema (Issue #490).
        if ((point.Y < edge.Top.Y + 2 || point.Y < next.Top.Y + 2) &&
            ((edge.Bottom.Y > point.Y) || (next.Bottom.Y > point.Y)))
        {
            // Issue #490.
            return;
        }

        if (checkCurrX)
        {
            if (PolygonUtilities.PerpendicularDistanceSquared(point, next.Bottom, next.Top) > 0.25D)
            {
                return;
            }
        }
        else if (edge.CurrentX != next.CurrentX)
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
        if (prevPoint.Point.X == nextPoint.Point.X)
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
        long currentY = outputPoint.Point.Y;
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
            // Mark invalid so sorting pushes it to the end.
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

            // Find overlapping segments to generate join candidates.
            for (int j = i + 1; j < k; j++)
            {
                HorizontalSegment segment2 = this.horizontalSegments[j];
                if ((segment2.LeftPoint!.Point.X >= segment1.RightPoint!.Point.X) ||
                    (segment2.LeftToRight == segment1.LeftToRight) ||
                    (segment2.RightPoint!.Point.X <= segment1.LeftPoint!.Point.X))
                {
                    continue;
                }

                long currentY = segment1.LeftPoint.Point.Y;
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
    private static List<Vertex64> BuildCleanContour(OutputPoint outputPoint)
    {
        List<Vertex64> result = [];
        OutputPoint outputPoint2 = outputPoint;
        while (outputPoint2.Next != outputPoint &&
            ((outputPoint2.Point.X == outputPoint2.Next!.Point.X &&
              outputPoint2.Point.X == outputPoint2.Prev.Point.X) ||
             (outputPoint2.Point.Y == outputPoint2.Next.Point.Y &&
              outputPoint2.Point.Y == outputPoint2.Prev.Point.Y)))
        {
            outputPoint2 = outputPoint2.Next;
        }

        result.Add(outputPoint2.Point);
        OutputPoint prevOp = outputPoint2;
        outputPoint2 = outputPoint2.Next;
        while (outputPoint2 != outputPoint)
        {
            if ((outputPoint2.Point.X != outputPoint2.Next!.Point.X || outputPoint2.Point.X != prevOp.Point.X) &&
                (outputPoint2.Point.Y != outputPoint2.Next.Point.Y || outputPoint2.Point.Y != prevOp.Point.Y))
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
    private static PointInPolygonResult PointInOutputPolygon(Vertex64 point, OutputPoint outputPoint)
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

        // Degenerate ring.
        if (outputPoint.Point.Y == point.Y)
        {
            return PointInPolygonResult.Outside;
        }

        // Point is strictly above or below the starting Y.
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

            // The scanline must touch or cross point.Y an even number of times.
            // Handle horizontal touches explicitly.
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
                    // Toggle parity.
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
    public static bool IsPathInsidePath(OutputPoint outputPoint1, OutputPoint outputPoint2)
    {
        // Allow for rounding error; don't decide based solely on the first vertex.
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

        // Result is unclear, so try again using cleaned paths (Issue #973).
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

            // This join may split a single output record.
            if (outputRecord1 == outputRecord2)
            {
                outputRecord2 = this.CreateOutputRecord();
                outputRecord2.Points = op1b;
                FixOutputRecordPoints(outputRecord2);

                // If outputRecord1.Points moved to outputRecord2, update outputRecord1.Points.
                if (outputRecord1.Points!.OutputRecord == outputRecord2)
                {
                    outputRecord1.Points = join.LeftToRight;
                    outputRecord1.Points.OutputRecord = outputRecord1;
                }

                // Issue references: #498, #520, #584, #576, #618
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

                    // Issue #618.
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
            Vertex64 delta = a.Point - b.Point;
            if (delta.Y != 0)
            {
                return delta.Y > 0 ? -1 : 1;
            }

            if (delta.X == 0)
            {
                return 0;
            }

            return delta.X < 0 ? -1 : 1;
        }
    }
}

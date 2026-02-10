// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System.Runtime.CompilerServices;

namespace SixLabors.PolygonClipper;

/// <summary>
/// Builds output contours and polygon hierarchies for self-intersection removal.
/// </summary>
internal sealed class SelfIntersectionUnionClipper
{
    private readonly SelfIntersectionSweepLine sweepLine;
    private bool buildHierarchy;

    /// <summary>
    /// Initializes a new instance of the <see cref="SelfIntersectionUnionClipper"/> class.
    /// </summary>
    internal SelfIntersectionUnionClipper() => this.sweepLine = new SelfIntersectionSweepLine();

    /// <summary>
    /// Gets or sets a value indicating whether collinear output points are preserved.
    /// </summary>
    internal bool PreserveCollinear
    {
        get => this.sweepLine.PreserveCollinear;
        set => this.sweepLine.PreserveCollinear = value;
    }

    /// <summary>
    /// Gets or sets a value indicating whether the output orientation is reversed.
    /// </summary>
    internal bool ReverseSolution { get; set; }

    /// <summary>
    /// Clears all cached input and output data.
    /// </summary>
    internal void Clear() => this.sweepLine.Clear();

    /// <summary>
    /// Adds subject contours to the sweep-line clipper.
    /// </summary>
    /// <param name="paths">The subject contours to add.</param>
    internal void AddSubject(List<Contour> paths) => this.sweepLine.AddSubject(paths);

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
        Vertex delta = Vertex.Abs(firstPoint - secondPoint);
        return delta.X < tolerance && delta.Y < tolerance;
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

        return result;
    }

    /// <summary>
    /// Creates a new output record with the next stable index.
    /// </summary>
    /// <returns>The created output record.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private OutputRecord CreateOutputRecord()
    {
        int idx = this.sweepLine.OutputRecords.Count;
        OutputRecord result = this.sweepLine.OutputRecords.Add();
        result.Index = idx;
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
        OutputPoint result = this.sweepLine.OutputPoints.Add(outputPoint.Point, outputPoint.OutputRecord);
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
    /// Removes collinear points and resolves self-intersections in an output record.
    /// </summary>
    /// <param name="outputRecord">The output record to clean.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void CleanCollinearEdges(OutputRecord? outputRecord)
    {
        outputRecord = SelfIntersectionSweepLine.ResolveOutputRecord(outputRecord);

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
            // When preserving collinear, only remove 180-degree spikes.
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
        // The segments (splitOp.Prev, splitOp) and (splitOp.Next, splitOp.Next.Next) intersect.
        OutputPoint prevOp = splitOp.Prev;
        OutputPoint nextNextOp = splitOp.Next!.Next!;
        outputRecord.Points = prevOp;

        PolygonUtilities.TryGetLineIntersection(
            prevOp.Point, splitOp.Point, splitOp.Next.Point, nextNextOp.Point, out Vertex intersectionPoint);

        double area1 = SelfIntersectionSweepLine.ComputeSignedArea(prevOp);
        double absArea1 = Math.Abs(area1);

        if (absArea1 < PolygonUtilities.SmallAreaTolerance2)
        {
            outputRecord.Points = null;
            return;
        }

        double area2 = PolygonUtilities.SignedArea(intersectionPoint, splitOp.Point, splitOp.Next.Point);
        double absArea2 = Math.Abs(area2);

        // Remove the crossing segment and insert the intersection point.
        if (PolygonUtilities.PointEquals(intersectionPoint, prevOp.Point) || PolygonUtilities.PointEquals(intersectionPoint, nextNextOp.Point))
        {
            nextNextOp.Prev = prevOp;
            prevOp.Next = nextNextOp;
        }
        else
        {
            OutputPoint newOp2 = this.sweepLine.OutputPoints.Add(intersectionPoint, outputRecord);
            newOp2.Prev = prevOp;
            newOp2.Next = nextNextOp;
            nextNextOp.Prev = newOp2;
            prevOp.Next = newOp2;
        }

        // Note: area1 is the path's signed area *before* splitting, whereas area2 is
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

        OutputPoint newOp = this.sweepLine.OutputPoints.Add(intersectionPoint, newOutputRecord);
        newOp.Prev = splitOp.Next;
        newOp.Next = splitOp;
        newOutputRecord.Points = newOp;
        splitOp.Prev = newOp;
        splitOp.Next.Next = newOp;

        if (!this.buildHierarchy)
        {
            return;
        }

        if (SelfIntersectionSweepLine.IsPathInsidePath(prevOp, newOp))
        {
            newOutputRecord.Splits ??= [];
            newOutputRecord.Splits.Add(outputRecord.Index);
        }
        else
        {
            outputRecord.Splits ??= [];
            outputRecord.Splits.Add(newOutputRecord.Index);
        }
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
            // Triangles cannot self-intersect.
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
                    // Adjacent intersections (micro self-intersection).
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

                    // Triangles cannot self-intersect.
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
            Vertex current = new(currentPoint.Point.X, currentPoint.Point.Y);
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
        solution.EnsureCapacity(this.sweepLine.OutputRecords.Count);

        int i = 0;
        while (i < this.sweepLine.OutputRecords.Count)
        {
            OutputRecord outputRecord = this.sweepLine.OutputRecords[i++];
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
        // Use indexing because splits can be modified during iteration (Issue #1029).
        for (int i = 0; i < splits!.Count; i++)
        {
            OutputRecord? splitRecord = this.sweepLine.OutputRecords[splits[i]];
            if (splitRecord.Points == null && splitRecord.Splits != null &&
                this.CheckSplitOwner(outputRecord, splitRecord.Splits))
            {
                // Issue #942.
                return true;
            }

            splitRecord = SelfIntersectionSweepLine.ResolveOutputRecord(splitRecord);
            if (splitRecord == null || splitRecord == outputRecord || splitRecord.RecursiveSplit == outputRecord)
            {
                continue;
            }

            // Issue #599.
            splitRecord.RecursiveSplit = outputRecord;

            if (splitRecord.Splits != null && this.CheckSplitOwner(outputRecord, splitRecord.Splits))
            {
                return true;
            }

            if (!this.CheckOutputBounds(splitRecord) ||
                    !splitRecord.Bounds.Contains(outputRecord.Bounds) ||
                    !SelfIntersectionSweepLine.IsPathInsidePath(outputRecord.Points!, splitRecord.Points!))
            {
                continue;
            }

            // splitRecord is owned by outputRecord (Issue #957).
            if (!SelfIntersectionSweepLine.IsOwnerValid(outputRecord, splitRecord))
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
                SelfIntersectionSweepLine.IsPathInsidePath(outputRecord.Points!, outputRecord.Owner.Points!))
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
        List<OutputRecord> closedOutputRecords = new(this.sweepLine.OutputRecords.Count);

        int i = 0;

        // outputRecordPool.Count may change because CheckOutputBounds can add records
        // via FixOutputRecordPoints and CleanCollinearEdges.
        while (i < this.sweepLine.OutputRecords.Count)
        {
            OutputRecord outputRecord = this.sweepLine.OutputRecords[i++];
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

            // Depth is the number of owning contours in the chain.
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
                // Map parent links to hole indices for quick traversal.
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
        bool succeeded = this.sweepLine.Execute(fillRule, true);
        if (succeeded)
        {
            this.BuildPolygon(polygon);
        }

        this.sweepLine.ClearSolutionData();
        return succeeded;
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
        bool succeeded = this.sweepLine.Execute(fillRule, false);
        if (succeeded)
        {
            this.BuildContours(solution);
        }

        this.sweepLine.ClearSolutionData();
        return succeeded;
    }
}

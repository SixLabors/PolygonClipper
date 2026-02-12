// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System.Runtime.CompilerServices;

namespace SixLabors.PolygonClipper;

/// <summary>
/// Provides functionality to remove self-intersections from polygons using a sweep line algorithm.
/// </summary>
/// <remarks>
/// <para>
/// This class implements a sweep line algorithm that applies the requested fill rule
/// to resolve self-intersections within a polygon.
/// </para>
/// <para>
/// The algorithm works in three phases:
/// </para>
/// <list type="number">
/// <item><description>
/// <b>Intersection Detection:</b> Uses a sweep line to find all points where segments
/// intersect each other (both self-intersections and cross-contour intersections).
/// </description></item>
/// <item><description>
/// <b>Segment Splitting:</b> Divides segments at intersection points, creating new
/// vertices where crossings occur.
/// </description></item>
/// <item><description>
/// <b>Boundary Extraction:</b> Evaluates the chosen fill rule and keeps only edges that
/// form the boundary between filled and unfilled regions.
/// </description></item>
/// </list>
/// </remarks>
internal static class SelfIntersectionRemover
{
    /// <summary>
    /// Processes a polygon to remove self-intersections.
    /// </summary>
    /// <param name="polygon">The polygon to process.</param>
    /// <param name="options">
    /// Optional fixed-precision options for quantization and fill rule override.
    /// </param>
    /// <returns>
    /// A new <see cref="Polygon"/> with self-intersections resolved. Regions considered
    /// filled by the effective fill rule are preserved.
    /// </returns>
    public static Polygon Process(Polygon polygon, ClipperOptions? options = null)
    {
        if (polygon.Count == 0)
        {
            return [];
        }

        FixedPrecisionContext context = FixedPrecisionContext.Create(options, [polygon]);
        bool hasExplicitFillRule = options?.FillRuleOverride is FillRule;
        List<List<Vertex64>> subject = BuildSubjectPaths(polygon, context, !hasExplicitFillRule);
        FillRule effectiveFillRule = options?.FillRuleOverride ?? FillRule.Positive;
        bool reverseSolution = false;

        // When no explicit rule is provided, normalize contours for positive fill and
        // infer the final positive/negative winding orientation from the outer contour.
        if (!hasExplicitFillRule)
        {
            GetLowestPathInfo(subject, out int lowestPathIdx, out bool isNegativeArea);
            reverseSolution = lowestPathIdx >= 0 && isNegativeArea;
            effectiveFillRule = reverseSolution ? FillRule.Negative : FillRule.Positive;
        }

        return UnionWithClipper(subject, effectiveFillRule, context, polygon.Count, reverseSolution);
    }

    /// <summary>
    /// Executes a union using the internal clipper with the specified fill rule.
    /// </summary>
    /// <param name="subject">The quantized subject contours to union.</param>
    /// <param name="fillRule">The fill rule used to resolve inside/outside.</param>
    /// <param name="context">The fixed-precision context.</param>
    /// <param name="resultCapacity">The initial contour capacity for the output polygon.</param>
    /// <param name="reverseSolution">Whether output contour orientation should be reversed.</param>
    /// <returns>A polygon containing the unioned contours.</returns>
    private static Polygon UnionWithClipper(
        List<List<Vertex64>> subject,
        FillRule fillRule,
        FixedPrecisionContext context,
        int resultCapacity,
        bool reverseSolution)
    {
        OutputBuilder builder = new();
        builder.SetContext(context);
        builder.PreserveCollinear = false;
        builder.ReverseSolution = reverseSolution;
        builder.AddSubject(subject);

        Polygon result = new(resultCapacity);
        builder.Execute(fillRule, result);
        return result;
    }

    /// <summary>
    /// Determines the lowest point across all paths and whether its contour area is negative.
    /// </summary>
    /// <param name="paths">The paths to examine.</param>
    /// <param name="lowestPathIdx">The index of the path containing the lowest point.</param>
    /// <param name="isNegativeArea">True when the lowest path has negative area.</param>
    private static void GetLowestPathInfo(List<List<Vertex64>> paths, out int lowestPathIdx, out bool isNegativeArea)
    {
        lowestPathIdx = -1;
        isNegativeArea = false;

        if (paths.Count == 0)
        {
            return;
        }

        Vertex64 lowestPoint = default;
        bool hasPoint = false;
        for (int i = 0; i < paths.Count; i++)
        {
            List<Vertex64> path = paths[i];
            if (path.Count == 0)
            {
                continue;
            }

            Vertex64 candidate = GetLowestPoint(path);
            if (!hasPoint || candidate.Y > lowestPoint.Y || (candidate.Y == lowestPoint.Y && candidate.X < lowestPoint.X))
            {
                lowestPoint = candidate;
                lowestPathIdx = i;
                hasPoint = true;
            }
        }

        if (lowestPathIdx >= 0)
        {
            isNegativeArea = GetSignedArea(paths[lowestPathIdx]) < 0D;
        }
    }

    private static Vertex64 GetLowestPoint(List<Vertex64> path)
    {
        int count = path.Count;
        int lastIndex = count - 1;
        if (count > 1 && path[0] == path[^1])
        {
            lastIndex = count - 2;
        }

        Vertex64 lowest = path[0];
        for (int i = 1; i <= lastIndex; i++)
        {
            Vertex64 candidate = path[i];
            if (candidate.Y > lowest.Y || (candidate.Y == lowest.Y && candidate.X < lowest.X))
            {
                lowest = candidate;
            }
        }

        return lowest;
    }

    private static int GetDepth(int index, int[] parentIndices)
    {
        int depth = 0;
        int current = parentIndices[index];
        while (current >= 0)
        {
            depth++;
            current = parentIndices[current];
        }

        return depth;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vertex64 GetContourTestPoint(List<Vertex64> contour)
    {
        if (contour.Count == 0)
        {
            return default;
        }

        Vertex64 first = contour[0];
        if (contour.Count > 1 && first == contour[^1])
        {
            return contour[1];
        }

        return first;
    }

    private static double GetSignedArea(List<Vertex64> contour)
    {
        Int128 area = 0;
        for (int i = 0; i < contour.Count; i++)
        {
            Vertex64 current = contour[i];
            Vertex64 next = contour[(i + 1) % contour.Count];
            area += Vertex64.Cross(current, next);
        }

        return (double)area * 0.5D;
    }

    private static bool HasSelfIntersection(List<Vertex64> contour)
    {
        int vertexCount = contour.Count - 1;
        if (vertexCount < 4)
        {
            return false;
        }

        for (int i = 0; i < vertexCount; i++)
        {
            Vertex64 segA1 = contour[i];
            Vertex64 segA2 = contour[i + 1];
            if (segA1 == segA2)
            {
                continue;
            }

            for (int j = i + 1; j < vertexCount; j++)
            {
                if (j == i || j == i + 1 || (i == 0 && j == vertexCount - 1))
                {
                    continue;
                }

                Vertex64 segB1 = contour[j];
                Vertex64 segB2 = contour[j + 1];
                if (segB1 == segB2)
                {
                    continue;
                }

                if (FixedPolygonUtilities.SegmentsIntersect(segA1, segA2, segB1, segB2, true) ||
                    (FixedPolygonUtilities.IsCollinear(segA1, segA2, segB1) &&
                     FixedPolygonUtilities.IsCollinear(segA1, segA2, segB2) &&
                     SegmentsOverlap(segA1, segA2, segB1, segB2)))
                {
                    return true;
                }
            }
        }

        return false;
    }

    private static bool ContoursIntersect(
        List<Vertex64> left,
        List<Vertex64> right,
        in Box64 leftBounds,
        in Box64 rightBounds)
    {
        if (!leftBounds.Intersects(rightBounds))
        {
            return false;
        }

        int leftCount = left.Count - 1;
        int rightCount = right.Count - 1;
        for (int i = 0; i < leftCount; i++)
        {
            Vertex64 leftSeg1 = left[i];
            Vertex64 leftSeg2 = left[i + 1];
            if (leftSeg1 == leftSeg2)
            {
                continue;
            }

            for (int j = 0; j < rightCount; j++)
            {
                Vertex64 rightSeg1 = right[j];
                Vertex64 rightSeg2 = right[j + 1];
                if (rightSeg1 == rightSeg2)
                {
                    continue;
                }

                if (FixedPolygonUtilities.SegmentsIntersect(leftSeg1, leftSeg2, rightSeg1, rightSeg2, true) ||
                    (FixedPolygonUtilities.IsCollinear(leftSeg1, leftSeg2, rightSeg1) &&
                     FixedPolygonUtilities.IsCollinear(leftSeg1, leftSeg2, rightSeg2) &&
                     SegmentsOverlap(leftSeg1, leftSeg2, rightSeg1, rightSeg2)))
                {
                    return true;
                }
            }
        }

        return false;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool SegmentsOverlap(in Vertex64 a1, in Vertex64 a2, in Vertex64 b1, in Vertex64 b2)
    {
        long minAx = Math.Min(a1.X, a2.X);
        long maxAx = Math.Max(a1.X, a2.X);
        long minAy = Math.Min(a1.Y, a2.Y);
        long maxAy = Math.Max(a1.Y, a2.Y);
        long minBx = Math.Min(b1.X, b2.X);
        long maxBx = Math.Max(b1.X, b2.X);
        long minBy = Math.Min(b1.Y, b2.Y);
        long maxBy = Math.Max(b1.Y, b2.Y);

        return maxAx >= minBx && maxBx >= minAx && maxAy >= minBy && maxBy >= minAy;
    }

    /// <summary>
    /// Builds subject paths from a polygon.
    /// </summary>
    /// <param name="polygon">The polygon to convert.</param>
    /// <param name="context">The fixed-precision context.</param>
    /// <param name="normalizeForPositiveFill">
    /// Whether to normalize contour orientation for positive fill semantics.
    /// </param>
    /// <returns>A list of fixed-precision vertex paths ready for clipping.</returns>
    private static List<List<Vertex64>> BuildSubjectPaths(
        Polygon polygon,
        FixedPrecisionContext context,
        bool normalizeForPositiveFill)
    {
        List<List<Vertex64>> subject = new(polygon.Count);
        List<int> sourceIndices = normalizeForPositiveFill ? new List<int>(polygon.Count) : [];
        for (int i = 0; i < polygon.Count; i++)
        {
            Contour contour = polygon[i];
            if (contour.Count == 0)
            {
                continue;
            }

            bool isClosed = contour.Count > 1 && contour[0] == contour[^1];
            int capacity = contour.Count + (isClosed ? 0 : 1);
            List<Vertex64> path = new(capacity);

            CopyContourVertices(contour, isClosed, path, context);
            subject.Add(path);

            if (normalizeForPositiveFill)
            {
                sourceIndices.Add(i);
            }
        }

        if (normalizeForPositiveFill)
        {
            ApplyPositiveFillOrientation(polygon, subject, sourceIndices);
        }

        return subject;
    }

    private static void CopyContourVertices(
        Contour source,
        bool isClosed,
        List<Vertex64> destination,
        FixedPrecisionContext context)
    {
        for (int i = 0; i < source.Count; i++)
        {
            Vertex vertex = source[i];
            destination.Add(context.Quantize(vertex));
        }

        if (!isClosed && destination.Count > 1 && destination[^1] != destination[0])
        {
            destination.Add(destination[0]);
        }
    }

    private static void ApplyPositiveFillOrientation(
        Polygon source,
        List<List<Vertex64>> subject,
        List<int> sourceIndices)
    {
        bool[]? reverseFlags = BuildPositiveFillReversalFlags(source, subject, sourceIndices);
        if (reverseFlags == null)
        {
            return;
        }

        for (int i = 0; i < reverseFlags.Length; i++)
        {
            if (reverseFlags[i])
            {
                subject[i].Reverse();
            }
        }
    }

    private static bool[]? BuildPositiveFillReversalFlags(
        Polygon source,
        List<List<Vertex64>> subject,
        List<int> sourceIndices)
    {
        int count = subject.Count;
        if (count <= 1)
        {
            return null;
        }

        int[] parentIndices = new int[count];
        Array.Fill(parentIndices, -1);

        bool hasHierarchy = false;
        for (int i = 0; i < count; i++)
        {
            Contour contour = source[sourceIndices[i]];
            if (contour.ParentIndex == null && contour.HoleCount <= 0)
            {
                continue;
            }

            hasHierarchy = true;
            break;
        }

        if (hasHierarchy)
        {
            int[] sourceToSubject = new int[source.Count];
            Array.Fill(sourceToSubject, -1);
            for (int i = 0; i < sourceIndices.Count; i++)
            {
                sourceToSubject[sourceIndices[i]] = i;
            }

            for (int i = 0; i < count; i++)
            {
                int sourceIndex = sourceIndices[i];
                int parentIndex = source[sourceIndex].ParentIndex ?? -1;
                parentIndices[i] = parentIndex >= 0 && parentIndex < sourceToSubject.Length
                    ? sourceToSubject[parentIndex]
                    : -1;
            }
        }
        else
        {
            Box64[] bounds = new Box64[count];
            double[] absAreas = new double[count];
            for (int i = 0; i < count; i++)
            {
                List<Vertex64> contour = subject[i];
                bounds[i] = FixedPolygonUtilities.GetBounds(contour);
                absAreas[i] = Math.Abs(GetSignedArea(contour));

                if (HasSelfIntersection(contour))
                {
                    // Avoid reorienting inputs that are already self-intersecting.
                    return null;
                }
            }

            for (int i = 0; i < count; i++)
            {
                for (int j = i + 1; j < count; j++)
                {
                    if (ContoursIntersect(subject[i], subject[j], bounds[i], bounds[j]))
                    {
                        // Overlapping contours can change semantics when reoriented.
                        return null;
                    }
                }
            }

            for (int i = 0; i < count; i++)
            {
                List<Vertex64> contour = subject[i];
                if (contour.Count == 0)
                {
                    continue;
                }

                Vertex64 testPoint = GetContourTestPoint(contour);
                double smallestArea = double.PositiveInfinity;
                int parentIndex = -1;
                for (int j = 0; j < count; j++)
                {
                    if (i == j || !bounds[j].Contains(testPoint))
                    {
                        continue;
                    }

                    if (FixedPolygonUtilities.PointInPolygon(testPoint, subject[j]) != PointInPolygonResult.Inside)
                    {
                        continue;
                    }

                    if (absAreas[j] < smallestArea)
                    {
                        smallestArea = absAreas[j];
                        parentIndex = j;
                    }
                }

                parentIndices[i] = parentIndex;
            }
        }

        bool[] reverseFlags = new bool[count];
        bool needsReversal = false;
        for (int i = 0; i < count; i++)
        {
            List<Vertex64> contour = subject[i];
            if (contour.Count == 0)
            {
                continue;
            }

            int depth = GetDepth(i, parentIndices);
            bool shouldBeCounterClockwise = (depth & 1) == 0;
            bool isCounterClockwise = GetSignedArea(contour) >= 0D;
            if (isCounterClockwise != shouldBeCounterClockwise)
            {
                reverseFlags[i] = true;
                needsReversal = true;
            }
        }

        return needsReversal ? reverseFlags : null;
    }

    private sealed class OutputBuilder
    {
        private readonly SelfIntersectionSweepLine sweepLine;
        private bool buildHierarchy;
        private FixedPrecisionContext context;

        /// <summary>
        /// Initializes a new instance of the <see cref="OutputBuilder"/> class.
        /// </summary>
        public OutputBuilder() => this.sweepLine = new SelfIntersectionSweepLine();

        /// <summary>
        /// Gets or sets a value indicating whether collinear output points are preserved.
        /// </summary>
        public bool PreserveCollinear
        {
            get => this.sweepLine.PreserveCollinear;
            set => this.sweepLine.PreserveCollinear = value;
        }

        /// <summary>
        /// Gets or sets a value indicating whether the output orientation is reversed.
        /// </summary>
        public bool ReverseSolution { get; set; }

        /// <summary>
        /// Clears all cached input and output data.
        /// </summary>
        public void Clear() => this.sweepLine.Clear();

        /// <summary>
        /// Sets the fixed-precision context used to dequantize output vertices.
        /// </summary>
        /// <param name="context">The fixed-precision context.</param>
        public void SetContext(FixedPrecisionContext context) => this.context = context;

        /// <summary>
        /// Adds subject contours to the sweep-line clipper.
        /// </summary>
        /// <param name="paths">The subject contours to add.</param>
        public void AddSubject(List<List<Vertex64>> paths) => this.sweepLine.AddSubject(paths);

        /// <summary>
        /// Determines whether two points are within a tight tolerance.
        /// </summary>
        /// <param name="firstPoint">The first point.</param>
        /// <param name="secondPoint">The second point.</param>
        /// <returns><see langword="true"/> if the points are nearly coincident.</returns>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static bool ArePointsVeryClose(in Vertex64 firstPoint, in Vertex64 secondPoint)
        {
            Vertex64 delta = Vertex64.Abs(firstPoint - secondPoint);
            return delta.X < 2 && delta.Y < 2;
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
                if (FixedPolygonUtilities.IsCollinear(outputPoint2!.Prev.Point, outputPoint2.Point, outputPoint2.Next!.Point) &&
                    (outputPoint2.Point == outputPoint2.Prev.Point || outputPoint2.Point == outputPoint2.Next.Point || !this.PreserveCollinear ||
                    (FixedPolygonUtilities.Dot(outputPoint2.Prev.Point, outputPoint2.Point, outputPoint2.Next.Point) < 0)))
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

            FixedPolygonUtilities.TryGetLineIntersection(
                prevOp.Point, splitOp.Point, splitOp.Next.Point, nextNextOp.Point, out Vertex64 intersectionPoint);

            double area1 = SelfIntersectionSweepLine.ComputeSignedArea(prevOp);
            double absArea1 = Math.Abs(area1);

            if (absArea1 < 2D)
            {
                outputRecord.Points = null;
                return;
            }

            double area2 = AreaTriangle(intersectionPoint, splitOp.Point, splitOp.Next.Point);
            double absArea2 = Math.Abs(area2);

            // Remove the crossing segment and insert the intersection point.
            if (intersectionPoint == prevOp.Point || intersectionPoint == nextNextOp.Point)
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
            if (!(absArea2 > 1D) ||
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

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static double AreaTriangle(in Vertex64 pt1, in Vertex64 pt2, in Vertex64 pt3)
            => ((double)(pt3.Y + pt1.Y) * (pt3.X - pt1.X)) +
               ((double)(pt1.Y + pt2.Y) * (pt1.X - pt2.X)) +
               ((double)(pt2.Y + pt3.Y) * (pt2.X - pt3.X));

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
                if (FixedPolygonUtilities.SegmentsIntersect(
                    outputPoint2!.Prev.Point,
                    outputPoint2.Point,
                    outputPoint2.Next!.Point,
                    outputPoint2.Next.Next!.Point))
                {
                    if (FixedPolygonUtilities.SegmentsIntersect(
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
        private static bool BuildPath(OutputPoint? outputPoint, bool reverse, List<Vertex64> path)
        {
            if (outputPoint == null || outputPoint.Next == outputPoint || outputPoint.Next == outputPoint.Prev)
            {
                return false;
            }

            path.Clear();

            Vertex64 lastPoint;
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
                if (currentPoint.Point != lastPoint)
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
        private bool BuildContour(OutputPoint? outputPoint, bool reverse, Contour contour)
        {
            if (outputPoint == null || outputPoint.Next == outputPoint || outputPoint.Next == outputPoint.Prev)
            {
                return false;
            }

            contour.Clear();

            Vertex64 lastPoint;
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

            contour.Add(this.context.Dequantize(lastPoint));

            while (currentPoint != outputPoint)
            {
                Vertex64 current = currentPoint.Point;
                if (current != lastPoint)
                {
                    lastPoint = current;
                    contour.Add(this.context.Dequantize(lastPoint));
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

                int estimatedCapacity = outputRecord.OutputPointCount > 0
                    ? outputRecord.OutputPointCount + 1
                    : 0;
                Contour contour = estimatedCapacity > 0 ? new Contour(estimatedCapacity) : [];
                this.CleanCollinearEdges(outputRecord);
                if (this.BuildContour(outputRecord.Points, this.ReverseSolution, contour))
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
            if (outputRecord.Points == null)
            {
                return false;
            }

            if (outputRecord.OutputPointCount > 0)
            {
                outputRecord.Path.EnsureCapacity(outputRecord.OutputPointCount);
            }

            if (!BuildPath(outputRecord.Points, this.ReverseSolution, outputRecord.Path))
            {
                return false;
            }

            outputRecord.Bounds = FixedPolygonUtilities.GetBounds(outputRecord.Path);
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
                int estimatedCapacity = outputRecord.OutputPointCount > 0
                    ? outputRecord.OutputPointCount + 1
                    : 0;
                Contour contour = estimatedCapacity > 0 ? new Contour(estimatedCapacity) : [];
                if (!this.BuildContour(outputRecord.Points, this.ReverseSolution, contour))
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
        public bool Execute(
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
        public bool Execute(
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
}

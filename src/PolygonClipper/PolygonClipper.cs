// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System.Runtime.CompilerServices;

namespace SixLabors.PolygonClipper;

/// <summary>
/// Performs Boolean operations on the filled areas of directed contours.
/// </summary>
/// <remarks>
/// <para>
/// Intersection, union, difference, and exclusive-or interpret the original contour directions using the selected fill rules.
/// Input vertices and hierarchy are not modified. Results own their contour data.
/// </para>
/// <para>
/// Instance members are not thread-safe for concurrent use.
/// </para>
/// <para>
/// Fill rules interpret the input contours. Output uses positive orientation regardless of the input rules:
/// counterclockwise exteriors and islands, and clockwise holes in Cartesian coordinates.
/// Use NonZero, EvenOdd, or Positive filling to reuse that resolved area.
/// Contours use implicit closure without a repeated closing vertex.
/// </para>
/// <para>
/// Unary union and Normalize return full containment hierarchies. Binary operations group each exterior contour with its holes.
/// </para>
/// <para>
/// Coordinates must be finite. Only coordinates used by an operation are validated.
/// Intersection with an empty operand returns before inspecting the other operand's coordinates.
/// </para>
/// </remarks>
public class PolygonClipper
{
    // Keep a small per-thread hot set of clipper instances. Depth 4 covers common
    // burst usage without retaining many heavyweight buffers per thread.
    private const int MaxClipperPoolDepth = 4;

    // Upper bound for retained event/status capacities when returning to the pool.
    // 131_072 (~128K) keeps normal workloads warm while dropping pathological runs.
    private const int MaxRetainedEventCapacityScore = 131_072;

    private static readonly Polygon EmptyFillOperand = [];

    [ThreadStatic]
    private static Stack<PolygonClipper>? clipperPool;

    private readonly SweepEventComparer comparer = SweepEventComparer.Instance;
    private readonly SweepEventPoolList sortedEvents = [];
    private readonly StatusLine statusLine = new();

    private readonly List<QueueEntry> eventStorage = [];
    private readonly StablePriorityQueue<QueueEntry, SweepEventComparer> events;
    private readonly SweepEventPoolList segments = [];
    private readonly StatusLine activeResult = new();
    private readonly List<SweepEvent> outgoing = [];
    private readonly List<SweepEvent> bundled = [];
    private readonly List<SweepEvent> pendingBoundaries = [];
    private readonly Stack<PriorityQueue<WindingCount, Vertex>> endpointQueues = new();
    private int retainedEndpointCapacity;

    private Polygon? subject;
    private Polygon? clipping;
    private BooleanOperation operation;
    private ContourOutput contourOutput;
    private FillRule subjectFillRule;
    private FillRule clipFillRule;

    /// <summary>
    /// Initializes a new instance of the <see cref="PolygonClipper"/> class with independent operand fill rules.
    /// </summary>
    /// <param name="subject">The left-hand operand.</param>
    /// <param name="clip">The right-hand operand.</param>
    /// <param name="operation">The Boolean operation.</param>
    /// <param name="subjectFillRule">The rule interpreting the subject's directed contours.</param>
    /// <param name="clipFillRule">The rule interpreting the clipping operand's directed contours.</param>
    public PolygonClipper(Polygon subject, Polygon clip, BooleanOperation operation, FillRule subjectFillRule, FillRule clipFillRule)
        : this()
    {
        ValidateFillArguments(subject, clip, operation, subjectFillRule, clipFillRule);
        this.Configure(subject, clip, operation, subjectFillRule, clipFillRule);
    }

    /// <summary>
    /// Initializes a new instance of the <see cref="PolygonClipper"/> class for pooled execution.
    /// </summary>
    private PolygonClipper()
        => this.events = new(this.comparer, this.eventStorage);

    /// <summary>
    /// Specifies the form of returned contours.
    /// </summary>
    private enum ContourOutput
    {
        /// <summary>
        /// Groups exterior contours with their holes.
        /// </summary>
        Grouped,

        /// <summary>
        /// Preserves every immediate owner in a nesting chain.
        /// </summary>
        Nested
    }

    /// <summary>
    /// Intersects two filled areas using a common fill rule.
    /// </summary>
    /// <param name="subject">The left-hand operand.</param>
    /// <param name="clip">The right-hand operand.</param>
    /// <param name="fillRule">The rule interpreting both operands.</param>
    /// <returns>The filled area common to both operands.</returns>
    public static Polygon Intersection(Polygon subject, Polygon clip, FillRule fillRule)
        => Intersection(subject, clip, fillRule, fillRule);

    /// <summary>
    /// Intersects two filled areas using independent fill rules.
    /// </summary>
    /// <param name="subject">The left-hand operand.</param>
    /// <param name="clip">The right-hand operand.</param>
    /// <param name="subjectFillRule">The subject's fill rule.</param>
    /// <param name="clipFillRule">The clipping operand's fill rule.</param>
    /// <returns>The filled area common to both operands.</returns>
    public static Polygon Intersection(Polygon subject, Polygon clip, FillRule subjectFillRule, FillRule clipFillRule)
        => ExecuteFillOperation(subject, clip, BooleanOperation.Intersection, subjectFillRule, clipFillRule, ContourOutput.Grouped);

    /// <summary>
    /// Unites two filled areas using a common fill rule.
    /// </summary>
    /// <param name="subject">The left-hand operand.</param>
    /// <param name="clip">The right-hand operand.</param>
    /// <param name="fillRule">The rule interpreting both operands.</param>
    /// <returns>The filled area belonging to either operand.</returns>
    public static Polygon Union(Polygon subject, Polygon clip, FillRule fillRule)
        => Union(subject, clip, fillRule, fillRule);

    /// <summary>
    /// Unites two filled areas using independent fill rules.
    /// </summary>
    /// <param name="subject">The left-hand operand.</param>
    /// <param name="clip">The right-hand operand.</param>
    /// <param name="subjectFillRule">The subject's fill rule.</param>
    /// <param name="clipFillRule">The clipping operand's fill rule.</param>
    /// <returns>The filled area belonging to either operand.</returns>
    public static Polygon Union(Polygon subject, Polygon clip, FillRule subjectFillRule, FillRule clipFillRule)
        => ExecuteFillOperation(subject, clip, BooleanOperation.Union, subjectFillRule, clipFillRule, ContourOutput.Grouped);

    /// <summary>
    /// Resolves directed contours into their filled area.
    /// </summary>
    /// <param name="polygon">The raw directed contours.</param>
    /// <param name="fillRule">The rule interpreting the contours.</param>
    /// <returns>The resolved filled area with the full containment hierarchy, without changing the input contours.</returns>
    /// <remarks>Input hierarchy does not override contour direction or the selected fill rule.</remarks>
    public static Polygon Union(Polygon polygon, FillRule fillRule)
        => ExecuteFillOperation(polygon, EmptyFillOperand, BooleanOperation.Union, fillRule, FillRule.EvenOdd, ContourOutput.Nested);

    /// <summary>
    /// Subtracts one filled area from another using a common fill rule.
    /// </summary>
    /// <param name="subject">The area to retain.</param>
    /// <param name="clip">The area to subtract.</param>
    /// <param name="fillRule">The rule interpreting both operands.</param>
    /// <returns>The subject area outside the clipping operand.</returns>
    public static Polygon Difference(Polygon subject, Polygon clip, FillRule fillRule)
        => Difference(subject, clip, fillRule, fillRule);

    /// <summary>
    /// Subtracts one filled area from another using independent fill rules.
    /// </summary>
    /// <param name="subject">The area to retain.</param>
    /// <param name="clip">The area to subtract.</param>
    /// <param name="subjectFillRule">The subject's fill rule.</param>
    /// <param name="clipFillRule">The clipping operand's fill rule.</param>
    /// <returns>The subject area outside the clipping operand.</returns>
    public static Polygon Difference(Polygon subject, Polygon clip, FillRule subjectFillRule, FillRule clipFillRule)
        => ExecuteFillOperation(subject, clip, BooleanOperation.Difference, subjectFillRule, clipFillRule, ContourOutput.Grouped);

    /// <summary>
    /// Computes exclusive filled coverage using a common fill rule.
    /// </summary>
    /// <param name="subject">The left-hand operand.</param>
    /// <param name="clip">The right-hand operand.</param>
    /// <param name="fillRule">The rule interpreting both operands.</param>
    /// <returns>The filled area belonging to exactly one operand.</returns>
    public static Polygon Xor(Polygon subject, Polygon clip, FillRule fillRule)
        => Xor(subject, clip, fillRule, fillRule);

    /// <summary>
    /// Computes exclusive filled coverage using independent fill rules.
    /// </summary>
    /// <param name="subject">The left-hand operand.</param>
    /// <param name="clip">The right-hand operand.</param>
    /// <param name="subjectFillRule">The subject's fill rule.</param>
    /// <param name="clipFillRule">The clipping operand's fill rule.</param>
    /// <returns>The filled area belonging to exactly one operand.</returns>
    public static Polygon Xor(Polygon subject, Polygon clip, FillRule subjectFillRule, FillRule clipFillRule)
        => ExecuteFillOperation(subject, clip, BooleanOperation.Xor, subjectFillRule, clipFillRule, ContourOutput.Grouped);

    /// <summary>
    /// Resolves self-intersections and overlaps using the specified fill rule.
    /// </summary>
    /// <param name="polygon">The source contours. Their vertices and directions are not modified.</param>
    /// <param name="fillRule">The rule that determines the source polygon's filled area.</param>
    /// <returns>
    /// The same cleanup as unary union, with closed contours and their full containment hierarchy.
    /// Exterior contours and islands are counterclockwise. Holes are clockwise.
    /// The first vertex is not repeated at the end of a contour.
    /// </returns>
    /// <remarks>
    /// The fill rule interprets the input. Output orientation is positive even when the input rule is Negative.
    /// Use NonZero, EvenOdd, or Positive filling when reusing the cleaned polygon.
    /// </remarks>
    public static Polygon Normalize(Polygon polygon, FillRule fillRule)
        => Union(polygon, fillRule);

    /// <summary>
    /// Executes the configured Boolean operation using the operand fill rules.
    /// </summary>
    /// <returns>The connected filled boundaries.</returns>
    /// <remarks>Instance execution is not thread-safe for concurrent use.</remarks>
    public Polygon Run()
    {
        Polygon subject = this.subject!;
        Polygon clip = this.clipping!;
        BooleanOperation operation = this.operation;
        FillRule subjectRule = this.subjectFillRule;
        FillRule clipRule = this.clipFillRule;
        SweepEventPoolList resultEvents = this.sortedEvents;
        SweepEventComparer resultComparer = this.comparer;
        resultEvents.Clear();

        this.ResetSweep();
        if (operation == BooleanOperation.Intersection && (subject.Count == 0 || clip.Count == 0))
        {
            return [];
        }

        // Reuse the existing capacity APIs and reserve source-sized storage
        // before ingestion. Repeated geometric growth otherwise copies the
        // larger endpoint records and can exceed the shared retention bound.
        int vertexCount = subject.VertexCount + clip.VertexCount;
        this.eventStorage.EnsureCapacity(vertexCount);
        this.segments.EnsureCapacity(vertexCount);
        resultEvents.EnsureCapacity(vertexCount * 2);

        Box2 subjectBounds = this.ProcessSegments(subject, true);
        Box2 clipBounds = this.ProcessSegments(clip, false);
        if (operation == BooleanOperation.Intersection && !subjectBounds.Intersects(clipBounds))
        {
            return [];
        }

        double stopX = operation switch
        {
            BooleanOperation.Intersection => Math.Min(subjectBounds.Max.X, clipBounds.Max.X),
            BooleanOperation.Difference => subjectBounds.Max.X,
            _ => double.PositiveInfinity
        };

        this.events.Heapify();

        while (this.events.Count > 0)
        {
            // Bounds exclude any further filled result. Finish active boundaries
            // first because an ordering repair can move their endpoint by one ULP.
            if (this.events.Peek().Point.X > stopX && this.activeResult.Count == 0)
            {
                break;
            }

            QueueEntry next = this.events.Dequeue();
            if (!next.IsValid)
            {
                continue;
            }

            Vertex point = next.Point;
            bool sourcePoint = next.IsStart || point == next.Edge.SourceEnd;
            this.outgoing.Clear();
            this.pendingBoundaries.Clear();
            int firstEnd = int.MaxValue;
            int lastEnd = -1;
            if (next.IsStart)
            {
                this.outgoing.Add(next.Edge);
            }
            else
            {
                firstEnd = lastEnd = next.Edge.PosSL;
            }

            // A vertex is a transaction, not a sequence of parity toggles.
            // Collect every start/end at this exact lexicographic point.
            while (this.events.Count > 0 && this.events.Peek().Point == point)
            {
                next = this.events.Dequeue();
                if (next.IsValid && (next.IsStart || point == next.Edge.SourceEnd))
                {
                    sourcePoint = true;
                }

                if (next.IsValid && next.IsStart)
                {
                    this.outgoing.Add(next.Edge);
                }
                else if (next.IsValid)
                {
                    firstEnd = Math.Min(firstEnd, next.Edge.PosSL);
                    lastEnd = Math.Max(lastEnd, next.Edge.PosSL);
                }
            }

            int first = lastEnd < 0 ? this.statusLine.FindLower(point) : firstEnd;
            while (first > 0 && this.statusLine[first - 1].ContainsPoint(point))
            {
                first--;
            }

            int after = Math.Max(first, lastEnd + 1);
            while (after < this.statusLine.Count && this.statusLine[after].ContainsPoint(point))
            {
                after++;
            }

            // The result index is a filtered view of the very same block.
            // Derive removals from the owned interval identities, so rounded
            // incidence tests cannot remove a boundary independently of its
            // source interval and leave a stale birth context behind.
            int firstResult = first == 0 ? 0 : this.statusLine[first - 1].ResultRank;
            int afterResult = firstResult;
            for (int i = first; i < after; i++)
            {
                SweepEvent incoming = this.statusLine[i];
                if (incoming.Boundary is not null)
                {
                    afterResult++;
                }
            }

            // The surviving lower interval is outside the changed block.
            // Closure conserves the sum of signed contributions, so its
            // prefix and every surviving prefix above the block stay valid.
            for (int i = first; i < after; i++)
            {
                this.StageIncoming(this.statusLine[i], point);
            }

            this.statusLine.RemoveRange(first, after - first);
            this.activeResult.RemoveRange(firstResult, afterResult - firstResult);

            while (true)
            {
                this.outgoing.Sort(SegmentComparer.CompareRays);
                this.BundleOutgoing();
                for (int i = 0; i < this.bundled.Count; i++)
                {
                    SweepEvent edge = this.bundled[i];
                    edge.Active = true;
                    this.statusLine.Insert(first + i, edge);
                }

                // Rounded intersections can equal this point even when the
                // incidence predicate missed a neighboring interval. Absorb it
                // before committing winding or immutable boundary history.
                bool lowerIncident = this.PossibleIntersection(first - 1, first, point, sourcePoint);
                bool upperIncident = this.bundled.Count == 0
                    ? lowerIncident
                    : !lowerIncident && this.PossibleIntersection(first + this.bundled.Count - 1, first + this.bundled.Count, point, sourcePoint);
                if (!lowerIncident && !upperIncident)
                {
                    break;
                }

                this.statusLine.RemoveRange(first, this.bundled.Count);
                this.outgoing.Clear();
                this.outgoing.AddRange(this.bundled);
                foreach (SweepEvent edge in this.outgoing)
                {
                    // Rebundling can replace a representative. Invalidate any
                    // future endpoint queued while checking the staged block.
                    edge.Active = false;
                    edge.Version++;
                }

                if (lowerIncident)
                {
                    SweepEvent incoming = this.statusLine[--first];
                    if (incoming.Boundary is not null)
                    {
                        this.activeResult.RemoveAt(--firstResult);
                    }

                    this.statusLine.RemoveAt(first);
                    this.StageIncoming(incoming, point);
                }

                if (upperIncident)
                {
                    SweepEvent incoming = this.statusLine[first];
                    if (incoming.Boundary is not null)
                    {
                        this.activeResult.RemoveAt(firstResult);
                    }

                    this.statusLine.RemoveAt(first);
                    this.StageIncoming(incoming, point);
                }
            }

            foreach (SweepEvent boundary in this.pendingBoundaries)
            {
                // The point is closed. Complete each historical piece without
                // changing its original transition or predecessor.
                SweepEvent right = resultEvents.Add(point, false, boundary, ResultTransition.Neutral, null!, boundary.ContourId);
                boundary.OtherEvent = right;
            }

            WindingCount winding = first == 0 ? default : this.statusLine[first - 1].Above;
            SweepEvent? previousBoundary = firstResult == 0 ? null : this.activeResult[firstResult - 1].Boundary;
            if (firstResult > 0 && this.activeResult[firstResult - 1].Point.X == this.activeResult[firstResult - 1].End.X)
            {
                previousBoundary = previousBoundary!.PrevInResult;
            }

            int resultIndex = firstResult;
            for (int i = 0; i < this.bundled.Count; i++)
            {
                SweepEvent edge = this.bundled[i];
                bool below = InResult(winding, operation, subjectRule, clipRule);
                winding = winding.Add(edge.Delta);
                edge.Above = winding;
                bool above = InResult(winding, operation, subjectRule, clipRule);
                if (below != above)
                {
                    // Endpoint pairing is completed when the interval ends.
                    // No geometry consumer reads this event until connection.
                    edge.Boundary = resultEvents.Add(
                        point,
                        true,
                        null!,
                        above ? ResultTransition.Contributing : ResultTransition.NonContributing,
                        previousBoundary!,
                        edge.ContourId);

                    if (edge.Point.X != edge.End.X)
                    {
                        previousBoundary = edge.Boundary;
                    }

                    this.activeResult.Insert(resultIndex++, edge);
                }

                this.events.Enqueue(new QueueEntry(edge.End, edge, false));
            }

            // List insertion/removal already shifts this suffix. Keep the
            // endpoint anchors and filtered-boundary ranks in the same order.
            // Winding counts above the changed block do not need recomputing.
            int resultRank = firstResult;
            for (int i = first; i < this.statusLine.Count; i++)
            {
                SweepEvent edge = this.statusLine[i];
                edge.PosSL = i;
                if (edge.Boundary is not null)
                {
                    resultRank++;
                }

                edge.ResultRank = resultRank;
            }
        }

        // Boundary events are completed at their right endpoints. Restore the
        // ordering expected by the existing connector in O(B log B), rather
        // than making its nearly-sorted repair loop sort reversed history.
        resultEvents.Sort(resultComparer);
        Polygon result = this.ConnectEdges(resultEvents);
        for (int i = 0; i < result.Count; i++)
        {
            Contour contour = result[i];
            if (contour.Count > 1 && contour[0] == contour[^1])
            {
                contour.RemoveVertexAt(contour.Count - 1);
            }

            if ((contour.Depth & 1) == 0)
            {
                contour.SetCounterClockwise();
            }
            else
            {
                contour.SetClockwise();
            }
        }

        return result;
    }

    /// <summary>
    /// Executes an explicit fill-rule operation through the existing clipper pool.
    /// </summary>
    /// <param name="subject">The left-hand operand.</param>
    /// <param name="clip">The right-hand operand.</param>
    /// <param name="operation">The Boolean operation.</param>
    /// <param name="subjectRule">The subject's fill rule.</param>
    /// <param name="clipRule">The clipping operand's fill rule.</param>
    /// <param name="output">The requested contour hierarchy.</param>
    /// <returns>The resolved filled area.</returns>
    private static Polygon ExecuteFillOperation(
        Polygon subject,
        Polygon clip,
        BooleanOperation operation,
        FillRule subjectRule,
        FillRule clipRule,
        ContourOutput output)
    {
        ValidateFillArguments(subject, clip, operation, subjectRule, clipRule);
        PolygonClipper clipper = Rent(subject, clip, operation, subjectRule, clipRule);
        clipper.contourOutput = output;

        try
        {
            return clipper.Run();
        }
        finally
        {
            Return(clipper);
        }
    }

    /// <summary>
    /// Validates public operation arguments before taking geometry shortcuts.
    /// </summary>
    /// <param name="subject">The left-hand operand.</param>
    /// <param name="clip">The right-hand operand.</param>
    /// <param name="operation">The Boolean operation.</param>
    /// <param name="subjectRule">The subject's fill rule.</param>
    /// <param name="clipRule">The clipping operand's fill rule.</param>
    private static void ValidateFillArguments(Polygon subject, Polygon clip, BooleanOperation operation, FillRule subjectRule, FillRule clipRule)
    {
        ArgumentNullException.ThrowIfNull(subject);
        ArgumentNullException.ThrowIfNull(clip);
        ArgumentOutOfRangeException.ThrowIfGreaterThan((uint)operation, (uint)BooleanOperation.Xor, nameof(operation));
        ArgumentOutOfRangeException.ThrowIfGreaterThan((uint)subjectRule, (uint)FillRule.Negative, nameof(subjectRule));
        ArgumentOutOfRangeException.ThrowIfGreaterThan((uint)clipRule, (uint)FillRule.Negative, nameof(clipRule));
    }

    /// <summary>
    /// Acquires an instance configured for this operation and both operand rules.
    /// </summary>
    /// <param name="subject">The subject contours.</param>
    /// <param name="clip">The clipping contours.</param>
    /// <param name="operation">The Boolean operation.</param>
    /// <param name="subjectRule">The subject fill rule.</param>
    /// <param name="clipRule">The clipping fill rule.</param>
    /// <returns>The clipper configured for this operation.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static PolygonClipper Rent(Polygon subject, Polygon clip, BooleanOperation operation, FillRule subjectRule, FillRule clipRule)
    {
        Stack<PolygonClipper>? pool = clipperPool;
        if (pool != null && pool.Count > 0)
        {
            PolygonClipper clipper = pool.Pop();
            clipper.Configure(subject, clip, operation, subjectRule, clipRule);
            return clipper;
        }

        // The public entry point already validated these arguments. Configure the
        // private instance directly so a cold pool does not validate them twice.
        PolygonClipper created = new();
        created.Configure(subject, clip, operation, subjectRule, clipRule);
        return created;
    }

    /// <summary>
    /// Releases operation references and retains only bounded reusable state.
    /// </summary>
    /// <param name="clipper">The completed clipping instance.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void Return(PolygonClipper clipper)
    {
        clipper.subject = null;
        clipper.clipping = null;
        clipper.sortedEvents.Clear();
        clipper.ResetSweep();

        // Avoid keeping very large event buffers alive in thread-static pools.
        if (clipper.GetRetainedEventCapacityScore() > MaxRetainedEventCapacityScore)
        {
            return;
        }

        Stack<PolygonClipper> pool = clipperPool ??= new Stack<PolygonClipper>(MaxClipperPoolDepth);
        if (pool.Count < MaxClipperPoolDepth)
        {
            pool.Push(clipper);
        }
    }

    /// <summary>
    /// Configures the inputs and fill rules before any event is created.
    /// </summary>
    /// <param name="subject">The subject contours.</param>
    /// <param name="clip">The clipping contours.</param>
    /// <param name="operation">The Boolean operation.</param>
    /// <param name="subjectRule">The subject fill rule.</param>
    /// <param name="clipRule">The clipping fill rule.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void Configure(Polygon subject, Polygon clip, BooleanOperation operation, FillRule subjectRule, FillRule clipRule)
    {
        this.subject = subject;
        this.clipping = clip;
        this.operation = operation;
        this.contourOutput = ContourOutput.Grouped;
        this.subjectFillRule = subjectRule;
        this.clipFillRule = clipRule;
    }

    /// <summary>
    /// Gets the combined retained event and status capacities.
    /// </summary>
    /// <returns>The total capacity retained by the clipping buffers.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private int GetRetainedEventCapacityScore()
        => this.sortedEvents.Capacity + this.eventStorage.Capacity + this.segments.Capacity + this.statusLine.RetainedCapacity +
           this.activeResult.RetainedCapacity + this.outgoing.Capacity + this.bundled.Capacity +
           this.pendingBoundaries.Capacity + this.retainedEndpointCapacity;

    /// <summary>
    /// Counts the edges of an implicitly closed filled boundary.
    /// </summary>
    /// <param name="contour">The contour to inspect.</param>
    /// <returns>The number of segments to iterate.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static int GetContourSegmentCount(Contour contour)
    {
        int count = contour.Count;
        if (count < 3)
        {
            return 0;
        }

        return contour[0] == contour[^1] ? count - 1 : count;
    }

    /// <summary>
    /// Releases per-operation references while retaining bounded scratch storage.
    /// </summary>
    private void ResetSweep()
    {
        this.eventStorage.Clear();
        this.statusLine.Reset(0);
        this.activeResult.Reset(0);
        this.outgoing.Clear();
        this.bundled.Clear();
        this.pendingBoundaries.Clear();
        for (int i = 0; i < this.segments.Count; i++)
        {
            if (this.segments[i].Ends is PriorityQueue<WindingCount, Vertex> ends)
            {
                this.ReturnEndpoints(ends);
            }

            this.segments[i].Ends = null;
            this.segments[i].Boundary = null;
        }

        this.segments.Clear();
    }

    /// <summary>
    /// Adds directed input segments without normalizing or reorienting them.
    /// </summary>
    /// <param name="polygon">The input contours.</param>
    /// <param name="subject">Whether these contours belong to the subject.</param>
    /// <returns>The bounds collected during the existing input traversal.</returns>
    private Box2 ProcessSegments(Polygon polygon, bool subject)
    {
        Box2 bounds = Box2.Invalid;
        for (int i = 0; i < polygon.Count; i++)
        {
            Contour contour = polygon[i];
            int count = GetContourSegmentCount(contour);
            if (count == 0)
            {
                // These external contours have no closed segments to process.
                continue;
            }

            for (int j = 0; j < count; j++)
            {
                Segment segment = contour.GetSegment(j);
                Vertex start = segment.Source;
                Vertex end = segment.Target;
                if (!double.IsFinite(start.X) || !double.IsFinite(start.Y))
                {
                    throw new ArgumentException("Polygon coordinates must be finite.", nameof(polygon));
                }

                bounds = bounds.Add(new Box2(start));
                if (start == end)
                {
                    continue;
                }

                int direction = 1;
                if (SweepEventComparer.Instance.Compare(start, end) > 0)
                {
                    (start, end) = (end, start);
                    direction = -1;
                }

                // Lexicographic direction is the formal x+epsilon*y sweep
                // direction, including vertical source segments. Epsilon is
                // never evaluated or applied to the stored double coordinates.
                WindingCount delta = subject ? new(direction, 0) : new(0, direction);
                SweepEvent edge = this.segments.Add(start, end, delta);
                this.eventStorage.Add(new QueueEntry(start, edge, true));
            }
        }

        return bounds;
    }

    /// <summary>
    /// Removes an incoming interval while retaining its history until the point is closed.
    /// </summary>
    /// <param name="incoming">The incident active interval.</param>
    /// <param name="point">The staged event point.</param>
    private void StageIncoming(SweepEvent incoming, Vertex point)
    {
        incoming.Active = false;
        incoming.Version++;
        if (incoming.Boundary is SweepEvent boundary)
        {
            this.pendingBoundaries.Add(boundary);
        }

        this.ContinueAt(incoming, point);
    }

    /// <summary>
    /// Preserves a split interval's longer source tails and endpoint deltas.
    /// </summary>
    /// <param name="incoming">The interval ending at this point.</param>
    /// <param name="point">The complete event point.</param>
    private void ContinueAt(SweepEvent incoming, Vertex point)
    {
        if (incoming.Ends is PriorityQueue<WindingCount, Vertex> ends)
        {
            WindingCount delta = incoming.Delta;
            while (ends.TryPeek(out WindingCount ending, out Vertex endpoint) && endpoint == point)
            {
                delta = delta.Subtract(ending);
                ends.Dequeue();
            }

            if (ends.Count == 0)
            {
                incoming.Ends = null;
                this.ReturnEndpoints(ends);
                return;
            }

            incoming.Delta = delta;
            incoming.SourceEnd = FirstEndpoint(ends);
        }
        else if (SweepEventComparer.Instance.Compare(point, incoming.SourceEnd) >= 0)
        {
            return;
        }

        // Output history owns independent SweepEvent objects. Reuse only the
        // live interval, whose version was advanced before closing its old
        // piece. Stale queue records stay invalid without retaining a new
        // interval object for every intersection in the operation.
        incoming.Point = point;
        incoming.End = incoming.SourceEnd;
        incoming.Boundary = null;
        this.outgoing.Add(incoming);
    }

    /// <summary>
    /// Combines coincident outgoing intervals without losing longer continuations.
    /// </summary>
    private void BundleOutgoing()
    {
        this.bundled.Clear();
        int i = 0;
        while (i < this.outgoing.Count)
        {
            SweepEvent edge = this.outgoing[i++];
            while (i < this.outgoing.Count && SegmentComparer.CompareRays(edge, this.outgoing[i]) == 0)
            {
                SweepEvent other = this.outgoing[i++];
                if (SweepEventComparer.Instance.Compare(other.End, edge.End) < 0)
                {
                    // A staged neighbor check may already have found an earlier
                    // crossing. Keep that node when rebuilding a coincident bundle.
                    edge.End = other.End;
                }

                // A bundle may outlive its representative's original segment.
                // Its support must cover every retained endpoint-ledger tail.
                if (SweepEventComparer.Instance.Compare(other.Support.Target, edge.Support.Target) > 0)
                {
                    edge.Support = new Segment(edge.Support.Source, other.Support.Target);
                }

                if (edge.Ends is null && other.Ends is null && edge.SourceEnd == other.SourceEnd)
                {
                    // The common duplicate-edge case needs no endpoint ledger.
                    edge.Delta = edge.Delta.Add(other.Delta);
                    continue;
                }

                if (edge.Ends is null)
                {
                    edge.Ends = this.RentEndpoints();
                    if (!edge.Delta.IsZero)
                    {
                        edge.Ends.Enqueue(edge.Delta, edge.SourceEnd);
                    }
                }

                PriorityQueue<WindingCount, Vertex> ends = edge.Ends;
                if (other.Ends is PriorityQueue<WindingCount, Vertex> otherEnds)
                {
                    foreach ((WindingCount delta, Vertex endpoint) in otherEnds.UnorderedItems)
                    {
                        ends.Enqueue(delta, endpoint);
                    }

                    other.Ends = null;
                    this.ReturnEndpoints(otherEnds);
                }
                else if (!other.Delta.IsZero)
                {
                    ends.Enqueue(other.Delta, other.SourceEnd);
                }

                edge.Delta = edge.Delta.Add(other.Delta);
            }

            if (edge.Ends is PriorityQueue<WindingCount, Vertex> ledger)
            {
                if (ledger.Count == 0)
                {
                    edge.Ends = null;
                    this.ReturnEndpoints(ledger);
                    continue;
                }

                edge.SourceEnd = FirstEndpoint(ledger);
                if (SweepEventComparer.Instance.Compare(edge.SourceEnd, edge.End) < 0)
                {
                    edge.End = edge.SourceEnd;
                }
            }
            else if (edge.Delta.IsZero)
            {
                continue;
            }

            // A zero current sum with non-zero later endpoint changes must
            // retain its continuation, even though it emits no boundary now.
            this.bundled.Add(edge);
        }
    }

    /// <summary>
    /// Gets the earliest endpoint of a non-empty contribution queue.
    /// </summary>
    /// <param name="ends">The non-empty endpoint queue.</param>
    /// <returns>The earliest endpoint.</returns>
    private static Vertex FirstEndpoint(PriorityQueue<WindingCount, Vertex> ends)
    {
        ends.TryPeek(out _, out Vertex endpoint);
        return endpoint;
    }

    /// <summary>
    /// Rents endpoint storage without allocating tree nodes for each contribution.
    /// </summary>
    /// <returns>An empty reusable priority queue.</returns>
    private PriorityQueue<WindingCount, Vertex> RentEndpoints()
    {
        if (this.endpointQueues.TryPop(out PriorityQueue<WindingCount, Vertex>? ends))
        {
            this.retainedEndpointCapacity -= ends.EnsureCapacity(0);
            return ends;
        }

        return new(SweepEventComparer.Instance);
    }

    /// <summary>
    /// Returns endpoint storage after its interval releases ownership.
    /// </summary>
    /// <param name="ends">The queue no longer referenced by a live interval.</param>
    private void ReturnEndpoints(PriorityQueue<WindingCount, Vertex> ends)
    {
        // Retain the array for the next bundle. Include it in the clipper's
        // capacity limit so a large overlap cannot pin unbounded scratch storage.
        ends.Clear();
        this.retainedEndpointCapacity += ends.EnsureCapacity(0);
        this.endpointQueues.Push(ends);
    }

    /// <summary>
    /// Schedules the next intersection of newly adjacent active intervals.
    /// </summary>
    /// <param name="lower">The lower neighbor's index.</param>
    /// <param name="upper">The upper neighbor's index.</param>
    /// <param name="point">The event already processed.</param>
    /// <param name="sourcePoint">Whether this event contains an original source endpoint.</param>
    /// <returns>Whether the new adjacency also meets at the staged point.</returns>
    private bool PossibleIntersection(int lower, int upper, Vertex point, bool sourcePoint)
    {
        if (lower < 0 || upper >= this.statusLine.Count)
        {
            return false;
        }

        SweepEvent first = this.statusLine[lower];
        SweepEvent second = this.statusLine[upper];

        // Keep the original support lines across splits. Reconstructing a
        // line from a rounded intersection rotates it and can create a second,
        // slightly different intersection at an existing source vertex.
        if (first.ContourId > second.ContourId)
        {
            (first, second) = (second, first);
        }

        int count = PolygonUtilities.FindIntersection(
            first.Support, second.Support, out Vertex intersection, out Vertex last);
        if (count > 0)
        {
            if (intersection == point || (count == 2 && last == point))
            {
                if (!sourcePoint ||
                    (PolygonUtilities.SignedArea(first.Support.Source, first.Support.Target, point) == 0 &&
                     PolygonUtilities.SignedArea(second.Support.Source, second.Support.Target, point) == 0))
                {
                    return true;
                }

                // A real interior crossing can round to a nearby source vertex.
                // Do not merge those distinct events. Preserve a vertical support's
                // x coordinate; otherwise use the existing one-ULP x-order repair.
                intersection = first.Point.X == first.SourceEnd.X || second.Point.X == second.SourceEnd.X
                    ? new Vertex(point.X, point.Y.NextAfter(double.PositiveInfinity))
                    : new Vertex(point.X.NextAfter(double.PositiveInfinity), point.Y);
            }

            // Match the established Martinez repair: a rounded crossing on
            // this x coordinate but below the completed event cannot be
            // visited backwards. Advance x by one representable double.
            if (intersection.X == point.X && intersection.Y < point.Y)
            {
                intersection = new Vertex(intersection.X.NextAfter(double.PositiveInfinity), intersection.Y);
            }

            if (SweepEventComparer.Instance.Compare(intersection, first.Point) < 0 || SweepEventComparer.Instance.Compare(intersection, second.Point) < 0 ||
                (SweepEventComparer.Instance.Compare(intersection, first.End) > 0 && (first.End != first.SourceEnd || intersection.X != first.End.X)) ||
                (SweepEventComparer.Instance.Compare(intersection, second.End) > 0 && (second.End != second.SourceEnd || intersection.X != second.End.X)))
            {
                return false;
            }

            this.DivideSegment(first, intersection, point);
            this.DivideSegment(second, intersection, point);
            if (count == 2)
            {
                this.DivideSegment(first, last, point);
                this.DivideSegment(second, last, point);
            }
        }

        return false;
    }

    /// <summary>
    /// Shortens a live interval to its earliest discovered future node.
    /// </summary>
    /// <param name="edge">The live interval.</param>
    /// <param name="intersection">The discovered node.</param>
    /// <param name="current">The completed event point.</param>
    private void DivideSegment(SweepEvent edge, Vertex intersection, Vertex current)
    {
        if (SweepEventComparer.Instance.Compare(intersection, current) <= 0 || intersection == edge.End)
        {
            return;
        }

        if (SweepEventComparer.Instance.Compare(intersection, edge.End) > 0)
        {
            if (edge.End != edge.SourceEnd || intersection.X != edge.End.X || edge.Point.X == edge.End.X)
            {
                return;
            }

            // The second established Martinez rounding case reverses the
            // tiny final piece: the source endpoint precedes the rounded
            // crossing on the same x coordinate. Represent that piece as a
            // new forward interval with the opposite signed contribution.
            SweepEvent tail = this.segments.Add(edge.SourceEnd, intersection, default(WindingCount).Subtract(edge.Delta));
            this.events.Enqueue(new QueueEntry(tail.Point, tail, true));
            edge.SourceEnd = intersection;
        }

        // SourceEnd/Ends retain the source's real terminal changes. End only
        // limits this open interval; its continuation is made at the new node.
        edge.End = intersection;
        edge.Version++;
        this.events.Enqueue(new QueueEntry(intersection, edge, false));
    }

    /// <summary>
    /// Evaluates Boolean membership from independent signed winding counts.
    /// </summary>
    /// <param name="count">The region's winding counts.</param>
    /// <param name="operation">The Boolean operation.</param>
    /// <param name="subjectRule">The subject's fill rule.</param>
    /// <param name="clipRule">The clipping operand's fill rule.</param>
    /// <returns>Whether the region belongs to the result.</returns>
    private static bool InResult(WindingCount count, BooleanOperation operation, FillRule subjectRule, FillRule clipRule)
    {
        bool subject = Filled(count.Subject, subjectRule);
        bool clip = Filled(count.Clip, clipRule);
        return operation switch
        {
            BooleanOperation.Intersection => subject && clip,
            BooleanOperation.Union => subject || clip,
            BooleanOperation.Difference => subject && !clip,
            _ => subject ^ clip
        };
    }

    /// <summary>
    /// Interprets a winding count using a validated fill rule.
    /// </summary>
    /// <param name="count">The signed winding count.</param>
    /// <param name="rule">The validated fill rule.</param>
    /// <returns>Whether the region is filled.</returns>
    private static bool Filled(int count, FillRule rule)
        => rule switch
        {
            FillRule.EvenOdd => (count & 1) != 0,
            FillRule.NonZero => count != 0,
            FillRule.Positive => count > 0,
            _ => count < 0
        };

    /// <summary>
    /// Connects edges in the result polygon by processing the sweep events
    /// and constructing contours for the final result.
    /// </summary>
    /// <param name="resultEvents">The sorted, paired filled-boundary events.</param>
    /// <returns>The connected polygon.</returns>
    private Polygon ConnectEdges(SweepEventPoolList resultEvents)
    {
        // Run already sorts completed boundary events. Work directly on their
        // pooled storage; no filtering copy or second ordering pass is needed.
        // Assign positions to events
        // The first loop ensures that every event gets its initial position based on its index in the list.
        // This must be completed for all events before adjustments are made for right events to avoid inconsistent state.
        for (int i = 0; i < resultEvents.Count; i++)
        {
            resultEvents[i].Pos = i;
        }

        // Adjust positions for right events
        // The second loop handles swapping positions for right events with their corresponding left events.
        // This ensures that the `Pos` values are consistent between paired events after the initial assignment.
        for (int i = 0; i < resultEvents.Count; i++)
        {
            SweepEvent sweepEvent = resultEvents[i];
            if (sweepEvent.Left)
            {
                (sweepEvent.OtherEvent.Pos, sweepEvent.Pos) = (sweepEvent.Pos, sweepEvent.OtherEvent.Pos);
            }
        }

        ReadOnlySpan<int> iterationMap = PrecomputeIterationOrder(resultEvents);

        Polygon result = [];
        Span<bool> processed = new bool[resultEvents.Count];
        for (int i = 0; i < resultEvents.Count; i++)
        {
            if (processed[i])
            {
                continue;
            }

            int contourId = result.Count;
            Contour contour = this.InitializeContourFromContext(resultEvents[i], result, contourId);

            int pos = i;
            Vertex initial = resultEvents[i].Point;
            contour.Add(initial);

            // Main loop to process the contour
            do
            {
                MarkProcessed(resultEvents[pos], processed, pos, contourId);
                pos = resultEvents[pos].Pos;

                MarkProcessed(resultEvents[pos], processed, pos, contourId);

                Vertex point = resultEvents[pos].Point;
                contour.Add(point);

                pos = NextPos(pos, processed, iterationMap, out bool found);
                if (!found)
                {
                    break;
                }
            }
            while (resultEvents[pos].Point != initial);

            result.Add(contour);
        }

        if (this.contourOutput == ContourOutput.Nested)
        {
            return result;
        }

        Polygon polygon = [];
        using Buffer<int> contourIndices = new(result.Count);
        Span<int> indices = contourIndices.GetSpan();

        for (int i = 0; i < result.Count; i++)
        {
            Contour contour = result[i];
            if (contour.IsExternal)
            {
                // The exterior ring goes first
                indices[i] = polygon.Count;
                polygon.Add(contour);

                // Followed by holes if any
                for (int j = 0; j < contour.HoleCount; j++)
                {
                    int holeId = contour.GetHoleIndex(j);
                    indices[holeId] = polygon.Count;
                    polygon.Add(result[holeId]);
                }
            }
        }

        // Connectivity used the original contour order. Exported indices must
        // instead address the exterior/hole order returned to the caller.
        for (int i = 0; i < polygon.Count; i++)
        {
            polygon[i].RemapHierarchyIndices(indices);
        }

        return polygon;
    }

    /// <summary>
    /// Builds the incident boundary order used to trace connected contours.
    /// </summary>
    /// <param name="data">The fully sorted, paired output events.</param>
    /// <returns>The next incident event index for each boundary endpoint.</returns>
    private static ReadOnlySpan<int> PrecomputeIterationOrder(SweepEventPoolList data)
    {
        Span<int> map = new int[data.Count];

        int i = 0;
        while (i < data.Count)
        {
            SweepEvent xRef = data[i];

            // Find index range of R events
            int rFrom = i;
            while (i < data.Count && xRef.Point == data[i].Point && !data[i].Left)
            {
                i++;
            }

            int rUptoExclusive = i;

            // Find index range of L events
            int lFrom = i;
            while (i < data.Count && xRef.Point == data[i].Point)
            {
                if (!data[i].Left)
                {
                    throw new InvalidOperationException("Expected left event");
                }

                i++;
            }

            int lUptoExclusive = i;

            bool hasREvents = rUptoExclusive > rFrom;
            bool hasLEvents = lUptoExclusive > lFrom;

            if (hasREvents)
            {
                int rUpto = rUptoExclusive - 1;

                // Connect elements in [rFrom, rUpto) to larger index
                for (int j = rFrom; j < rUpto; j++)
                {
                    map[j] = j + 1;
                }

                // Special handling of *last* element: Connect either the last L event
                // or loop back to start of R events (if no L events).
                if (hasLEvents)
                {
                    map[rUpto] = lUptoExclusive - 1;
                }
                else
                {
                    map[rUpto] = rFrom;
                }
            }

            if (hasLEvents)
            {
                int lUpto = lUptoExclusive - 1;

                // Connect elements in (lFrom, lUpto] to lower index
                for (int j = lFrom + 1; j <= lUpto; j++)
                {
                    map[j] = j - 1;
                }

                // Special handling of *first* element: Connect either to the first R event
                // or loop back to end of L events (if no R events).
                if (hasREvents)
                {
                    map[lFrom] = rFrom;
                }
                else
                {
                    map[lFrom] = lUpto;
                }
            }
        }

        return map;
    }

    /// <summary>
    /// Marks a connected endpoint and records its owning output contour.
    /// </summary>
    /// <param name="sweepEvent">The connected endpoint.</param>
    /// <param name="processed">The traversal state.</param>
    /// <param name="pos">The endpoint position.</param>
    /// <param name="contourId">The output contour index.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void MarkProcessed(SweepEvent sweepEvent, Span<bool> processed, int pos, int contourId)
    {
        processed[pos] = true;
        sweepEvent.OutputContourId = contourId;
    }

    /// <summary>
    /// Initializes a contour based on its context in relation to previous events and contours.
    /// Implements the 4 cases of parent contours from the Martinez paper (Fig. 4).
    /// </summary>
    /// <param name="sweepEvent">The current sweep event.</param>
    /// <param name="polygon">The collection of contours processed so far.</param>
    /// <param name="contourId">The ID for the new contour.</param>
    /// <returns>The initialized <see cref="Contour"/>.</returns>
    private Contour InitializeContourFromContext(SweepEvent sweepEvent, Polygon polygon, int contourId)
    {
        Contour contour = [];
        if (this.contourOutput == ContourOutput.Nested)
        {
            if (sweepEvent.PrevInResult is SweepEvent lower)
            {
                int lowerId = lower.OutputContourId;
                Contour lowerContour = polygon[lowerId];
                bool filledAbove = lower.ResultTransition > 0;
                bool lowerExterior = (lowerContour.Depth & 1) == 0;

                // At a contour's leftmost point, the surviving lower boundary
                // locates its containing region. Crossing into the lower contour
                // makes it the owner; crossing out returns to that contour's owner.
                contour.ParentIndex = filledAbove == lowerExterior ? lowerId : lowerContour.ParentIndex;
                if (contour.ParentIndex is int parent)
                {
                    contour.Depth = polygon[parent].Depth + 1;
                    polygon[parent].AddHoleIndex(contourId);
                }
            }

            return contour;
        }

        // Check if there is a "previous in result" event
        if (sweepEvent.PrevInResult != null)
        {
            SweepEvent prevInResult = sweepEvent.PrevInResult;

            // It is valid to query PrevInResult's outputContourId because it must have already been processed
            int lowerContourId = prevInResult.OutputContourId;
            ResultTransition lowerResultTransition = prevInResult.ResultTransition;

            if (lowerResultTransition > 0)
            {
                // We are inside. Check if the lower contour is a hole or an exterior contour.
                Contour lowerContour = polygon[lowerContourId];

                if (lowerContour.ParentIndex != null)
                {
                    // The lower contour is a hole: Connect the new contour as a hole to its parent and use the same depth.
                    int parentContourId = lowerContour.ParentIndex.Value;
                    polygon[parentContourId].AddHoleIndex(contourId);
                    contour.ParentIndex = parentContourId;
                    contour.Depth = polygon[lowerContourId].Depth;
                }
                else
                {
                    // The lower contour is an exterior contour: Connect the new contour as a hole and increment depth.
                    polygon[lowerContourId].AddHoleIndex(contourId);
                    contour.ParentIndex = lowerContourId;
                    contour.Depth = polygon[lowerContourId].Depth + 1;
                }
            }
            else
            {
                // Exterior/hole grouping starts each exterior at depth zero.
                contour.ParentIndex = null;
                contour.Depth = 0;
            }
        }
        else
        {
            // There is no "previous in result" event: This contour is an exterior contour with depth 0.
            contour.ParentIndex = null;
            contour.Depth = 0;
        }

        return contour;
    }

    /// <summary>
    /// Finds the next unprocessed boundary event incident to the current point.
    /// </summary>
    /// <param name="pos">The current position in the result events.</param>
    /// <param name="processed">Whether each boundary event has already been traced.</param>
    /// <param name="iterationMap">The cyclic order of incident boundary events.</param>
    /// <param name="found">Whether an unprocessed incident event was found.</param>
    /// <returns>The next event index, or <see cref="int.MinValue"/> when all incident events are processed.</returns>
    /// <remarks>
    /// The incident order selects the boundary continuation around the filled region.
    /// Returning to the starting event without finding an unprocessed event ends the search.
    /// </remarks>
    private static int NextPos(
        int pos,
        ReadOnlySpan<bool> processed,
        ReadOnlySpan<int> iterationMap,
        out bool found)
    {
        int startPos = pos;

        while (true)
        {
            pos = iterationMap[pos];
            if (pos == startPos)
            {
                // Entire group is already processed?
                found = false;
                return int.MinValue;
            }

            if (!processed[pos])
            {
                found = true;
                return pos;
            }
        }
    }

    /// <summary>
    /// Stores separate subject and clip contributions without cross-operand cancellation.
    /// </summary>
    internal readonly struct WindingCount
    {
        /// <summary>
        /// The subject winding.
        /// </summary>
        public readonly int Subject;

        /// <summary>
        /// The clipping operand winding.
        /// </summary>
        public readonly int Clip;

        /// <summary>
        /// Initializes a new instance of the <see cref="WindingCount"/> struct.
        /// </summary>
        /// <param name="subject">The subject component.</param>
        /// <param name="clip">The clipping component.</param>
        public WindingCount(int subject, int clip)
        {
            this.Subject = subject;
            this.Clip = clip;
        }

        /// <summary>
        /// Gets a value indicating whether both contributions cancel.
        /// </summary>
        public bool IsZero => this.Subject == 0 && this.Clip == 0;

        /// <summary>
        /// Adds an interval's contribution.
        /// </summary>
        /// <param name="other">The contribution to add.</param>
        /// <returns>The summed winding.</returns>
        public WindingCount Add(WindingCount other) => new(this.Subject + other.Subject, this.Clip + other.Clip);

        /// <summary>
        /// Removes contributions terminating at an endpoint.
        /// </summary>
        /// <param name="other">The terminating contributions.</param>
        /// <returns>The remaining winding.</returns>
        public WindingCount Subtract(WindingCount other) => new(this.Subject - other.Subject, this.Clip - other.Clip);
    }

    /// <summary>
    /// Stores a versioned endpoint in the existing priority queue.
    /// </summary>
    internal readonly struct QueueEntry
    {
        /// <summary>
        /// The queued geometric point.
        /// </summary>
        public readonly Vertex Point;

        /// <summary>
        /// The associated interval.
        /// </summary>
        public readonly SweepEvent Edge;

        /// <summary>
        /// The interval version when queued.
        /// </summary>
        public readonly int Version;

        /// <summary>
        /// Whether this record starts a source interval.
        /// </summary>
        public readonly bool IsStart;

        /// <summary>
        /// Initializes a new instance of the <see cref="QueueEntry"/> struct.
        /// </summary>
        /// <param name="point">The endpoint.</param>
        /// <param name="edge">The interval.</param>
        /// <param name="isStart">Whether the record starts the interval.</param>
        public QueueEntry(Vertex point, SweepEvent edge, bool isStart)
        {
            this.Point = point;
            this.Edge = edge;
            this.Version = edge.Version;
            this.IsStart = isStart;
        }

        /// <summary>
        /// Gets a value indicating whether the representative still owns this queued record.
        /// </summary>
        public bool IsValid => this.Version == this.Edge.Version && (this.IsStart || this.Edge.Active);
    }
}

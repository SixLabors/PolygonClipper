// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace SixLabors.PolygonClipper;

/// <summary>
/// Performs Boolean clipping on closed filled boundaries and open paths.
/// </summary>
/// <remarks>
/// <para>
/// Fill rules interpret the original directions of closed contours. Open contours have no filled interior.
/// Input vertices and hierarchy are not modified. Results own their contour data.
/// </para>
/// <para>
/// Instance members are not thread-safe for concurrent use.
/// </para>
/// <para>
/// Closed output uses positive orientation regardless of the input rules:
/// counterclockwise exteriors and islands, and clockwise holes in Cartesian coordinates.
/// Use NonZero, EvenOdd, or Positive filling to reuse that resolved area.
/// Closed output uses implicit closure without a repeated closing vertex.
/// Open output has no closing edge. A one-vertex open contour represents an isolated point intersection.
/// Line pieces retain their cut endpoints. An explicit line cycle retains its repeated terminal vertex and remains open.
/// </para>
/// <para>
/// Normalize returns the full containment hierarchy. Binary operations group each exterior contour with its holes.
/// Open output has no parent or holes. Closed-only clipping returns filled-area boundaries.
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
    private readonly SweepEventPoolList outgoing = [];
    private readonly SweepEventPoolList bundled = [];
    private readonly SweepEventPoolList pendingBoundaries = [];
    private readonly Stack<PriorityQueue<EndpointContribution, Vertex>> endpointQueues = new();
    private int retainedEndpointCapacity;
    private int activeBoundaryCount;

    // Output buffers are initialized only for open input. Live membership and
    // endpoint changes stay on the source intervals held by the status line.
    private List<Vertex> openPoints = null!;
    private List<OpenNode> openNodes = null!;
    private List<OpenEdge> openEdges = null!;
    private int openPointLines;
    private int openPointAreas;
    private bool openPointAreaBoundary;
    private bool openPointAreaResult;
    private bool openPointLineResult;
    private int openPointNode;
    private int openLineEndpoints;
    private bool hasOpenContours;

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
    /// Supplies a compile-time switch for open output in the shared sweep.
    /// </summary>
    private interface IOpenPathMode
    {
        /// <summary>
        /// Gets a value indicating whether the sweep collects open output.
        /// </summary>
        static abstract bool Enabled { get; }
    }

    /// <summary>
    /// Intersects two operands using a common fill rule for their closed contours.
    /// </summary>
    /// <param name="subject">The left-hand operand.</param>
    /// <param name="clip">The right-hand operand.</param>
    /// <param name="fillRule">The rule interpreting both operands.</param>
    /// <returns>The shared filled areas, open line pieces, and isolated open-path intersection points.</returns>
    public static Polygon Intersection(Polygon subject, Polygon clip, FillRule fillRule)
        => Intersection(subject, clip, fillRule, fillRule);

    /// <summary>
    /// Intersects two operands using independent fill rules for their closed contours.
    /// </summary>
    /// <param name="subject">The left-hand operand.</param>
    /// <param name="clip">The right-hand operand.</param>
    /// <param name="subjectFillRule">The subject's fill rule.</param>
    /// <param name="clipFillRule">The clipping operand's fill rule.</param>
    /// <returns>The shared filled areas, open line pieces, and isolated open-path intersection points.</returns>
    public static Polygon Intersection(Polygon subject, Polygon clip, FillRule subjectFillRule, FillRule clipFillRule)
        => ExecuteFillOperation(subject, clip, BooleanOperation.Intersection, subjectFillRule, clipFillRule, ContourOutput.Grouped);

    /// <summary>
    /// Unites two operands using a common fill rule for their closed contours.
    /// </summary>
    /// <param name="subject">The left-hand operand.</param>
    /// <param name="clip">The right-hand operand.</param>
    /// <param name="fillRule">The rule interpreting both operands.</param>
    /// <returns>The filled areas and open path pieces belonging to either operand.</returns>
    public static Polygon Union(Polygon subject, Polygon clip, FillRule fillRule)
        => Union(subject, clip, fillRule, fillRule);

    /// <summary>
    /// Unites two operands using independent fill rules for their closed contours.
    /// </summary>
    /// <param name="subject">The left-hand operand.</param>
    /// <param name="clip">The right-hand operand.</param>
    /// <param name="subjectFillRule">The subject's fill rule.</param>
    /// <param name="clipFillRule">The clipping operand's fill rule.</param>
    /// <returns>The filled areas and open path pieces belonging to either operand.</returns>
    public static Polygon Union(Polygon subject, Polygon clip, FillRule subjectFillRule, FillRule clipFillRule)
        => ExecuteFillOperation(subject, clip, BooleanOperation.Union, subjectFillRule, clipFillRule, ContourOutput.Grouped);

    /// <summary>
    /// Subtracts one operand from another using a common fill rule for their closed contours.
    /// </summary>
    /// <param name="subject">The area to retain.</param>
    /// <param name="clip">The area to subtract.</param>
    /// <param name="fillRule">The rule interpreting both operands.</param>
    /// <returns>The subject's retained filled areas and open path pieces, including their cut endpoints.</returns>
    public static Polygon Difference(Polygon subject, Polygon clip, FillRule fillRule)
        => Difference(subject, clip, fillRule, fillRule);

    /// <summary>
    /// Subtracts one operand from another using independent fill rules for their closed contours.
    /// </summary>
    /// <param name="subject">The area to retain.</param>
    /// <param name="clip">The area to subtract.</param>
    /// <param name="subjectFillRule">The subject's fill rule.</param>
    /// <param name="clipFillRule">The clipping operand's fill rule.</param>
    /// <returns>The subject's retained filled areas and open path pieces, including their cut endpoints.</returns>
    public static Polygon Difference(Polygon subject, Polygon clip, FillRule subjectFillRule, FillRule clipFillRule)
        => ExecuteFillOperation(subject, clip, BooleanOperation.Difference, subjectFillRule, clipFillRule, ContourOutput.Grouped);

    /// <summary>
    /// Computes exclusive geometry using a common fill rule for closed contours.
    /// </summary>
    /// <param name="subject">The left-hand operand.</param>
    /// <param name="clip">The right-hand operand.</param>
    /// <param name="fillRule">The rule interpreting both operands.</param>
    /// <returns>The selected filled areas and open path pieces, including their cut endpoints.</returns>
    public static Polygon Xor(Polygon subject, Polygon clip, FillRule fillRule)
        => Xor(subject, clip, fillRule, fillRule);

    /// <summary>
    /// Computes exclusive geometry using independent fill rules for closed contours.
    /// </summary>
    /// <param name="subject">The left-hand operand.</param>
    /// <param name="clip">The right-hand operand.</param>
    /// <param name="subjectFillRule">The subject's fill rule.</param>
    /// <param name="clipFillRule">The clipping operand's fill rule.</param>
    /// <returns>The selected filled areas and open path pieces, including their cut endpoints.</returns>
    public static Polygon Xor(Polygon subject, Polygon clip, FillRule subjectFillRule, FillRule clipFillRule)
        => ExecuteFillOperation(subject, clip, BooleanOperation.Xor, subjectFillRule, clipFillRule, ContourOutput.Grouped);

    /// <summary>
    /// Resolves self-intersections and overlaps using the specified fill rule for closed contours.
    /// </summary>
    /// <param name="polygon">The source contours. Their vertices and directions are not modified.</param>
    /// <param name="fillRule">The rule that determines the source polygon's filled area.</param>
    /// <returns>
    /// Filled boundaries with their full containment hierarchy, plus uncovered open path pieces and isolated points.
    /// Exterior contours and islands are counterclockwise. Holes are clockwise.
    /// Closed contours do not repeat their first vertex at the end.
    /// </returns>
    /// <remarks>
    /// The fill rule interprets the input. Output orientation is positive even when the input rule is Negative.
    /// Input hierarchy does not override contour direction or the selected fill rule.
    /// Use NonZero, EvenOdd, or Positive filling when reusing the cleaned polygon.
    /// </remarks>
    public static Polygon Normalize(Polygon polygon, FillRule fillRule)
        => ExecuteFillOperation(polygon, EmptyFillOperand, BooleanOperation.Union, fillRule, FillRule.EvenOdd, ContourOutput.Nested);

    /// <summary>
    /// Executes the configured Boolean operation using the operand fill rules for closed contours.
    /// </summary>
    /// <returns>The clipped filled boundaries, open paths, and isolated points.</returns>
    /// <remarks>Instance execution is not thread-safe for concurrent use.</remarks>
    public Polygon Run()
    {
        Polygon subject = this.subject!;
        Polygon clip = this.clipping!;
        BooleanOperation operation = this.operation;
        SweepEventPoolList resultEvents = this.sortedEvents;
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

        // The value-type mode makes open output a compile-time choice for the JIT.
        // Both modes use the same Martinez sweep. Closed-only machine code does
        // not carry point/line output checks or their extra live local variables.
        return this.hasOpenContours
            ? this.RunSweep<WithOpenPaths>(subjectBounds, clipBounds)
            : this.RunSweep<WithoutOpenPaths>(subjectBounds, clipBounds);
    }

    /// <summary>
    /// Runs the shared clipping sweep with open output enabled only for inputs that contain open contours.
    /// </summary>
    /// <typeparam name="TOpenPaths">The compile-time switch for open output.</typeparam>
    /// <param name="subjectBounds">The subject bounds collected during ingestion.</param>
    /// <param name="clipBounds">The clipping bounds collected during ingestion.</param>
    /// <returns>The clipped geometry.</returns>
    private Polygon RunSweep<TOpenPaths>(Box2 subjectBounds, Box2 clipBounds)
        where TOpenPaths : struct, IOpenPathMode
    {
        BooleanOperation operation = this.operation;
        FillRule subjectRule = this.subjectFillRule;
        FillRule clipRule = this.clipFillRule;
        SweepEventPoolList resultEvents = this.sortedEvents;
        SweepEventComparer resultComparer = this.comparer;
        bool open = TOpenPaths.Enabled;
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
            if (this.events.Peek().Point.X > stopX && this.activeBoundaryCount == 0)
            {
                break;
            }

            QueueEntry next = this.events.Dequeue();
            SweepEvent queuedEdge = this.segments[next.EdgeIndex];
            if (!next.IsValid(queuedEdge))
            {
                continue;
            }

            Vertex point = next.Point;
            if (open)
            {
                this.openPointLines = this.openPointAreas = 0;
                this.openPointAreaBoundary = this.openPointAreaResult = this.openPointLineResult = false;
                this.openPointNode = -1;
            }

            bool sourcePoint = next.IsStart || point == queuedEdge.SourceEnd;
            this.outgoing.Clear();
            this.pendingBoundaries.Clear();
            int firstEnd = int.MaxValue;
            int lastEnd = -1;
            if (next.IsStart)
            {
                if (open && queuedEdge.Point == queuedEdge.SourceEnd)
                {
                    this.ObserveOpenSource(queuedEdge);
                }
                else
                {
                    this.outgoing.Add(queuedEdge);
                }
            }
            else
            {
                firstEnd = lastEnd = queuedEdge.Pos;
            }

            // A vertex is a transaction, not a sequence of parity toggles.
            // Collect every start/end at this exact lexicographic point.
            while (this.events.Count > 0 && this.events.Peek().Point == point)
            {
                next = this.events.Dequeue();
                queuedEdge = this.segments[next.EdgeIndex];
                bool valid = next.IsValid(queuedEdge);
                if (valid && (next.IsStart || point == queuedEdge.SourceEnd))
                {
                    sourcePoint = true;
                }

                if (valid && next.IsStart)
                {
                    if (open && queuedEdge.Point == queuedEdge.SourceEnd)
                    {
                        this.ObserveOpenSource(queuedEdge);
                    }
                    else
                    {
                        this.outgoing.Add(queuedEdge);
                    }
                }
                else if (valid)
                {
                    firstEnd = Math.Min(firstEnd, queuedEdge.Pos);
                    lastEnd = Math.Max(lastEnd, queuedEdge.Pos);
                }
            }

            int first = lastEnd < 0 ? this.statusLine.FindLower(point) : firstEnd;
            if (open)
            {
                foreach (SweepEvent source in this.outgoing)
                {
                    this.ObserveOpenSource(source);
                }

                this.ObserveOpenRegion(first == 0 ? default : this.statusLine[first - 1].Above);
            }

            while (first > 0 && this.statusLine[first - 1].ContainsPoint(point))
            {
                first--;
            }

            int after = Math.Max(first, lastEnd + 1);
            while (after < this.statusLine.Count && this.statusLine[after].ContainsPoint(point))
            {
                after++;
            }

            // The surviving lower interval is outside the changed block.
            // Closure conserves the sum of signed contributions, so its
            // prefix and every surviving prefix above the block stay valid.
            for (int i = first; i < after; i++)
            {
                this.StageIncoming(this.statusLine[i], point, open);
            }

            int statusCount = this.statusLine.Count;
            int blockCount = after - first;

            while (true)
            {
                this.BundleOutgoing();
                this.statusLine.ReplaceRange(first, blockCount, this.bundled.GetEventsSpan());
                blockCount = this.bundled.Count;
                for (int i = 0; i < this.bundled.Count; i++)
                {
                    SweepEvent edge = this.bundled[i];
                    edge.Active = true;
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

                this.outgoing.Clear();
                this.outgoing.AddRange(this.bundled);
                foreach (SweepEvent edge in this.outgoing)
                {
                    // Rebundling can replace a representative. Invalidate any
                    // future endpoint queued while checking the staged block.
                    edge.Active = false;
                    edge.Version++;
                    edge.EndQueued = false;
                }

                if (lowerIncident)
                {
                    SweepEvent incoming = this.statusLine[--first];
                    blockCount++;
                    this.StageIncoming(incoming, point, open);
                }

                if (upperIncident)
                {
                    SweepEvent incoming = this.statusLine[first + blockCount];
                    blockCount++;
                    this.StageIncoming(incoming, point, open);
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
            bool below = InResult(winding, operation, subjectRule, clipRule);
            SweepEvent? previousBoundary = null;
            bool hasBoundaryContext = false;
            for (int i = 0; i < this.bundled.Count; i++)
            {
                SweepEvent edge = this.bundled[i];
                winding = winding.Add(edge.Delta);
                edge.Above = winding;
                bool above = InResult(winding, operation, subjectRule, clipRule);
                if (open)
                {
                    this.SelectOpenOutgoing(edge, winding.Subtract(edge.Delta), below, above);
                }

                if (below != above)
                {
                    // Interior crossings emit nothing and need no output
                    // predecessor. For a boundary, the first contributing
                    // interval below this complete block owns that context.
                    if (!hasBoundaryContext)
                    {
                        for (int j = first - 1; j >= 0; j--)
                        {
                            SweepEvent lower = this.statusLine[j];
                            if (lower.Boundary is not null)
                            {
                                previousBoundary = lower.Point.X == lower.End.X ? lower.Boundary.PrevInResult : lower.Boundary;
                                break;
                            }
                        }

                        hasBoundaryContext = true;
                    }

                    // Endpoint pairing is completed when the interval ends.
                    // No geometry consumer reads this event until connection.
                    edge.Boundary = resultEvents.Add(
                        point,
                        true,
                        null!,
                        above ? ResultTransition.Contributing : ResultTransition.NonContributing,
                        previousBoundary!,
                        edge.ContourId);
                    this.activeBoundaryCount++;

                    if (edge.Point.X != edge.End.X)
                    {
                        previousBoundary = edge.Boundary;
                    }
                }

                // The next interval starts in this interval's upper region.
                // Carry its membership instead of evaluating the same winding twice.
                below = above;

                // Neighbor intersection discovery can already have queued this
                // version's shortened endpoint. Keep one record per version.
                if (!edge.EndQueued)
                {
                    this.events.Enqueue(new QueueEntry(edge.End, edge, false));
                    edge.EndQueued = true;
                }

                edge.Pos = first + i;
            }

            // Crossings keep full-status positions unchanged. Only births,
            // deaths, and changing bundles move the unchanged suffix.
            if (this.statusLine.Count != statusCount)
            {
                for (int i = first + this.bundled.Count; i < this.statusLine.Count; i++)
                {
                    SweepEvent edge = this.statusLine[i];
                    edge.Pos = i;
                }
            }

            if (open && this.openPointLines != 0)
            {
                this.CompleteOpenPoint(point);
            }

            // Each source interval owns at most one valid queued start or end.
            // Above twice the interval count, at least half the heap is obsolete.
            // Rebuild only then, so each scan is paid for by discarded versions
            // and old endpoints cannot grow the queue with every intersection.
            if (this.eventStorage.Count > this.segments.Count * 2)
            {
                Span<QueueEntry> queued = CollectionsMarshal.AsSpan(this.eventStorage);
                int count = 0;
                for (int i = 0; i < queued.Length; i++)
                {
                    if (queued[i].IsValid(this.segments[queued[i].EdgeIndex]))
                    {
                        queued[count++] = queued[i];
                    }
                }

                this.eventStorage.RemoveRange(count, this.eventStorage.Count - count);
                this.events.Heapify();
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

        if (open)
        {
            this.AppendOpenOutput(result);
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
           this.outgoing.Capacity + this.bundled.Capacity +
           this.pendingBoundaries.Capacity + this.retainedEndpointCapacity +
           (this.openPoints?.Capacity ?? 0) + (this.openNodes?.Capacity ?? 0) + (this.openEdges?.Capacity ?? 0);

    /// <summary>
    /// Counts the edges selected by a contour's explicit closure state.
    /// </summary>
    /// <param name="contour">The contour to inspect.</param>
    /// <returns>The number of segments to iterate.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static int GetContourSegmentCount(Contour contour)
    {
        int count = contour.Count;
        if (!contour.IsClosed)
        {
            return Math.Max(0, count - 1);
        }

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
        if (this.hasOpenContours)
        {
            this.openPoints.Clear();
            this.openNodes.Clear();
            this.openEdges.Clear();
            this.openLineEndpoints = 0;
        }

        this.hasOpenContours = false;
        this.eventStorage.Clear();
        this.statusLine.Reset(0);
        this.outgoing.Clear();
        this.bundled.Clear();
        this.pendingBoundaries.Clear();
        this.activeBoundaryCount = 0;
        for (int i = 0; i < this.segments.Count; i++)
        {
            if (this.segments[i].Ends is PriorityQueue<EndpointContribution, Vertex> ends)
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
            bool open = !contour.IsClosed;
            if (open && contour.Count > 0)
            {
                this.openPoints ??= [];
                this.openNodes ??= [];
                this.openEdges ??= [];
                this.hasOpenContours = true;

                // Closed ingestion visits every vertex as a segment start.
                // Open ingestion also validates the terminal vertex, which is
                // never a start and must not bypass the public finite-coordinate contract.
                Vertex terminal = contour[contour.Count - 1];
                if (!double.IsFinite(terminal.X) || !double.IsFinite(terminal.Y))
                {
                    throw new ArgumentException("Polygon coordinates must be finite.", nameof(polygon));
                }

                if (contour.Count == 1)
                {
                    bounds = bounds.Add(new Box2(terminal));
                    SweepEvent source = this.segments.Add(terminal, terminal, default);
                    source.Lines = subject ? new(1, 0) : new(0, 1);
                    this.eventStorage.Add(new QueueEntry(terminal, source, true));
                    continue;
                }

                bounds = bounds.Add(new Box2(terminal));
            }

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
                    if (open)
                    {
                        SweepEvent source = this.segments.Add(start, end, default);
                        source.Lines = subject ? new(1, 0) : new(0, 1);
                        this.eventStorage.Add(new QueueEntry(start, source, true));
                    }

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
                // Open edges participate in intersection events without changing
                // area winding. Their unsigned operand counts are tracked separately.
                WindingCount delta = open ? default : subject ? new(direction, 0) : new(0, direction);
                SweepEvent edge = this.segments.Add(start, end, delta);
                if (open)
                {
                    edge.Lines = subject ? new(1, 0) : new(0, 1);
                }

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
    /// <param name="open">Whether open output is collected in this sweep.</param>
    // Open-path bookkeeping pushes this method above the JIT's normal inline budget.
    // Keep interval retirement in the sweep loop, as it was for closed-only clipping.
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void StageIncoming(SweepEvent incoming, Vertex point, bool open)
    {
        if (open)
        {
            this.StageOpenIncoming(incoming, point);
        }

        incoming.Active = false;
        incoming.Version++;
        incoming.EndQueued = false;
        if (incoming.Boundary is SweepEvent boundary)
        {
            this.pendingBoundaries.Add(boundary);
            this.activeBoundaryCount--;
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
        if (incoming.Ends is PriorityQueue<EndpointContribution, Vertex> ends)
        {
            WindingCount delta = incoming.Delta;
            WindingCount lines = incoming.Lines;
            while (ends.TryPeek(out EndpointContribution ending, out Vertex endpoint) && endpoint == point)
            {
                delta = delta.Subtract(ending.Area);
                lines = lines.Subtract(ending.Lines);
                ends.Dequeue();
            }

            if (ends.Count == 0)
            {
                incoming.Ends = null;
                this.ReturnEndpoints(ends);
                return;
            }

            incoming.Delta = delta;
            incoming.Lines = lines;
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
    /// Orders outgoing rays and combines coincident intervals without losing longer continuations.
    /// </summary>
    private void BundleOutgoing()
    {
        Span<SweepEvent> outgoing = this.outgoing.GetEventsSpan();
        int pairOrder = 0;

        // Most crossings have two outgoing rays. Their order also tells us
        // whether they coincide, so reuse that comparison when bundling.
        if (outgoing.Length == 2)
        {
            pairOrder = SegmentComparer.CompareRays(outgoing[0], outgoing[1]);
            if (pairOrder > 0)
            {
                (outgoing[0], outgoing[1]) = (outgoing[1], outgoing[0]);
            }
        }
        else if (outgoing.Length > 2)
        {
            outgoing.Sort(SegmentComparer.CompareRays);
        }

        this.bundled.Clear();
        int i = 0;
        while (i < outgoing.Length)
        {
            SweepEvent edge = outgoing[i++];
            while (i < outgoing.Length && (outgoing.Length == 2 ? pairOrder == 0 : SegmentComparer.CompareRays(edge, outgoing[i]) == 0))
            {
                SweepEvent other = outgoing[i++];
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
                    edge.Lines = edge.Lines.Add(other.Lines);
                    continue;
                }

                if (edge.Ends is null)
                {
                    edge.Ends = this.RentEndpoints();
                    if (!edge.Delta.IsZero || !edge.Lines.IsZero)
                    {
                        edge.Ends.Enqueue(new EndpointContribution(edge.Delta, edge.Lines), edge.SourceEnd);
                    }
                }

                PriorityQueue<EndpointContribution, Vertex> ends = edge.Ends;
                if (other.Ends is PriorityQueue<EndpointContribution, Vertex> otherEnds)
                {
                    foreach ((EndpointContribution delta, Vertex endpoint) in otherEnds.UnorderedItems)
                    {
                        ends.Enqueue(delta, endpoint);
                    }

                    other.Ends = null;
                    this.ReturnEndpoints(otherEnds);
                }
                else if (!other.Delta.IsZero || !other.Lines.IsZero)
                {
                    ends.Enqueue(new EndpointContribution(other.Delta, other.Lines), other.SourceEnd);
                }

                edge.Delta = edge.Delta.Add(other.Delta);
                edge.Lines = edge.Lines.Add(other.Lines);
            }

            if (edge.Ends is PriorityQueue<EndpointContribution, Vertex> ledger)
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
            else if (edge.Delta.IsZero && edge.Lines.IsZero)
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
    private static Vertex FirstEndpoint(PriorityQueue<EndpointContribution, Vertex> ends)
    {
        ends.TryPeek(out _, out Vertex endpoint);
        return endpoint;
    }

    /// <summary>
    /// Rents endpoint storage without allocating tree nodes for each contribution.
    /// </summary>
    /// <returns>An empty reusable priority queue.</returns>
    private PriorityQueue<EndpointContribution, Vertex> RentEndpoints()
    {
        if (this.endpointQueues.TryPop(out PriorityQueue<EndpointContribution, Vertex>? ends))
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
    private void ReturnEndpoints(PriorityQueue<EndpointContribution, Vertex> ends)
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

            // Line membership is unsigned even when this tiny area contribution reverses.
            tail.Lines = edge.Lines;

            this.events.Enqueue(new QueueEntry(tail.Point, tail, true));
            edge.SourceEnd = intersection;
        }

        // SourceEnd/Ends retain the source's real terminal changes. End only
        // limits this open interval; its continuation is made at the new node.
        edge.End = intersection;
        edge.Version++;
        this.events.Enqueue(new QueueEntry(intersection, edge, false));
        edge.EndQueued = true;
    }

    /// <summary>
    /// Evaluates Boolean membership from independent signed winding counts.
    /// </summary>
    /// <param name="count">The region's winding counts.</param>
    /// <param name="operation">The Boolean operation.</param>
    /// <param name="subjectRule">The subject's fill rule.</param>
    /// <param name="clipRule">The clipping operand's fill rule.</param>
    /// <returns>Whether the region belongs to the result.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static bool InResult(WindingCount count, BooleanOperation operation, FillRule subjectRule, FillRule clipRule)
    {
        return operation switch
        {
            BooleanOperation.Intersection => Filled(count.Subject, subjectRule) && Filled(count.Clip, clipRule),
            BooleanOperation.Union => Filled(count.Subject, subjectRule) || Filled(count.Clip, clipRule),
            BooleanOperation.Difference => Filled(count.Subject, subjectRule) && !Filled(count.Clip, clipRule),
            _ => Filled(count.Subject, subjectRule) ^ Filled(count.Clip, clipRule)
        };
    }

    /// <summary>
    /// Interprets a winding count using a validated fill rule.
    /// </summary>
    /// <param name="count">The signed winding count.</param>
    /// <param name="rule">The validated fill rule.</param>
    /// <returns>Whether the region is filled.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static bool Filled(int count, FillRule rule)
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

        using Buffer<int> iterationBuffer = new(resultEvents.Count);
        Span<int> iterationMap = iterationBuffer.GetSpan();
        PrecomputeIterationOrder(resultEvents, iterationMap);

        Polygon result = [];
        using Buffer<bool> processedBuffer = new(resultEvents.Count);
        Span<bool> processed = processedBuffer.GetSpan();

        // Every iteration-map slot is assigned, but processed flags must
        // start clear because pooled storage can contain flags from a prior run.
        processed.Clear();
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

                // Closing the contour needs no further edge. Searching here
                // would revisit processed rays at a shared starting vertex.
                if (point == initial)
                {
                    break;
                }

                pos = NextPos(pos, processed, iterationMap, out bool found);
                if (!found)
                {
                    break;
                }
            }
            while (true);

            result.Add(contour);
        }

        if (this.contourOutput == ContourOutput.Nested)
        {
            return result;
        }

        Polygon polygon = new(result.Count);
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
    /// <param name="map">Receives the next incident event index for every boundary endpoint.</param>
    private static void PrecomputeIterationOrder(SweepEventPoolList data, Span<int> map)
    {
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
        // A triangle needs three vertices and its closing vertex during tracing.
        // Reserve those slots instead of growing the list on the first Add.
        Contour contour = new(4);
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
    /// Retains an isolated point or marks the line junctions at a completed event.
    /// </summary>
    /// <param name="point">The completed event point.</param>
    private void CompleteOpenPoint(Vertex point)
    {
        if (this.openPointNode >= 0)
        {
            ref OpenNode node = ref CollectionsMarshal.AsSpan(this.openNodes)[this.openPointNode];
            node.Split = this.openPointLines == 3 || this.openPointAreaBoundary;
            if (node.Degree != 2 || node.Split)
            {
                this.openLineEndpoints += node.Degree;
            }
        }

        int membership = this.openPointLines | this.openPointAreas;
        if (!this.openPointAreaResult && !this.openPointLineResult && SelectOpenMembership((membership & 1) != 0, (membership & 2) != 0, this.operation))
        {
            this.openPoints.Add(point);
        }
    }

    /// <summary>
    /// Completes an open output piece before its live interval is continued or reused.
    /// </summary>
    /// <param name="incoming">The incoming interval.</param>
    /// <param name="point">The completed endpoint.</param>
    private void StageOpenIncoming(SweepEvent incoming, Vertex point)
    {
        // Keep open output construction outside the closed-only sweep's hot loop.
        // The same sweep supplies its completed points, intervals, and area membership.
        this.ObserveOpenSource(incoming);
        this.ObserveOpenRegion(incoming.Above);
        this.ObserveOpenRegion(incoming.Above.Subtract(incoming.Delta));
        if (incoming.LineIncluded && incoming.Point != point)
        {
            int end = this.GetOpenPointNode(point);
            ref OpenNode first = ref CollectionsMarshal.AsSpan(this.openNodes)[incoming.LineStartNode];
            ref OpenNode last = ref CollectionsMarshal.AsSpan(this.openNodes)[end];
            int index = this.openEdges.Count;
            this.openEdges.Add(new OpenEdge(incoming.LineStartNode, end, first.First, last.First));
            first.First = last.First = index;
            last.Degree++;
            this.openPointLineResult = true;
        }

        incoming.LineIncluded = false;
    }

    /// <summary>
    /// Selects an outgoing line piece and reserves its output incidence at this event.
    /// </summary>
    /// <param name="edge">The outgoing interval.</param>
    /// <param name="areaWindingBelow">The area winding below the interval.</param>
    /// <param name="areaBelow">Whether the result contains the area below the interval.</param>
    /// <param name="areaAbove">Whether the result contains the area above the interval.</param>
    private void SelectOpenOutgoing(SweepEvent edge, WindingCount areaWindingBelow, bool areaBelow, bool areaAbove)
    {
        this.ObserveOpenSource(edge);
        this.ObserveOpenRegion(areaWindingBelow);
        this.ObserveOpenRegion(edge.Above);
        if (edge.Lines.IsZero)
        {
            edge.LineIncluded = false;
        }
        else
        {
            // A boundary belongs to a filled operand when either adjacent sector
            // is filled. A retained area already represents any line on its boundary.
            bool lineSubject = edge.Lines.Subject > 0 || Filled(areaWindingBelow.Subject, this.subjectFillRule) ||
                Filled(edge.Above.Subject, this.subjectFillRule);
            bool lineClip = edge.Lines.Clip > 0 || Filled(areaWindingBelow.Clip, this.clipFillRule) ||
                Filled(edge.Above.Clip, this.clipFillRule);
            edge.LineIncluded = !areaBelow && !areaAbove && SelectOpenMembership(lineSubject, lineClip, this.operation);
            if (edge.LineIncluded)
            {
                edge.LineStartNode = this.GetOpenPointNode(edge.Point);

                // The completed point already knows each selected outgoing ray.
                // Count its incidence now; its adjacency link is completed when
                // the interval ends. No coordinate dictionary or second pass is needed.
                CollectionsMarshal.AsSpan(this.openNodes)[edge.LineStartNode].Degree++;
            }
        }

        this.openPointLineResult |= edge.LineIncluded;
    }

    /// <summary>
    /// Includes a source endpoint or a one-vertex input in the current point event.
    /// </summary>
    /// <param name="edge">The incident source interval.</param>
    private void ObserveOpenSource(SweepEvent edge)
    {
        WindingCount lines = edge.Lines;
        this.openPointLines |= (lines.Subject > 0 ? 1 : 0) | (lines.Clip > 0 ? 2 : 0);
        this.openPointAreaBoundary |= !edge.Delta.IsZero;
    }

    /// <summary>
    /// Records filled sectors without treating line contributions as winding.
    /// </summary>
    /// <param name="winding">The winding in an incident sector.</param>
    private void ObserveOpenRegion(WindingCount winding)
    {
        this.openPointAreas |= (Filled(winding.Subject, this.subjectFillRule) ? 1 : 0) |
            (Filled(winding.Clip, this.clipFillRule) ? 2 : 0);
        this.openPointAreaResult |= InResult(winding, this.operation, this.subjectFillRule, this.clipFillRule);
    }

    /// <summary>
    /// Applies the Boolean operation to line or point membership without signed area winding.
    /// </summary>
    /// <param name="subject">Whether the subject contains this location.</param>
    /// <param name="clip">Whether the clipping operand contains this location.</param>
    /// <param name="operation">The Boolean operation.</param>
    /// <returns>Whether this location belongs to the result.</returns>
    private static bool SelectOpenMembership(bool subject, bool clip, BooleanOperation operation)
        => InResult(new(subject ? 1 : 0, clip ? 1 : 0), operation, FillRule.NonZero, FillRule.NonZero);

    /// <summary>
    /// Appends open contours and isolated points after the closed contours have their final hierarchy indices.
    /// </summary>
    /// <param name="result">The result that owns the generated contours.</param>
    private void AppendOpenOutput(Polygon result)
    {
        // Every non-cycle line has two endpoint incidences. Complete-point
        // processing counted them, so reserve their contours without another
        // graph scan. Endpoint-free cycles can grow the result normally.
        result.EnsureCapacity(result.Count + (this.openLineEndpoints / 2) + this.openPoints.Count);

        // Join ordinary degree-two source vertices. Every intersection node
        // and branch remains an endpoint, so separate pieces retain their cut vertex.
        for (int i = 0; i < this.openNodes.Count; i++)
        {
            OpenNode node = this.openNodes[i];
            if (node.Degree == 2 && !node.Split)
            {
                continue;
            }

            int edge = node.First;
            while (edge >= 0)
            {
                if (!this.openEdges[edge].Visited)
                {
                    result.Add(this.TraceOpenPath(i, edge));
                }

                edge = this.NextOpenEdge(edge, i);
            }
        }

        // A line can form a cycle without an implicit closing edge. Trace the
        // remaining cycles and retain their explicit terminal source vertex.
        for (int i = 0; i < this.openEdges.Count; i++)
        {
            if (!this.openEdges[i].Visited)
            {
                result.Add(this.TraceOpenPath(this.openEdges[i].Start, i));
            }
        }

        foreach (Vertex point in this.openPoints)
        {
            Contour contour = new(1) { IsClosed = false };
            contour.Add(point);
            result.Add(contour);
        }
    }

    /// <summary>
    /// Gets the current complete point's output node without a coordinate lookup.
    /// </summary>
    /// <param name="point">The shared endpoint.</param>
    /// <returns>The stable output node index.</returns>
    private int GetOpenPointNode(Vertex point)
    {
        if (this.openPointNode < 0)
        {
            // The sweep completes every incidence at this exact point before
            // advancing. All selected rays therefore share this one node index.
            this.openPointNode = this.openNodes.Count;
            this.openNodes.Add(new OpenNode(point));
        }

        return this.openPointNode;
    }

    /// <summary>
    /// Gets the next edge in one node's incident list.
    /// </summary>
    /// <param name="edge">The current incident edge.</param>
    /// <param name="node">The node owning this adjacency list.</param>
    /// <returns>The next incident edge index, or -1 at the end.</returns>
    private int NextOpenEdge(int edge, int node)
    {
        OpenEdge current = this.openEdges[edge];
        return current.Start == node ? current.NextStart : current.NextEnd;
    }

    /// <summary>
    /// Traces a maximal unbranched line between cut endpoints.
    /// </summary>
    /// <param name="start">The first node.</param>
    /// <param name="index">The first unvisited incident edge.</param>
    /// <returns>An open contour that owns its vertices.</returns>
    private Contour TraceOpenPath(int start, int index)
    {
        Contour contour = new(2) { IsClosed = false };
        contour.Add(this.openNodes[start].Point);
        int node = start;
        while (true)
        {
            ref OpenEdge edge = ref CollectionsMarshal.AsSpan(this.openEdges)[index];
            edge.Visited = true;
            node = edge.Start == node ? edge.End : edge.Start;
            OpenNode next = this.openNodes[node];
            contour.Add(next.Point);
            if (node == start || next.Degree != 2 || next.Split)
            {
                return contour;
            }

            index = next.First == index ? this.NextOpenEdge(index, node) : next.First;
        }
    }

    /// <summary>
    /// Removes open output work when the JIT compiles the closed-only sweep.
    /// </summary>
    private readonly struct WithoutOpenPaths : IOpenPathMode
    {
        /// <summary>
        /// Gets a value indicating whether the sweep collects open output.
        /// </summary>
        public static bool Enabled => false;
    }

    /// <summary>
    /// Includes open output work when the JIT compiles the shared sweep.
    /// </summary>
    private readonly struct WithOpenPaths : IOpenPathMode
    {
        /// <summary>
        /// Gets a value indicating whether the sweep collects open output.
        /// </summary>
        public static bool Enabled => true;
    }

    /// <summary>
    /// Stores the incident output edges of one exact vertex.
    /// </summary>
    private struct OpenNode
    {
        /// <summary>
        /// The node coordinate.
        /// </summary>
        public Vertex Point;

        /// <summary>
        /// The first incident edge index, or -1 for a new node.
        /// </summary>
        public int First;

        /// <summary>
        /// The number of incident output edges.
        /// </summary>
        public int Degree;

        /// <summary>
        /// Whether this event must remain a cut endpoint.
        /// </summary>
        public bool Split;

        /// <summary>
        /// Initializes a new instance of the <see cref="OpenNode"/> struct.
        /// </summary>
        /// <param name="point">The exact output vertex.</param>
        public OpenNode(Vertex point)
        {
            this.Point = point;
            this.First = -1;
            this.Degree = 0;
            this.Split = false;
        }
    }

    /// <summary>
    /// Stores one undirected line piece and its two adjacency links.
    /// </summary>
    private struct OpenEdge
    {
        /// <summary>
        /// The first endpoint's node index.
        /// </summary>
        public int Start;

        /// <summary>
        /// The second endpoint's node index.
        /// </summary>
        public int End;

        /// <summary>
        /// The next edge incident to Start.
        /// </summary>
        public int NextStart;

        /// <summary>
        /// The next edge incident to End.
        /// </summary>
        public int NextEnd;

        /// <summary>
        /// Whether output traversal has consumed this edge.
        /// </summary>
        public bool Visited;

        /// <summary>
        /// Initializes a new instance of the <see cref="OpenEdge"/> struct.
        /// </summary>
        /// <param name="start">The first endpoint index.</param>
        /// <param name="end">The second endpoint index.</param>
        /// <param name="nextStart">The previous adjacency head at Start.</param>
        /// <param name="nextEnd">The previous adjacency head at End.</param>
        public OpenEdge(int start, int end, int nextStart, int nextEnd)
        {
            this.Start = start;
            this.End = end;
            this.NextStart = nextStart;
            this.NextEnd = nextEnd;
            this.Visited = false;
        }
    }
}

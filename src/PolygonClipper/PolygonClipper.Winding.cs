// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

namespace SixLabors.PolygonClipper;

/// <summary>Provides native signed-winding execution for polygon clipping.</summary>
public partial class PolygonClipper
{
    /// <summary>Compares points in the formal sweep's lexicographic order.</summary>
    /// <param name="first">The first point.</param>
    /// <param name="second">The second point.</param>
    /// <returns>Their sweep ordering.</returns>
    private static int ComparePoints(Vertex first, Vertex second)
    {
        int x = first.X.CompareTo(second.X);
        return x == 0 ? first.Y.CompareTo(second.Y) : x;
    }

    /// <summary>Stores separate subject and clip contributions without cross-operand cancellation.</summary>
    private readonly struct WindingCount
    {
        /// <summary>The subject winding.</summary>
        public readonly int Subject;

        /// <summary>The clipping operand winding.</summary>
        public readonly int Clip;

        /// <summary>Initializes a new instance of the <see cref="WindingCount"/> struct.</summary>
        /// <param name="subject">The subject component.</param>
        /// <param name="clip">The clipping component.</param>
        public WindingCount(int subject, int clip)
        {
            this.Subject = subject;
            this.Clip = clip;
        }

        /// <summary>Gets a value indicating whether both contributions cancel.</summary>
        public bool IsZero => this.Subject == 0 && this.Clip == 0;

        /// <summary>Adds an interval's contribution.</summary>
        /// <param name="other">The contribution to add.</param>
        /// <returns>The summed winding.</returns>
        public WindingCount Add(WindingCount other) => new(this.Subject + other.Subject, this.Clip + other.Clip);

        /// <summary>Removes contributions terminating at an endpoint.</summary>
        /// <param name="other">The terminating contributions.</param>
        /// <returns>The remaining winding.</returns>
        public WindingCount Subtract(WindingCount other) => new(this.Subject - other.Subject, this.Clip - other.Clip);
    }

    /// <summary>Stores a versioned endpoint in the existing priority queue.</summary>
    private readonly struct WindingEvent
    {
        /// <summary>The queued geometric point.</summary>
        public readonly Vertex Point;

        /// <summary>The associated interval.</summary>
        public readonly WindingEdge Edge;

        /// <summary>The interval version when queued.</summary>
        public readonly int Version;

        /// <summary>Whether this record starts a source interval.</summary>
        public readonly bool IsStart;

        /// <summary>Initializes a new instance of the <see cref="WindingEvent"/> struct.</summary>
        /// <param name="point">The endpoint.</param>
        /// <param name="edge">The interval.</param>
        /// <param name="isStart">Whether the record starts the interval.</param>
        public WindingEvent(Vertex point, WindingEdge edge, bool isStart)
        {
            this.Point = point;
            this.Edge = edge;
            this.Version = edge.Version;
            this.IsStart = isStart;
        }

        /// <summary>Gets a value indicating whether the representative still owns this queued record.</summary>
        public bool IsValid => this.Version == this.Edge.Version && (this.IsStart || this.Edge.Active);
    }

    /// <summary>
    /// Stores the additional state needed by explicit fill rules. Legacy parity calls
    /// do not allocate this workspace or enlarge their individual sweep events.
    /// </summary>
    private sealed class WindingSweep
    {
        private readonly List<WindingEvent> eventStorage = [];
        private readonly StablePriorityQueue<WindingEvent, WindingEventComparer> events;
        private readonly WindingEdgePool edges = new();
        private readonly List<WindingEdge> active = [];
        private readonly List<WindingEdge> activeResult = [];
        private readonly List<WindingEdge> outgoing = [];
        private readonly List<WindingEdge> bundled = [];

        /// <summary>Initializes a new instance of the <see cref="WindingSweep"/> class.</summary>
        public WindingSweep() => this.events = new(WindingEventComparer.Instance, this.eventStorage);

        /// <summary>Gets the storage retained by this native sweep.</summary>
        public int RetainedCapacity => this.edges.Capacity + this.eventStorage.Capacity + this.active.Capacity +
            this.activeResult.Capacity + this.outgoing.Capacity + this.bundled.Capacity;

        /// <summary>Releases per-operation references while retaining bounded scratch storage.</summary>
        public void Reset()
        {
            this.eventStorage.Clear();
            this.active.Clear();
            this.activeResult.Clear();
            this.outgoing.Clear();
            this.bundled.Clear();
            for (int i = 0; i < this.edges.Count; i++)
            {
                this.edges[i].Ends = null;
                this.edges[i].Boundary = null;
            }

            this.edges.Clear();
        }

        /// <summary>Clips raw directed contours in one Martinez intersection sweep.</summary>
        /// <param name="subject">The subject contours.</param>
        /// <param name="clip">The clipping contours.</param>
        /// <param name="operation">The Boolean operation.</param>
        /// <param name="subjectRule">The subject's fill rule.</param>
        /// <param name="clipRule">The clipping operand's fill rule.</param>
        /// <param name="resultEvents">The existing clipper's output-event storage.</param>
        /// <param name="resultComparer">The existing edge-connection event comparer.</param>
        /// <returns>The connected result boundaries.</returns>
        public Polygon Run(
            Polygon subject,
            Polygon clip,
            BooleanOperation operation,
            FillRule subjectRule,
            FillRule clipRule,
            SweepEventPoolList resultEvents,
            SweepEventComparer resultComparer)
        {
            this.Reset();
            if (operation == BooleanOperation.Intersection && (subject.Count == 0 || clip.Count == 0))
            {
                return [];
            }

            // Reuse the existing capacity APIs and reserve source-sized storage
            // before ingestion. Repeated geometric growth otherwise copies the
            // larger endpoint records and can exceed the shared retention bound.
            int vertexCount = subject.VertexCount + clip.VertexCount;
            this.eventStorage.EnsureCapacity(vertexCount);
            this.edges.EnsureCapacity(vertexCount);
            resultEvents.EnsureCapacity(vertexCount * 2);

            this.AddInput(subject, true);
            this.AddInput(clip, false);
            this.events.Heapify();

            while (this.events.Count > 0)
            {
                WindingEvent next = this.events.Dequeue();
                if (!next.IsValid)
                {
                    continue;
                }

                Vertex point = next.Point;
                this.outgoing.Clear();
                int firstEnd = int.MaxValue;
                int lastEnd = -1;
                if (next.IsStart)
                {
                    this.outgoing.Add(next.Edge);
                }
                else
                {
                    firstEnd = lastEnd = next.Edge.Position;
                }

                // A vertex is a transaction, not a sequence of parity toggles.
                // Collect every start/end at this exact lexicographic point.
                while (this.events.Count > 0 && this.events.Peek().Point == point)
                {
                    next = this.events.Dequeue();
                    if (next.IsValid && next.IsStart)
                    {
                        this.outgoing.Add(next.Edge);
                    }
                    else if (next.IsValid)
                    {
                        firstEnd = Math.Min(firstEnd, next.Edge.Position);
                        lastEnd = Math.Max(lastEnd, next.Edge.Position);
                    }
                }

                int first = lastEnd < 0 ? FindLower(this.active, point) : firstEnd;
                while (first > 0 && ContainsPoint(this.active[first - 1], point))
                {
                    first--;
                }

                int after = Math.Max(first, lastEnd + 1);
                while (after < this.active.Count && ContainsPoint(this.active[after], point))
                {
                    after++;
                }

                // The result index is a filtered view of the very same block.
                // Derive removals from the owned interval identities, so rounded
                // incidence tests cannot remove a boundary independently of its
                // source interval and leave a stale birth context behind.
                int firstResult = first == 0 ? 0 : this.active[first - 1].ResultRank;
                int afterResult = firstResult;
                for (int i = first; i < after; i++)
                {
                    WindingEdge incoming = this.active[i];
                    if (incoming.Boundary is not null)
                    {
                        afterResult++;
                    }
                }

                // The surviving lower interval is outside the changed block.
                // Closure conserves the sum of signed contributions, so its
                // prefix and every surviving prefix above the block stay valid.
                WindingCount winding = first == 0 ? default : this.active[first - 1].Above;
                SweepEvent? previousBoundary = firstResult == 0 ? null : this.activeResult[firstResult - 1].Boundary;
                if (firstResult > 0 && this.activeResult[firstResult - 1].Start.X == this.activeResult[firstResult - 1].End.X)
                {
                    previousBoundary = previousBoundary!.PrevInResult;
                }

                for (int i = first; i < after; i++)
                {
                    WindingEdge incoming = this.active[i];
                    incoming.Active = false;
                    incoming.Version++;
                    if (incoming.Boundary is SweepEvent boundary)
                    {
                        // A historical contribution is never rewritten when its
                        // continuation changes fill membership. Only its actual
                        // terminal endpoint becomes known at this event.
                        SweepEvent right = new(point, false, boundary, PolygonType.Subject)
                        {
                            ContourId = boundary.ContourId
                        };
                        boundary.OtherEvent = right;
                        resultEvents.Add(boundary);
                        resultEvents.Add(right);
                    }

                    this.ContinueAt(incoming, point);
                }

                this.active.RemoveRange(first, after - first);
                this.activeResult.RemoveRange(firstResult, afterResult - firstResult);

                // All active intervals through point were included above, not
                // merely those named by queued events. Outgoing rays all start
                // here, so non-collinear rays cannot meet again in their interiors.
                this.outgoing.Sort(WindingRayComparer.Instance);
                this.BundleOutgoing();

                int resultIndex = firstResult;
                for (int i = 0; i < this.bundled.Count; i++)
                {
                    WindingEdge edge = this.bundled[i];
                    bool below = Membership(winding, operation, subjectRule, clipRule);
                    winding = winding.Add(edge.Delta);
                    edge.Above = winding;
                    bool above = Membership(winding, operation, subjectRule, clipRule);
                    if (below != above)
                    {
                        // Endpoint pairing is completed when the interval ends.
                        // No geometry consumer reads this event until connection.
                        edge.Boundary = new SweepEvent(point, true, PolygonType.Subject)
                        {
                            ContourId = edge.Id,
                            PrevInResult = previousBoundary!,
                            ResultTransition = above ? ResultTransition.Contributing : ResultTransition.NonContributing
                        };
                        if (edge.Start.X != edge.End.X)
                        {
                            previousBoundary = edge.Boundary;
                        }

                        this.activeResult.Insert(resultIndex++, edge);
                    }

                    edge.Active = true;
                    this.active.Insert(first + i, edge);
                    this.events.Enqueue(new WindingEvent(edge.End, edge, false));
                }

                // List insertion/removal already shifts this suffix. Keep the
                // endpoint anchors and filtered-boundary ranks in the same order.
                // Winding counts above the changed block do not need recomputing.
                int resultRank = firstResult;
                for (int i = first; i < this.active.Count; i++)
                {
                    WindingEdge edge = this.active[i];
                    edge.Position = i;
                    if (edge.Boundary is not null)
                    {
                        resultRank++;
                    }

                    edge.ResultRank = resultRank;
                }

                // Interior outgoing pairs share this already closed event point.
                // Only the two new exterior adjacencies can expose a future crossing.
                if (this.bundled.Count == 0)
                {
                    this.CheckNeighbors(first - 1, first, point);
                }
                else
                {
                    this.CheckNeighbors(first - 1, first, point);
                    this.CheckNeighbors(first + this.bundled.Count - 1, first + this.bundled.Count, point);
                }
            }

            // Native events are completed at their right endpoints. Restore the
            // ordering expected by the existing connector in O(B log B), rather
            // than making its nearly-sorted repair loop sort reversed history.
            resultEvents.Sort(resultComparer);
            Polygon result = ConnectEdges(resultEvents, resultComparer);
            for (int i = 0; i < result.Count; i++)
            {
                Contour contour = result[i];
                if (contour.Count > 1 && contour[0] == contour[^1])
                {
                    contour.RemoveVertexAt(contour.Count - 1);
                }

                if (contour.IsExternal)
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

        /// <summary>Adds directed input segments without normalizing or reorienting them.</summary>
        /// <param name="polygon">The input contours.</param>
        /// <param name="subject">Whether these contours belong to the subject.</param>
        private void AddInput(Polygon polygon, bool subject)
        {
            for (int i = 0; i < polygon.Count; i++)
            {
                Contour contour = polygon[i];
                int count = GetContourSegmentCount(contour);
                for (int j = 0; j < count; j++)
                {
                    Segment segment = contour.GetSegment(j);
                    Vertex start = segment.Source;
                    Vertex end = segment.Target;
                    if (!double.IsFinite(start.X) || !double.IsFinite(start.Y))
                    {
                        throw new ArgumentException("Polygon coordinates must be finite.", nameof(polygon));
                    }

                    if (start == end)
                    {
                        continue;
                    }

                    int direction = 1;
                    if (ComparePoints(start, end) > 0)
                    {
                        (start, end) = (end, start);
                        direction = -1;
                    }

                    // Lexicographic direction is the formal x+epsilon*y sweep
                    // direction, including vertical source segments. Epsilon is
                    // never evaluated or applied to the stored double coordinates.
                    WindingCount delta = subject ? new(direction, 0) : new(0, direction);
                    WindingEdge edge = this.edges.Add(start, end, delta);
                    this.eventStorage.Add(new WindingEvent(start, edge, true));
                }
            }
        }

        /// <summary>Preserves a split interval's longer source tails and endpoint deltas.</summary>
        /// <param name="incoming">The interval ending at this point.</param>
        /// <param name="point">The complete event point.</param>
        private void ContinueAt(WindingEdge incoming, Vertex point)
        {
            if (incoming.Ends is SortedDictionary<Vertex, WindingCount> ends)
            {
                WindingCount delta = incoming.Delta;
                if (ends.Remove(point, out WindingCount ending))
                {
                    delta = delta.Subtract(ending);
                }

                if (ends.Count == 0)
                {
                    incoming.Ends = null;
                    return;
                }

                incoming.Delta = delta;
                incoming.SourceEnd = FirstEndpoint(ends);
            }
            else if (ComparePoints(point, incoming.SourceEnd) >= 0)
            {
                return;
            }

            // Output history owns independent SweepEvent objects. Reuse only the
            // live interval, whose version was advanced before closing its old
            // piece. Stale queue records stay invalid without retaining a new
            // interval object for every intersection in the operation.
            incoming.Start = point;
            incoming.End = incoming.SourceEnd;
            incoming.Boundary = null;
            this.outgoing.Add(incoming);
        }

        /// <summary>Combines coincident outgoing intervals without losing longer continuations.</summary>
        private void BundleOutgoing()
        {
            this.bundled.Clear();
            int i = 0;
            while (i < this.outgoing.Count)
            {
                WindingEdge edge = this.outgoing[i++];
                while (i < this.outgoing.Count && WindingRayComparer.Instance.Compare(edge, this.outgoing[i]) == 0)
                {
                    WindingEdge other = this.outgoing[i++];

                    // A bundle may outlive its representative's original segment.
                    // Its support must cover every retained endpoint-ledger tail.
                    if (ComparePoints(other.Support.Target, edge.Support.Target) > 0)
                    {
                        edge.Support = new Segment(edge.Support.Source, other.Support.Target);
                    }

                    if (edge.Ends is null && other.Ends is null && edge.SourceEnd == other.SourceEnd)
                    {
                        // The common duplicate-edge case needs no endpoint ledger.
                        edge.Delta = edge.Delta.Add(other.Delta);
                        continue;
                    }

                    SortedDictionary<Vertex, WindingCount> ends = edge.Ends ??=
                        new SortedDictionary<Vertex, WindingCount>(WindingPointComparer.Instance)
                        {
                            [edge.SourceEnd] = edge.Delta
                        };
                    if (other.Ends is SortedDictionary<Vertex, WindingCount> otherEnds)
                    {
                        foreach (KeyValuePair<Vertex, WindingCount> endpoint in otherEnds)
                        {
                            AddEndpoint(ends, endpoint.Key, endpoint.Value);
                        }

                        other.Ends = null;
                    }
                    else
                    {
                        AddEndpoint(ends, other.SourceEnd, other.Delta);
                    }

                    edge.Delta = edge.Delta.Add(other.Delta);
                }

                if (edge.Ends is SortedDictionary<Vertex, WindingCount> ledger)
                {
                    if (ledger.Count == 0)
                    {
                        continue;
                    }

                    edge.SourceEnd = edge.End = FirstEndpoint(ledger);
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

        /// <summary>Accumulates one coincident contribution's terminal change.</summary>
        /// <param name="ends">The bundle's future endpoint changes.</param>
        /// <param name="point">The terminal point.</param>
        /// <param name="delta">The contribution ending there.</param>
        private static void AddEndpoint(SortedDictionary<Vertex, WindingCount> ends, Vertex point, WindingCount delta)
        {
            if (ends.TryGetValue(point, out WindingCount existing))
            {
                delta = delta.Add(existing);
            }

            if (delta.IsZero)
            {
                ends.Remove(point);
            }
            else
            {
                ends[point] = delta;
            }
        }

        /// <summary>Gets the next terminal change of a non-empty bundle ledger.</summary>
        /// <param name="ends">The non-empty endpoint ledger.</param>
        /// <returns>The first endpoint.</returns>
        private static Vertex FirstEndpoint(SortedDictionary<Vertex, WindingCount> ends)
        {
            using SortedDictionary<Vertex, WindingCount>.Enumerator enumerator = ends.GetEnumerator();
            enumerator.MoveNext();
            return enumerator.Current.Key;
        }

        /// <summary>Schedules the next intersection of newly adjacent active intervals.</summary>
        /// <param name="lower">The lower neighbor's index.</param>
        /// <param name="upper">The upper neighbor's index.</param>
        /// <param name="point">The event already processed.</param>
        private void CheckNeighbors(int lower, int upper, Vertex point)
        {
            if (lower < 0 || upper >= this.active.Count)
            {
                return;
            }

            WindingEdge first = this.active[lower];
            WindingEdge second = this.active[upper];

            // Keep the original support lines across splits. Reconstructing a
            // line from a rounded intersection rotates it and can create a second,
            // slightly different intersection at an existing source vertex.
            if (first.LineId > second.LineId)
            {
                (first, second) = (second, first);
            }

            int count = PolygonUtilities.FindIntersectionPrecise(
                first.Support, second.Support, out Vertex intersection, out Vertex last);
            if (count > 0)
            {
                // Match the established Martinez repair: a rounded crossing on
                // this x coordinate but below the completed event cannot be
                // visited backwards. Advance x by one representable double.
                if (intersection.X == point.X && intersection.Y < point.Y)
                {
                    intersection = new Vertex(intersection.X.NextAfter(double.PositiveInfinity), intersection.Y);
                }

                if (ComparePoints(intersection, first.Start) < 0 || ComparePoints(intersection, second.Start) < 0 ||
                    (ComparePoints(intersection, first.End) > 0 && (first.End != first.SourceEnd || intersection.X != first.End.X)) ||
                    (ComparePoints(intersection, second.End) > 0 && (second.End != second.SourceEnd || intersection.X != second.End.X)))
                {
                    return;
                }

                this.ScheduleSplit(first, intersection, point);
                this.ScheduleSplit(second, intersection, point);
                if (count == 2)
                {
                    this.ScheduleSplit(first, last, point);
                    this.ScheduleSplit(second, last, point);
                }
            }
        }

        /// <summary>Shortens a live interval to its earliest discovered future node.</summary>
        /// <param name="edge">The live interval.</param>
        /// <param name="intersection">The discovered node.</param>
        /// <param name="current">The completed event point.</param>
        private void ScheduleSplit(WindingEdge edge, Vertex intersection, Vertex current)
        {
            if (ComparePoints(intersection, current) <= 0 || intersection == edge.End)
            {
                return;
            }

            if (ComparePoints(intersection, edge.End) > 0)
            {
                if (edge.End != edge.SourceEnd || intersection.X != edge.End.X || edge.Start.X == edge.End.X)
                {
                    return;
                }

                // The second established Martinez rounding case reverses the
                // tiny final piece: the source endpoint precedes the rounded
                // crossing on the same x coordinate. Represent that piece as a
                // new forward interval with the opposite signed contribution.
                WindingEdge tail = this.edges.Add(edge.SourceEnd, intersection, default(WindingCount).Subtract(edge.Delta));
                this.events.Enqueue(new WindingEvent(tail.Start, tail, true));
                edge.SourceEnd = intersection;
            }

            // SourceEnd/Ends retain the source's real terminal changes. End only
            // limits this open interval; its continuation is made at the new node.
            edge.End = intersection;
            edge.Version++;
            this.events.Enqueue(new WindingEvent(intersection, edge, false));
        }

        /// <summary>Finds the first live interval not strictly below an event point.</summary>
        /// <param name="status">The already ordered status list.</param>
        /// <param name="point">The event point.</param>
        /// <returns>The lower insertion bound.</returns>
        private static int FindLower(List<WindingEdge> status, Vertex point)
        {
            int left = 0;
            int right = status.Count;
            while (left < right)
            {
                int middle = left + ((right - left) >> 1);
                WindingEdge edge = status[middle];
                if (!ContainsPoint(edge, point) && PolygonUtilities.SignedArea(edge.Start, edge.End, point) > 0)
                {
                    left = middle + 1;
                }
                else
                {
                    right = middle;
                }
            }

            return left;
        }

        /// <summary>Tests incidence on both the noded interval and its preserved source support.</summary>
        /// <param name="edge">The active interval.</param>
        /// <param name="point">The event point.</param>
        /// <returns>Whether the interval participates in this event.</returns>
        private static bool ContainsPoint(WindingEdge edge, Vertex point)
        {
            // The two equivalent determinant origins can round differently.
            // Preserve endpoint incidence on either representation, and source
            // vertices on the support even after a rounded split shortened End.
            return ComparePoints(edge.Start, point) <= 0 && ComparePoints(point, edge.End) <= 0 &&
               (PolygonUtilities.SignedArea(edge.Start, edge.End, point) == 0 ||
                Vertex.Cross(edge.End - edge.Start, point - edge.Start) == 0 ||
                Vertex.Cross(edge.Support.Target - edge.Support.Source, point - edge.Support.Source) == 0);
        }

        /// <summary>Evaluates Boolean membership from independent signed winding counts.</summary>
        /// <param name="count">The region's winding counts.</param>
        /// <param name="operation">The Boolean operation.</param>
        /// <param name="subjectRule">The subject's fill rule.</param>
        /// <param name="clipRule">The clipping operand's fill rule.</param>
        /// <returns>Whether the region belongs to the result.</returns>
        private static bool Membership(WindingCount count, BooleanOperation operation, FillRule subjectRule, FillRule clipRule)
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

        /// <summary>Interprets a winding count using a validated fill rule.</summary>
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
    }

    /// <summary>Stores one live native interval, separately from completed output history.</summary>
    private sealed class WindingEdge
    {
        /// <summary>Gets or sets the stable identity within this operation.</summary>
        public int Id { get; set; }

        /// <summary>Gets or sets the canonical start point.</summary>
        public Vertex Start { get; set; }

        /// <summary>Gets or sets the next discovered endpoint or intersection.</summary>
        public Vertex End { get; set; }

        /// <summary>Gets or sets the next actual source contribution endpoint.</summary>
        public Vertex SourceEnd { get; set; }

        /// <summary>Gets or sets the original directed support, unaffected by rounded split points.</summary>
        public Segment Support { get; set; }

        /// <summary>Gets or sets the original segment identity used to construct intersections consistently.</summary>
        public int LineId { get; set; }

        /// <summary>Gets or sets the live status position, used to anchor constructed intersections.</summary>
        public int Position { get; set; }

        /// <summary>Gets or sets the count of live result boundaries at or below this interval.</summary>
        public int ResultRank { get; set; }

        /// <summary>Gets or sets the signed contribution of the live interval.</summary>
        public WindingCount Delta { get; set; }

        /// <summary>Gets or sets the cached winding immediately above the live interval.</summary>
        public WindingCount Above { get; set; }

        /// <summary>Gets or sets the longer-tail endpoint ledger for a coincident bundle.</summary>
        public SortedDictionary<Vertex, WindingCount>? Ends { get; set; }

        /// <summary>Gets or sets the fixed contribution and birth context when this interval contributes.</summary>
        public SweepEvent? Boundary { get; set; }

        /// <summary>Gets or sets the version used to discard superseded queued endpoints.</summary>
        public int Version { get; set; }

        /// <summary>Gets or sets a value indicating whether this representative remains in the active status.</summary>
        public bool Active { get; set; }
    }

    /// <summary>Reuses native interval objects through the existing append-only pool mechanism.</summary>
    private sealed class WindingEdgePool : PooledList<WindingEdge>
    {
        /// <summary>Acquires an interval for a new open piece.</summary>
        /// <param name="start">The canonical start.</param>
        /// <param name="end">The source endpoint.</param>
        /// <param name="delta">The signed contribution.</param>
        /// <returns>The initialized interval.</returns>
        public WindingEdge Add(Vertex start, Vertex end, WindingCount delta)
        {
            this.TryGrow();
            WindingEdge edge = this.Items[this.Size] ??= new WindingEdge();
            edge.Id = this.Size++;
            edge.LineId = edge.Id;
            edge.Start = start;
            edge.End = edge.SourceEnd = end;
            edge.Support = new(start, end);
            edge.Delta = delta;
            edge.Above = default;
            edge.Ends = null;
            edge.Boundary = null;
            edge.Version = 0;
            edge.Active = false;
            return edge;
        }
    }

    /// <summary>Orders native endpoint records deterministically without changing coordinates.</summary>
    private sealed class WindingEventComparer : IComparer<WindingEvent>
    {
        /// <summary>The shared stateless comparer.</summary>
        public static readonly WindingEventComparer Instance = new();

        /// <inheritdoc/>
        public int Compare(WindingEvent x, WindingEvent y)
        {
            int order = ComparePoints(x.Point, y.Point);
            if (order == 0)
            {
                order = x.Edge.Id.CompareTo(y.Edge.Id);
            }

            if (order == 0)
            {
                order = x.Version.CompareTo(y.Version);
            }

            return order == 0 ? x.IsStart.CompareTo(y.IsStart) : order;
        }
    }

    /// <summary>Orders rays immediately after their shared event point.</summary>
    private sealed class WindingRayComparer : IComparer<WindingEdge>
    {
        /// <summary>The shared stateless comparer.</summary>
        public static readonly WindingRayComparer Instance = new();

        /// <inheritdoc/>
        public int Compare(WindingEdge? x, WindingEdge? y)
        {
            double cross = Vertex.Cross(x!.SourceEnd - x.Start, y!.SourceEnd - y.Start);
            return cross > 0 ? -1 : cross < 0 ? 1 : 0;
        }
    }

    /// <summary>Orders contribution endpoints within a coincident bundle.</summary>
    private sealed class WindingPointComparer : IComparer<Vertex>
    {
        /// <summary>The shared stateless comparer.</summary>
        public static readonly WindingPointComparer Instance = new();

        /// <inheritdoc/>
        public int Compare(Vertex x, Vertex y) => ComparePoints(x, y);
    }
}

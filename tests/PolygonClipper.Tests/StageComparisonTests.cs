// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

#nullable enable

using System.Collections;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using GeoJson.Feature;
using Xunit;

namespace SixLabors.PolygonClipper.Tests;

public class StageComparisonTests
{
    public static TheoryData<string> FailingTestCases()
        => new(
            "fatal3.geojson",
            "fatal4.geojson",
            "vertical_ulp_slopes1.geojson",
            "vertical_ulp_slopes2.geojson",
            "issue99.geojson",
            "issue103.geojson",
            "issue110.geojson",
            "daef_cross_selfintersecting.geojson",
            "overlap_y.geojson",
            "issue76.geojson");

    [Theory]
    [MemberData(nameof(FailingTestCases))]
    public void FillQueueStage_MatchesDoubleAndFixed(string testCaseFile)
    {
        FeatureCollection data = TestData.Generic.GetFeatureCollection(testCaseFile);
        (Polygon subject, Polygon clipping) = TestPolygonUtilities.BuildPolygon(data);

        FixedPrecisionContext context = FixedPrecisionContext.Create(null, [subject, clipping]);

        QueueSnapshot doubleSnapshot = StageDouble.BuildEventQueue(subject, clipping, context);
        QueueSnapshot fixedSnapshot = StageFixed.BuildEventQueue(subject, clipping, context);

        Assert.Equal(doubleSnapshot.SubjectBounds, fixedSnapshot.SubjectBounds);
        Assert.Equal(doubleSnapshot.ClippingBounds, fixedSnapshot.ClippingBounds);
        Assert.Equal(doubleSnapshot.Events.Count, fixedSnapshot.Events.Count);

        for (int i = 0; i < doubleSnapshot.Events.Count; i++)
        {
            EventSnapshot expected = doubleSnapshot.Events[i];
            EventSnapshot actual = fixedSnapshot.Events[i];
            Assert.Equal(expected, actual);
        }
    }

    [Theory]
    [MemberData(nameof(FailingTestCases))]
    public void PossibleIntersectionStage_MatchesDoubleAndFixed(string testCaseFile)
    {
        FeatureCollection data = TestData.Generic.GetFeatureCollection(testCaseFile);
        (Polygon subject, Polygon clipping) = TestPolygonUtilities.BuildPolygon(data);

        FixedPrecisionContext context = FixedPrecisionContext.Create(null, [subject, clipping]);

        List<DoubleSweepEvent> doubleEvents = StageDouble.BuildSortedEvents(subject, clipping, context);
        List<SweepEvent> fixedEvents = StageFixed.BuildSortedEvents(subject, clipping, context);

        bool found = TryFindIntersectingPair(doubleEvents, fixedEvents, context, out DoubleSweepEvent doubleA, out DoubleSweepEvent doubleB, out SweepEvent fixedA, out SweepEvent fixedB);
        if (!found)
        {
            return;
        }

        DoubleSweepEventComparer doubleComparer = new();
        StablePriorityQueue<DoubleSweepEvent, DoubleSweepEventComparer> doubleQueue = new(doubleComparer);
        int doubleResult = StageDouble.PossibleIntersection(doubleA, doubleB, doubleQueue);
        List<EventSnapshot> doubleInserted = DrainQueue(doubleQueue, context);

        SweepEventComparer fixedComparer = new();
        StablePriorityQueue<SweepEvent, SweepEventComparer> fixedQueue = new(fixedComparer);
        int fixedResult = StageFixed.PossibleIntersection(fixedA, fixedB, fixedQueue, context);
        List<EventSnapshot> fixedInserted = DrainQueue(fixedQueue);

        Assert.Equal(doubleResult, fixedResult);
        Assert.Equal(doubleInserted.Count, fixedInserted.Count);
        for (int i = 0; i < doubleInserted.Count; i++)
        {
            Assert.Equal(doubleInserted[i], fixedInserted[i]);
        }
    }

    [Theory]
    [MemberData(nameof(FailingTestCases))]
    public void SubdivideSegmentsStage_MatchesDoubleAndFixed(string testCaseFile)
    {
        FeatureCollection data = TestData.Generic.GetFeatureCollection(testCaseFile);
        (Polygon subject, Polygon clipping) = TestPolygonUtilities.BuildPolygon(data);

        FixedPrecisionContext context = FixedPrecisionContext.Create(null, [subject, clipping]);

        List<EventSnapshot> doubleSegments = StageDouble.SubdivideSegments(subject, clipping, context);
        List<EventSnapshot> fixedSegments = StageFixed.SubdivideSegments(subject, clipping, context);

        Assert.Equal(doubleSegments.Count, fixedSegments.Count);
        for (int i = 0; i < doubleSegments.Count; i++)
        {
            Assert.Equal(doubleSegments[i], fixedSegments[i]);
        }
    }

    [Theory]
    [MemberData(nameof(FailingTestCases))]
    public void ComputeFieldsStage_MatchesDoubleAndFixed(string testCaseFile)
    {
        FeatureCollection data = TestData.Generic.GetFeatureCollection(testCaseFile);
        (Polygon subject, Polygon clipping) = TestPolygonUtilities.BuildPolygon(data);

        FixedPrecisionContext context = FixedPrecisionContext.Create(null, [subject, clipping]);
        List<BooleanOperation> operations = GetOperations(data);

        for (int opIndex = 0; opIndex < operations.Count; opIndex++)
        {
            BooleanOperation operation = operations[opIndex];
            List<FieldSnapshot> doubleSnapshots = StageDouble.ComputeFieldSnapshots(subject, clipping, context, operation);
            List<FieldSnapshot> fixedSnapshots = StageFixed.ComputeFieldSnapshots(subject, clipping, context, operation);

            Assert.Equal(doubleSnapshots.Count, fixedSnapshots.Count);
            for (int i = 0; i < doubleSnapshots.Count; i++)
            {
                Assert.Equal(doubleSnapshots[i], fixedSnapshots[i]);
            }
        }
    }

    private readonly record struct EventSnapshot(
        bool Left,
        Vertex64 Point,
        Vertex64 OtherPoint,
        PolygonType PolygonType,
        int ContourId);

    private readonly record struct QueueSnapshot(
        Box64 SubjectBounds,
        Box64 ClippingBounds,
        List<EventSnapshot> Events);

    private readonly record struct FieldSnapshot(
        bool Left,
        Vertex64 Point,
        Vertex64 OtherPoint,
        PolygonType PolygonType,
        EdgeType EdgeType,
        bool InOut,
        bool OtherInOut,
        ResultTransition ResultTransition,
        int ContourId,
        bool HasPrev,
        Vertex64 PrevPoint,
        Vertex64 PrevOtherPoint);

    private static bool TryFindIntersectingPair(
        List<DoubleSweepEvent> doubleEvents,
        List<SweepEvent> fixedEvents,
        FixedPrecisionContext context,
        out DoubleSweepEvent doubleA,
        out DoubleSweepEvent doubleB,
        out SweepEvent fixedA,
        out SweepEvent fixedB)
    {
        doubleA = null!;
        doubleB = null!;
        fixedA = null!;
        fixedB = null!;

        Dictionary<EventSnapshot, List<SweepEvent>> fixedLookup = BuildEventLookup(fixedEvents);

        List<DoubleSweepEvent> leftEvents = [];
        for (int i = 0; i < doubleEvents.Count; i++)
        {
            if (doubleEvents[i].Left)
            {
                leftEvents.Add(doubleEvents[i]);
            }
        }

        int limit = Math.Min(leftEvents.Count, 256);
        for (int i = 0; i < limit; i++)
        {
            DoubleSweepEvent a = leftEvents[i];
            DoubleSegment segA = a.GetSegment();
            for (int j = i + 1; j < limit; j++)
            {
                DoubleSweepEvent b = leftEvents[j];
                if (DoublePolygonUtilities.FindIntersection(segA, b.GetSegment(), out _, out _) == 0)
                {
                    continue;
                }

                if (!TryMapFixedEvent(a, fixedLookup, context, out SweepEvent matchA) ||
                    !TryMapFixedEvent(b, fixedLookup, context, out SweepEvent matchB))
                {
                    continue;
                }

                doubleA = a;
                doubleB = b;
                fixedA = matchA;
                fixedB = matchB;
                return true;
            }
        }

        return false;
    }

    private static List<BooleanOperation> GetOperations(FeatureCollection data)
    {
        List<BooleanOperation> operations = [];
        for (int i = 2; i < data.Features.Count; i++)
        {
            Feature feature = data.Features[i];
            if (!feature.Properties.TryGetValue("operation", out object? rawOp))
            {
                continue;
            }

            string? mode = rawOp?.ToString();
            if (string.IsNullOrWhiteSpace(mode))
            {
                continue;
            }

            operations.Add(mode switch
            {
                "union" => BooleanOperation.Union,
                "intersection" => BooleanOperation.Intersection,
                "xor" => BooleanOperation.Xor,
                "diff" => BooleanOperation.Difference,
                "diff_ba" => BooleanOperation.Difference,
                _ => throw new InvalidOperationException($"Unsupported operation: {mode}")
            });
        }

        return operations;
    }

    private static Dictionary<EventSnapshot, List<SweepEvent>> BuildEventLookup(List<SweepEvent> events)
    {
        Dictionary<EventSnapshot, List<SweepEvent>> lookup = new();
        for (int i = 0; i < events.Count; i++)
        {
            SweepEvent sweepEvent = events[i];
            EventSnapshot snapshot = new(
                sweepEvent.Left,
                sweepEvent.Point,
                sweepEvent.OtherEvent.Point,
                sweepEvent.PolygonType,
                sweepEvent.ContourId);

            if (!lookup.TryGetValue(snapshot, out List<SweepEvent>? bucket))
            {
                bucket = [];
                lookup.Add(snapshot, bucket);
            }

            bucket.Add(sweepEvent);
        }

        return lookup;
    }

    private static bool TryMapFixedEvent(
        DoubleSweepEvent doubleEvent,
        Dictionary<EventSnapshot, List<SweepEvent>> lookup,
        FixedPrecisionContext context,
        out SweepEvent match)
    {
        EventSnapshot key = new(
            doubleEvent.Left,
            context.Quantize(doubleEvent.Point),
            context.Quantize(doubleEvent.OtherEvent.Point),
            doubleEvent.PolygonType,
            doubleEvent.ContourId);

        if (lookup.TryGetValue(key, out List<SweepEvent>? bucket) && bucket.Count > 0)
        {
            match = bucket[0];
            return true;
        }

        match = null!;
        return false;
    }

    private static List<EventSnapshot> DrainQueue(
        StablePriorityQueue<DoubleSweepEvent, DoubleSweepEventComparer> queue,
        FixedPrecisionContext context)
    {
        List<EventSnapshot> events = new(queue.Count);
        while (queue.Count > 0)
        {
            DoubleSweepEvent sweepEvent = queue.Dequeue();
            events.Add(new EventSnapshot(
                sweepEvent.Left,
                context.Quantize(sweepEvent.Point),
                context.Quantize(sweepEvent.OtherEvent.Point),
                sweepEvent.PolygonType,
                sweepEvent.ContourId));
        }

        return events;
    }

    private static List<EventSnapshot> DrainQueue(StablePriorityQueue<SweepEvent, SweepEventComparer> queue)
    {
        List<EventSnapshot> events = new(queue.Count);
        while (queue.Count > 0)
        {
            SweepEvent sweepEvent = queue.Dequeue();
            events.Add(new EventSnapshot(
                sweepEvent.Left,
                sweepEvent.Point,
                sweepEvent.OtherEvent.Point,
                sweepEvent.PolygonType,
                sweepEvent.ContourId));
        }

        return events;
    }

    private static class StageFixed
    {
        public static QueueSnapshot BuildEventQueue(Polygon subject, Polygon clipping, FixedPrecisionContext context)
        {
            PolygonUtilities.SetFloatingScale(context.InvScale);
            try
            {
                List<SweepEvent> unordered = [];
                Box64 subjectBounds = BuildEventQueue(subject, PolygonType.Subject, context, unordered);
                Box64 clippingBounds = BuildEventQueue(clipping, PolygonType.Clipping, context, unordered);

                SweepEventComparer comparer = new();
                StablePriorityQueue<SweepEvent, SweepEventComparer> queue = new(comparer, unordered);
                List<EventSnapshot> events = new(queue.Count);
                while (queue.Count > 0)
                {
                    SweepEvent sweepEvent = queue.Dequeue();
                    events.Add(new EventSnapshot(
                        sweepEvent.Left,
                        sweepEvent.Point,
                        sweepEvent.OtherEvent.Point,
                        sweepEvent.PolygonType,
                        sweepEvent.ContourId));
                }

                return new QueueSnapshot(subjectBounds, clippingBounds, events);
            }
            finally
            {
                PolygonUtilities.ClearFloatingScale();
            }
        }

        public static List<SweepEvent> BuildSortedEvents(Polygon subject, Polygon clipping, FixedPrecisionContext context)
        {
            PolygonUtilities.SetFloatingScale(context.InvScale);
            try
            {
                List<SweepEvent> unordered = [];
                BuildEventQueue(subject, PolygonType.Subject, context, unordered);
                BuildEventQueue(clipping, PolygonType.Clipping, context, unordered);

                SweepEventComparer comparer = new();
                StablePriorityQueue<SweepEvent, SweepEventComparer> queue = new(comparer, unordered);
                List<SweepEvent> events = new(queue.Count);
                while (queue.Count > 0)
                {
                    events.Add(queue.Dequeue());
                }

                return events;
            }
            finally
            {
                PolygonUtilities.ClearFloatingScale();
            }
        }

        public static List<EventSnapshot> SubdivideSegments(Polygon subject, Polygon clipping, FixedPrecisionContext context)
        {
            PolygonUtilities.SetFloatingScale(context.InvScale);
            try
            {
                List<SweepEvent> unordered = [];
                BuildEventQueue(subject, PolygonType.Subject, context, unordered);
                BuildEventQueue(clipping, PolygonType.Clipping, context, unordered);

                SweepEventComparer comparer = new();
                StablePriorityQueue<SweepEvent, SweepEventComparer> queue = new(comparer, unordered);
                StatusLine statusLine = new(unordered.Count >> 1);
                List<EventSnapshot> leftEvents = new();

                Span<SweepEvent> workspace = new SweepEvent[4];
                while (queue.Count > 0)
                {
                    SweepEvent sweepEvent = queue.Dequeue();
                    if (sweepEvent.Left)
                    {
                        int index = sweepEvent.PosSL = statusLine.Add(sweepEvent);
                        SweepEvent? prevEvent = statusLine.Prev(index);
                        SweepEvent? nextEvent = statusLine.Next(index);

                        if (nextEvent != null && PossibleIntersection(sweepEvent, nextEvent, queue, context) == 2)
                        {
                            // no-op for subdivision stage
                        }

                        if (prevEvent != null && PossibleIntersection(prevEvent, sweepEvent, queue, context) == 2)
                        {
                            // no-op for subdivision stage
                        }

                        leftEvents.Add(new EventSnapshot(
                            sweepEvent.Left,
                            sweepEvent.Point,
                            sweepEvent.OtherEvent.Point,
                            sweepEvent.PolygonType,
                            sweepEvent.ContourId));
                    }
                    else
                    {
                        sweepEvent = sweepEvent.OtherEvent;
                        int index = sweepEvent.PosSL;
                        SweepEvent? prevEvent = statusLine.Prev(index);
                        SweepEvent? nextEvent = statusLine.Next(index);
                        if (prevEvent != null && nextEvent != null)
                        {
                            _ = PossibleIntersection(prevEvent, nextEvent, queue, context);
                        }

                        statusLine.RemoveAt(index);
                    }
                }

                return leftEvents;
            }
            finally
            {
                PolygonUtilities.ClearFloatingScale();
            }
        }

        public static List<FieldSnapshot> ComputeFieldSnapshots(
            Polygon subject,
            Polygon clipping,
            FixedPrecisionContext context,
            BooleanOperation operation)
        {
            PolygonUtilities.SetFloatingScale(context.InvScale);
            try
            {
                List<SweepEvent> unordered = [];
                BuildEventQueue(subject, PolygonType.Subject, context, unordered);
                BuildEventQueue(clipping, PolygonType.Clipping, context, unordered);

                SweepEventComparer comparer = new();
                StablePriorityQueue<SweepEvent, SweepEventComparer> queue = new(comparer, unordered);
                StatusLine statusLine = new(unordered.Count >> 1);
                List<FieldSnapshot> snapshots = [];

                Span<SweepEvent> workspace = new SweepEvent[4];
                while (queue.Count > 0)
                {
                    SweepEvent sweepEvent = queue.Dequeue();
                    if (sweepEvent.Left)
                    {
                        int index = sweepEvent.PosSL = statusLine.Add(sweepEvent);
                        SweepEvent? prevEvent = statusLine.Prev(index);
                        SweepEvent? nextEvent = statusLine.Next(index);

                        ComputeFields(sweepEvent, prevEvent, operation);

                        if (nextEvent != null && PolygonClipperPossibleIntersection(sweepEvent, nextEvent, queue, workspace, context) == 2)
                        {
                            ComputeFields(sweepEvent, prevEvent, operation);
                            ComputeFields(nextEvent, sweepEvent, operation);
                        }

                        if (prevEvent != null && PolygonClipperPossibleIntersection(prevEvent, sweepEvent, queue, workspace, context) == 2)
                        {
                            SweepEvent? prevPrevEvent = statusLine.Prev(prevEvent.PosSL);
                            ComputeFields(prevEvent, prevPrevEvent, operation);
                            ComputeFields(sweepEvent, prevEvent, operation);
                        }

                        snapshots.Add(CreateFieldSnapshot(sweepEvent));
                    }
                    else
                    {
                        sweepEvent = sweepEvent.OtherEvent;
                        int index = sweepEvent.PosSL;
                        SweepEvent? prevEvent = statusLine.Prev(index);
                        SweepEvent? nextEvent = statusLine.Next(index);
                        if (prevEvent != null && nextEvent != null)
                        {
                            _ = PolygonClipperPossibleIntersection(prevEvent, nextEvent, queue, workspace, context);
                        }

                        statusLine.RemoveAt(index);
                    }
                }

                return snapshots;
            }
            finally
            {
                PolygonUtilities.ClearFloatingScale();
            }
        }

        private static Box64 BuildEventQueue(
            Polygon polygon,
            PolygonType polygonType,
            FixedPrecisionContext context,
            List<SweepEvent> eventQueue)
        {
            long minX = long.MaxValue;
            long minY = long.MaxValue;
            long maxX = long.MinValue;
            long maxY = long.MinValue;
            int contourId = 0;

            for (int i = 0; i < polygon.Count; i++)
            {
                Contour contour = polygon[i];
                int count = contour.Count;
                int segmentCount = GetSegmentCount(contour, context);
                if (segmentCount == 0)
                {
                    continue;
                }

                contourId++;
                SweepEventComparer comparer = new();
                for (int j = 0; j < segmentCount; j++)
                {
                    int nextIndex = j + 1;
                    if (nextIndex == count)
                    {
                        nextIndex = 0;
                    }

                    ProcessSegment(
                        contourId,
                        contour[j],
                        contour[nextIndex],
                        polygonType,
                        context,
                        eventQueue,
                        comparer,
                        ref minX,
                        ref minY,
                        ref maxX,
                        ref maxY);
                }
            }

            return minX == long.MaxValue
                ? default
                : new Box64(new Vertex64(minX, minY), new Vertex64(maxX, maxY));
        }

        private static int GetSegmentCount(Contour contour, FixedPrecisionContext context)
        {
            int count = contour.Count;
            if (count < 3)
            {
                return 0;
            }

            Vertex64 first = context.Quantize(contour[0]);
            Vertex64 last = context.Quantize(contour[^1]);
            return first == last ? count - 1 : count;
        }

        private static void ProcessSegment(
            int contourId,
            in Vertex start,
            in Vertex end,
            PolygonType polygonType,
            FixedPrecisionContext context,
            List<SweepEvent> eventQueue,
            SweepEventComparer comparer,
            ref long minX,
            ref long minY,
            ref long maxX,
            ref long maxY)
        {
            Vertex64 source = context.Quantize(start);
            Vertex64 target = context.Quantize(end);
            if (source == target)
            {
                return;
            }

            SweepEvent e1 = new(source, true, polygonType);
            SweepEvent e2 = new(target, true, e1, polygonType);
            e1.OtherEvent = e2;
            e1.ContourId = e2.ContourId = contourId;
            e1.PointDouble = start;
            e2.PointDouble = end;

            if (comparer.Compare(e1, e2) < 0)
            {
                e2.Left = false;
            }
            else
            {
                e1.Left = false;
            }

            Vertex64 min = Vertex64.Min(source, target);
            Vertex64 max = Vertex64.Max(source, target);
            if (min.X < minX)
            {
                minX = min.X;
            }

            if (min.Y < minY)
            {
                minY = min.Y;
            }

            if (max.X > maxX)
            {
                maxX = max.X;
            }

            if (max.Y > maxY)
            {
                maxY = max.Y;
            }

            eventQueue.Add(e1);
            eventQueue.Add(e2);
        }

        public static int PossibleIntersection(
            SweepEvent leftEvent,
            SweepEvent rightEvent,
            StablePriorityQueue<SweepEvent, SweepEventComparer> eventQueue,
            FixedPrecisionContext context)
        {
            Span<SweepEvent> workspace = new SweepEvent[4];
            return PolygonClipperPossibleIntersection(leftEvent, rightEvent, eventQueue, workspace, context);
        }

        private static int PolygonClipperPossibleIntersection(
            SweepEvent le1,
            SweepEvent le2,
            StablePriorityQueue<SweepEvent, SweepEventComparer> eventQueue,
            Span<SweepEvent> workspace,
            FixedPrecisionContext context)
        {
            if (le1.OtherEvent == null || le2.OtherEvent == null)
            {
                return 0;
            }

            bool useFloating = PolygonUtilities.UseFloatingScale;
            int nIntersections;
            Vertex64 ip1Fixed = default;
            Vertex ip1Double = default;
            if (useFloating)
            {
                nIntersections = PolygonUtilities.FindIntersectionDouble(
                    le1.PointDouble,
                    le1.OtherEvent.PointDouble,
                    le2.PointDouble,
                    le2.OtherEvent.PointDouble,
                    out ip1Double,
                    out Vertex _);
                if (nIntersections != 0)
                {
                    ip1Fixed = context.Quantize(ip1Double);
                }
            }
            else
            {
                nIntersections = PolygonUtilities.FindIntersection(
                    le1.GetSegment(),
                    le2.GetSegment(),
                    out ip1Fixed,
                    out Vertex64 _);
            }

            if (nIntersections == 0)
            {
                return 0;
            }

            if (nIntersections == 1)
            {
                bool leftMatch = useFloating
                    ? le1.PointDouble == le2.PointDouble
                    : le1.Point == le2.Point;
                bool rightMatch = useFloating
                    ? le1.OtherEvent.PointDouble == le2.OtherEvent.PointDouble
                    : le1.OtherEvent.Point == le2.OtherEvent.Point;
                if (leftMatch || rightMatch)
                {
                    return 0;
                }
            }

            if (nIntersections == 2 && le1.PolygonType == le2.PolygonType)
            {
                return 0;
            }

            SweepEventComparer comparer = eventQueue.Comparer;
            if (nIntersections == 1)
            {
                if (useFloating)
                {
                    if (le1.PointDouble != ip1Double && le1.OtherEvent.PointDouble != ip1Double)
                    {
                        DivideSegment(le1, ip1Fixed, ip1Double, eventQueue, comparer, context);
                    }

                    if (le2.PointDouble != ip1Double && le2.OtherEvent.PointDouble != ip1Double)
                    {
                        DivideSegment(le2, ip1Fixed, ip1Double, eventQueue, comparer, context);
                    }
                }
                else
                {
                    if (le1.Point != ip1Fixed && le1.OtherEvent.Point != ip1Fixed)
                    {
                        DivideSegment(le1, ip1Fixed, new Vertex(ip1Fixed.X, ip1Fixed.Y), eventQueue, comparer, context);
                    }

                    if (le2.Point != ip1Fixed && le2.OtherEvent.Point != ip1Fixed)
                    {
                        DivideSegment(le2, ip1Fixed, new Vertex(ip1Fixed.X, ip1Fixed.Y), eventQueue, comparer, context);
                    }
                }

                return 1;
            }

            bool leftCoincide = useFloating
                ? le1.PointDouble == le2.PointDouble
                : le1.Point == le2.Point;
            bool rightCoincide = useFloating
                ? le1.OtherEvent.PointDouble == le2.OtherEvent.PointDouble
                : le1.OtherEvent.Point == le2.OtherEvent.Point;

            ref SweepEvent wRef = ref MemoryMarshal.GetReference(workspace);
            SweepEvent splitEvent;
            Vertex64 splitPoint;
            Vertex splitPointDouble;
            if (!leftCoincide)
            {
                if (comparer.Compare(le1, le2) > 0)
                {
                    Unsafe.Add(ref wRef, 0u) = le2;
                    Unsafe.Add(ref wRef, 1u) = le1;
                }
                else
                {
                    Unsafe.Add(ref wRef, 0u) = le1;
                    Unsafe.Add(ref wRef, 1u) = le2;
                }

                if (!rightCoincide)
                {
                    Unsafe.Add(ref wRef, 2u) = le1.OtherEvent;
                    Unsafe.Add(ref wRef, 3u) = le2.OtherEvent;
                }
                else
                {
                    Unsafe.Add(ref wRef, 2u) = le2.OtherEvent;
                    Unsafe.Add(ref wRef, 3u) = le1.OtherEvent;
                }
            }
            else if (!rightCoincide)
            {
                if (comparer.Compare(le1.OtherEvent, le2.OtherEvent) > 0)
                {
                    Unsafe.Add(ref wRef, 0u) = le2.OtherEvent;
                    Unsafe.Add(ref wRef, 1u) = le1.OtherEvent;
                }
                else
                {
                    Unsafe.Add(ref wRef, 0u) = le1.OtherEvent;
                    Unsafe.Add(ref wRef, 1u) = le2.OtherEvent;
                }
            }

            if (leftCoincide)
            {
                le2.EdgeType = EdgeType.NonContributing;
                le1.EdgeType = (le2.InOut == le1.InOut)
                    ? EdgeType.SameTransition
                    : EdgeType.DifferentTransition;

                if (!rightCoincide)
                {
                    splitEvent = Unsafe.Add(ref wRef, 0u);
                    splitPoint = useFloating ? context.Quantize(splitEvent.PointDouble) : splitEvent.Point;
                    splitPointDouble = useFloating ? splitEvent.PointDouble : new Vertex(splitPoint.X, splitPoint.Y);
                    DivideSegment(Unsafe.Add(ref wRef, 1u).OtherEvent, splitPoint, splitPointDouble, eventQueue, comparer, context);
                }

                return 2;
            }

            if (rightCoincide)
            {
                splitEvent = Unsafe.Add(ref wRef, 1u);
                splitPoint = useFloating ? context.Quantize(splitEvent.PointDouble) : splitEvent.Point;
                splitPointDouble = useFloating ? splitEvent.PointDouble : new Vertex(splitPoint.X, splitPoint.Y);
                DivideSegment(Unsafe.Add(ref wRef, 0u), splitPoint, splitPointDouble, eventQueue, comparer, context);
                return 3;
            }

            if (Unsafe.Add(ref wRef, 0u) != Unsafe.Add(ref wRef, 3u).OtherEvent)
            {
                splitEvent = Unsafe.Add(ref wRef, 1u);
                splitPoint = useFloating ? context.Quantize(splitEvent.PointDouble) : splitEvent.Point;
                splitPointDouble = useFloating ? splitEvent.PointDouble : new Vertex(splitPoint.X, splitPoint.Y);
                DivideSegment(Unsafe.Add(ref wRef, 0u), splitPoint, splitPointDouble, eventQueue, comparer, context);

                splitEvent = Unsafe.Add(ref wRef, 2u);
                splitPoint = useFloating ? context.Quantize(splitEvent.PointDouble) : splitEvent.Point;
                splitPointDouble = useFloating ? splitEvent.PointDouble : new Vertex(splitPoint.X, splitPoint.Y);
                DivideSegment(Unsafe.Add(ref wRef, 1u), splitPoint, splitPointDouble, eventQueue, comparer, context);
                return 3;
            }

            splitEvent = Unsafe.Add(ref wRef, 1u);
            splitPoint = useFloating ? context.Quantize(splitEvent.PointDouble) : splitEvent.Point;
            splitPointDouble = useFloating ? splitEvent.PointDouble : new Vertex(splitPoint.X, splitPoint.Y);
            DivideSegment(Unsafe.Add(ref wRef, 0u), splitPoint, splitPointDouble, eventQueue, comparer, context);

            splitEvent = Unsafe.Add(ref wRef, 2u);
            splitPoint = useFloating ? context.Quantize(splitEvent.PointDouble) : splitEvent.Point;
            splitPointDouble = useFloating ? splitEvent.PointDouble : new Vertex(splitPoint.X, splitPoint.Y);
            DivideSegment(Unsafe.Add(ref wRef, 3u).OtherEvent, splitPoint, splitPointDouble, eventQueue, comparer, context);
            return 3;
        }

        private static void DivideSegment(
            SweepEvent le,
            Vertex64 point,
            Vertex pointDouble,
            StablePriorityQueue<SweepEvent, SweepEventComparer> eventQueue,
            SweepEventComparer comparer,
            FixedPrecisionContext context)
        {
            if (le.OtherEvent == null)
            {
                return;
            }

            SweepEvent re = le.OtherEvent;

            if (PolygonUtilities.UseFloatingScale)
            {
                if (pointDouble.X == le.PointDouble.X && pointDouble.Y < le.PointDouble.Y)
                {
                    pointDouble = new Vertex(pointDouble.X.NextAfter(double.PositiveInfinity), pointDouble.Y);
                    point = context.Quantize(pointDouble);
                }
            }
            else if (point.X == le.Point.X && point.Y < le.Point.Y)
            {
                point = new Vertex64(point.X + 1, point.Y);
                pointDouble = new Vertex(point.X, point.Y);
            }

            SweepEvent r = new(point, false, le, le.PolygonType);
            SweepEvent l = new(point, true, re, le.PolygonType);
            r.PointDouble = pointDouble;
            l.PointDouble = pointDouble;

            r.ContourId = l.ContourId = le.ContourId;

            if (comparer.Compare(l, re) > 0)
            {
                re.Left = true;
                l.Left = false;
            }

            re.OtherEvent = l;
            le.OtherEvent = r;

            eventQueue.Enqueue(l);
            eventQueue.Enqueue(r);
        }

        private static void ComputeFields(SweepEvent le, SweepEvent? prev, BooleanOperation operation)
        {
            if (prev == null)
            {
                le.InOut = false;
                le.OtherInOut = true;
                le.PrevInResult = null;
            }
            else if (le.PolygonType == prev.PolygonType)
            {
                le.InOut = !prev.InOut;
                le.OtherInOut = prev.OtherInOut;
            }
            else
            {
                le.InOut = !prev.OtherInOut;
                le.OtherInOut = prev.IsVertical() ? !prev.InOut : prev.InOut;
            }

            if (prev != null)
            {
                le.PrevInResult = (!InResult(prev, operation) || prev.IsVertical())
                    ? prev.PrevInResult
                    : prev;
            }

            bool inResult = InResult(le, operation);
            le.ResultTransition = inResult
                ? DetermineResultTransition(le, operation)
                : ResultTransition.Neutral;
        }

        private static ResultTransition DetermineResultTransition(SweepEvent sweepEvent, BooleanOperation operation)
        {
            bool thisIn = !sweepEvent.InOut;
            bool thatIn = !sweepEvent.OtherInOut;
            bool isIn;

            switch (operation)
            {
                case BooleanOperation.Intersection:
                    isIn = thisIn && thatIn;
                    break;
                case BooleanOperation.Union:
                    isIn = thisIn || thatIn;
                    break;
                case BooleanOperation.Xor:
                    isIn = thisIn ^ thatIn;
                    break;
                case BooleanOperation.Difference:
                    if (sweepEvent.PolygonType == PolygonType.Subject)
                    {
                        isIn = thisIn && !thatIn;
                    }
                    else
                    {
                        isIn = thatIn && !thisIn;
                    }

                    break;
                default:
                    throw new InvalidOperationException("Invalid boolean operation.");
            }

            return isIn ? ResultTransition.Contributing : ResultTransition.NonContributing;
        }

        private static bool InResult(SweepEvent sweepEvent, BooleanOperation operation)
            => sweepEvent.EdgeType switch
            {
                EdgeType.Normal => operation switch
                {
                    BooleanOperation.Intersection => !sweepEvent.OtherInOut,
                    BooleanOperation.Union => sweepEvent.OtherInOut,
                    BooleanOperation.Difference =>
                        (sweepEvent.OtherInOut && sweepEvent.PolygonType == PolygonType.Subject) ||
                        (!sweepEvent.OtherInOut && sweepEvent.PolygonType == PolygonType.Clipping),
                    BooleanOperation.Xor => true,
                    _ => false,
                },
                EdgeType.NonContributing => false,
                EdgeType.SameTransition => operation is BooleanOperation.Intersection or BooleanOperation.Union,
                EdgeType.DifferentTransition => operation == BooleanOperation.Difference,
                _ => false,
            };

        private static FieldSnapshot CreateFieldSnapshot(SweepEvent sweepEvent)
        {
            SweepEvent? prev = sweepEvent.PrevInResult;
            Vertex64 prevPoint = prev == null ? default : prev.Point;
            Vertex64 prevOtherPoint = prev == null ? default : prev.OtherEvent.Point;
            return new FieldSnapshot(
                sweepEvent.Left,
                sweepEvent.Point,
                sweepEvent.OtherEvent.Point,
                sweepEvent.PolygonType,
                sweepEvent.EdgeType,
                sweepEvent.InOut,
                sweepEvent.OtherInOut,
                sweepEvent.ResultTransition,
                sweepEvent.ContourId,
                prev != null,
                prevPoint,
                prevOtherPoint);
        }
    }

    private static class StageDouble
    {
        public static QueueSnapshot BuildEventQueue(Polygon subject, Polygon clipping, FixedPrecisionContext context)
        {
            List<DoubleSweepEvent> unordered = [];
            Box2 subjectBounds = BuildEventQueue(subject, PolygonType.Subject, unordered, context);
            Box2 clippingBounds = BuildEventQueue(clipping, PolygonType.Clipping, unordered, context);

            DoubleSweepEventComparer comparer = new();
            StablePriorityQueue<DoubleSweepEvent, DoubleSweepEventComparer> queue = new(comparer, unordered);
            List<EventSnapshot> events = new(queue.Count);
            while (queue.Count > 0)
            {
                DoubleSweepEvent sweepEvent = queue.Dequeue();
                events.Add(new EventSnapshot(
                    sweepEvent.Left,
                    context.Quantize(sweepEvent.Point),
                    context.Quantize(sweepEvent.OtherEvent.Point),
                    sweepEvent.PolygonType,
                    sweepEvent.ContourId));
            }

            return new QueueSnapshot(QuantizeBounds(subjectBounds, context), QuantizeBounds(clippingBounds, context), events);
        }

        public static List<DoubleSweepEvent> BuildSortedEvents(Polygon subject, Polygon clipping, FixedPrecisionContext context)
        {
            List<DoubleSweepEvent> unordered = [];
            BuildEventQueue(subject, PolygonType.Subject, unordered, context);
            BuildEventQueue(clipping, PolygonType.Clipping, unordered, context);

            DoubleSweepEventComparer comparer = new();
            StablePriorityQueue<DoubleSweepEvent, DoubleSweepEventComparer> queue = new(comparer, unordered);
            List<DoubleSweepEvent> events = new(queue.Count);
            while (queue.Count > 0)
            {
                events.Add(queue.Dequeue());
            }

            return events;
        }

        public static List<EventSnapshot> SubdivideSegments(Polygon subject, Polygon clipping, FixedPrecisionContext context)
        {
            List<DoubleSweepEvent> unordered = [];
            BuildEventQueue(subject, PolygonType.Subject, unordered, context);
            BuildEventQueue(clipping, PolygonType.Clipping, unordered, context);

            DoubleSweepEventComparer comparer = new();
            StablePriorityQueue<DoubleSweepEvent, DoubleSweepEventComparer> queue = new(comparer, unordered);
            DoubleStatusLine statusLine = new(unordered.Count >> 1);
            List<EventSnapshot> leftEvents = new();

            Span<DoubleSweepEvent> workspace = new DoubleSweepEvent[4];
            while (queue.Count > 0)
            {
                DoubleSweepEvent sweepEvent = queue.Dequeue();
                if (sweepEvent.Left)
                {
                    int index = sweepEvent.PosSL = statusLine.Add(sweepEvent);
                    DoubleSweepEvent? prevEvent = statusLine.Prev(index);
                    DoubleSweepEvent? nextEvent = statusLine.Next(index);

                    if (nextEvent != null && PossibleIntersection(sweepEvent, nextEvent, queue) == 2)
                    {
                        // no-op for subdivision stage
                    }

                    if (prevEvent != null && PossibleIntersection(prevEvent, sweepEvent, queue) == 2)
                    {
                        // no-op for subdivision stage
                    }

                    leftEvents.Add(new EventSnapshot(
                        sweepEvent.Left,
                        context.Quantize(sweepEvent.Point),
                        context.Quantize(sweepEvent.OtherEvent.Point),
                        sweepEvent.PolygonType,
                        sweepEvent.ContourId));
                }
                else
                {
                    sweepEvent = sweepEvent.OtherEvent;
                    int index = sweepEvent.PosSL;
                    DoubleSweepEvent? prevEvent = statusLine.Prev(index);
                    DoubleSweepEvent? nextEvent = statusLine.Next(index);
                    if (prevEvent != null && nextEvent != null)
                    {
                        _ = PossibleIntersection(prevEvent, nextEvent, queue);
                    }

                    statusLine.RemoveAt(index);
                }
            }

            return leftEvents;
        }

        public static List<FieldSnapshot> ComputeFieldSnapshots(
            Polygon subject,
            Polygon clipping,
            FixedPrecisionContext context,
            BooleanOperation operation)
        {
            List<DoubleSweepEvent> unordered = [];
            BuildEventQueue(subject, PolygonType.Subject, unordered, context);
            BuildEventQueue(clipping, PolygonType.Clipping, unordered, context);

            DoubleSweepEventComparer comparer = new();
            StablePriorityQueue<DoubleSweepEvent, DoubleSweepEventComparer> queue = new(comparer, unordered);
            DoubleStatusLine statusLine = new(unordered.Count >> 1);
            List<FieldSnapshot> snapshots = [];

            Span<DoubleSweepEvent> workspace = new DoubleSweepEvent[4];
            while (queue.Count > 0)
            {
                DoubleSweepEvent sweepEvent = queue.Dequeue();
                if (sweepEvent.Left)
                {
                    int index = sweepEvent.PosSL = statusLine.Add(sweepEvent);
                    DoubleSweepEvent? prevEvent = statusLine.Prev(index);
                    DoubleSweepEvent? nextEvent = statusLine.Next(index);

                    ComputeFields(sweepEvent, prevEvent, operation);

                    if (nextEvent != null && PolygonClipperPossibleIntersection(sweepEvent, nextEvent, queue, workspace) == 2)
                    {
                        ComputeFields(sweepEvent, prevEvent, operation);
                        ComputeFields(nextEvent, sweepEvent, operation);
                    }

                    if (prevEvent != null && PolygonClipperPossibleIntersection(prevEvent, sweepEvent, queue, workspace) == 2)
                    {
                        DoubleSweepEvent? prevPrevEvent = statusLine.Prev(prevEvent.PosSL);
                        ComputeFields(prevEvent, prevPrevEvent, operation);
                        ComputeFields(sweepEvent, prevEvent, operation);
                    }

                    snapshots.Add(CreateFieldSnapshot(sweepEvent, context));
                }
                else
                {
                    sweepEvent = sweepEvent.OtherEvent;
                    int index = sweepEvent.PosSL;
                    DoubleSweepEvent? prevEvent = statusLine.Prev(index);
                    DoubleSweepEvent? nextEvent = statusLine.Next(index);
                    if (prevEvent != null && nextEvent != null)
                    {
                        _ = PolygonClipperPossibleIntersection(prevEvent, nextEvent, queue, workspace);
                    }

                    statusLine.RemoveAt(index);
                }
            }

            return snapshots;
        }

        private static Box2 BuildEventQueue(
            Polygon polygon,
            PolygonType polygonType,
            List<DoubleSweepEvent> eventQueue,
            FixedPrecisionContext context)
        {
            Vertex min = new(double.PositiveInfinity, double.PositiveInfinity);
            Vertex max = new(double.NegativeInfinity, double.NegativeInfinity);
            int contourId = 0;

            for (int i = 0; i < polygon.Count; i++)
            {
                Contour contour = polygon[i];
                int count = contour.Count;
                if (count < 3)
                {
                    continue;
                }

                contourId++;
                DoubleSweepEventComparer comparer = new();
                for (int j = 0; j < count - 1; j++)
                {
                    Vertex start = contour[j];
                    Vertex end = contour[j + 1];
                    DoubleSegment segment = new(start, end);
                    ProcessSegment(
                        contourId,
                        segment,
                        polygonType,
                        eventQueue,
                        comparer,
                        ref min,
                        ref max);
                }
            }

            return new Box2(min, max);
        }

        private static void ProcessSegment(
            int contourId,
            DoubleSegment segment,
            PolygonType polygonType,
            List<DoubleSweepEvent> eventQueue,
            DoubleSweepEventComparer comparer,
            ref Vertex min,
            ref Vertex max)
        {
            if (segment.Source == segment.Target)
            {
                return;
            }

            DoubleSweepEvent e1 = new(segment.Source, true, null, polygonType);
            DoubleSweepEvent e2 = new(segment.Target, true, e1, polygonType);
            e1.OtherEvent = e2;
            e1.ContourId = e2.ContourId = contourId;

            if (comparer.Compare(e1, e2) < 0)
            {
                e2.Left = false;
            }
            else
            {
                e1.Left = false;
            }

            min = Vertex.Min(min, segment.Min);
            max = Vertex.Max(max, segment.Max);

            eventQueue.Add(e1);
            eventQueue.Add(e2);
        }

        public static int PossibleIntersection(
            DoubleSweepEvent leftEvent,
            DoubleSweepEvent rightEvent,
            StablePriorityQueue<DoubleSweepEvent, DoubleSweepEventComparer> eventQueue)
        {
            Span<DoubleSweepEvent> workspace = new DoubleSweepEvent[4];
            return PolygonClipperPossibleIntersection(leftEvent, rightEvent, eventQueue, workspace);
        }

        private static int PolygonClipperPossibleIntersection(
            DoubleSweepEvent le1,
            DoubleSweepEvent le2,
            StablePriorityQueue<DoubleSweepEvent, DoubleSweepEventComparer> eventQueue,
            Span<DoubleSweepEvent> workspace)
        {
            if (le1.OtherEvent == null || le2.OtherEvent == null)
            {
                return 0;
            }

            int nIntersections = DoublePolygonUtilities.FindIntersection(
                le1.GetSegment(),
                le2.GetSegment(),
                out Vertex ip1,
                out Vertex _);

            if (nIntersections == 0)
            {
                return 0;
            }

            if (nIntersections == 1 &&
                (le1.Point == le2.Point || le1.OtherEvent.Point == le2.OtherEvent.Point))
            {
                return 0;
            }

            if (nIntersections == 2 && le1.PolygonType == le2.PolygonType)
            {
                return 0;
            }

            DoubleSweepEventComparer comparer = eventQueue.Comparer;
            if (nIntersections == 1)
            {
                if (le1.Point != ip1 && le1.OtherEvent.Point != ip1)
                {
                    DivideSegment(le1, ip1, eventQueue, comparer);
                }

                if (le2.Point != ip1 && le2.OtherEvent.Point != ip1)
                {
                    DivideSegment(le2, ip1, eventQueue, comparer);
                }

                return 1;
            }

            bool leftCoincide = le1.Point == le2.Point;
            bool rightCoincide = le1.OtherEvent.Point == le2.OtherEvent.Point;

            ref DoubleSweepEvent wRef = ref MemoryMarshal.GetReference(workspace);
            if (!leftCoincide)
            {
                if (comparer.Compare(le1, le2) > 0)
                {
                    Unsafe.Add(ref wRef, 0u) = le2;
                    Unsafe.Add(ref wRef, 1u) = le1;
                }
                else
                {
                    Unsafe.Add(ref wRef, 0u) = le1;
                    Unsafe.Add(ref wRef, 1u) = le2;
                }

                if (!rightCoincide)
                {
                    Unsafe.Add(ref wRef, 2u) = le1.OtherEvent;
                    Unsafe.Add(ref wRef, 3u) = le2.OtherEvent;
                }
                else
                {
                    Unsafe.Add(ref wRef, 2u) = le2.OtherEvent;
                    Unsafe.Add(ref wRef, 3u) = le1.OtherEvent;
                }
            }
            else if (!rightCoincide)
            {
                if (comparer.Compare(le1.OtherEvent, le2.OtherEvent) > 0)
                {
                    Unsafe.Add(ref wRef, 0u) = le2.OtherEvent;
                    Unsafe.Add(ref wRef, 1u) = le1.OtherEvent;
                }
                else
                {
                    Unsafe.Add(ref wRef, 0u) = le1.OtherEvent;
                    Unsafe.Add(ref wRef, 1u) = le2.OtherEvent;
                }
            }

            if (leftCoincide)
            {
                le2.EdgeType = EdgeType.NonContributing;
                le1.EdgeType = le1.InOut == le2.InOut
                    ? EdgeType.SameTransition
                    : EdgeType.DifferentTransition;

                if (!rightCoincide)
                {
                    DivideSegment(Unsafe.Add(ref wRef, 1u).OtherEvent, Unsafe.Add(ref wRef, 0u).Point, eventQueue, comparer);
                }

                return 2;
            }

            if (rightCoincide)
            {
                DivideSegment(Unsafe.Add(ref wRef, 0u), Unsafe.Add(ref wRef, 1u).Point, eventQueue, comparer);
                return 3;
            }

            if (Unsafe.Add(ref wRef, 0u) != Unsafe.Add(ref wRef, 3u).OtherEvent)
            {
                DivideSegment(Unsafe.Add(ref wRef, 0u), Unsafe.Add(ref wRef, 1u).Point, eventQueue, comparer);
                DivideSegment(Unsafe.Add(ref wRef, 1u), Unsafe.Add(ref wRef, 2u).Point, eventQueue, comparer);
                return 3;
            }

            DivideSegment(Unsafe.Add(ref wRef, 0u), Unsafe.Add(ref wRef, 1u).Point, eventQueue, comparer);
            DivideSegment(Unsafe.Add(ref wRef, 3u).OtherEvent, Unsafe.Add(ref wRef, 2u).Point, eventQueue, comparer);
            return 3;
        }

        private static void DivideSegment(
            DoubleSweepEvent le,
            Vertex point,
            StablePriorityQueue<DoubleSweepEvent, DoubleSweepEventComparer> eventQueue,
            DoubleSweepEventComparer comparer)
        {
            if (le.OtherEvent == null)
            {
                return;
            }

            DoubleSweepEvent re = le.OtherEvent;

            if (point.X == le.Point.X && point.Y < le.Point.Y)
            {
                point = new Vertex(point.X.NextAfter(double.PositiveInfinity), point.Y);
            }

            DoubleSweepEvent r = new(point, false, le, le.PolygonType);
            DoubleSweepEvent l = new(point, true, re, le.PolygonType);

            r.ContourId = l.ContourId = le.ContourId;

            if (comparer.Compare(l, re) > 0)
            {
                re.Left = true;
                l.Left = false;
            }

            re.OtherEvent = l;
            le.OtherEvent = r;

            eventQueue.Enqueue(l);
            eventQueue.Enqueue(r);
        }

        private static void ComputeFields(DoubleSweepEvent le, DoubleSweepEvent? prev, BooleanOperation operation)
        {
            if (prev == null)
            {
                le.InOut = false;
                le.OtherInOut = true;
                le.PrevInResult = null;
            }
            else if (le.PolygonType == prev.PolygonType)
            {
                le.InOut = !prev.InOut;
                le.OtherInOut = prev.OtherInOut;
            }
            else
            {
                le.InOut = !prev.OtherInOut;
                le.OtherInOut = prev.IsVertical() ? !prev.InOut : prev.InOut;
            }

            if (prev != null)
            {
                le.PrevInResult = (!InResult(prev, operation) || prev.IsVertical())
                    ? prev.PrevInResult
                    : prev;
            }

            bool inResult = InResult(le, operation);
            le.ResultTransition = inResult
                ? DetermineResultTransition(le, operation)
                : ResultTransition.Neutral;
        }

        private static ResultTransition DetermineResultTransition(DoubleSweepEvent sweepEvent, BooleanOperation operation)
        {
            bool thisIn = !sweepEvent.InOut;
            bool thatIn = !sweepEvent.OtherInOut;
            bool isIn;

            switch (operation)
            {
                case BooleanOperation.Intersection:
                    isIn = thisIn && thatIn;
                    break;
                case BooleanOperation.Union:
                    isIn = thisIn || thatIn;
                    break;
                case BooleanOperation.Xor:
                    isIn = thisIn ^ thatIn;
                    break;
                case BooleanOperation.Difference:
                    if (sweepEvent.PolygonType == PolygonType.Subject)
                    {
                        isIn = thisIn && !thatIn;
                    }
                    else
                    {
                        isIn = thatIn && !thisIn;
                    }

                    break;
                default:
                    throw new InvalidOperationException("Invalid boolean operation.");
            }

            return isIn ? ResultTransition.Contributing : ResultTransition.NonContributing;
        }

        private static bool InResult(DoubleSweepEvent sweepEvent, BooleanOperation operation)
            => sweepEvent.EdgeType switch
            {
                EdgeType.Normal => operation switch
                {
                    BooleanOperation.Intersection => !sweepEvent.OtherInOut,
                    BooleanOperation.Union => sweepEvent.OtherInOut,
                    BooleanOperation.Difference =>
                        (sweepEvent.OtherInOut && sweepEvent.PolygonType == PolygonType.Subject) ||
                        (!sweepEvent.OtherInOut && sweepEvent.PolygonType == PolygonType.Clipping),
                    BooleanOperation.Xor => true,
                    _ => false,
                },
                EdgeType.NonContributing => false,
                EdgeType.SameTransition => operation is BooleanOperation.Intersection or BooleanOperation.Union,
                EdgeType.DifferentTransition => operation == BooleanOperation.Difference,
                _ => false,
            };

        private static FieldSnapshot CreateFieldSnapshot(DoubleSweepEvent sweepEvent, FixedPrecisionContext context)
        {
            DoubleSweepEvent? prev = sweepEvent.PrevInResult;
            Vertex64 prevPoint = prev == null ? default : context.Quantize(prev.Point);
            Vertex64 prevOtherPoint = prev == null ? default : context.Quantize(prev.OtherEvent.Point);
            return new FieldSnapshot(
                sweepEvent.Left,
                context.Quantize(sweepEvent.Point),
                context.Quantize(sweepEvent.OtherEvent.Point),
                sweepEvent.PolygonType,
                sweepEvent.EdgeType,
                sweepEvent.InOut,
                sweepEvent.OtherInOut,
                sweepEvent.ResultTransition,
                sweepEvent.ContourId,
                prev != null,
                prevPoint,
                prevOtherPoint);
        }


        private static Box64 QuantizeBounds(Box2 bounds, FixedPrecisionContext context)
            => new(context.Quantize(bounds.Min), context.Quantize(bounds.Max));
    }

    private sealed class DoubleSweepEvent
    {
        public DoubleSweepEvent(
            Vertex point,
            bool left,
            DoubleSweepEvent? otherEvent = null,
            PolygonType polygonType = PolygonType.Subject,
            EdgeType edgeType = EdgeType.Normal)
        {
            this.Point = point;
            this.Left = left;
            this.OtherEvent = otherEvent!;
            this.PolygonType = polygonType;
            this.EdgeType = edgeType;
            this.SegmentSource = point;
            this.SegmentTarget = point;
        }

        public Vertex Point { get; }

        public bool Left { get; set; }

        public DoubleSweepEvent OtherEvent { get; set; }

        public PolygonType PolygonType { get; }

        public EdgeType EdgeType { get; set; }

        public bool InOut { get; set; }

        public bool OtherInOut { get; set; }

        public DoubleSweepEvent? PrevInResult { get; set; }

        public ResultTransition ResultTransition { get; set; }

        public bool InResult => this.ResultTransition != ResultTransition.Neutral;

        public int ContourId { get; set; }

        public int PosSL { get; set; }

        public Vertex SegmentSource { get; set; }

        public Vertex SegmentTarget { get; set; }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool IsVertical() => this.Point.X == this.OtherEvent.Point.X;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool IsBefore(DoubleSweepEvent other)
        {
            if (this.Point.X != other.Point.X)
            {
                return this.Point.X < other.Point.X;
            }

            return this.Point.Y < other.Point.Y;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public DoubleSegment GetSegment() => new(this.Point, this.OtherEvent.Point);
    }

    private readonly struct DoubleSegment : IEquatable<DoubleSegment>
    {
        public DoubleSegment(in Vertex source, in Vertex target)
        {
            this.Source = source;
            this.Target = target;
            this.Min = Vertex.Min(source, target);
            this.Max = Vertex.Max(source, target);
        }

        public Vertex Source { get; }

        public Vertex Target { get; }

        public Vertex Min { get; }

        public Vertex Max { get; }

        public static bool operator ==(DoubleSegment left, DoubleSegment right) => left.Equals(right);

        public static bool operator !=(DoubleSegment left, DoubleSegment right) => !left.Equals(right);

        public bool Equals(DoubleSegment other)
            => this.Source.Equals(other.Source) && this.Target.Equals(other.Target);

        public override bool Equals(object? obj) => obj is DoubleSegment segment && this.Equals(segment);

        public override int GetHashCode() => HashCode.Combine(this.Source, this.Target);
    }

    private sealed class DoubleSweepEventComparer : IComparer<DoubleSweepEvent>, IComparer
    {
        public int Compare(DoubleSweepEvent? x, DoubleSweepEvent? y)
        {
            if (x == null)
            {
                return -1;
            }

            if (y == null)
            {
                return 1;
            }

            Vertex xPoint = x.Point;
            Vertex yPoint = y.Point;

            if (xPoint.X > yPoint.X)
            {
                return 1;
            }

            if (xPoint.X < yPoint.X)
            {
                return -1;
            }

            if (xPoint.Y != yPoint.Y)
            {
                return xPoint.Y > yPoint.Y ? 1 : -1;
            }

            if (x.Left != y.Left)
            {
                return x.Left ? 1 : -1;
            }

            Vertex xOtherPoint = x.OtherEvent.Point;
            Vertex yOtherPoint = y.OtherEvent.Point;

            double area = DoublePolygonUtilities.SignedArea(xPoint, xOtherPoint, yOtherPoint);
            if (area == 0D)
            {
                return x.PolygonType != PolygonType.Subject && y.PolygonType == PolygonType.Subject ? 1 : -1;
            }

            bool isBelow = x.Left ? area > 0D : area < 0D;
            return isBelow ? -1 : 1;
        }

        public int Compare(object? x, object? y)
        {
            if (x is DoubleSweepEvent a && y is DoubleSweepEvent b)
            {
                return this.Compare(a, b);
            }

            throw new ArgumentException("Both arguments must be of type DoubleSweepEvent.", nameof(x));
        }
    }

    private sealed class DoubleSegmentComparer : IComparer<DoubleSweepEvent>, IComparer
    {
        public int Compare(DoubleSweepEvent? x, DoubleSweepEvent? y)
        {
            if (ReferenceEquals(x, y))
            {
                return 0;
            }

            if (x == null)
            {
                return -1;
            }

            if (y == null)
            {
                return 1;
            }

            bool inversed = !x.IsBefore(y);
            DoubleSweepEvent perhapsInversedX = inversed ? y : x;
            DoubleSweepEvent perhapsInversedY = inversed ? x : y;

            Vertex xPoint = perhapsInversedX.Point;
            Vertex yPoint = perhapsInversedY.Point;
            Vertex xOtherPoint = perhapsInversedX.OtherEvent.Point;
            Vertex yOtherPoint = perhapsInversedY.OtherEvent.Point;

            double area1 = DoublePolygonUtilities.SignedArea(xPoint, xOtherPoint, yPoint);
            double area2 = DoublePolygonUtilities.SignedArea(xPoint, xOtherPoint, yOtherPoint);

            if (area1 != 0D || area2 != 0D)
            {
                if (xPoint == yPoint)
                {
                    bool isBelow = perhapsInversedX.Left ? area2 > 0D : area2 < 0D;
                    return LessIf(isBelow, inversed);
                }

                if (xPoint.X == yPoint.X)
                {
                    return LessIf(xPoint.Y < yPoint.Y, inversed);
                }

                if ((area1 > 0D) == (area2 > 0D))
                {
                    return LessIf(area1 > 0D, inversed);
                }

                if (area1 == 0D)
                {
                    return LessIf(area2 > 0D, inversed);
                }

                DoubleSegment seg0 = new(xPoint, xOtherPoint);
                DoubleSegment seg1 = new(yPoint, yOtherPoint);

                int interResult = DoublePolygonUtilities.FindIntersection(seg0, seg1, out Vertex pi0, out Vertex _);

                if (interResult == 0)
                {
                    return LessIf(area1 > 0D, inversed);
                }

                if (interResult == 1)
                {
                    if (pi0 == y.Point)
                    {
                        return LessIf(area2 > 0D, inversed);
                    }

                    return LessIf(area1 > 0D, inversed);
                }
            }

            if (perhapsInversedX.PolygonType == perhapsInversedY.PolygonType)
            {
                if (xPoint == yPoint)
                {
                    return LessIf(perhapsInversedX.ContourId < perhapsInversedY.ContourId, inversed);
                }

                return LessIf(true, inversed);
            }

            return LessIf(perhapsInversedX.PolygonType == PolygonType.Subject, inversed);
        }

        public int Compare(object? x, object? y)
        {
            if (x == null)
            {
                return -1;
            }

            if (y == null)
            {
                return 1;
            }

            if (x is DoubleSweepEvent a && y is DoubleSweepEvent b)
            {
                return this.Compare(a, b);
            }

            throw new ArgumentException("Both arguments must be of type DoubleSweepEvent.", nameof(x));
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static int LessIf(bool condition, bool inversed = false) => condition ^ inversed ? -1 : 1;
    }

    private sealed class DoubleStatusLine
    {
        private const int DefaultCapacity = 16;
        private readonly List<DoubleSweepEvent> sortedEvents;
        private readonly DoubleSegmentComparer comparer = new();

        public DoubleStatusLine()
            : this(DefaultCapacity)
        {
        }

        public DoubleStatusLine(int capacity)
            => this.sortedEvents = new List<DoubleSweepEvent>(capacity > 0 ? capacity : DefaultCapacity);

        public int Count
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => this.sortedEvents.Count;
        }

        public DoubleSweepEvent this[int index]
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => this.sortedEvents[index];
        }

        public int Add(DoubleSweepEvent e)
        {
            int index = this.sortedEvents.BinarySearch(e, this.comparer);
            if (index < 0)
            {
                index = ~index;
            }

            this.sortedEvents.Insert(index, e);
            this.Up(index);
            return index;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void RemoveAt(int index)
        {
            this.sortedEvents.RemoveAt(index);
            this.Down(index);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public DoubleSweepEvent? Next(int index)
        {
            if (index >= 0 && index < this.sortedEvents.Count - 1)
            {
                return this.sortedEvents[index + 1];
            }

            return null;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public DoubleSweepEvent? Prev(int index)
        {
            if (index > 0 && index < this.sortedEvents.Count)
            {
                return this.sortedEvents[index - 1];
            }

            return null;
        }

        private void Up(int index)
        {
            List<DoubleSweepEvent> events = this.sortedEvents;
            for (int i = index + 1; i < events.Count; i++)
            {
                events[i].PosSL = i;
            }
        }

        private void Down(int index)
        {
            List<DoubleSweepEvent> events = this.sortedEvents;
            for (int i = index; i < events.Count; i++)
            {
                events[i].PosSL = i;
            }
        }
    }

    private static class DoublePolygonUtilities
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static double SignedArea(in Vertex p0, in Vertex p1, in Vertex p2)
            => ((p0.X - p2.X) * (p1.Y - p2.Y)) - ((p1.X - p2.X) * (p0.Y - p2.Y));

        public static int FindIntersection(in DoubleSegment seg0, in DoubleSegment seg1, out Vertex pi0, out Vertex pi1)
        {
            pi0 = default;
            pi1 = default;

            if (!TryGetIntersectionBoundingBox(seg0.Source, seg0.Target, seg1.Source, seg1.Target, out Box2 bbox))
            {
                return 0;
            }

            int interResult = FindIntersectionImpl(seg0, seg1, out pi0, out pi1);

            if (interResult == 1)
            {
                pi0 = ConstrainToBoundingBox(pi0, bbox);
            }
            else if (interResult == 2)
            {
                pi0 = ConstrainToBoundingBox(pi0, bbox);
                pi1 = ConstrainToBoundingBox(pi1, bbox);
            }

            return interResult;
        }

        private static int FindIntersectionImpl(in DoubleSegment seg0, in DoubleSegment seg1, out Vertex pi0, out Vertex pi1)
        {
            pi0 = default;
            pi1 = default;

            Vertex a1 = seg0.Source;
            Vertex a2 = seg1.Source;

            Vertex va = seg0.Target - a1;
            Vertex vb = seg1.Target - a2;
            Vertex e = a2 - a1;

            double kross = Vertex.Cross(va, vb);
            double sqrKross = kross * kross;
            double sqrLenA = Vertex.Dot(va, va);

            if (sqrKross > 0D)
            {
                double s = Vertex.Cross(e, vb) / kross;
                if (s is < 0D or > 1D)
                {
                    return 0;
                }

                double t = Vertex.Cross(e, va) / kross;
                if (t is < 0D or > 1D)
                {
                    return 0;
                }

                if (s is 0D or 1D)
                {
                    pi0 = a1 + (s * va);
                    return 1;
                }

                if (t is 0D or 1D)
                {
                    pi0 = a2 + (t * vb);
                    return 1;
                }

                pi0 = a1 + (s * va);
                return 1;
            }

            kross = Vertex.Cross(e, va);
            sqrKross = kross * kross;
            if (sqrKross > 0D)
            {
                return 0;
            }

            double sa = Vertex.Dot(va, e) / sqrLenA;
            double sb = sa + (Vertex.Dot(va, vb) / sqrLenA);
            double smin = Math.Min(sa, sb);
            double smax = Math.Max(sa, sb);

            if (smin <= 1D && smax >= 0D)
            {
                if (smin == 1D)
                {
                    pi0 = a1 + (smin * va);
                    return 1;
                }

                if (smax == 0D)
                {
                    pi0 = a1 + (smax * va);
                    return 1;
                }

                pi0 = a1 + (Math.Max(smin, 0D) * va);
                pi1 = a1 + (Math.Min(smax, 1D) * va);
                return 2;
            }

            return 0;
        }

        private static bool TryGetIntersectionBoundingBox(
            in Vertex a1,
            in Vertex a2,
            in Vertex b1,
            in Vertex b2,
            out Box2 result)
        {
            Vertex minA = Vertex.Min(a1, a2);
            Vertex maxA = Vertex.Max(a1, a2);
            Vertex minB = Vertex.Min(b1, b2);
            Vertex maxB = Vertex.Max(b1, b2);

            Vertex interMin = Vertex.Max(minA, minB);
            Vertex interMax = Vertex.Min(maxA, maxB);

            if (interMin.X <= interMax.X && interMin.Y <= interMax.Y)
            {
                result = new Box2(interMin, interMax);
                return true;
            }

            result = default;
            return false;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static Vertex ConstrainToBoundingBox(in Vertex p, in Box2 bbox)
            => Vertex.Min(Vertex.Max(p, bbox.Min), bbox.Max);
    }
}

// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

namespace SixLabors.PolygonClipper;

/// <summary>
/// Provides functionality to remove self-intersections from polygons using a sweep line algorithm.
/// </summary>
/// <remarks>
/// <para>
/// This class implements a dedicated algorithm for detecting and resolving self-intersections
/// within a single polygon. Unlike boolean operations which work on two polygons, this approach
/// processes only one polygon and rebuilds its contours after splitting segments at intersection points.
/// </para>
/// <para>
/// The algorithm works in three phases:
/// </para>
/// <list type="number">
/// <item><description>
/// <b>Intersection Detection:</b> Uses a sweep line to find all points where segments
/// from the same polygon intersect each other.
/// </description></item>
/// <item><description>
/// <b>Segment Splitting:</b> Divides segments at intersection points, creating new
/// vertices where crossings occur.
/// </description></item>
/// <item><description>
/// <b>Contour Reconstruction:</b> Rebuilds contours by grouping segments by their
/// original contour ID and connecting them by shared endpoints.
/// </description></item>
/// </list>
/// <para>
/// This implementation does not rely on any input metadata about contour types (external vs hole).
/// It preserves the original contour structure by tracking which segments belong to which
/// input contour throughout the process.
/// </para>
/// </remarks>
internal static class SelfIntersectionRemover
{
    /// <summary>
    /// Removes self-intersections from a polygon by detecting intersection points,
    /// splitting segments, and rebuilding contours.
    /// </summary>
    /// <param name="polygon">The polygon to process.</param>
    /// <returns>
    /// A new <see cref="Polygon"/> with all self-intersections resolved.
    /// If the input polygon has no self-intersections, returns a normalized copy.
    /// </returns>
    /// <remarks>
    /// <para>
    /// The method preserves the original contour structure of the polygon. Each input
    /// contour is processed independently, and segments are grouped back to their
    /// original contours after intersection splitting.
    /// </para>
    /// <para>
    /// For polygons with no self-intersections, this method returns a normalized copy
    /// of the input without running the full reconstruction.
    /// </para>
    /// </remarks>
    public static Polygon Process(Polygon polygon)
    {
        if (polygon.Count == 0)
        {
            return [];
        }

        // Phase 1 & 2: Run sweep line to detect intersections and split segments
        List<SweepEvent> processedEvents = RunSweepAndSplitSegments(polygon);

        // Phase 3: Rebuild contours from the processed segments
        return RebuildContours(processedEvents, polygon);
    }

    /// <summary>
    /// Runs the sweep line algorithm to detect self-intersections and split segments
    /// at intersection points.
    /// </summary>
    /// <param name="polygon">The input polygon.</param>
    /// <returns>
    /// A list of sweep events representing all segments after splitting at intersection points.
    /// Each event retains its original <see cref="SweepEvent.ContourId"/> for reconstruction.
    /// </returns>
    private static List<SweepEvent> RunSweepAndSplitSegments(Polygon polygon)
    {
        // Create sweep events for all segments
        SweepEventComparer comparer = new();
        List<SweepEvent> unorderedEvents = new(polygon.VertexCount * 2);

        int contourId = 0;
        for (int i = 0; i < polygon.Count; i++)
        {
            Contour contour = polygon[i];
            contourId++;

            for (int j = 0; j < contour.Count - 1; j++)
            {
                Segment segment = contour.GetSegment(j);
                if (segment.Source == segment.Target)
                {
                    // Skip degenerate zero-length segments
                    continue;
                }

                // Create left and right events for the segment
                SweepEvent e1 = new(segment.Source, true, PolygonType.Subject);
                SweepEvent e2 = new(segment.Target, true, e1, PolygonType.Subject);
                e1.OtherEvent = e2;
                e1.ContourId = e2.ContourId = contourId;
                e1.SegmentIndex = e2.SegmentIndex = j; // Track original position

                // Determine which endpoint is the left endpoint
                if (comparer.Compare(e1, e2) < 0)
                {
                    e2.Left = false;
                }
                else
                {
                    e1.Left = false;
                }

                unorderedEvents.Add(e1);
                unorderedEvents.Add(e2);
            }
        }

        // Process events using the sweep line
        StablePriorityQueue<SweepEvent, SweepEventComparer> eventQueue = new(comparer, unorderedEvents);
        List<SweepEvent> sortedEvents = new(unorderedEvents.Count);
        StatusLine statusLine = new(polygon.VertexCount >> 1);

        // Safety limit to prevent infinite loops
        int maxIterations = unorderedEvents.Count * 10;
        int iterations = 0;

        while (eventQueue.Count > 0)
        {
            if (++iterations > maxIterations)
            {
                // Exceeded safety limit - return what we have
                break;
            }

            SweepEvent sweepEvent = eventQueue.Dequeue();
            sortedEvents.Add(sweepEvent);

            if (sweepEvent.Left)
            {
                // Insert event into status line and get neighbors
                int position = sweepEvent.PosSL = statusLine.Add(sweepEvent);
                SweepEvent? prevEvent = statusLine.Prev(position);
                SweepEvent? nextEvent = statusLine.Next(position);

                // Check for intersections with neighbors
                if (nextEvent != null)
                {
                    CheckAndSplitIntersection(sweepEvent, nextEvent, eventQueue, comparer);
                }

                if (prevEvent != null)
                {
                    CheckAndSplitIntersection(prevEvent, sweepEvent, eventQueue, comparer);
                }
            }
            else
            {
                // Remove event from status line
                // Need to find the corresponding left event
                SweepEvent? leftEvent = sweepEvent.OtherEvent;
                if (leftEvent != null && leftEvent.PosSL >= 0 && leftEvent.PosSL < statusLine.Count)
                {
                    int position = leftEvent.PosSL;
                    SweepEvent? prevEvent = statusLine.Prev(position);
                    SweepEvent? nextEvent = statusLine.Next(position);

                    // Check for intersections between neighbors that are now adjacent
                    if (prevEvent != null && nextEvent != null)
                    {
                        CheckAndSplitIntersection(prevEvent, nextEvent, eventQueue, comparer);
                    }

                    statusLine.RemoveAt(position);
                }
            }
        }

        return sortedEvents;
    }

    /// <summary>
    /// Checks if two segments intersect and splits them at the intersection point if needed.
    /// </summary>
    /// <param name="event1">The first sweep event.</param>
    /// <param name="event2">The second sweep event.</param>
    /// <param name="eventQueue">The event queue to add new split events to.</param>
    /// <param name="comparer">The sweep event comparer.</param>
    private static void CheckAndSplitIntersection(
        SweepEvent event1,
        SweepEvent event2,
        StablePriorityQueue<SweepEvent, SweepEventComparer> eventQueue,
        SweepEventComparer comparer)
    {
        if (event1.OtherEvent == null || event2.OtherEvent == null)
        {
            return;
        }

        // For SELF-intersection removal, only process intersections within the same contour
        // Cross-contour intersections are not self-intersections
        if (event1.ContourId != event2.ContourId)
        {
            return;
        }

        // Find intersection points
        int intersectionCount = PolygonUtilities.FindIntersection(
            event1.GetSegment(),
            event2.GetSegment(),
            out Vertex intersectionPoint,
            out Vertex _);

        if (intersectionCount == 0)
        {
            return;
        }

        // Ignore trivial intersections at shared endpoints
        // This includes any case where the segments share an endpoint
        if (intersectionCount == 1)
        {
            bool shareEndpoint =
                event1.Point == event2.Point ||
                event1.Point == event2.OtherEvent.Point ||
                event1.OtherEvent.Point == event2.Point ||
                event1.OtherEvent.Point == event2.OtherEvent.Point;

            if (shareEndpoint)
            {
                return;
            }
        }

        // For overlapping segments (intersectionCount == 2), skip - they're part of original geometry
        if (intersectionCount == 2)
        {
            return;
        }

        // Split segments at intersection point(s)
        if (intersectionCount == 1)
        {
            // Split event1 if intersection is not at its endpoints
            if (event1.Point != intersectionPoint && event1.OtherEvent.Point != intersectionPoint)
            {
                DivideSegment(event1, intersectionPoint, eventQueue, comparer);
            }

            // Split event2 if intersection is not at its endpoints
            if (event2.Point != intersectionPoint && event2.OtherEvent.Point != intersectionPoint)
            {
                DivideSegment(event2, intersectionPoint, eventQueue, comparer);
            }
        }
    }

    /// <summary>
    /// Divides a segment at the specified point, creating two new segments.
    /// </summary>
    /// <param name="leftEvent">The left event of the segment to divide.</param>
    /// <param name="point">The point at which to divide the segment.</param>
    /// <param name="eventQueue">The event queue to add new events to.</param>
    /// <param name="comparer">The sweep event comparer.</param>
    private static void DivideSegment(
        SweepEvent leftEvent,
        Vertex point,
        StablePriorityQueue<SweepEvent, SweepEventComparer> eventQueue,
        SweepEventComparer comparer)
    {
        if (leftEvent.OtherEvent == null)
        {
            return;
        }

        SweepEvent rightEvent = leftEvent.OtherEvent;

        // Handle corner case: vertical segment at same X coordinate
        if (point.X == leftEvent.Point.X && point.Y < leftEvent.Point.Y)
        {
            point = new Vertex(point.X.NextAfter(double.PositiveInfinity), point.Y);
        }

        // Create new events for the split point
        SweepEvent newRight = new(point, false, leftEvent, leftEvent.PolygonType);
        SweepEvent newLeft = new(point, true, rightEvent, leftEvent.PolygonType);

        // Preserve contour ID for reconstruction
        // Mark as split segment by using a different segment index (negative)
        newRight.ContourId = newLeft.ContourId = leftEvent.ContourId;
        newRight.SegmentIndex = newLeft.SegmentIndex = -1; // Indicates split segment

        // Handle corner case: vertical segment ordering
        if (comparer.Compare(newLeft, rightEvent) > 0)
        {
            rightEvent.Left = true;
            newLeft.Left = false;
        }

        // Update event linkages
        rightEvent.OtherEvent = newLeft;
        leftEvent.OtherEvent = newRight;

        // Add new events to the queue
        eventQueue.Enqueue(newLeft);
        eventQueue.Enqueue(newRight);
    }

    /// <summary>
    /// Rebuilds contours from processed sweep events after intersection splitting.
    /// Segments are grouped by their original ContourId to preserve the input structure.
    /// When segments have been split (due to intersections), multiple contours may result.
    /// </summary>
    /// <param name="events">The processed sweep events after intersection splitting.</param>
    /// <param name="originalPolygon">The original polygon for fast-path when no splits occurred.</param>
    /// <returns>A new polygon with rebuilt contours.</returns>
    private static Polygon RebuildContours(List<SweepEvent> events, Polygon originalPolygon)
    {
        // Check if any segments were split
        bool anySplits = false;
        for (int i = 0; i < events.Count; i++)
        {
            if (events[i].SegmentIndex < 0)
            {
                anySplits = true;
                break;
            }
        }

        // If no splits occurred, use sweep events to determine hierarchy and copy original contours
        if (!anySplits)
        {
            // Determine containment depth for each contour from sweep events
            // When a contour's segment is added to the status line, the segment immediately
            // below it tells us which contour we're inside of (if any)
            int[] contourDepth = new int[originalPolygon.Count];
            int[] contourParent = new int[originalPolygon.Count];
            for (int i = 0; i < originalPolygon.Count; i++)
            {
                contourParent[i] = -1;
            }

            // Process events and track the status line to find containment
            // For each contour, record the parent from its first left event
            Dictionary<int, bool> contourFirstEventSeen = [];
            SweepEventComparer comparer = new();
            StatusLine statusLine = new(originalPolygon.VertexCount >> 1);

            foreach (SweepEvent evt in events)
            {
                if (evt.Left)
                {
                    int contourId = evt.ContourId;
                    bool isFirstEvent = !contourFirstEventSeen.ContainsKey(contourId);

                    // Add to status line
                    int position = statusLine.Add(evt);
                    evt.PosSL = position;

                    if (isFirstEvent)
                    {
                        contourFirstEventSeen[contourId] = true;

                        // The segment immediately below us tells us what we're inside of
                        SweepEvent? prevEvent = statusLine.Prev(position);
                        if (prevEvent != null && prevEvent.ContourId != contourId)
                        {
                            // We're inside the contour of prevEvent
                            int parentIdx = prevEvent.ContourId - 1;
                            int contourIdx = contourId - 1;

                            if (contourIdx >= 0 && contourIdx < originalPolygon.Count &&
                                parentIdx >= 0 && parentIdx < originalPolygon.Count)
                            {
                                contourParent[contourIdx] = parentIdx;

                                // Depth is parent's depth + 1
                                contourDepth[contourIdx] = contourDepth[parentIdx] + 1;
                            }
                        }
                    }
                }
                else
                {
                    // Remove from status line
                    SweepEvent? leftEvent = evt.OtherEvent;
                    if (leftEvent != null && leftEvent.PosSL >= 0 && leftEvent.PosSL < statusLine.Count)
                    {
                        statusLine.RemoveAt(leftEvent.PosSL);
                    }
                }
            }

            // Build result polygon with proper hierarchy
            List<Contour> originalContours = [];
            for (int i = 0; i < originalPolygon.Count; i++)
            {
                Contour copy = [];
                Contour original = originalPolygon[i];
                for (int j = 0; j < original.Count; j++)
                {
                    copy.AddVertex(original[j]);
                }

                copy.Depth = contourDepth[i];
                if (contourParent[i] >= 0)
                {
                    copy.ParentIndex = contourParent[i];
                }

                originalContours.Add(copy);
            }

            // Sort: externals first (depth 0), then their holes (depth 1), etc.
            return BuildPolygonFromContours(originalContours, contourDepth, contourParent);
        }

        // Splits occurred - need to rebuild from segments
        Dictionary<int, List<(Vertex Start, Vertex End, int SegmentIndex)>> segmentsByContour = [];
        Dictionary<int, bool> contourHasSplits = [];

        for (int i = 0; i < events.Count; i++)
        {
            SweepEvent evt = events[i];

            // Only process left events (each segment once)
            if (!evt.Left || evt.OtherEvent == null)
            {
                continue;
            }

            int contourId = evt.ContourId;
            if (!segmentsByContour.TryGetValue(contourId, out List<(Vertex Start, Vertex End, int SegmentIndex)>? segments))
            {
                segments = [];
                segmentsByContour[contourId] = segments;
                contourHasSplits[contourId] = false;
            }

            if (evt.SegmentIndex < 0)
            {
                contourHasSplits[contourId] = true;
            }

            segments.Add((evt.Point, evt.OtherEvent.Point, evt.SegmentIndex));
        }

        // Build contours per ContourId
        List<Contour> allContours = [];
        foreach (KeyValuePair<int, List<(Vertex Start, Vertex End, int SegmentIndex)>> kvp in segmentsByContour)
        {
            bool hasSplits = contourHasSplits[kvp.Key];
            List<Contour> contours = ChainSegmentsIntoContours(kvp.Value, hasSplits);
            allContours.AddRange(contours);
        }

        // Compute containment hierarchy and build final polygon
        return BuildPolygonWithHierarchy(allContours);
    }

    /// <summary>
    /// Builds the final polygon from contours with known depth and parent information.
    /// </summary>
    private static Polygon BuildPolygonFromContours(List<Contour> contours, int[] depth, int[] parent)
    {
        Polygon result = [];

        // Add contours sorted by depth (externals first, then holes)
        List<int> indices = [];
        for (int i = 0; i < contours.Count; i++)
        {
            indices.Add(i);
        }

        // Sort by depth ascending
        indices.Sort((a, b) => depth[a].CompareTo(depth[b]));

        foreach (int i in indices)
        {
            result.Add(contours[i]);
        }

        return result;
    }

    /// <summary>
    /// Chains segments into closed contours by matching endpoints.
    /// When hasSplits is false, follows original path order to produce a single contour.
    /// When hasSplits is true, allows multiple contours by following any available path.
    /// </summary>
    /// <param name="segments">List of segments with ordering information.</param>
    /// <param name="hasSplits">Whether this contour had segments split by intersections.</param>
    /// <returns>List of closed contours.</returns>
    private static List<Contour> ChainSegmentsIntoContours(
        List<(Vertex Start, Vertex End, int SegmentIndex)> segments,
        bool hasSplits)
    {
        List<Contour> result = [];
        if (segments.Count == 0)
        {
            return result;
        }

        // Sort segments by their original index (split segments have index -1, put them at end)
        segments.Sort((a, b) =>
        {
            if (a.SegmentIndex < 0 && b.SegmentIndex < 0)
            {
                return 0;
            }

            if (a.SegmentIndex < 0)
            {
                return 1;
            }

            if (b.SegmentIndex < 0)
            {
                return -1;
            }

            return a.SegmentIndex.CompareTo(b.SegmentIndex);
        });

        // Build map from SegmentIndex to list index for fast lookup
        Dictionary<int, int> segmentIndexToListIndex = [];
        for (int i = 0; i < segments.Count; i++)
        {
            int segIdx = segments[i].SegmentIndex;
            if (segIdx >= 0 && !segmentIndexToListIndex.ContainsKey(segIdx))
            {
                segmentIndexToListIndex[segIdx] = i;
            }
        }

        // Build adjacency: for each vertex, which segments connect to it (by list index after sorting)
        Dictionary<Vertex, List<int>> vertexToSegments = [];
        for (int i = 0; i < segments.Count; i++)
        {
            (Vertex start, Vertex end, _) = segments[i];

            if (!vertexToSegments.TryGetValue(start, out List<int>? list1))
            {
                list1 = [];
                vertexToSegments[start] = list1;
            }

            list1.Add(i);

            if (!vertexToSegments.TryGetValue(end, out List<int>? list2))
            {
                list2 = [];
                vertexToSegments[end] = list2;
            }

            list2.Add(i);
        }

        bool[] used = new bool[segments.Count];

        for (int startIdx = 0; startIdx < segments.Count; startIdx++)
        {
            if (used[startIdx])
            {
                continue;
            }

            Contour contour = [];
            int currentIdx = startIdx;
            Vertex startVertex = segments[startIdx].Start;
            Vertex currentVertex = startVertex;

            contour.AddVertex(currentVertex);
            used[currentIdx] = true;

            while (true)
            {
                // Move to the other end of current segment
                (Vertex segStart, Vertex segEnd, int currentSegIndex) = segments[currentIdx];
                Vertex nextVertex = (currentVertex == segStart) ? segEnd : segStart;

                contour.AddVertex(nextVertex);

                // Check if we've closed the loop
                if (nextVertex == startVertex)
                {
                    break;
                }

                // Find next segment at this vertex
                currentVertex = nextVertex;
                int nextIdx = -1;

                if (vertexToSegments.TryGetValue(currentVertex, out List<int>? connectedSegments))
                {
                    if (!hasSplits && currentSegIndex >= 0)
                    {
                        // No splits: prefer the segment with the next sequential SegmentIndex
                        int expectedNextSegIndex = currentSegIndex + 1;
                        if (segmentIndexToListIndex.TryGetValue(expectedNextSegIndex, out int expectedListIdx))
                        {
                            if (!used[expectedListIdx] && connectedSegments.Contains(expectedListIdx))
                            {
                                nextIdx = expectedListIdx;
                            }
                        }
                    }

                    // If no sequential match (or hasSplits), use any available segment
                    if (nextIdx == -1)
                    {
                        foreach (int idx in connectedSegments)
                        {
                            if (!used[idx])
                            {
                                nextIdx = idx;
                                break;
                            }
                        }
                    }
                }

                if (nextIdx == -1)
                {
                    // No more segments - contour ends here (open contour)
                    break;
                }

                currentIdx = nextIdx;
                used[currentIdx] = true;
            }

            if (contour.Count >= 4) // Need at least 3 unique vertices + closing vertex
            {
                result.Add(contour);
            }
        }

        return result;
    }

    /// <summary>
    /// Builds a polygon with proper parent/child hierarchy by computing containment relationships
    /// using a sweep line algorithm.
    /// </summary>
    /// <param name="contours">The list of contours to organize.</param>
    /// <returns>A polygon with contours properly organized as externals and holes.</returns>
    private static Polygon BuildPolygonWithHierarchy(List<Contour> contours)
    {
        if (contours.Count == 0)
        {
            return [];
        }

        int count = contours.Count;

        // Use sweep line to determine containment hierarchy
        // Create sweep events for all contours
        SweepEventComparer comparer = new();
        List<SweepEvent> allEvents = [];

        for (int contourIdx = 0; contourIdx < count; contourIdx++)
        {
            Contour contour = contours[contourIdx];
            for (int j = 0; j < contour.Count - 1; j++)
            {
                Segment segment = new(contour[j], contour[j + 1]);
                if (segment.Source == segment.Target)
                {
                    continue;
                }

                SweepEvent e1 = new(segment.Source, true, PolygonType.Subject);
                SweepEvent e2 = new(segment.Target, true, e1, PolygonType.Subject);
                e1.OtherEvent = e2;
                e1.ContourId = e2.ContourId = contourIdx + 1;

                if (comparer.Compare(e1, e2) < 0)
                {
                    e2.Left = false;
                }
                else
                {
                    e1.Left = false;
                }

                allEvents.Add(e1);
                allEvents.Add(e2);
            }
        }

        // Process events to determine containment
        int[] contourDepth = new int[count];
        int[] contourParent = new int[count];
        for (int i = 0; i < count; i++)
        {
            contourParent[i] = -1;
        }

        StablePriorityQueue<SweepEvent, SweepEventComparer> eventQueue = new(comparer, allEvents);
        StatusLine statusLine = new(allEvents.Count >> 1);
        Dictionary<int, bool> contourFirstEventSeen = [];

        while (eventQueue.Count > 0)
        {
            SweepEvent evt = eventQueue.Dequeue();

            if (evt.Left)
            {
                int contourId = evt.ContourId;
                bool isFirstEvent = !contourFirstEventSeen.ContainsKey(contourId);

                int position = statusLine.Add(evt);
                evt.PosSL = position;

                if (isFirstEvent)
                {
                    contourFirstEventSeen[contourId] = true;

                    SweepEvent? prevEvent = statusLine.Prev(position);
                    if (prevEvent != null && prevEvent.ContourId != contourId)
                    {
                        int parentIdx = prevEvent.ContourId - 1;
                        int contourIdx = contourId - 1;

                        if (contourIdx >= 0 && contourIdx < count &&
                            parentIdx >= 0 && parentIdx < count)
                        {
                            contourParent[contourIdx] = parentIdx;
                            contourDepth[contourIdx] = contourDepth[parentIdx] + 1;
                        }
                    }
                }
            }
            else
            {
                SweepEvent? leftEvent = evt.OtherEvent;
                if (leftEvent != null && leftEvent.PosSL >= 0 && leftEvent.PosSL < statusLine.Count)
                {
                    statusLine.RemoveAt(leftEvent.PosSL);
                }
            }
        }

        // Set contour properties
        for (int i = 0; i < count; i++)
        {
            contours[i].Depth = contourDepth[i];
            if (contourParent[i] >= 0)
            {
                contours[i].ParentIndex = contourParent[i];
            }
        }

        // Build result sorted by depth
        Polygon result = [];
        List<int> indices = [count];
        for (int i = 0; i < count; i++)
        {
            indices.Add(i);
        }

        indices.Sort((a, b) => contourDepth[a].CompareTo(contourDepth[b]));

        foreach (int i in indices)
        {
            result.Add(contours[i]);
        }

        return result;
    }
}

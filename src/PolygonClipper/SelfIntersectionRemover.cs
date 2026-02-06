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
            // When a contour's first left event is processed, count how many other contours
            // have segments both above AND below it in the status line
            int[] contourDepth = new int[originalPolygon.Count];
            int[] contourParent = new int[originalPolygon.Count];
            for (int i = 0; i < originalPolygon.Count; i++)
            {
                contourParent[i] = -1;
            }

            // Find the first left event for each contour and compute its depth
            Dictionary<int, bool> contourFirstEventSeen = [];
            foreach (SweepEvent evt in events)
            {
                if (!evt.Left)
                {
                    continue;
                }

                int contourId = evt.ContourId;
                if (contourFirstEventSeen.ContainsKey(contourId))
                {
                    continue;
                }

                contourFirstEventSeen[contourId] = true;

                // Count segments from other contours that are below this one in the status line
                // The number of such segments (considering in/out) tells us the depth
                int depth = 0;
                int parentContourId = -1;
                double parentArea = double.MaxValue;

                // Check events that were processed before this one at the same or earlier X
                // and are still "active" (their right endpoint hasn't been reached)
                foreach (SweepEvent other in events)
                {
                    if (other == evt || !other.Left || other.ContourId == contourId)
                    {
                        continue;
                    }

                    if (other.OtherEvent == null)
                    {
                        continue;
                    }

                    // Check if 'other' segment is active at evt's X position
                    // (started before or at evt.Point.X and ends after evt.Point.X)
                    double otherLeftX = Math.Min(other.Point.X, other.OtherEvent.Point.X);
                    double otherRightX = Math.Max(other.Point.X, other.OtherEvent.Point.X);

                    if (otherLeftX > evt.Point.X || otherRightX <= evt.Point.X)
                    {
                        continue;
                    }

                    // Compute Y of 'other' segment at evt.Point.X
                    double otherY = ComputeYAtX(other.Point, other.OtherEvent.Point, evt.Point.X);

                    // If other segment is below evt, we're inside that contour
                    if (otherY < evt.Point.Y)
                    {
                        depth++;
                        int otherContourIdx = other.ContourId - 1;
                        if (otherContourIdx >= 0 && otherContourIdx < originalPolygon.Count)
                        {
                            double area = Math.Abs(ComputeArea(originalPolygon[otherContourIdx]));
                            if (area < parentArea)
                            {
                                parentArea = area;
                                parentContourId = other.ContourId;
                            }
                        }
                    }
                }

                int contourIdx = contourId - 1;
                if (contourIdx >= 0 && contourIdx < originalPolygon.Count)
                {
                    contourDepth[contourIdx] = depth;
                    contourParent[contourIdx] = parentContourId - 1;
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
    /// Computes the Y coordinate of a line segment at a given X coordinate.
    /// </summary>
    private static double ComputeYAtX(Vertex p1, Vertex p2, double x)
    {
        if (Math.Abs(p2.X - p1.X) < 1e-10)
        {
            return (p1.Y + p2.Y) / 2;
        }

        double t = (x - p1.X) / (p2.X - p1.X);
        return p1.Y + (t * (p2.Y - p1.Y));
    }

    /// <summary>
    /// Builds the final polygon from contours with known depth and parent information.
    /// </summary>
    private static Polygon BuildPolygonFromContours(List<Contour> contours, int[] depth, int[] parent)
    {
        Polygon result = [];
        bool[] added = new bool[contours.Count];

        // Add externals (depth even) first, sorted by depth then by area
        List<int> indices = [];
        for (int i = 0; i < contours.Count; i++)
        {
            indices.Add(i);
        }

        // Sort: externals (even depth) first by depth ascending, then their holes
        indices.Sort((a, b) =>
        {
            int depthCompare = depth[a].CompareTo(depth[b]);
            if (depthCompare != 0)
            {
                return depthCompare;
            }

            // Same depth - sort by area descending (larger first)
            double areaA = Math.Abs(ComputeArea(contours[a]));
            double areaB = Math.Abs(ComputeArea(contours[b]));
            return areaB.CompareTo(areaA);
        });

        foreach (int i in indices)
        {
            if (!added[i])
            {
                result.Add(contours[i]);
                added[i] = true;
            }
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
    /// Builds a polygon with proper parent/child hierarchy by computing containment relationships.
    /// </summary>
    /// <param name="contours">The list of contours to organize.</param>
    /// <returns>A polygon with contours properly organized as externals and holes.</returns>
    /// <remarks>
    /// <para>
    /// This method determines which contours are external (shells) and which are holes
    /// by computing containment relationships. A contour is a hole if it is contained
    /// within another contour.
    /// </para>
    /// <para>
    /// The algorithm:
    /// </para>
    /// <list type="number">
    /// <item><description>For each contour, find which other contours contain it.</description></item>
    /// <item><description>The direct parent is the smallest containing contour.</description></item>
    /// <item><description>Contours with no parent are external; others are holes.</description></item>
    /// </list>
    /// </remarks>
    private static Polygon BuildPolygonWithHierarchy(List<Contour> contours)
    {
        if (contours.Count == 0)
        {
            return [];
        }

        int count = contours.Count;

        // Compute areas and bounding boxes for all contours
        double[] areas = new double[count];
        (double MinX, double MinY, double MaxX, double MaxY)[] bounds = new (double, double, double, double)[count];

        for (int i = 0; i < count; i++)
        {
            areas[i] = Math.Abs(ComputeArea(contours[i]));
            bounds[i] = ComputeBounds(contours[i]);
        }

        // For each contour, find its parent (smallest containing contour)
        int[] parentIndex = new int[count];
        for (int i = 0; i < count; i++)
        {
            parentIndex[i] = -1; // -1 means no parent (external)
        }

        // Compute containment: for each pair, check if one contains the other
        for (int i = 0; i < count; i++)
        {
            Contour contourI = contours[i];
            if (contourI.Count < 3)
            {
                continue;
            }

            // Get a test point - try the first vertex, then centroid as fallback
            Vertex testPoint = contourI[0];

            double smallestArea = double.MaxValue;
            int smallestContainingIndex = -1;

            for (int j = 0; j < count; j++)
            {
                if (i == j)
                {
                    continue;
                }

                Contour contourJ = contours[j];
                if (contourJ.Count < 3)
                {
                    continue;
                }

                // Quick rejection: if J's bounding box doesn't contain the test point, skip
                (double minX, double minY, double maxX, double maxY) = bounds[j];
                if (testPoint.X < minX || testPoint.X > maxX || testPoint.Y < minY || testPoint.Y > maxY)
                {
                    continue;
                }

                // Check if contourJ contains the test point from contourI
                if (ContainsPoint(contourJ, testPoint))
                {
                    if (areas[j] < smallestArea)
                    {
                        smallestArea = areas[j];
                        smallestContainingIndex = j;
                    }
                }
            }

            parentIndex[i] = smallestContainingIndex;
        }

        // Build the result polygon with proper hierarchy
        // We need to compute depth for each contour
        int[] depth = new int[count];
        for (int i = 0; i < count; i++)
        {
            int d = 0;
            int current = parentIndex[i];
            HashSet<int> visited = [i];
            while (current != -1 && d < count)
            {
                if (visited.Contains(current))
                {
                    // Cycle detected - break to prevent infinite loop
                    break;
                }

                visited.Add(current);
                d++;
                current = parentIndex[current];
            }

            depth[i] = d;
        }

        // Set contour properties based on depth
        // Even depth = external, odd depth = hole
        for (int i = 0; i < count; i++)
        {
            Contour contour = contours[i];
            contour.Depth = depth[i];

            if (parentIndex[i] != -1)
            {
                contour.ParentIndex = parentIndex[i];
            }
        }

        // Build result: externals first (sorted by area descending), then their holes
        Polygon result = [];
        bool[] added = new bool[count];

        // Sort externals by area (largest first)
        List<int> externals = [];
        for (int i = 0; i < count; i++)
        {
            if (depth[i] % 2 == 0)
            {
                externals.Add(i);
            }
        }

        externals.Sort((a, b) => areas[b].CompareTo(areas[a]));

        // Add external contours and their holes
        foreach (int i in externals)
        {
            if (added[i])
            {
                continue;
            }

            // This is an external contour
            result.Add(contours[i]);
            added[i] = true;

            // Add its direct holes (depth = this depth + 1 and parent = i)
            for (int j = 0; j < count; j++)
            {
                if (!added[j] && parentIndex[j] == i)
                {
                    result.Add(contours[j]);
                    added[j] = true;
                }
            }
        }

        // Add any remaining contours (shouldn't happen with valid input)
        for (int i = 0; i < count; i++)
        {
            if (!added[i])
            {
                result.Add(contours[i]);
            }
        }

        return result;
    }

    /// <summary>
    /// Computes the bounding box of a contour.
    /// </summary>
    private static (double MinX, double MinY, double MaxX, double MaxY) ComputeBounds(Contour contour)
    {
        if (contour.Count == 0)
        {
            return (0, 0, 0, 0);
        }

        double minX = double.MaxValue;
        double minY = double.MaxValue;
        double maxX = double.MinValue;
        double maxY = double.MinValue;

        for (int i = 0; i < contour.Count; i++)
        {
            Vertex v = contour[i];
            if (v.X < minX)
            {
                minX = v.X;
            }

            if (v.X > maxX)
            {
                maxX = v.X;
            }

            if (v.Y < minY)
            {
                minY = v.Y;
            }

            if (v.Y > maxY)
            {
                maxY = v.Y;
            }
        }

        return (minX, minY, maxX, maxY);
    }

    /// <summary>
    /// Determines if a point is inside a contour using the ray casting algorithm.
    /// </summary>
    /// <param name="contour">The contour to test against.</param>
    /// <param name="point">The point to test.</param>
    /// <returns><see langword="true"/> if the point is inside the contour; otherwise <see langword="false"/>.</returns>
    private static bool ContainsPoint(Contour contour, Vertex point)
    {
        int count = contour.Count;
        if (count < 3)
        {
            return false;
        }

        bool inside = false;
        Vertex p1 = contour[count - 1];

        for (int i = 0; i < count; i++)
        {
            Vertex p2 = contour[i];

            if (point.Y > Math.Min(p1.Y, p2.Y) &&
                point.Y <= Math.Max(p1.Y, p2.Y) &&
                point.X <= Math.Max(p1.X, p2.X))
            {
                double xIntersection = ((point.Y - p1.Y) * (p2.X - p1.X) / (p2.Y - p1.Y)) + p1.X;
                if (p1.X == p2.X || point.X <= xIntersection)
                {
                    inside = !inside;
                }
            }

            p1 = p2;
        }

        return inside;
    }

    /// <summary>
    /// Computes the signed area of a contour using the shoelace formula.
    /// </summary>
    /// <param name="contour">The contour to compute the area for.</param>
    /// <returns>The signed area (positive for counter-clockwise, negative for clockwise).</returns>
    private static double ComputeArea(Contour contour)
    {
        int count = contour.Count;
        if (count < 3)
        {
            return 0;
        }

        double area = 0;
        for (int i = 0; i < count; i++)
        {
            Vertex current = contour[i];
            Vertex next = contour[(i + 1) % count];
            area += (current.X * next.Y) - (next.X * current.Y);
        }

        return area * .5d;
    }
}

// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System.Diagnostics.CodeAnalysis;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace SixLabors.PolygonClipper;

/// <summary>
/// Provides functionality to remove self-intersections from polygons using the positive
/// fill rule (non-zero winding) and a sweep line algorithm.
/// </summary>
/// <remarks>
/// <para>
/// This class implements a sweep line algorithm that applies the positive fill rule
/// to resolve self-intersections within a polygon. A region is considered "inside"
/// if its winding number is greater than zero.
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
/// <b>Boundary Extraction:</b> Computes winding numbers and keeps only edges that
/// form the boundary between filled (winding > 0) and unfilled (winding ≤ 0) regions.
/// </description></item>
/// </list>
/// </remarks>
internal static class SelfIntersectionRemover
{
    /// <summary>
    /// Processes a polygon to remove self-intersections using the positive fill rule.
    /// </summary>
    /// <param name="polygon">The polygon to process.</param>
    /// <returns>
    /// A new <see cref="Polygon"/> with self-intersections resolved. Regions with
    /// positive winding number are considered filled.
    /// </returns>
    public static Polygon Process(Polygon polygon)
    {
        if (polygon.Count == 0)
        {
            return [];
        }

        // Normalize contour orientation so positive fill (Clipper2 union) is consistent.
        Polygon orientedPolygon = OrientContoursForPositiveFill(polygon);

        // Sweep and split to build a planar arrangement with explicit intersection vertices.
        List<DirectedSegment> segments = BuildArrangementSegments(orientedPolygon);
        if (segments.Count == 0)
        {
            return [];
        }

        // Build half-edges, enumerate faces, and classify boundary edges by winding.
        HalfEdgeGraph graph = BuildHalfEdgeGraph(segments);
        List<Face> faces = EnumerateFaces(graph);
        ComputeFaceWindings(graph, faces, segments);
        List<Contour> contours = ExtractBoundaryContours(graph);
        List<Contour> mergedContours = MergeTouchingContours(contours);

        List<Contour> result = new(mergedContours.Count);
        foreach (Contour contour in mergedContours)
        {
            CleanupContour(contour);
            if (contour.Count > 2)
            {
                RotateContourToLowestPoint(contour);
                result.Add(contour);
            }
        }

        // Match Clipper2 ordering: highest Y (then lowest X) first.
        result.Sort(CompareContoursByLowestPoint);
        Polygon resultPolygon = new(result.Count);
        for (int i = 0; i < result.Count; i++)
        {
            resultPolygon.Add(result[i]);
        }

        AssignHierarchy(resultPolygon);
        return resultPolygon;
    }

    /// <summary>
    /// Compares two contours using the Y coordinate of their lowest vertex, breaking ties by X.
    /// </summary>
    /// <param name="left">The first contour to compare.</param>
    /// <param name="right">The second contour to compare.</param>
    /// <returns>A value indicating the relative ordering for sorting.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static int CompareContoursByLowestPoint(Contour left, Contour right)
    {
        Vertex leftPoint = GetLowestPoint(left);
        Vertex rightPoint = GetLowestPoint(right);

        if (leftPoint.Y > rightPoint.Y)
        {
            return -1;
        }

        return leftPoint.Y < rightPoint.Y ? 1 : leftPoint.X.CompareTo(rightPoint.X);
    }

    /// <summary>
    /// Finds the lowest vertex in a contour, ignoring the duplicated closing vertex if present.
    /// </summary>
    /// <param name="contour">The contour to scan.</param>
    /// <returns>The lowest vertex, or <see cref="Vertex"/> default when the contour is empty.</returns>
    private static Vertex GetLowestPoint(Contour contour)
    {
        int count = contour.Count;
        if (count == 0)
        {
            return default;
        }

        int lastIndex = count - 1;
        if (count > 1 && contour[0] == contour[^1])
        {
            lastIndex = count - 2;
        }

        Vertex lowest = contour[0];
        for (int i = 1; i <= lastIndex; i++)
        {
            Vertex candidate = contour[i];
            if (candidate.Y > lowest.Y || (candidate.Y == lowest.Y && candidate.X < lowest.X))
            {
                lowest = candidate;
            }
        }

        return lowest;
    }

    /// <summary>
    /// Rotates the contour so that its lowest vertex appears first, keeping relative ordering intact.
    /// </summary>
    /// <param name="contour">The contour to normalize.</param>
    private static void RotateContourToLowestPoint(Contour contour)
    {
        int count = contour.Count;
        if (count < 2)
        {
            return;
        }

        bool isClosed = contour[0] == contour[^1];
        int limit = isClosed ? count - 1 : count;

        int lowestIndex = 0;
        Vertex lowest = contour[0];
        for (int i = 1; i < limit; i++)
        {
            Vertex candidate = contour[i];
            if (candidate.Y > lowest.Y || (candidate.Y == lowest.Y && candidate.X < lowest.X))
            {
                lowest = candidate;
                lowestIndex = i;
            }
        }

        if (lowestIndex == 0)
        {
            return;
        }

        List<Vertex> rotated = new(limit + (isClosed ? 1 : 0));
        for (int i = 0; i < limit; i++)
        {
            rotated.Add(contour[(i + lowestIndex) % limit]);
        }

        if (isClosed)
        {
            rotated.Add(rotated[0]);
        }

        contour.Clear();
        for (int i = 0; i < rotated.Count; i++)
        {
            contour.Add(rotated[i]);
        }
    }

    /// <summary>
    /// Ensures every contour is oriented with positive winding for outer rings and negative for holes.
    /// </summary>
    /// <param name="polygon">The polygon whose contours need reorientation.</param>
    /// <returns>A polygon copy with consistent winding order.</returns>
    private static Polygon OrientContoursForPositiveFill(Polygon polygon)
    {
        if (polygon.Count == 0)
        {
            return polygon;
        }

        int count = polygon.Count;
        int[] parentIndices = new int[count];
        Array.Fill(parentIndices, -1);

        // If the input already carries hierarchy info, keep it; otherwise infer containment.
        bool hasHierarchy = false;
        for (int i = 0; i < count; i++)
        {
            Contour contour = polygon[i];
            if (contour.ParentIndex != null || contour.HoleCount > 0)
            {
                hasHierarchy = true;
                break;
            }
        }

        if (hasHierarchy)
        {
            for (int i = 0; i < count; i++)
            {
                parentIndices[i] = polygon[i].ParentIndex ?? -1;
            }
        }
        else
        {
            Box2[] bounds = new Box2[count];
            for (int i = 0; i < count; i++)
            {
                bounds[i] = polygon[i].GetBoundingBox();
            }

            for (int i = 0; i < count; i++)
            {
                Vertex testPoint = GetContourTestPoint(polygon[i]);
                double smallestArea = double.PositiveInfinity;
                int parentIndex = -1;

                for (int j = 0; j < count; j++)
                {
                    if (i == j)
                    {
                        continue;
                    }

                    // Quick reject on bounds before doing full containment and intersection checks.
                    if (!BoundsOverlap(bounds[i], bounds[j]) || !PointInBounds(testPoint, bounds[j]))
                    {
                        continue;
                    }

                    Contour candidate = polygon[j];
                    if (PointInContour(testPoint, candidate))
                    {
                        // If contours intersect or touch, do not treat the candidate as a parent.
                        if (ContoursHaveIntersection(polygon[i], candidate, bounds[i], bounds[j]))
                        {
                            continue;
                        }

                        // Choose the tightest containing contour as the parent.
                        double area = Math.Abs(GetSignedArea(candidate));
                        if (area < smallestArea)
                        {
                            smallestArea = area;
                            parentIndex = j;
                        }
                    }
                }

                parentIndices[i] = parentIndex;
            }
        }

        int[] depths = new int[count];
        for (int i = 0; i < count; i++)
        {
            // Depth parity determines external vs hole orientation.
            depths[i] = GetDepth(i, parentIndices);
        }

        Polygon oriented = new(count);
        for (int i = 0; i < count; i++)
        {
            Contour copy = [];
            Contour original = polygon[i];
            for (int j = 0; j < original.Count; j++)
            {
                copy.Add(original[j]);
            }

            if (depths[i] % 2 == 0)
            {
                copy.SetCounterClockwise();
            }
            else
            {
                copy.SetClockwise();
            }

            oriented.Add(copy);
        }

        return oriented;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool BoundsOverlap(in Box2 left, in Box2 right)
        => left.Min.X <= right.Max.X && left.Max.X >= right.Min.X &&
           left.Min.Y <= right.Max.Y && left.Max.Y >= right.Min.Y;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool PointInBounds(in Vertex point, in Box2 bounds)
        => point.X >= bounds.Min.X && point.X <= bounds.Max.X &&
           point.Y >= bounds.Min.Y && point.Y <= bounds.Max.Y;

    private static bool ContoursHaveIntersection(Contour subject, Contour clipping, Box2 subjectBounds, Box2 clippingBounds)
    {
        if (!BoundsOverlap(subjectBounds, clippingBounds))
        {
            return false;
        }

        // If any segment pair intersects, containment is ambiguous, so avoid nesting.
        for (int i = 0; i < subject.Count; i++)
        {
            Segment subjectSegment = subject.GetSegment(i);
            if (subjectSegment.IsDegenerate())
            {
                continue;
            }

            for (int j = 0; j < clipping.Count; j++)
            {
                Segment clippingSegment = clipping.GetSegment(j);
                if (clippingSegment.IsDegenerate())
                {
                    continue;
                }

                if (PolygonUtilities.FindIntersection(subjectSegment, clippingSegment, out _, out _) != 0)
                {
                    return true;
                }
            }
        }

        return false;
    }

    /// <summary>
    /// Computes the nesting depth for a contour relative to its parents.
    /// </summary>
    /// <param name="index">The index of the contour.</param>
    /// <param name="parentIndices">The parent lookup table.</param>
    /// <returns>The zero-based depth.</returns>
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

    /// <summary>
    /// Selects a representative vertex from a contour for containment tests.
    /// </summary>
    /// <param name="contour">The contour to examine.</param>
    /// <returns>A vertex guaranteed not to be the duplicated closing point.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vertex GetContourTestPoint(Contour contour)
    {
        if (contour.Count == 0)
        {
            return default;
        }

        Vertex first = contour[0];
        if (contour.Count > 1 && first == contour[^1])
        {
            return contour[1];
        }

        return first;
    }

    /// <summary>
    /// Determines whether a point lies inside a contour using ray casting. Points on the boundary count as inside.
    /// </summary>
    /// <param name="point">The query point.</param>
    /// <param name="contour">The contour to test against.</param>
    /// <returns><see langword="true"/> if the point lies inside or on the edge.</returns>
    private static bool PointInContour(in Vertex point, Contour contour)
    {
        int count = contour.Count;
        if (count < 3)
        {
            return false;
        }

        bool inside = false;
        Vertex previous = contour[count - 1];

        for (int i = 0; i < count; i++)
        {
            Vertex current = contour[i];
            if (current == previous)
            {
                previous = current;
                continue;
            }

            if (IsPointOnSegment(point, previous, current))
            {
                return true;
            }

            bool intersects = (current.Y > point.Y) != (previous.Y > point.Y);
            if (intersects)
            {
                double xIntersection = ((previous.X - current.X) * (point.Y - current.Y) / (previous.Y - current.Y)) + current.X;
                if (point.X < xIntersection)
                {
                    inside = !inside;
                }
            }

            previous = current;
        }

        return inside;
    }

    /// <summary>
    /// Tests whether a point lies on the closed segment between two vertices.
    /// </summary>
    /// <param name="point">The point to check.</param>
    /// <param name="a">Segment start.</param>
    /// <param name="b">Segment end.</param>
    /// <returns><see langword="true"/> when the point is collinear and within the segment bounds.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsPointOnSegment(in Vertex point, in Vertex a, in Vertex b)
    {
        if (PolygonUtilities.SignedArea(a, b, point) != 0D)
        {
            return false;
        }

        Vertex min = Vertex.Min(a, b);
        Vertex max = Vertex.Max(a, b);

        return point.X >= min.X && point.X <= max.X && point.Y >= min.Y && point.Y <= max.Y;
    }

    /// <summary>
    /// Calculates the signed area of a contour. Positive values indicate counter-clockwise orientation.
    /// </summary>
    /// <param name="contour">The contour to measure.</param>
    /// <returns>The signed area in square units.</returns>
    private static double GetSignedArea(Contour contour)
    {
        double area = 0D;
        for (int i = 0; i < contour.Count; i++)
        {
            Vertex current = contour[i];
            Vertex next = contour[(i + 1) % contour.Count];
            area += Vertex.Cross(current, next);
        }

        return area * 0.5D;
    }

    /// <summary>
    /// Converts oriented contours into directed segments, splitting where necessary using a sweep-line.
    /// </summary>
    /// <param name="polygon">The positively oriented polygon.</param>
    /// <returns>A list of directed segments describing the arrangement.</returns>
    private static List<DirectedSegment> BuildArrangementSegments(Polygon polygon)
    {
        SweepEventComparer comparer = new();
        List<SweepEvent> unorderedEvents = new(polygon.VertexCount * 2);
        List<SegmentSplit> segmentSplits = new(polygon.VertexCount);

        int contourId = 0;
        int segmentId = 0;
        for (int i = 0; i < polygon.Count; i++)
        {
            Contour contour = polygon[i];
            contourId++;

            // Each original segment contributes +1 to the winding delta for positive fill.
            const int windDelta = 1;

            for (int j = 0; j < contour.Count - 1; j++)
            {
                Segment segment = contour.GetSegment(j);
                if (segment.Source == segment.Target)
                {
                    continue;
                }

                // Track the original segment so we can accumulate all split points by ID.
                // Each vertex pair produces two sweep events (left/right) so the sweep-line can detect intersections.
                SweepEvent e1 = new(segment.Source, true, PolygonType.Subject);
                SweepEvent e2 = new(segment.Target, true, e1, PolygonType.Subject);
                e1.OtherEvent = e2;
                e1.ContourId = e2.ContourId = contourId;
                e1.SegmentId = e2.SegmentId = segmentId;
                e1.IsContourSource = true;
                e2.IsContourSource = false;
                e1.SegmentSource = segment.Source;
                e1.SegmentTarget = segment.Target;
                e2.SegmentSource = segment.Source;
                e2.SegmentTarget = segment.Target;

                if (comparer.Compare(e1, e2) < 0)
                {
                    e2.Left = false;
                }
                else
                {
                    e1.Left = false;
                }

                // WindDx stays with the original segment orientation even as it is split.
                int windDx = e1.Left ? 1 : -1;
                e1.WindDx = e2.WindDx = windDx;

                unorderedEvents.Add(e1);
                unorderedEvents.Add(e2);

                segmentSplits.Add(new SegmentSplit(segment.Source, segment.Target, windDelta));
                segmentId++;
            }
        }

        if (unorderedEvents.Count == 0)
        {
            return [];
        }

        StablePriorityQueue<SweepEvent, SweepEventComparer> eventQueue = new(comparer, unorderedEvents);
        StatusLine statusLine = new(polygon.VertexCount >> 1);
        Span<SegmentSplit> segmentSplitSpan = CollectionsMarshal.AsSpan(segmentSplits);

        // Guard against pathological input that could otherwise leave the sweep stuck in an infinite loop.
        int maxIterations = unorderedEvents.Count * 10;
        int iterations = 0;
        Span<SweepEvent> workspace = new SweepEvent[4];

        while (eventQueue.Count > 0)
        {
            if (++iterations > maxIterations)
            {
                break;
            }

            SweepEvent sweepEvent = eventQueue.Dequeue();

            if (sweepEvent.Left)
            {
                // Add to the sweep line, then check for intersections with immediate neighbors.
                int position = sweepEvent.PosSL = statusLine.Add(sweepEvent);
                SweepEvent? prevEvent = statusLine.Prev(position);
                SweepEvent? nextEvent = statusLine.Next(position);

                if (nextEvent != null)
                {
                    _ = PossibleIntersection(sweepEvent, nextEvent, eventQueue, workspace, comparer, segmentSplitSpan);
                }

                if (prevEvent != null)
                {
                    _ = PossibleIntersection(prevEvent, sweepEvent, eventQueue, workspace, comparer, segmentSplitSpan);
                }
            }
            else
            {
                // Remove the corresponding left event and check for new neighbor intersections.
                SweepEvent? leftEvent = sweepEvent.OtherEvent;
                if (leftEvent != null && leftEvent.PosSL >= 0 && leftEvent.PosSL < statusLine.Count)
                {
                    int position = leftEvent.PosSL;
                    SweepEvent? prevEvent = statusLine.Prev(position);
                    SweepEvent? nextEvent = statusLine.Next(position);

                    if (prevEvent != null && nextEvent != null)
                    {
                        _ = PossibleIntersection(prevEvent, nextEvent, eventQueue, workspace, comparer, segmentSplitSpan);
                    }

                    statusLine.RemoveAt(position);
                }
            }
        }

        List<DirectedSegment> segments = BuildSegmentsFromSplits(segmentSplits);
        return MergeDuplicateSegments(segments);
    }

    /// <summary>
    /// Builds the final directed segments by sorting and emitting split points per original segment.
    /// </summary>
    private static List<DirectedSegment> BuildSegmentsFromSplits(List<SegmentSplit> segmentSplits)
    {
        List<DirectedSegment> segments = new(segmentSplits.Count * 2);

        for (int i = 0; i < segmentSplits.Count; i++)
        {
            SegmentSplit segmentSplit = segmentSplits[i];
            List<Vertex>? splits = segmentSplit.Splits;
            if (splits == null)
            {
                if (segmentSplit.Source != segmentSplit.Target)
                {
                    segments.Add(new DirectedSegment(segmentSplit.Source, segmentSplit.Target, segmentSplit.WindDelta));
                }

                continue;
            }

            if (splits.Count < 2)
            {
                continue;
            }

            Vertex source = segmentSplit.Source;
            Vertex target = segmentSplit.Target;
            int windDelta = segmentSplit.WindDelta;
            double dx = target.X - source.X;
            double dy = target.Y - source.Y;
            if (dx == 0D && dy == 0D)
            {
                continue;
            }

            // Sort split points by parameter along the original segment to preserve direction.
            // Use the dominant axis to avoid unstable division when one component is near zero.
            if (Math.Abs(dx) >= Math.Abs(dy))
            {
                if (dx > 0D)
                {
                    splits.Sort((a, b) => a.X.CompareTo(b.X));
                }
                else
                {
                    splits.Sort((a, b) => b.X.CompareTo(a.X));
                }
            }
            else
            {
                if (dy > 0D)
                {
                    splits.Sort((a, b) => a.Y.CompareTo(b.Y));
                }
                else
                {
                    splits.Sort((a, b) => b.Y.CompareTo(a.Y));
                }
            }

            Vertex last = splits[0];
            for (int j = 1; j < splits.Count; j++)
            {
                Vertex next = splits[j];
                if (ArePointsClose(last, next))
                {
                    continue;
                }

                // Emit directed sub-segments that follow the original segment direction.
                segments.Add(new DirectedSegment(last, next, windDelta));
                last = next;
            }
        }

        return segments;
    }

    /// <summary>
    /// Combines identical directed segments so their winding deltas accumulate once per unique edge.
    /// </summary>
    /// <param name="segments">The raw directed segments.</param>
    /// <returns>The merged segment list.</returns>
    private static List<DirectedSegment> MergeDuplicateSegments(List<DirectedSegment> segments)
    {
        if (segments.Count < 2)
        {
            return segments;
        }

        // Merge overlapping directed segments so winding deltas accumulate once per unique edge.
        Dictionary<(Vertex Source, Vertex Target), int> merged = new(segments.Count);
        for (int i = 0; i < segments.Count; i++)
        {
            DirectedSegment segment = segments[i];
            (Vertex Source, Vertex Target) key = (segment.Source, segment.Target);
            merged[key] = merged.TryGetValue(key, out int count)
                ? count + segment.WindDelta
                : segment.WindDelta;
        }

        List<DirectedSegment> result = new(merged.Count);
        foreach (KeyValuePair<(Vertex Source, Vertex Target), int> entry in merged)
        {
            if (entry.Value == 0)
            {
                continue;
            }

            result.Add(new DirectedSegment(entry.Key.Source, entry.Key.Target, entry.Value));
        }

        return result;
    }

    /// <summary>
    /// Builds a half-edge graph from the directed segments so faces and windings can be enumerated.
    /// </summary>
    /// <param name="segments">The segments produced by the arrangement phase.</param>
    /// <returns>A populated half-edge graph.</returns>
    private static HalfEdgeGraph BuildHalfEdgeGraph(List<DirectedSegment> segments)
    {
        int edgeCapacity = segments.Count * 2;
        HalfEdgeGraph graph = new(segments.Count, edgeCapacity);
        Dictionary<Vertex, Node> nodeMap = new(segments.Count);

        Node GetNode(Vertex vertex)
        {
            if (!nodeMap.TryGetValue(vertex, out Node? node))
            {
                node = new Node(vertex);
                nodeMap.Add(vertex, node);
                graph.Nodes.Add(node);
            }

            return node;
        }

        foreach (DirectedSegment segment in segments)
        {
            Node origin = GetNode(segment.Source);
            Node destination = GetNode(segment.Target);

            // Each geometric edge is represented by two half-edges so we can walk faces and reference the opposite side.
            HalfEdge edge = new(origin, destination);
            HalfEdge twin = new(destination, origin);
            edge.Twin = twin;
            twin.Twin = edge;
            edge.WindDelta = segment.WindDelta;
            twin.WindDelta = -segment.WindDelta;

            // Pre-compute polar angles so outgoing edges can be sorted counter-clockwise per node.
            edge.Angle = Math.Atan2(destination.Point.Y - origin.Point.Y, destination.Point.X - origin.Point.X);
            twin.Angle = Math.Atan2(origin.Point.Y - destination.Point.Y, origin.Point.X - destination.Point.X);

            origin.Outgoing.Add(edge);
            destination.Outgoing.Add(twin);

            graph.Edges.Add(edge);
            graph.Edges.Add(twin);
        }

        return graph;
    }

    /// <summary>
    /// Walks the half-edge graph to create face records with their boundaries.
    /// </summary>
    /// <param name="graph">The graph to traverse.</param>
    /// <returns>A list of faces with edge loops.</returns>
    private static List<Face> EnumerateFaces(HalfEdgeGraph graph)
    {
        foreach (Node node in graph.Nodes)
        {
            // Sorting outgoing edges by polar angle lets us stitch the half-edge cycle for each face.
            node.Outgoing.Sort((a, b) => a.Angle.CompareTo(b.Angle));
            for (int i = 0; i < node.Outgoing.Count; i++)
            {
                node.Outgoing[i].OutgoingIndex = i;
            }
        }

        foreach (HalfEdge edge in graph.Edges)
        {
            // For a half-edge, the next edge on its left face is the previous outgoing edge at the destination.
            Node dest = edge.Destination;
            List<HalfEdge> outgoing = dest.Outgoing;
            int nextIdx = edge.Twin!.OutgoingIndex - 1;
            if (nextIdx < 0)
            {
                nextIdx = outgoing.Count - 1;
            }

            edge.Next = outgoing[nextIdx];
        }

        List<Face> faces = new(graph.Edges.Count / 2);
        foreach (HalfEdge edge in graph.Edges)
        {
            if (edge.Face != null)
            {
                continue;
            }

            Face face = new();
            HalfEdge? current = edge;
            int guard = 0;
            while (current != null && current.Face == null && guard++ < graph.Edges.Count)
            {
                current.Face = face;
                face.Edges.Add(current);
                current = current.Next;
                if (current == edge)
                {
                    break;
                }
            }

            if (face.Edges.Count > 0)
            {
                faces.Add(face);
            }
        }

        return faces;
    }

    /// <summary>
    /// Computes winding numbers per face by sampling one face per connected component and propagating across edges.
    /// </summary>
    /// <param name="graph">The graph containing the half-edges.</param>
    /// <param name="faces">The faces produced by the half-edge traversal.</param>
    /// <param name="segments">The original segments used to seed component windings.</param>
    private static void ComputeFaceWindings(HalfEdgeGraph graph, List<Face> faces, List<DirectedSegment> segments)
    {
        // If the topology is ambiguous (multiple edges with identical angles), fall back to per-edge sampling.
        if (HasAmbiguousVertices(graph))
        {
            ComputeEdgeWindingBySampling(graph, segments);
            return;
        }

        const int unassigned = int.MinValue;
        for (int i = 0; i < faces.Count; i++)
        {
            faces[i].Winding = unassigned;
        }

        Stack<Face> stack = new();
        for (int i = 0; i < faces.Count; i++)
        {
            Face face = faces[i];
            if (face.Winding != unassigned)
            {
                continue;
            }

            // Seed each connected component by sampling a point inside one of its faces.
            Vertex sample = GetFaceSamplePoint(face);
            face.Winding = ComputeWindingNumber(sample, segments);
            stack.Push(face);

            while (stack.Count > 0)
            {
                Face current = stack.Pop();
                for (int e = 0; e < current.Edges.Count; e++)
                {
                    HalfEdge edge = current.Edges[e];
                    Face? neighbor = edge.Twin?.Face;
                    if (neighbor == null)
                    {
                        continue;
                    }

                    // left - right = windDelta, so right = left - windDelta.
                    int neighborWinding = current.Winding - edge.WindDelta;
                    if (neighbor.Winding == unassigned)
                    {
                        neighbor.Winding = neighborWinding;
                        stack.Push(neighbor);
                    }
                }
            }
        }

        for (int i = 0; i < graph.Edges.Count; i++)
        {
            HalfEdge edge = graph.Edges[i];
            edge.LeftWinding = edge.Face?.Winding ?? 0;
        }
    }

    /// <summary>
    /// Detects vertices where multiple outgoing edges share the same direction.
    /// </summary>
    /// <param name="graph">The half-edge graph to inspect.</param>
    /// <returns><see langword="true"/> when collinear ambiguity is detected.</returns>
    private static bool HasAmbiguousVertices(HalfEdgeGraph graph)
    {
        const double angleEpsilon = 1e-12;
        const double twoPi = Math.PI * 2D;

        // Outgoing edges with identical direction imply overlapping/collinear edges or non-manifold joins.
        for (int i = 0; i < graph.Nodes.Count; i++)
        {
            List<HalfEdge> outgoing = graph.Nodes[i].Outgoing;
            int count = outgoing.Count;
            if (count < 2)
            {
                continue;
            }

            for (int j = 1; j < count; j++)
            {
                if (Math.Abs(outgoing[j].Angle - outgoing[j - 1].Angle) <= angleEpsilon)
                {
                    return true;
                }
            }

            double wrapDelta = Math.Abs((outgoing[0].Angle + twoPi) - outgoing[^1].Angle);
            if (wrapDelta <= angleEpsilon)
            {
                return true;
            }
        }

        return false;
    }

    /// <summary>
    /// Computes windings by sampling a point to the left of each edge.
    /// </summary>
    /// <param name="graph">The half-edge graph.</param>
    /// <param name="segments">The directed segments used for winding tests.</param>
    private static void ComputeEdgeWindingBySampling(HalfEdgeGraph graph, List<DirectedSegment> segments)
    {
        // Sample per edge when face propagation cannot safely resolve windings.
        for (int i = 0; i < graph.Edges.Count; i++)
        {
            HalfEdge edge = graph.Edges[i];
            Vertex a = edge.Origin.Point;
            Vertex b = edge.Destination.Point;
            Vertex delta = b - a;
            double length = delta.Length();
            if (length == 0)
            {
                edge.LeftWinding = 0;
                continue;
            }

            // Sample a point slightly to the left of the edge so we can query the winding number of that region.
            double epsilon = Math.Max(1e-6, length * 1e-6);
            Vertex midpoint = (a + b) * 0.5;
            Vertex normal = new Vertex(-delta.Y, delta.X) / length;
            Vertex sample = midpoint + (normal * epsilon);
            edge.LeftWinding = ComputeWindingNumber(sample, segments);
        }
    }

    /// <summary>
    /// Finds a point that lies strictly inside the face boundary.
    /// </summary>
    /// <param name="face">The face to sample.</param>
    /// <returns>A point inside the face or a best-effort fallback.</returns>
    private static Vertex GetFaceSamplePoint(Face face)
    {
        int edgeCount = face.Edges.Count;
        if (edgeCount == 0)
        {
            return default;
        }

        // Try offsets from multiple edges to find a sample strictly inside the face boundary.
        for (int i = 0; i < edgeCount; i++)
        {
            HalfEdge edge = face.Edges[i];
            Vertex candidate = OffsetAlongLeftNormal(edge);
            if (IsPointInsideFace(candidate, face))
            {
                return candidate;
            }
        }

        return OffsetAlongLeftNormal(face.Edges[0]);
    }

    /// <summary>
    /// Offsets a midpoint slightly to the left of the given edge.
    /// </summary>
    /// <param name="edge">The edge providing the direction.</param>
    /// <returns>The offset point.</returns>
    private static Vertex OffsetAlongLeftNormal(HalfEdge edge)
    {
        // Move slightly to the left of the edge so we query the face on its left side.
        Vertex a = edge.Origin.Point;
        Vertex b = edge.Destination.Point;
        Vertex delta = b - a;
        double length = delta.Length();
        if (length == 0)
        {
            return a;
        }

        double epsilon = Math.Max(1e-6, length * 1e-6);
        Vertex midpoint = (a + b) * 0.5;
        Vertex normal = new Vertex(-delta.Y, delta.X) / length;
        return midpoint + (normal * epsilon);
    }

    /// <summary>
    /// Tests whether a point lies strictly inside the face boundary.
    /// </summary>
    /// <param name="point">The point to test.</param>
    /// <param name="face">The face to test.</param>
    /// <returns><see langword="true"/> if the point is inside the face.</returns>
    private static bool IsPointInsideFace(in Vertex point, Face face)
    {
        int count = face.Edges.Count;
        if (count < 3)
        {
            return false;
        }

        // Standard ray-casting, but treat boundary points as outside to avoid ambiguous samples.
        bool inside = false;
        Vertex previous = face.Edges[^1].Origin.Point;

        for (int i = 0; i < count; i++)
        {
            Vertex current = face.Edges[i].Origin.Point;
            if (current == previous)
            {
                previous = current;
                continue;
            }

            if (IsPointOnSegment(point, previous, current))
            {
                return false;
            }

            bool intersects = (current.Y > point.Y) != (previous.Y > point.Y);
            if (intersects)
            {
                double xIntersection = ((previous.X - current.X) * (point.Y - current.Y) / (previous.Y - current.Y)) + current.X;
                if (point.X < xIntersection)
                {
                    inside = !inside;
                }
            }

            previous = current;
        }

        return inside;
    }

    /// <summary>
    /// Extracts contours that separate filled and empty regions using the computed windings.
    /// </summary>
    /// <param name="graph">The graph to trace.</param>
    /// <returns>A list of boundary contours following the positive fill rule.</returns>
    private static List<Contour> ExtractBoundaryContours(HalfEdgeGraph graph)
    {
        List<Contour> contours = new(graph.Edges.Count / 2);

        foreach (HalfEdge edge in graph.Edges)
        {
            if (!IsBoundaryEdge(edge) || edge.Visited)
            {
                continue;
            }

            Contour contour = [];
            HalfEdge? current = edge;
            int guard = 0;
            while (current != null && guard++ < 100000)
            {
                // Track which edges have already been walked so we do not duplicate contours.
                current.Visited = true;
                Vertex point = current.Origin.Point;
                if (contour.Count == 0 || !ArePointsClose(contour[^1], point))
                {
                    contour.Add(point);
                }

                // Follow the boundary by taking the smallest CCW turn at each vertex.
                current = NextBoundaryEdge(current);
                if (current != null && current != edge && current.Visited)
                {
                    break;
                }

                if (current == null || current == edge)
                {
                    break;
                }
            }

            if (contour.Count > 0 && contour[0] != contour[^1])
            {
                contour.Add(contour[0]);
            }

            contours.Add(contour);
        }

        return contours;
    }

    // Merge boundary cycles that meet at a single vertex so the union is represented as one contour.
    private static List<Contour> MergeTouchingContours(List<Contour> contours)
    {
        if (contours.Count < 2)
        {
            return contours;
        }

        bool merged = true;
        while (merged)
        {
            merged = false;
            for (int i = 0; i < contours.Count && !merged; i++)
            {
                for (int j = i + 1; j < contours.Count && !merged; j++)
                {
                    // Merge the first pair of contours that share a vertex, then restart the scan.
                    if (!TryFindSharedVertex(contours[i], contours[j], out int leftIndex, out int rightIndex, out Vertex shared))
                    {
                        continue;
                    }

                    Contour mergedContour = MergeContoursAtSharedVertex(contours[i], leftIndex, contours[j], rightIndex, shared);
                    contours[i] = mergedContour;
                    contours.RemoveAt(j);
                    merged = true;
                }
            }
        }

        return contours;
    }

    private static bool TryFindSharedVertex(
        Contour left,
        Contour right,
        out int leftIndex,
        out int rightIndex,
        out Vertex shared)
    {
        leftIndex = -1;
        rightIndex = -1;
        shared = default;

        // Ignore duplicated closing vertices when searching for a shared point.
        int leftCount = GetContourPointCount(left);
        int rightCount = GetContourPointCount(right);

        for (int i = 0; i < leftCount; i++)
        {
            Vertex leftPoint = left[i];
            for (int j = 0; j < rightCount; j++)
            {
                // Use a tight epsilon so numerically close endpoints are treated as the same vertex.
                if (!ArePointsClose(leftPoint, right[j]))
                {
                    continue;
                }

                leftIndex = i;
                rightIndex = j;
                shared = leftPoint;
                return true;
            }
        }

        return false;
    }

    private static int GetContourPointCount(Contour contour)
    {
        int count = contour.Count;

        // Contours may be explicitly closed; drop the duplicate terminal vertex when present.
        if (count > 1 && contour[0] == contour[^1])
        {
            return count - 1;
        }

        return count;
    }

    private static List<Vertex> BuildRotatedVertices(Contour contour, int startIndex)
    {
        int count = GetContourPointCount(contour);
        List<Vertex> rotated = new(count);

        // Rotate so the shared vertex is at index 0, preserving the original winding order.
        for (int i = 0; i < count; i++)
        {
            rotated.Add(contour[(startIndex + i) % count]);
        }

        return rotated;
    }

    private static Contour MergeContoursAtSharedVertex(
        Contour left,
        int leftIndex,
        Contour right,
        int rightIndex,
        Vertex shared)
    {
        List<Vertex> leftVerts = BuildRotatedVertices(left, leftIndex);
        List<Vertex> rightVerts = BuildRotatedVertices(right, rightIndex);

        // Stitch the two boundary cycles at the shared vertex, keeping explicit closure.
        Contour merged = new(leftVerts.Count + rightVerts.Count + 1)
        {
            shared
        };
        for (int i = 1; i < leftVerts.Count; i++)
        {
            merged.Add(leftVerts[i]);
        }

        merged.Add(shared);
        for (int i = 1; i < rightVerts.Count; i++)
        {
            merged.Add(rightVerts[i]);
        }

        merged.Add(shared);
        return merged;
    }

    /// <summary>
    /// Computes parent/child relationships and hole lists for the resulting polygon.
    /// </summary>
    /// <param name="polygon">The polygon whose contours will be linked.</param>
    private static void AssignHierarchy(Polygon polygon)
    {
        int count = polygon.Count;
        if (count == 0)
        {
            return;
        }

        int[] parentIndices = new int[count];
        Array.Fill(parentIndices, -1);

        for (int i = 0; i < count; i++)
        {
            polygon[i].ParentIndex = null;
            polygon[i].Depth = 0;
            polygon[i].ClearHoles();
        }

        for (int i = 0; i < count; i++)
        {
            Vertex testPoint = GetContourTestPoint(polygon[i]);
            double smallestBounds = double.PositiveInfinity;
            int parentIndex = -1;

            for (int j = 0; j < count; j++)
            {
                if (i == j)
                {
                    continue;
                }

                Contour candidate = polygon[j];
                if (PointInContour(testPoint, candidate))
                {
                    // Prefer the tightest bounding box to reduce the amount of hierarchy re-parenting later.
                    Box2 bounds = candidate.GetBoundingBox();
                    double width = bounds.Max.X - bounds.Min.X;
                    double height = bounds.Max.Y - bounds.Min.Y;
                    double area = width * height;
                    if (area < smallestBounds)
                    {
                        smallestBounds = area;
                        parentIndex = j;
                    }
                }
            }

            parentIndices[i] = parentIndex;
            if (parentIndex >= 0)
            {
                polygon[i].ParentIndex = parentIndex;
            }
        }

        for (int i = 0; i < count; i++)
        {
            int depth = 0;
            int current = parentIndices[i];
            while (current >= 0)
            {
                depth++;
                current = parentIndices[current];
            }

            polygon[i].Depth = depth;
        }

        for (int i = 0; i < count; i++)
        {
            int parentIndex = parentIndices[i];
            if (parentIndex >= 0)
            {
                polygon[parentIndex].AddHoleIndex(i);
            }
        }
    }

    /// <summary>
    /// Determines whether a half-edge borders a filled region on the left and empty space on the right.
    /// </summary>
    /// <param name="edge">The edge to test.</param>
    /// <returns><see langword="true"/> when the edge contributes to the final boundary.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsBoundaryEdge(HalfEdge edge)
    {
        bool leftInside = edge.LeftWinding > 0;
        bool rightInside = edge.Twin != null && edge.Twin.LeftWinding > 0;
        return leftInside && !rightInside;
    }

    /// <summary>
    /// Advances to the next boundary edge, wrapping when the walk returns to the starting edge.
    /// </summary>
    /// <param name="edge">The current boundary edge.</param>
    /// <returns>The next boundary edge or <see langword="null"/> if traversal fails.</returns>
    private static HalfEdge? NextBoundaryEdge(HalfEdge edge)
    {
        List<HalfEdge> outgoing = edge.Destination.Outgoing;
        if (outgoing.Count == 0)
        {
            return null;
        }

        // Prefer the smallest positive delta in angle to keep a consistent boundary walk.
        double inAngle = edge.Angle;
        double bestDelta = double.PositiveInfinity;
        HalfEdge? best = null;

        for (int i = 0; i < outgoing.Count; i++)
        {
            HalfEdge candidate = outgoing[i];
            if (!IsBoundaryEdge(candidate))
            {
                continue;
            }

            double delta = candidate.Angle - inAngle;
            if (delta < 0)
            {
                delta += Math.PI * 2;
            }

            if (delta < bestDelta)
            {
                bestDelta = delta;
                best = candidate;
            }
        }

        return best;
    }

    /// <summary>
    /// Computes the winding number of the arrangement at the specified point.
    /// </summary>
    /// <param name="point">The sample point.</param>
    /// <param name="segments">The directed segments defining the arrangement.</param>
    /// <returns>The winding number relative to the positive fill rule.</returns>
    private static int ComputeWindingNumber(Vertex point, List<DirectedSegment> segments)
    {
        int winding = 0;
        for (int i = 0; i < segments.Count; i++)
        {
            Vertex a = segments[i].Source;
            Vertex b = segments[i].Target;

            // Standard winding-number test using upward/downward crossings of a horizontal ray.
            if (a.Y <= point.Y)
            {
                if (b.Y > point.Y && IsLeft(a, b, point) > 0)
                {
                    winding++;
                }
            }
            else
            {
                if (b.Y <= point.Y && IsLeft(a, b, point) < 0)
                {
                    winding--;
                }
            }
        }

        return winding;
    }

    /// <summary>
    /// Returns the signed area of the triangle formed by (a, b, p); positive values place p to the left of segment ab.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static double IsLeft(Vertex a, Vertex b, Vertex p)
        => ((b.X - a.X) * (p.Y - a.Y)) - ((p.X - a.X) * (b.Y - a.Y));

    /// <summary>
    /// Removes degenerate vertices, collinear runs, and duplicate endpoints from a contour.
    /// </summary>
    /// <param name="contour">The contour to clean in-place.</param>
    private static void CleanupContour(Contour contour)
    {
        if (contour.Count == 0)
        {
            return;
        }

        // Move through a linked structure so removals are O(1) without reallocating the contour.
        // Copy to a mutable list so we can build a circular linked structure without touching the contour directly.
        // The contour indexer is read-only, so we never mutate it via a setter; all changes happen through this temporary list
        // and the ContourNode helpers before the contour is cleared and repopulated.
        List<Vertex> vertices = [.. contour];
        if (vertices.Count > 1 && vertices[0] == vertices[^1])
        {
            vertices.RemoveAt(vertices.Count - 1);
        }

        if (!TryBuildContourNodes(vertices, out ContourNode? start))
        {
            contour.Clear();
            return;
        }

        if (!IsValidClosedPath(start))
        {
            contour.Clear();
            return;
        }

        // Remove collinear or duplicate runs, then rebuild a simplified vertex list.
        if (!TryCleanCollinear(start, preserveCollinear: false, out start) || !IsValidClosedPath(start))
        {
            contour.Clear();
            return;
        }

        List<Vertex> cleaned = BuildPath(start);
        cleaned = TrimCollinear(cleaned);
        if (cleaned.Count < 3 || (cleaned.Count == 3 && IsVerySmallTriangle(cleaned)))
        {
            contour.Clear();
            return;
        }

        if (cleaned[0] != cleaned[^1])
        {
            cleaned.Add(cleaned[0]);
        }

        // Replace the original contour with the cleaned vertex list.
        contour.Clear();
        for (int i = 0; i < cleaned.Count; i++)
        {
            contour.Add(cleaned[i]);
        }
    }

    /// <summary>
    /// Creates a circular doubly linked list from the supplied vertices.
    /// </summary>
    /// <param name="vertices">The vertex list to wrap.</param>
    /// <param name="node">The resulting head node.</param>
    /// <returns><see langword="true"/> when the list contains enough vertices.</returns>
    private static bool TryBuildContourNodes(List<Vertex> vertices, [NotNullWhen(true)] out ContourNode? node)
    {
        if (vertices.Count < 3)
        {
            node = null;
            return false;
        }

        // Build a circular doubly linked list for efficient removals during cleanup.
        ContourNode first = new(vertices[0]);
        ContourNode prev = first;
        for (int i = 1; i < vertices.Count; i++)
        {
            ContourNode n = new(vertices[i])
            {
                Prev = prev
            };
            prev.Next = n;
            prev = n;
        }

        prev.Next = first;
        first.Prev = prev;
        node = first;
        return true;
    }

    /// <summary>
    /// Checks whether the linked contour contains at least three distinct nodes.
    /// </summary>
    /// <param name="node">Any node within the contour.</param>
    /// <returns><see langword="true"/> when the walk can continue.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsValidClosedPath(ContourNode? node)
        => node != null && node.Next != node && (node.Next != node.Prev || !IsVerySmallTriangle(node));

    /// <summary>
    /// Determines whether three consecutive nodes form a degenerate triangle.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsVerySmallTriangle(ContourNode node)
        => node.Next.Next == node.Prev &&
            (PtsReallyClose(node.Prev.Point, node.Next.Point) ||
             PtsReallyClose(node.Point, node.Next.Point) ||
             PtsReallyClose(node.Point, node.Prev.Point));

    /// <summary>
    /// Tests whether a vertex list represents a triangle with clustered points.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsVerySmallTriangle(List<Vertex> vertices)
        => vertices.Count == 3 &&
            (PtsReallyClose(vertices[0], vertices[1]) ||
             PtsReallyClose(vertices[1], vertices[2]) ||
             PtsReallyClose(vertices[2], vertices[0]));

    /// <summary>
    /// Determines whether two vertices are nearly coincident in screen space terms.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool PtsReallyClose(Vertex a, Vertex b)
        => Math.Abs(a.X - b.X) < 2 && Math.Abs(a.Y - b.Y) < 2;

    /// <summary>
    /// Removes a node from the circular list and returns the next node (or null if exhausted).
    /// </summary>
    private static ContourNode? DisposeNode(ContourNode node)
    {
        ContourNode? result = node.Next == node ? null : node.Next;
        node.Prev.Next = node.Next;
        node.Next.Prev = node.Prev;
        return result;
    }

    /// <summary>
    /// Removes nodes that are collinear (or almost) while optionally preserving explicitly collinear edges.
    /// </summary>
    /// <param name="start">The starting node for traversal.</param>
    /// <param name="preserveCollinear">Whether perfectly collinear nodes should be retained.</param>
    /// <param name="result">The new starting node when successful.</param>
    /// <returns><see langword="true"/> when a valid closed path remains.</returns>
    private static bool TryCleanCollinear(ContourNode start, bool preserveCollinear, [NotNullWhen(true)] out ContourNode? result)
    {
        ContourNode? op2 = start;
        ContourNode? currentStart = start;

        while (true)
        {
            if (IsCollinear(op2!.Prev.Point, op2.Point, op2.Next.Point) &&
                (ArePointsClose(op2.Point, op2.Prev.Point) || ArePointsClose(op2.Point, op2.Next.Point) || !preserveCollinear ||
                 (DotProduct(op2.Prev.Point, op2.Point, op2.Next.Point) < 0)))
            {
                bool removedStart = op2 == currentStart;
                op2 = DisposeNode(op2);
                if (!IsValidClosedPath(op2))
                {
                    result = null;
                    return false;
                }

                if (removedStart)
                {
                    currentStart = op2;
                }

                continue;
            }

            op2 = op2.Next;
            if (op2 == currentStart)
            {
                break;
            }
        }

        result = currentStart;
        return true;
    }

    /// <summary>
    /// Converts the linked contour back into a list of vertices while removing duplicates.
    /// </summary>
    /// <param name="start">The node from which to begin traversal.</param>
    /// <returns>The ordered vertex path.</returns>
    private static List<Vertex> BuildPath(ContourNode start)
    {
        List<Vertex> path = [];
        Vertex last = start.Point;
        path.Add(last);
        ContourNode node = start.Next;
        while (node != start)
        {
            if (!ArePointsClose(node.Point, last))
            {
                last = node.Point;
                path.Add(last);
            }

            node = node.Next;
        }

        return path;
    }

    /// <summary>
    /// Trims remaining collinear vertices from an open path while keeping the polygon valid.
    /// </summary>
    /// <param name="path">The vertex sequence to trim.</param>
    /// <returns>A simplified path or an empty list when it degenerates.</returns>
    private static List<Vertex> TrimCollinear(List<Vertex> path)
    {
        int len = path.Count;
        if (len < 3)
        {
            return path;
        }

        int i = 0;
        while (i < len - 1 && IsCollinear(path[len - 1], path[i], path[i + 1]))
        {
            i++;
        }

        while (i < len - 1 && IsCollinear(path[len - 2], path[len - 1], path[i]))
        {
            len--;
        }

        if (len - i < 3)
        {
            return [];
        }

        List<Vertex> result = new(len - i);
        Vertex last = path[i];
        result.Add(last);

        for (i++; i < len - 1; i++)
        {
            if (IsCollinear(last, path[i], path[i + 1]))
            {
                continue;
            }

            last = path[i];
            result.Add(last);
        }

        if (!IsCollinear(last, path[len - 1], result[0]))
        {
            result.Add(path[len - 1]);
        }
        else
        {
            while (result.Count > 2 && IsCollinear(result[^1], result[^2], result[0]))
            {
                result.RemoveAt(result.Count - 1);
            }

            if (result.Count < 3)
            {
                result.Clear();
            }
        }

        return result;
    }

    /// <summary>
    /// Tests whether three vertices lie on the same line within a small tolerance.
    /// </summary>
    private static bool IsCollinear(Vertex a, Vertex b, Vertex c)
    {
        Vertex ac = c - a;
        double lenSq = ac.LengthSquared();
        if (lenSq <= double.Epsilon)
        {
            return true;
        }

        Vertex ab = b - a;
        double cross = Math.Abs(Vertex.Cross(ab, ac));
        double distance = cross / Math.Sqrt(lenSq);
        return distance <= 1e-3;
    }

    /// <summary>
    /// Computes the dot product of two segments sharing the middle vertex.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static double DotProduct(Vertex pt1, Vertex pt2, Vertex pt3)
        => Vertex.Dot(pt2 - pt1, pt3 - pt2);

    /// <summary>
    /// Checks whether two vertices are almost coincident using a tight epsilon.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool ArePointsClose(Vertex a, Vertex b)
        => Math.Abs(a.X - b.X) <= 1e-5 && Math.Abs(a.Y - b.Y) <= 1e-5;

    /// <summary>
    /// Adds a split point to a segment's split list, ignoring near-duplicates.
    /// </summary>
    private static void AddSplitPoint(ref SegmentSplit segmentSplit, Vertex point)
    {
        if (segmentSplit.Splits == null)
        {
            if (ArePointsClose(segmentSplit.Source, point) || ArePointsClose(segmentSplit.Target, point))
            {
                return;
            }

            segmentSplit.Splits = new List<Vertex>(4)
            {
                segmentSplit.Source,
                segmentSplit.Target,
                point
            };
            return;
        }

        List<Vertex> splits = segmentSplit.Splits;
        for (int i = 0; i < splits.Count; i++)
        {
            if (ArePointsClose(splits[i], point))
            {
                return;
            }
        }

        splits.Add(point);
    }

    /// <summary>
    /// Determines the possible intersection of two sweep line segments and handles splitting.
    /// </summary>
    /// <param name="le1">The first sweep event.</param>
    /// <param name="le2">The second sweep event.</param>
    /// <param name="eventQueue">The event queue to add new events to.</param>
    /// <param name="workspace">Scratch space for temporary storage.</param>
    /// <param name="comparer">The sweep event comparer.</param>
    /// <param name="segmentSplits">The split-point lists keyed by original segment ID.</param>
    /// <returns>
    /// 0 if no intersection, 1 if single point intersection, 2 if overlapping with shared left endpoint, 3 otherwise.
    /// </returns>
    private static int PossibleIntersection(
        SweepEvent le1,
        SweepEvent le2,
        StablePriorityQueue<SweepEvent, SweepEventComparer> eventQueue,
        Span<SweepEvent> workspace,
        SweepEventComparer comparer,
        Span<SegmentSplit> segmentSplits)
    {
        if (le1.OtherEvent == null || le2.OtherEvent == null)
        {
            return 0;
        }

        int nIntersections = PolygonUtilities.FindIntersection(
            le1.GetSegment(),
            le2.GetSegment(),
            out Vertex ip1,
            out Vertex ip2);

        if (nIntersections == 0)
        {
            return 0;
        }

        // Record split points for both segments so we can rebuild all sub-segments after the sweep.
        if (nIntersections >= 1)
        {
            if (le1.SegmentId >= 0 && le1.SegmentId < segmentSplits.Length)
            {
                AddSplitPoint(ref segmentSplits[le1.SegmentId], ip1);
                if (nIntersections == 2)
                {
                    AddSplitPoint(ref segmentSplits[le1.SegmentId], ip2);
                }
            }

            if (le2.SegmentId >= 0 && le2.SegmentId < segmentSplits.Length)
            {
                AddSplitPoint(ref segmentSplits[le2.SegmentId], ip1);
                if (nIntersections == 2)
                {
                    AddSplitPoint(ref segmentSplits[le2.SegmentId], ip2);
                }
            }
        }

        // Ignore intersection if it occurs at the exact left or right endpoint of both segments
        if (nIntersections == 1 &&
            (le1.Point == le2.Point || le1.OtherEvent.Point == le2.OtherEvent.Point))
        {
            return 0;
        }

        // Handle overlapping segments
        if (nIntersections == 2)
        {
            // Mark overlapping segments appropriately
            bool leftCoincide = le1.Point == le2.Point;
            bool rightCoincide = le1.OtherEvent.Point == le2.OtherEvent.Point;

            ref SweepEvent wRef = ref MemoryMarshal.GetReference(workspace);
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
            else if (leftCoincide && !rightCoincide)
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
                // One segment is marked as non-contributing (overlapping)
                le2.EdgeType = EdgeType.NonContributing;
                le1.EdgeType = (le2.InOut == le1.InOut)
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

        // Single intersection point
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

    /// <summary>
    /// Divides a segment at the specified point, creating two new segments.
    /// </summary>
    private static void DivideSegment(
        SweepEvent le,
        Vertex p,
        StablePriorityQueue<SweepEvent, SweepEventComparer> eventQueue,
        SweepEventComparer comparer)
    {
        if (le.OtherEvent == null)
        {
            return;
        }

        SweepEvent re = le.OtherEvent;

        // Handle corner case: vertical segment at same X coordinate
        if (p.X == le.Point.X && p.Y < le.Point.Y)
        {
            p = new Vertex(p.X.NextAfter(double.PositiveInfinity), p.Y);
        }

        // Create the right event for the left segment (new right endpoint)
        SweepEvent r = new(p, false, le, le.PolygonType);

        // Create the left event for the right segment (new left endpoint)
        SweepEvent l = new(p, true, re, le.PolygonType);

        // Assign the same contour ID to maintain connectivity
        r.ContourId = l.ContourId = le.ContourId;
        r.SegmentId = l.SegmentId = le.SegmentId;

        // Preserve contour direction: split the segment while keeping original direction.
        Vertex source = le.SegmentSource;
        Vertex target = le.SegmentTarget;

        le.IsContourSource = le.Point == source;
        re.IsContourSource = re.Point == source;
        r.IsContourSource = r.Point == source;
        l.IsContourSource = l.Point == source;

        le.SegmentSource = source;
        le.SegmentTarget = p;
        r.SegmentSource = source;
        r.SegmentTarget = p;

        l.SegmentSource = p;
        l.SegmentTarget = target;
        re.SegmentSource = p;
        re.SegmentTarget = target;

        // Propagate the stable WindDx to split segments so they inherit the
        // same winding direction as the original segment.
        r.WindDx = l.WindDx = le.WindDx;

        // Corner case: swap if ordering is wrong
        if (comparer.Compare(l, re) > 0)
        {
            re.Left = true;
            l.Left = false;
        }

        // Update references to maintain correct linkage
        re.OtherEvent = l;
        le.OtherEvent = r;

        // Add the new events to the event queue
        eventQueue.Enqueue(l);
        eventQueue.Enqueue(r);
    }

    /// <summary>
    /// Represents a directed edge between two vertices in the arrangement.
    /// </summary>
    private readonly record struct DirectedSegment(Vertex Source, Vertex Target, int WindDelta);

    /// <summary>
    /// Holds split points for a segment, allocating only when intersections occur.
    /// </summary>
    private struct SegmentSplit
    {
        /// <summary>
        /// Initializes a new instance of the <see cref="SegmentSplit"/> struct.
        /// </summary>
        public SegmentSplit(Vertex source, Vertex target, int windDelta)
        {
            this.Source = source;
            this.Target = target;
            this.WindDelta = windDelta;
            this.Splits = null;
        }

        /// <summary>Gets the segment source.</summary>
        public Vertex Source { get; }

        /// <summary>Gets the segment target.</summary>
        public Vertex Target { get; }

        /// <summary>Gets the winding delta for the segment.</summary>
        public int WindDelta { get; }

        /// <summary>Gets or sets the split point list when intersections occur.</summary>
        public List<Vertex>? Splits { get; set; }
    }

    /// <summary>
    /// Represents a graph vertex with a collection of outgoing half-edges.
    /// </summary>
    private sealed class Node
    {
        /// <summary>
        /// Initializes a new instance of the <see cref="Node"/> class.
        /// </summary>
        public Node(Vertex point) => this.Point = point;

        /// <summary>Gets the coordinate stored at this node.</summary>
        public Vertex Point { get; }

        /// <summary>Gets the outgoing half-edges emanating from this node.</summary>
        public List<HalfEdge> Outgoing { get; } = [];
    }

    /// <summary>
    /// Represents a directed edge in the arrangement, paired with a twin in opposite direction.
    /// </summary>
    private sealed class HalfEdge
    {
        /// <summary>
        /// Initializes a new instance of the <see cref="HalfEdge"/> class.
        /// </summary>
        public HalfEdge(Node origin, Node destination)
        {
            this.Origin = origin;
            this.Destination = destination;
        }

        /// <summary>Gets the node the half-edge originates from.</summary>
        public Node Origin { get; }

        /// <summary>Gets the node the half-edge points to.</summary>
        public Node Destination { get; }

        /// <summary>Gets or sets the opposite half-edge with reversed direction.</summary>
        public HalfEdge? Twin { get; set; }

        /// <summary>Gets or sets the next half-edge when walking a face counter-clockwise.</summary>
        public HalfEdge? Next { get; set; }

        /// <summary>Gets or sets the face to the left of this half-edge.</summary>
        public Face? Face { get; set; }

        /// <summary>Gets or sets the cached polar angle used for sorting outgoing edges.</summary>
        public double Angle { get; set; }

        /// <summary>Gets or sets the winding value of the region on the left-hand side.</summary>
        public int LeftWinding { get; set; }

        /// <summary>Gets or sets the winding delta across this edge (left minus right).</summary>
        public int WindDelta { get; set; }

        /// <summary>Gets or sets the index of this half-edge in its origin node's outgoing list.</summary>
        public int OutgoingIndex { get; set; }

        /// <summary>Gets or sets a value indicating whether this half-edge has been walked during contour extraction.</summary>
        public bool Visited { get; set; }
    }

    /// <summary>
    /// Represents a face boundary and metadata discovered during graph traversal.
    /// </summary>
    private sealed class Face
    {
        /// <summary>Gets the half-edges encountered when walking this face.</summary>
        public List<HalfEdge> Edges { get; } = [];

        /// <summary>Gets or sets a value indicating whether this face represents exterior space.</summary>
        public bool IsExterior { get; set; }

        /// <summary>Gets or sets the winding number for the region represented by this face.</summary>
        public int Winding { get; set; }
    }

    /// <summary>
    /// Container for the half-edge graph used during boundary extraction.
    /// </summary>
    private sealed class HalfEdgeGraph
    {
        /// <summary>
        /// Initializes a new instance of the <see cref="HalfEdgeGraph"/> class with pre-sized collections.
        /// </summary>
        public HalfEdgeGraph(int nodeCapacity, int edgeCapacity)
        {
            this.Nodes = new List<Node>(nodeCapacity);
            this.Edges = new List<HalfEdge>(edgeCapacity);
        }

        /// <summary>Gets the collection of graph nodes.</summary>
        public List<Node> Nodes { get; }

        /// <summary>Gets the collection of half-edges (each physical edge contributes two entries).</summary>
        public List<HalfEdge> Edges { get; }
    }

    /// <summary>
    /// Linked-list node used to clean and simplify contour vertex lists.
    /// </summary>
    private sealed class ContourNode
    {
        /// <summary>
        /// Initializes a new instance of the <see cref="ContourNode"/> class.
        /// </summary>
        public ContourNode(Vertex point) => this.Point = point;

        /// <summary>Gets or sets the vertex stored at this node.</summary>
        public Vertex Point { get; set; }

        /// <summary>Gets or sets the previous node in the circular list.</summary>
        public ContourNode Prev { get; set; } = null!;

        /// <summary>Gets or sets the next node in the circular list.</summary>
        public ContourNode Next { get; set; } = null!;
    }
}

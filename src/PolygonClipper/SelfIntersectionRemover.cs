// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System;
using System.Collections.Generic;
using System.Linq;
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

        List<DirectedSegment> segments = BuildArrangementSegments(polygon);
        if (segments.Count == 0)
        {
            return [];
        }

        HalfEdgeGraph graph = BuildHalfEdgeGraph(segments);
        _ = EnumerateFaces(graph);
        ComputeEdgeWinding(graph, segments);
        List<Contour> contours = ExtractBoundaryContours(graph);

        Polygon result = [];
        foreach (Contour contour in contours)
        {
            CleanupContour(contour);
            if (contour.Count > 2)
            {
                result.Add(contour);
            }
        }

        return PolygonUtilities.BuildNormalizedPolygon(result);
    }

    private readonly record struct DirectedSegment(Vertex Source, Vertex Target);

    private sealed class Node
    {
        public Node(Vertex point)
        {
            this.Point = point;
        }

        public Vertex Point { get; }

        public List<HalfEdge> Outgoing { get; } = new();
    }

    private sealed class HalfEdge
    {
        public HalfEdge(Node origin, Node destination)
        {
            this.Origin = origin;
            this.Destination = destination;
        }

        public Node Origin { get; }

        public Node Destination { get; }

        public HalfEdge? Twin { get; set; }

        public HalfEdge? Next { get; set; }

        public Face? Face { get; set; }

        public double Angle { get; set; }

        public int LeftWinding { get; set; }
    }

    private sealed class Face
    {
        public List<Vertex> Boundary { get; } = new();

        public List<HalfEdge> Edges { get; } = new();

        public bool IsExterior { get; set; }

        public int Winding { get; set; }
    }

    private sealed class HalfEdgeGraph
    {
        public List<Node> Nodes { get; } = new();

        public List<HalfEdge> Edges { get; } = new();
    }

    private static List<DirectedSegment> BuildArrangementSegments(Polygon polygon)
    {
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
                    continue;
                }

                SweepEvent e1 = new(segment.Source, true, PolygonType.Subject);
                SweepEvent e2 = new(segment.Target, true, e1, PolygonType.Subject);
                e1.OtherEvent = e2;
                e1.ContourId = e2.ContourId = contourId;
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

                int windDx = e1.Left ? 1 : -1;
                e1.WindDx = e2.WindDx = windDx;

                unorderedEvents.Add(e1);
                unorderedEvents.Add(e2);
            }
        }

        if (unorderedEvents.Count == 0)
        {
            return [];
        }

        StablePriorityQueue<SweepEvent, SweepEventComparer> eventQueue = new(comparer, unorderedEvents);
        List<SweepEvent> sortedEvents = new(unorderedEvents.Count);
        StatusLine statusLine = new(polygon.VertexCount >> 1);

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
            sortedEvents.Add(sweepEvent);

            if (sweepEvent.Left)
            {
                int position = sweepEvent.PosSL = statusLine.Add(sweepEvent);
                SweepEvent? prevEvent = statusLine.Prev(position);
                SweepEvent? nextEvent = statusLine.Next(position);

                if (nextEvent != null)
                {
                    _ = PossibleIntersection(sweepEvent, nextEvent, eventQueue, workspace, comparer);
                }

                if (prevEvent != null)
                {
                    _ = PossibleIntersection(prevEvent, sweepEvent, eventQueue, workspace, comparer);
                }
            }
            else
            {
                SweepEvent? leftEvent = sweepEvent.OtherEvent;
                if (leftEvent != null && leftEvent.PosSL >= 0 && leftEvent.PosSL < statusLine.Count)
                {
                    int position = leftEvent.PosSL;
                    SweepEvent? prevEvent = statusLine.Prev(position);
                    SweepEvent? nextEvent = statusLine.Next(position);

                    if (prevEvent != null && nextEvent != null)
                    {
                        _ = PossibleIntersection(prevEvent, nextEvent, eventQueue, workspace, comparer);
                    }

                    statusLine.RemoveAt(position);
                }
            }
        }

        List<DirectedSegment> segments = new(sortedEvents.Count);
        for (int i = 0; i < sortedEvents.Count; i++)
        {
            SweepEvent sweepEvent = sortedEvents[i];
            if (!sweepEvent.Left || sweepEvent.OtherEvent == null)
            {
                continue;
            }

            Vertex source = sweepEvent.SegmentSource;
            Vertex target = sweepEvent.SegmentTarget;
            if (source != target)
            {
                segments.Add(new DirectedSegment(source, target));
            }
        }

        return segments;
    }

    private static HalfEdgeGraph BuildHalfEdgeGraph(List<DirectedSegment> segments)
    {
        HalfEdgeGraph graph = new();
        Dictionary<Vertex, Node> nodeMap = new();

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

            HalfEdge edge = new(origin, destination);
            HalfEdge twin = new(destination, origin);
            edge.Twin = twin;
            twin.Twin = edge;

            edge.Angle = Math.Atan2(destination.Point.Y - origin.Point.Y, destination.Point.X - origin.Point.X);
            twin.Angle = Math.Atan2(origin.Point.Y - destination.Point.Y, origin.Point.X - destination.Point.X);

            origin.Outgoing.Add(edge);
            destination.Outgoing.Add(twin);

            graph.Edges.Add(edge);
            graph.Edges.Add(twin);
        }

        return graph;
    }

    private static List<Face> EnumerateFaces(HalfEdgeGraph graph)
    {
        foreach (Node node in graph.Nodes)
        {
            node.Outgoing.Sort((a, b) => a.Angle.CompareTo(b.Angle));
        }

        foreach (HalfEdge edge in graph.Edges)
        {
            Node dest = edge.Destination;
            List<HalfEdge> outgoing = dest.Outgoing;
            int idx = outgoing.IndexOf(edge.Twin!);
            if (idx < 0)
            {
                continue;
            }

            int nextIdx = idx - 1;
            if (nextIdx < 0)
            {
                nextIdx = outgoing.Count - 1;
            }

            edge.Next = outgoing[nextIdx];
        }

        List<Face> faces = new();
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
                face.Boundary.Add(current.Origin.Point);
                face.Edges.Add(current);
                current = current.Next;
                if (current == edge)
                {
                    break;
                }
            }

            if (face.Boundary.Count > 0)
            {
                faces.Add(face);
            }
        }

        return faces;
    }

    private static void ComputeEdgeWinding(HalfEdgeGraph graph, List<DirectedSegment> segments)
    {
        foreach (HalfEdge edge in graph.Edges)
        {
            Vertex a = edge.Origin.Point;
            Vertex b = edge.Destination.Point;
            double midX = (a.X + b.X) * 0.5;
            double midY = (a.Y + b.Y) * 0.5;
            double dx = b.X - a.X;
            double dy = b.Y - a.Y;
            double length = Math.Sqrt((dx * dx) + (dy * dy));
            if (length == 0)
            {
                edge.LeftWinding = 0;
                continue;
            }

            double nx = -dy / length;
            double ny = dx / length;
            double epsilon = Math.Max(1e-6, length * 1e-6);
            Vertex sample = new(midX + (nx * epsilon), midY + (ny * epsilon));
            edge.LeftWinding = ComputeWindingNumber(sample, segments);
        }
    }

    private static List<Contour> ExtractBoundaryContours(HalfEdgeGraph graph)
    {
        List<Contour> contours = new();
        HashSet<HalfEdge> visited = new();

        foreach (HalfEdge edge in graph.Edges)
        {
            if (!IsBoundaryEdge(edge) || visited.Contains(edge))
            {
                continue;
            }

            Contour contour = [];
            HalfEdge? current = edge;
            int guard = 0;
            while (current != null && guard++ < 100000)
            {
                visited.Add(current);
                contour.Add(current.Origin.Point);

                current = NextBoundaryEdge(current, edge);
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

    private static bool IsBoundaryEdge(HalfEdge edge)
    {
        bool leftInside = (edge.LeftWinding & 1) != 0;
        bool rightInside = edge.Twin != null && (edge.Twin.LeftWinding & 1) != 0;
        return leftInside && !rightInside;
    }

    private static HalfEdge? NextBoundaryEdge(HalfEdge edge, HalfEdge start)
    {
        HalfEdge? current = edge.Next;
        int guard = 0;
        while (current != null && guard++ < 100000)
        {
            if (IsBoundaryEdge(current))
            {
                return current;
            }

            current = current.Next;
            if (current == start)
            {
                return current;
            }
        }

        return null;
    }

    private static Vertex ComputeFaceSample(Face face)
    {
        HalfEdge edge = face.Edges[0];
        Vertex a = edge.Origin.Point;
        Vertex b = edge.Destination.Point;
        double midX = (a.X + b.X) * 0.5;
        double midY = (a.Y + b.Y) * 0.5;
        double dx = b.X - a.X;
        double dy = b.Y - a.Y;
        double length = Math.Sqrt((dx * dx) + (dy * dy));
        if (length == 0)
        {
            return new Vertex(midX, midY);
        }

        double nx = -dy / length;
        double ny = dx / length;

        double epsilon = Math.Max(1e-6, length * 1e-6);
        return new Vertex(midX + (nx * epsilon), midY + (ny * epsilon));
    }

    private static double ComputeSignedArea(List<Vertex> vertices)
    {
        double area = 0;
        int count = vertices.Count;
        for (int i = 0; i < count; i++)
        {
            Vertex a = vertices[i];
            Vertex b = vertices[(i + 1) % count];
            area += (a.X * b.Y) - (b.X * a.Y);
        }

        return area * 0.5;
    }

    private static int ComputeWindingNumber(Vertex point, List<DirectedSegment> segments)
    {
        int winding = 0;
        for (int i = 0; i < segments.Count; i++)
        {
            Vertex a = segments[i].Source;
            Vertex b = segments[i].Target;

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

    private static double IsLeft(Vertex a, Vertex b, Vertex p)
        => ((b.X - a.X) * (p.Y - a.Y)) - ((p.X - a.X) * (b.Y - a.Y));

    private static void CleanupContour(Contour contour)
    {
        if (contour.Count == 0)
        {
            return;
        }

        List<Vertex> vertices = contour.ToList();
        if (vertices.Count > 1 && vertices[0] == vertices[^1])
        {
            vertices.RemoveAt(vertices.Count - 1);
        }

        ContourNode? start = BuildContourNodes(vertices);
        if (!IsValidClosedPath(start))
        {
            contour.Clear();
            return;
        }

        start = CleanCollinear(start, preserveCollinear: true);
        if (!IsValidClosedPath(start))
        {
            contour.Clear();
            return;
        }

        List<Vertex> cleaned = BuildPath(start);
        if (cleaned.Count < 3 || (cleaned.Count == 3 && IsVerySmallTriangle(cleaned)))
        {
            contour.Clear();
            return;
        }

        if (cleaned[0] != cleaned[^1])
        {
            cleaned.Add(cleaned[0]);
        }

        contour.Clear();
        for (int i = 0; i < cleaned.Count; i++)
        {
            contour.Add(cleaned[i]);
        }
    }

    private sealed class ContourNode
    {
        public ContourNode(Vertex point)
        {
            this.Point = point;
        }

        public Vertex Point { get; set; }

        public ContourNode Prev { get; set; } = null!;

        public ContourNode Next { get; set; } = null!;
    }

    private static ContourNode? BuildContourNodes(List<Vertex> vertices)
    {
        if (vertices.Count < 3)
        {
            return null;
        }

        ContourNode first = new(vertices[0]);
        ContourNode prev = first;
        for (int i = 1; i < vertices.Count; i++)
        {
            ContourNode node = new(vertices[i]);
            node.Prev = prev;
            prev.Next = node;
            prev = node;
        }

        prev.Next = first;
        first.Prev = prev;
        return first;
    }

    private static bool IsValidClosedPath(ContourNode? node)
        => node != null && node.Next != node && (node.Next != node.Prev || !IsVerySmallTriangle(node));

    private static bool IsVerySmallTriangle(ContourNode node)
        => node.Next.Next == node.Prev &&
            (PtsReallyClose(node.Prev.Point, node.Next.Point) ||
             PtsReallyClose(node.Point, node.Next.Point) ||
             PtsReallyClose(node.Point, node.Prev.Point));

    private static bool IsVerySmallTriangle(List<Vertex> vertices)
        => vertices.Count == 3 &&
            (PtsReallyClose(vertices[0], vertices[1]) ||
             PtsReallyClose(vertices[1], vertices[2]) ||
             PtsReallyClose(vertices[2], vertices[0]));

    private static bool PtsReallyClose(Vertex a, Vertex b)
        => Math.Abs(a.X - b.X) < 2 && Math.Abs(a.Y - b.Y) < 2;

    private static ContourNode? DisposeNode(ContourNode node)
    {
        ContourNode? result = node.Next == node ? null : node.Next;
        node.Prev.Next = node.Next;
        node.Next.Prev = node.Prev;
        return result;
    }

    private static ContourNode? CleanCollinear(ContourNode start, bool preserveCollinear)
    {
        ContourNode? op2 = start;
        ContourNode? currentStart = start;

        for (;;)
        {
            if (IsCollinear(op2!.Prev.Point, op2.Point, op2.Next.Point) &&
                ((op2.Point == op2.Prev.Point) || (op2.Point == op2.Next.Point) || !preserveCollinear ||
                 (DotProduct(op2.Prev.Point, op2.Point, op2.Next.Point) < 0)))
            {
                if (op2 == currentStart)
                {
                    currentStart = op2.Prev;
                }

                op2 = DisposeNode(op2);
                if (!IsValidClosedPath(op2))
                {
                    return null;
                }

                currentStart = op2;
                continue;
            }

            op2 = op2.Next;
            if (op2 == currentStart)
            {
                break;
            }
        }

        return currentStart;
    }

    private static List<Vertex> BuildPath(ContourNode start)
    {
        List<Vertex> path = [];
        Vertex last = start.Point;
        path.Add(last);
        ContourNode node = start.Next;
        while (node != start)
        {
            if (node.Point != last)
            {
                last = node.Point;
                path.Add(last);
            }

            node = node.Next;
        }

        return path;
    }

    private static bool IsCollinear(Vertex a, Vertex b, Vertex c)
    {
        double area = IsLeft(a, b, c);
        double scale = Math.Abs(a.X) + Math.Abs(a.Y) + Math.Abs(b.X) + Math.Abs(b.Y) + Math.Abs(c.X) + Math.Abs(c.Y) + 1;
        double tolerance = 1e-9 * scale;
        return Math.Abs(area) <= tolerance;
    }

    private static double DotProduct(Vertex pt1, Vertex pt2, Vertex pt3)
        => ((pt1.X - pt2.X) * (pt3.X - pt2.X)) + ((pt1.Y - pt2.Y) * (pt3.Y - pt2.Y));

    /// <summary>
    /// Runs the sweep line algorithm with positive fill rule to process self-intersecting polygons.
    /// </summary>
    /// <param name="polygon">The input polygon.</param>
    /// <returns>A polygon with self-intersections resolved using positive fill rule.</returns>
    private static Polygon RunSweepWithPositiveFill(Polygon polygon)
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
                // e1 is at Source (start of segment in contour), e2 is at Target (end)
                SweepEvent e1 = new(segment.Source, true, PolygonType.Subject);
                SweepEvent e2 = new(segment.Target, true, e1, PolygonType.Subject);
                e1.OtherEvent = e2;
                e1.ContourId = e2.ContourId = contourId;

                // Track which point is the source in contour order
                e1.IsContourSource = true;
                e2.IsContourSource = false;

                // Determine which endpoint is the left endpoint
                if (comparer.Compare(e1, e2) < 0)
                {
                    e2.Left = false;
                }
                else
                {
                    e1.Left = false;
                }

                // Compute WindDx based on path direction relative to the sweep.
                // In a left-to-right sweep (analogous to Clipper2's ascending/descending):
                // - Path goes left to right (Source is left endpoint) → +1
                // - Path goes right to left (Source is right endpoint) → -1
                // e1 is always at Source, so e1.Left means Source is the left endpoint.
                int windDx = e1.Left ? 1 : -1;
                e1.WindDx = e2.WindDx = windDx;

                unorderedEvents.Add(e1);
                unorderedEvents.Add(e2);
            }
        }

        if (unorderedEvents.Count == 0)
        {
            return [];
        }

        // Process events using the sweep line
        StablePriorityQueue<SweepEvent, SweepEventComparer> eventQueue = new(comparer, unorderedEvents);
        List<SweepEvent> sortedEvents = new(unorderedEvents.Count);
        StatusLine statusLine = new(polygon.VertexCount >> 1);

        // Track if any segments were split (indicating actual intersections)
        bool hadIntersections = false;

        // Safety limit to prevent infinite loops
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
            sortedEvents.Add(sweepEvent);

            if (sweepEvent.Left)
            {
                // Insert event into status line and get neighbors
                int position = sweepEvent.PosSL = statusLine.Add(sweepEvent);
                SweepEvent? prevEvent = statusLine.Prev(position);
                SweepEvent? nextEvent = statusLine.Next(position);

                // Compute winding for this segment based on what's below it
                ComputeWindingFields(sweepEvent, prevEvent);

                // Check for intersections with neighbors (ALL intersections, not just same contour)
                if (nextEvent != null)
                {
                    int result = PossibleIntersection(sweepEvent, nextEvent, eventQueue, workspace, comparer);
                    if (result > 0)
                    {
                        hadIntersections = true;
                    }

                    if (result == 2)
                    {
                        ComputeWindingFields(sweepEvent, prevEvent);
                        ComputeWindingFields(nextEvent, sweepEvent);
                    }
                }

                if (prevEvent != null)
                {
                    int result = PossibleIntersection(prevEvent, sweepEvent, eventQueue, workspace, comparer);
                    if (result > 0)
                    {
                        hadIntersections = true;
                    }

                    if (result == 2)
                    {
                        SweepEvent? prevPrevEvent = statusLine.Prev(prevEvent.PosSL);
                        ComputeWindingFields(prevEvent, prevPrevEvent);
                        ComputeWindingFields(sweepEvent, prevEvent);
                    }
                }
            }
            else
            {
                // Remove event from status line
                SweepEvent? leftEvent = sweepEvent.OtherEvent;
                if (leftEvent != null && leftEvent.PosSL >= 0 && leftEvent.PosSL < statusLine.Count)
                {
                    int position = leftEvent.PosSL;
                    SweepEvent? prevEvent = statusLine.Prev(position);
                    SweepEvent? nextEvent = statusLine.Next(position);

                    // Check for intersections between neighbors that are now adjacent
                    if (prevEvent != null && nextEvent != null)
                    {
                        int result = PossibleIntersection(prevEvent, nextEvent, eventQueue, workspace, comparer);
                        if (result > 0)
                        {
                            hadIntersections = true;
                        }
                    }



                    statusLine.RemoveAt(position);
                }
            }
        }

        // If no intersections occurred, preserve original structure with hierarchy
        if (!hadIntersections)
        {
            return BuildPolygonWithHierarchy(polygon, sortedEvents);
        }

        // Build result using positive fill rule
        return ConnectEdgesWithPositiveFill(sortedEvents, comparer);
    }

    /// <summary>
    /// Computes winding-related fields for a sweep event based on what's below it in the status line.
    /// Implements Clipper2's SetWindCountForClosedPathEdge algorithm for positive fill rule.
    /// <para>
    /// Wind counts refer to polygon regions not edges, so an edge's WindingCount
    /// indicates the higher of the wind counts for the two regions touching the edge.
    /// Adjacent regions can only ever have their wind counts differ by one.
    /// </para>
    /// <para>
    /// For positive fill rule, an edge contributes to the result when WindingCount == 1,
    /// meaning it sits on the boundary between winding 0 (unfilled) and winding 1 (filled).
    /// </para>
    /// </summary>
    /// <param name="sweepEvent">The sweep event to compute fields for.</param>
    /// <param name="prev">The previous event in the status line (below this one).</param>
    private static void ComputeWindingFields(SweepEvent sweepEvent, SweepEvent? prev)
    {
        int windDx = sweepEvent.WindDx;

        if (prev == null)
        {
            sweepEvent.WindingCount = windDx;
        }
        else
        {
            // Clipper2's SetWindCountForClosedPathEdge for NonZero/Positive/Negative:
            // All edges are same polygon type, so we always use same-type logic.
            if (prev.WindingCount * prev.WindDx < 0)
            {
                // Opposite directions: 'e' is outside 'prev'
                if (Math.Abs(prev.WindingCount) > 1)
                {
                    // Outside prev poly but still inside another
                    if (prev.WindDx * windDx < 0)
                    {
                        // Reversing direction so use the same WC
                        sweepEvent.WindingCount = prev.WindingCount;
                    }
                    else
                    {
                        // Keep 'reducing' the WC by 1 (towards 0)
                        sweepEvent.WindingCount = prev.WindingCount + windDx;
                    }
                }
                else
                {
                    // Now outside all polys of same polytype
                    sweepEvent.WindingCount = windDx;
                }
            }
            else
            {
                // 'e' must be inside 'prev'
                if (prev.WindDx * windDx < 0)
                {
                    // Reversing direction so use the same WC
                    sweepEvent.WindingCount = prev.WindingCount;
                }
                else
                {
                    // Keep 'increasing' the WC by 1 (away from 0)
                    sweepEvent.WindingCount = prev.WindingCount + windDx;
                }
            }
        }

        // For positive fill: an edge is contributing when wind_cnt == 1.
        // This means it's the boundary between winding 0 (unfilled) and winding 1 (filled).
        bool inResult = sweepEvent.WindingCount == 1 && sweepEvent.EdgeType == EdgeType.Normal;

        // Compute PrevInResult field
        sweepEvent.PrevInResult = prev?.PrevInResult;
        if (prev != null && prev.ResultTransition != ResultTransition.Neutral)
        {
            sweepEvent.PrevInResult = prev;
        }

        // Set ResultTransition for this edge
        if (inResult)
        {
            // For positive fill with single polygon (self-union):
            // wind_cnt == 1 means this edge borders a filled region.
            // Determine direction: if windDx > 0, we're entering (Contributing);
            // if windDx < 0, we're exiting (NonContributing).
            sweepEvent.ResultTransition = windDx > 0
                ? ResultTransition.Contributing
                : ResultTransition.NonContributing;
        }
        else
        {
            sweepEvent.ResultTransition = ResultTransition.Neutral;
        }

        // Set InOut and OtherInOut for compatibility with overlap detection in PossibleIntersection
        sweepEvent.InOut = sweepEvent.WindingCount != 1;
        sweepEvent.OtherInOut = true;
    }

    /// <summary>
    /// Gets the winding delta for a segment. Returns the pre-computed WindDx.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static int GetWindingDelta(SweepEvent sweepEvent)
        => sweepEvent.WindDx != 0 ? sweepEvent.WindDx : 1;

    /// <summary>
    /// Determines if a sweep event is part of the result using positive fill rule.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsInResultPositiveFill(SweepEvent sweepEvent)
        => sweepEvent.InResult;

    /// <summary>
    /// Determines the possible intersection of two sweep line segments and handles splitting.
    /// </summary>
    /// <param name="le1">The first sweep event.</param>
    /// <param name="le2">The second sweep event.</param>
    /// <param name="eventQueue">The event queue to add new events to.</param>
    /// <param name="workspace">Scratch space for temporary storage.</param>
    /// <param name="comparer">The sweep event comparer.</param>
    /// <returns>
    /// 0 if no intersection, 1 if single point intersection, 2 if overlapping with shared left endpoint, 3 otherwise.
    /// </returns>
    private static int PossibleIntersection(
        SweepEvent le1,
        SweepEvent le2,
        StablePriorityQueue<SweepEvent, SweepEventComparer> eventQueue,
        Span<SweepEvent> workspace,
        SweepEventComparer comparer)
    {
        if (le1.OtherEvent == null || le2.OtherEvent == null)
        {
            return 0;
        }

        int nIntersections = PolygonUtilities.FindIntersection(
            le1.GetSegment(),
            le2.GetSegment(),
            out Vertex ip1,
            out Vertex _);

        if (nIntersections == 0)
        {
            return 0;
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
    /// Builds a polygon preserving the original contour structure when no intersections occurred.
    /// Computes hierarchy (depth and parent) based on containment using sweep events.
    /// </summary>
    private static Polygon BuildPolygonWithHierarchy(Polygon originalPolygon, List<SweepEvent> events)
    {
        int[] contourDepth = new int[originalPolygon.Count];
        int[] contourParent = new int[originalPolygon.Count];
        for (int i = 0; i < originalPolygon.Count; i++)
        {
            contourParent[i] = -1;
        }

        // Process events to find containment
        Dictionary<int, bool> contourFirstEventSeen = new(originalPolygon.Count);
        StatusLine statusLine = new(originalPolygon.VertexCount >> 1);

        foreach (SweepEvent evt in events)
        {
            if (evt.Left)
            {
                int contourId = evt.ContourId;
                bool isFirstEvent = !contourFirstEventSeen.ContainsKey(contourId);

                int position = statusLine.Add(evt);
                evt.PosSL = position;

                if (isFirstEvent)
                {
                    contourFirstEventSeen[contourId] = true;

                    // The segment immediately below tells us what we're inside of
                    SweepEvent? prevEvent = statusLine.Prev(position);
                    if (prevEvent != null && prevEvent.ContourId != contourId)
                    {
                        int parentIdx = prevEvent.ContourId - 1;
                        int contourIdx = contourId - 1;

                        if (contourIdx >= 0 && contourIdx < originalPolygon.Count &&
                            parentIdx >= 0 && parentIdx < originalPolygon.Count)
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

        // Build result polygon with proper hierarchy
        List<Contour> contours = new(originalPolygon.Count);
        for (int i = 0; i < originalPolygon.Count; i++)
        {
            Contour copy = [];
            Contour original = originalPolygon[i];
            for (int j = 0; j < original.Count; j++)
            {
                copy.Add(original[j]);
            }

            copy.Depth = contourDepth[i];
            if (contourParent[i] >= 0)
            {
                copy.ParentIndex = contourParent[i];
            }

            contours.Add(copy);
        }

        // Build final polygon sorted by depth
        Polygon result = [];
        List<int> indices = new(contours.Count);
        for (int i = 0; i < contours.Count; i++)
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

    /// <summary>
    /// Connects edges using positive fill rule to build the result polygon.
    /// Only edges that form the boundary between filled and unfilled regions are included.
    /// </summary>
    private static Polygon ConnectEdgesWithPositiveFill(List<SweepEvent> sortedEvents, SweepEventComparer comparer)
    {
        // Collect result events - edges that are part of the boundary
        List<SweepEvent> resultEvents = new(sortedEvents.Count);
        for (int i = 0; i < sortedEvents.Count; i++)
        {
            SweepEvent se = sortedEvents[i];

            if (se.Left && se.InResult)
            {
                resultEvents.Add(se);
            }
            else if (!se.Left && se.OtherEvent != null && se.OtherEvent.InResult)
            {
                resultEvents.Add(se);
            }
        }

        if (resultEvents.Count == 0)
        {
            return [];
        }

        // Sort result events
        bool sorted = false;
        while (!sorted)
        {
            sorted = true;
            for (int i = 0; i < resultEvents.Count - 1; i++)
            {
                if (comparer.Compare(resultEvents[i], resultEvents[i + 1]) > 0)
                {
                    (resultEvents[i], resultEvents[i + 1]) = (resultEvents[i + 1], resultEvents[i]);
                    sorted = false;
                }
            }
        }

        // Assign positions
        for (int i = 0; i < resultEvents.Count; i++)
        {
            resultEvents[i].Pos = i;
        }

        for (int i = 0; i < resultEvents.Count; i++)
        {
            SweepEvent sweepEvent = resultEvents[i];
            if (sweepEvent.Left)
            {
                (sweepEvent.OtherEvent!.Pos, sweepEvent.Pos) = (sweepEvent.Pos, sweepEvent.OtherEvent.Pos);
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
            Contour contour = InitializeContourFromContext(resultEvents[i], result, contourId);

            int pos = i;
            Vertex initial = resultEvents[i].Point;
            contour.Add(initial);

            do
            {
                MarkProcessed(resultEvents[pos], processed, pos, contourId);
                pos = resultEvents[pos].Pos;

                MarkProcessed(resultEvents[pos], processed, pos, contourId);

                contour.Add(resultEvents[pos].Point);
                pos = NextPos(pos, resultEvents, processed, iterationMap, out bool found);
                if (!found)
                {
                    break;
                }
            }
            while (resultEvents[pos].Point != initial);

            result.Add(contour);
        }

        // Build final polygon with externals first, then holes
        Polygon polygon = [];
        for (int i = 0; i < result.Count; i++)
        {
            Contour contour = result[i];
            if (contour.IsExternal)
            {
                polygon.Add(contour);

                for (int j = 0; j < contour.HoleCount; j++)
                {
                    int holeId = contour.GetHoleIndex(j);
                    polygon.Add(result[holeId]);
                }
            }
        }

        return polygon;
    }

    private static ReadOnlySpan<int> PrecomputeIterationOrder(List<SweepEvent> data)
    {
        Span<int> map = new int[data.Count];

        int i = 0;
        while (i < data.Count)
        {
            SweepEvent xRef = data[i];

            int rFrom = i;
            while (i < data.Count && xRef.Point == data[i].Point && !data[i].Left)
            {
                i++;
            }

            int rUptoExclusive = i;

            int lFrom = i;
            while (i < data.Count && xRef.Point == data[i].Point)
            {
                i++;
            }

            int lUptoExclusive = i;

            bool hasREvents = rUptoExclusive > rFrom;
            bool hasLEvents = lUptoExclusive > lFrom;

            if (hasREvents)
            {
                int rUpto = rUptoExclusive - 1;

                for (int j = rFrom; j < rUpto; j++)
                {
                    map[j] = j + 1;
                }

                map[rUpto] = hasLEvents ? lUptoExclusive - 1 : rFrom;
            }

            if (hasLEvents)
            {
                int lUpto = lUptoExclusive - 1;

                for (int j = lFrom + 1; j <= lUpto; j++)
                {
                    map[j] = j - 1;
                }

                map[lFrom] = hasREvents ? rFrom : lUpto;
            }
        }

        return map;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void MarkProcessed(SweepEvent sweepEvent, Span<bool> processed, int pos, int contourId)
    {
        processed[pos] = true;
        sweepEvent.OutputContourId = contourId;
    }

    private static Contour InitializeContourFromContext(SweepEvent sweepEvent, Polygon polygon, int contourId)
    {
        Contour contour = new();

        if (sweepEvent.PrevInResult != null)
        {
            SweepEvent prevInResult = sweepEvent.PrevInResult;
            int lowerContourId = prevInResult.OutputContourId;
            ResultTransition lowerResultTransition = prevInResult.ResultTransition;

            if (lowerResultTransition > 0)
            {
                Contour lowerContour = polygon[lowerContourId];

                if (lowerContour.ParentIndex != null)
                {
                    int parentContourId = lowerContour.ParentIndex.Value;
                    polygon[parentContourId].AddHoleIndex(contourId);
                    contour.ParentIndex = parentContourId;
                    contour.Depth = polygon[lowerContourId].Depth;
                }
                else
                {
                    polygon[lowerContourId].AddHoleIndex(contourId);
                    contour.ParentIndex = lowerContourId;
                    contour.Depth = polygon[lowerContourId].Depth + 1;
                }
            }
            else
            {
                contour.ParentIndex = null;
                contour.Depth = polygon[lowerContourId].Depth;
            }
        }
        else
        {
            contour.ParentIndex = null;
            contour.Depth = 0;
        }

        return contour;
    }

    private static int NextPos(
        int pos,
        List<SweepEvent> resultEvents,
        ReadOnlySpan<bool> processed,
        ReadOnlySpan<int> iterationMap,
        out bool found)
    {
        int startPos = pos;

        // Collect all unprocessed candidates at this vertex
        int candidateCount = 0;
        int firstCandidate = int.MinValue;
        int current = pos;
        while (true)
        {
            current = iterationMap[current];
            if (current == startPos)
            {
                break;
            }

            if (!processed[current])
            {
                candidateCount++;
                if (firstCandidate == int.MinValue)
                {
                    firstCandidate = current;
                }
            }
        }

        if (candidateCount == 0)
        {
            found = false;
            return int.MinValue;
        }

        // If only one candidate, return it directly (fast path)
        if (candidateCount == 1)
        {
            found = true;
            return firstCandidate;
        }


        // Multiple candidates at same vertex: pick the best one based on angle.
        // The incoming edge is from resultEvents[pos] (a right event that was just marked processed).
        // We need the incoming direction: from the other endpoint toward this vertex.
        SweepEvent arrivedAt = resultEvents[pos];
        Vertex vertex = arrivedAt.Point;
        Vertex incomingFrom = arrivedAt.OtherEvent?.Point ?? vertex;
        double inDx = vertex.X - incomingFrom.X;
        double inDy = vertex.Y - incomingFrom.Y;

        // The incoming angle (direction we came from)
        double inAngle = Math.Atan2(inDy, inDx);

        int bestCandidate = firstCandidate;
        double bestAngleDiff = double.MaxValue;

        current = pos;
        while (true)
        {
            current = iterationMap[current];
            if (current == startPos)
            {
                break;
            }

            if (!processed[current])
            {
                // Get the outgoing direction of this candidate edge
                SweepEvent candidate = resultEvents[current];
                Vertex outgoingTo = candidate.OtherEvent?.Point ?? vertex;
                double outDx = outgoingTo.X - vertex.X;
                double outDy = outgoingTo.Y - vertex.Y;
                double outAngle = Math.Atan2(outDy, outDx);

                // Compute the clockwise angle from the incoming direction to the outgoing direction.
                // We want the smallest clockwise turn (which corresponds to following the
                // boundary of the filled region on the right side).
                double angleDiff = inAngle - outAngle;
                if (angleDiff < 0)
                {
                    angleDiff += 2 * Math.PI;
                }

                if (angleDiff < 1e-10)
                {
                    angleDiff += 2 * Math.PI; // Avoid zero (straight reversal)
                }

                if (angleDiff < bestAngleDiff)
                {
                    bestAngleDiff = angleDiff;
                    bestCandidate = current;
                }
            }
        }

        found = true;
        return bestCandidate;
    }
}

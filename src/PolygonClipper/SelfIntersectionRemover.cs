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

        Polygon orientedPolygon = OrientContoursForPositiveFill(polygon);
        List<DirectedSegment> segments = BuildArrangementSegments(orientedPolygon);
        if (segments.Count == 0)
        {
            return [];
        }

        HalfEdgeGraph graph = BuildHalfEdgeGraph(segments);
        _ = EnumerateFaces(graph);
        ComputeEdgeWinding(graph, segments);
        List<Contour> contours = ExtractBoundaryContours(graph);

        List<Contour> result = [];
        foreach (Contour contour in contours)
        {
            CleanupContour(contour);
            if (contour.Count > 2)
            {
                RotateContourToLowestPoint(contour);
                result.Add(contour);
            }
        }

        result.Sort(CompareContoursByLowestPoint);
        Polygon resultPolygon = [];
        for (int i = 0; i < result.Count; i++)
        {
            resultPolygon.Add(result[i]);
        }

        AssignHierarchy(resultPolygon);
        return resultPolygon;
    }

    private static int CompareContoursByLowestPoint(Contour left, Contour right)
    {
        Vertex leftPoint = GetLowestPoint(left);
        Vertex rightPoint = GetLowestPoint(right);

        if (leftPoint.Y < rightPoint.Y)
        {
            return -1;
        }

        if (leftPoint.Y > rightPoint.Y)
        {
            return 1;
        }

        return rightPoint.X.CompareTo(leftPoint.X);
    }

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
            if (candidate.Y < lowest.Y || (candidate.Y == lowest.Y && candidate.X > lowest.X))
            {
                lowest = candidate;
            }
        }

        return lowest;
    }

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
            if (candidate.Y < lowest.Y || (candidate.Y == lowest.Y && candidate.X > lowest.X))
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

    private static Polygon OrientContoursForPositiveFill(Polygon polygon)
    {
        if (polygon.Count == 0)
        {
            return polygon;
        }

        int count = polygon.Count;
        int[] parentIndices = new int[count];
        Array.Fill(parentIndices, -1);

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

                    Contour candidate = polygon[j];
                    if (PointInContour(testPoint, candidate))
                    {
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
            depths[i] = GetDepth(i, parentIndices);
        }

        Polygon oriented = [];
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

    private static double GetSignedArea(Contour contour)
    {
        double area = 0D;
        for (int i = 0; i < contour.Count; i++)
        {
            Vertex current = contour[i];
            Vertex next = contour[(i + 1) % contour.Count];
            area += Vertex.Cross(current, next);
        }

        return area;
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
        Dictionary<Vertex, Node> nodeMap = [];

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

        List<Face> faces = [];
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
        List<Contour> contours = [];
        HashSet<HalfEdge> visited = [];

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
                Vertex point = current.Origin.Point;
                if (contour.Count == 0 || !ArePointsClose(contour[^1], point))
                {
                    contour.Add(point);
                }

                current = NextBoundaryEdge(current, edge);
                if (current != null && current != edge && visited.Contains(current))
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

    private static bool IsBoundaryEdge(HalfEdge edge)
    {
        bool leftInside = edge.LeftWinding > 0;
        bool rightInside = edge.Twin != null && edge.Twin.LeftWinding > 0;
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

        contour.Clear();
        for (int i = 0; i < cleaned.Count; i++)
        {
            contour.Add(cleaned[i]);
        }
    }

    private static bool TryBuildContourNodes(List<Vertex> vertices, [NotNullWhen(true)] out ContourNode? node)
    {
        if (vertices.Count < 3)
        {
            node = null;
            return false;
        }

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

    private static double DotProduct(Vertex pt1, Vertex pt2, Vertex pt3)
        => Vertex.Dot(pt2 - pt1, pt3 - pt2);

    private static bool ArePointsClose(Vertex a, Vertex b)
        => Math.Abs(a.X - b.X) <= 1e-5 && Math.Abs(a.Y - b.Y) <= 1e-5;

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

    private readonly record struct DirectedSegment(Vertex Source, Vertex Target);

    private sealed class Node
    {
        public Node(Vertex point) => this.Point = point;

        public Vertex Point { get; }

        public List<HalfEdge> Outgoing { get; } = [];
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
        public List<Vertex> Boundary { get; } = [];

        public List<HalfEdge> Edges { get; } = [];

        public bool IsExterior { get; set; }

        public int Winding { get; set; }
    }

    private sealed class HalfEdgeGraph
    {
        public List<Node> Nodes { get; } = [];

        public List<HalfEdge> Edges { get; } = [];
    }

    private sealed class ContourNode
    {
        public ContourNode(Vertex point) => this.Point = point;

        public Vertex Point { get; set; }

        public ContourNode Prev { get; set; } = null!;

        public ContourNode Next { get; set; } = null!;
    }
}

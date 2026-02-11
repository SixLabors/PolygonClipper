// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System.Diagnostics;
using System.Diagnostics.CodeAnalysis;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace SixLabors.PolygonClipper;

/// <summary>
/// Implements a robust algorithm for performing boolean operations on polygons.
/// </summary>
/// <remarks>
/// <para>
/// This class implements the algorithm described in the paper
/// "A Simple Algorithm for Boolean Operations on Polygons" by Francisco Martínez,
/// Carlos Ogayar, Juan R. Jiménez, and Antonio J. Rueda. The algorithm is designed
/// to handle boolean operations such as intersection, union, difference, and XOR
/// between two polygons efficiently and robustly.
/// </para>
/// <para>
/// The algorithm uses a sweep line approach combined with an event queue to process
/// polygon segments, ensuring robust handling of special cases, including overlapping edges,
/// trivial operations (e.g., non-overlapping polygons), and edge intersections.
/// </para>
/// <para>The main workflow is divided into the following stages:</para>
/// <list type="number">
/// <item><description>Preprocessing: Handles trivial operations and prepares segments for processing.</description></item>
/// <item><description>Sweeping: Processes events using a priority queue, handling segment insertions and removals.</description></item>
/// <item><description>Connecting edges: Constructs the resulting polygon by connecting valid segments.</description></item>
/// </list>
/// </remarks>
public class PolygonClipper
{
    private readonly Polygon subject;
    private readonly Polygon clipping;
    private readonly BooleanOperation operation;
    private readonly ClipperOptions? options;

    /// <summary>
    /// Initializes a new instance of the <see cref="PolygonClipper"/> class.
    /// </summary>
    /// <param name="subject">The subject polygon.</param>
    /// <param name="clip">The clipping polygon.</param>
    /// <param name="operation">The operation type.</param>
    public PolygonClipper(Polygon subject, Polygon clip, BooleanOperation operation, ClipperOptions? options = null)
    {
        this.subject = subject;
        this.clipping = clip;
        this.operation = operation;
        this.options = options;
    }

    /// <summary>
    /// Computes the intersection of two polygons. The resulting polygon contains the regions that are common to both input polygons.
    /// </summary>
    /// <param name="subject">The subject polygon.</param>
    /// <param name="clip">The clipping polygon.</param>
    /// <returns>A new <see cref="Polygon"/> representing the intersection of the two polygons.</returns>
    public static Polygon Intersection(Polygon subject, Polygon clip, ClipperOptions? options = null)
    {
        PolygonClipper clipper = new(subject, clip, BooleanOperation.Intersection, options);
        return clipper.Run();
    }

    /// <summary>
    /// Computes the union of two polygons. The resulting polygon contains the combined regions of the two input polygons.
    /// </summary>
    /// <param name="subject">The subject polygon.</param>
    /// <param name="clip">The clipping polygon.</param>
    /// <returns>A new <see cref="Polygon"/> representing the union of the two polygons.</returns>
    public static Polygon Union(Polygon subject, Polygon clip, ClipperOptions? options = null)
    {
        PolygonClipper clipper = new(subject, clip, BooleanOperation.Union, options);
        return clipper.Run();
    }

    /// <summary>
    /// Computes the difference of two polygons. The resulting polygon contains the regions of the subject polygon
    /// that are not shared with the clipping polygon.
    /// </summary>
    /// <param name="subject">The subject polygon.</param>
    /// <param name="clip">The clipping polygon.</param>
    /// <returns>A new <see cref="Polygon"/> representing the difference between the two polygons.</returns>
    public static Polygon Difference(Polygon subject, Polygon clip, ClipperOptions? options = null)
    {
        PolygonClipper clipper = new(subject, clip, BooleanOperation.Difference, options);
        return clipper.Run();
    }

    /// <summary>
    /// Computes the symmetric difference (XOR) of two polygons. The resulting polygon contains the regions that belong
    /// to either one of the input polygons but not to their intersection.
    /// </summary>
    /// <param name="subject">The subject polygon.</param>
    /// <param name="clip">The clipping polygon.</param>
    /// <returns>A new <see cref="Polygon"/> representing the symmetric difference of the two polygons.</returns>
    public static Polygon Xor(Polygon subject, Polygon clip, ClipperOptions? options = null)
    {
        PolygonClipper clipper = new(subject, clip, BooleanOperation.Xor, options);
        return clipper.Run();
    }

    /// <summary>
    /// Removes self-intersections from a polygon by detecting all intersection points and rebuilding
    /// the polygon with clean, non-self-intersecting contours. This is useful for cleaning up polygons
    /// generated by stroking/offset operations or other procedures that may produce self-intersections.
    /// </summary>
    /// <param name="polygon">The polygon to process.</param>
    /// <returns>A new <see cref="Polygon"/> without self-intersections.</returns>
    public static Polygon RemoveSelfIntersections(Polygon polygon, ClipperOptions? options = null)
        => SelfIntersectionRemover.Process(polygon, options);

    /// <summary>
    /// Executes the boolean operation using the sweep line algorithm.
    /// </summary>
    /// <returns>The resulting <see cref="Polygon"/>.</returns>
    public Polygon Run()
    {
        FixedPrecisionContext context = FixedPrecisionContext.Create(this.options, [this.subject, this.clipping]);
        return this.RunCore(context);
    }

    /// <summary>
    /// Executes the core sweep line algorithm without trivial optimization checks.
    /// Used internally for operations like self-intersection removal where we want
    /// the full sweep to process all segments.
    /// </summary>
    /// <returns>The resulting <see cref="Polygon"/>.</returns>
    private Polygon RunCore(FixedPrecisionContext context)
    {
        PolygonUtilities.SetFloatingScale(context.InvScale);
        try
        {
            Polygon subject = this.subject;
            Polygon clipping = this.clipping;
            BooleanOperation operation = this.operation;

            if (TryTrivialOperationForEmptyPolygons(subject, clipping, operation, out Polygon? result))
            {
                return result;
            }

            GetBoundsAndVertexCount(subject, context, out Box64 subjectBounds, out int subjectVertexCount);
            GetBoundsAndVertexCount(clipping, context, out Box64 clippingBounds, out int clippingVertexCount);

            if (clippingVertexCount > 0)
            {
                Box2 subjectBoundingBox = subject.GetBoundingBox();
                Box2 clippingBoundingBox = clipping.GetBoundingBox();
                if (TryTrivialOperationForNonOverlappingBoundingBoxes(
                    subject,
                    clipping,
                    subjectBoundingBox,
                    clippingBoundingBox,
                    operation,
                    out result))
                {
                    return result;
                }
            }

            int eventCount = (subjectVertexCount + clippingVertexCount) * 2;

            SweepEventComparer comparer = new();
            List<SweepEvent> unorderedEventQueue = new(eventCount);
            int contourId = 0;

            for (int i = 0; i < subject.Count; i++)
            {
                Contour contour = subject[i];
                int count = contour.Count;
                int segmentCount = GetSegmentCount(contour, context);
                if (segmentCount == 0)
                {
                    continue;
                }

                contourId++;
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
                        PolygonType.Subject,
                        unorderedEventQueue,
                        comparer,
                        context);
                }
            }

            for (int i = 0; i < clipping.Count; i++)
            {
                Contour contour = clipping[i];
                int count = contour.Count;
                int segmentCount = GetSegmentCount(contour, context);
                if (segmentCount == 0)
                {
                    continue;
                }

                contourId++;
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
                        PolygonType.Clipping,
                        unorderedEventQueue,
                        comparer,
                        context);
                }
            }

            // Sweep line algorithm: process events in the priority queue
            StablePriorityQueue<SweepEvent, SweepEventComparer> eventQueue = new(comparer, unorderedEventQueue);
            List<SweepEvent> sortedEvents = new(eventCount);

            // Heuristic capacity for the sweep line status structure.
            // At any given point during the sweep, only a subset of segments
            // are active, so we preallocate half the subject's vertex count
            // to reduce resizing without overcommitting memory.
            StatusLine statusLine = new(subjectVertexCount >> 1);
            long subjectMaxX = subjectBounds.Max.X;
            long minMaxX = Math.Min(subjectBounds.Max.X, clippingBounds.Max.X);

            SweepEvent? prevEvent;
            SweepEvent? nextEvent;
            Span<SweepEvent> workspace = new SweepEvent[4];
            while (eventQueue.Count > 0)
            {
                SweepEvent sweepEvent = eventQueue.Dequeue();
                sortedEvents.Add(sweepEvent);

                // Optimization: skip further processing if intersection is impossible
                if ((operation == BooleanOperation.Intersection && sweepEvent.Point.X > minMaxX) ||
                    (operation == BooleanOperation.Difference && sweepEvent.Point.X > subjectMaxX))
                {
                    return ConnectEdges(sortedEvents, comparer, context);
                }

                if (sweepEvent.Left)
                {
                    // Insert the event into the status line and get neighbors
                    int it = sweepEvent.PosSL = statusLine.Add(sweepEvent);
                    prevEvent = statusLine.Prev(it);
                    nextEvent = statusLine.Next(it);

                    // Compute fields for the current event
                    ComputeFields(sweepEvent, prevEvent, operation);

                    // Check intersection with the next neighbor
                    if (nextEvent != null)
                    {
                        // Check intersection with the next neighbor
                        if (PossibleIntersection(sweepEvent, nextEvent, eventQueue, workspace, context) == 2)
                        {
                            ComputeFields(sweepEvent, prevEvent, operation);
                            ComputeFields(nextEvent, sweepEvent, operation);
                        }
                    }

                    // Check intersection with the previous neighbor
                    if (prevEvent != null)
                    {
                        // Check intersection with the previous neighbor
                        if (PossibleIntersection(prevEvent, sweepEvent, eventQueue, workspace, context) == 2)
                        {
                            SweepEvent? prevPrevEvent = statusLine.Prev(prevEvent.PosSL);
                            ComputeFields(prevEvent, prevPrevEvent, operation);
                            ComputeFields(sweepEvent, prevEvent, operation);
                        }
                    }
                }
                else
                {
                    // Remove the event from the status line
                    sweepEvent = sweepEvent.OtherEvent;
                    int it = sweepEvent.PosSL;
                    prevEvent = statusLine.Prev(it);
                    nextEvent = statusLine.Next(it);

                    // Check intersection between neighbors
                    if (prevEvent != null && nextEvent != null)
                    {
                        _ = PossibleIntersection(prevEvent, nextEvent, eventQueue, workspace, context);
                    }

                    statusLine.RemoveAt(it);
                }
            }

            // Connect edges after processing all events
            return ConnectEdges(sortedEvents, comparer, context);
        }
        finally
        {
            PolygonUtilities.ClearFloatingScale();
        }
    }

    /// <summary>
    /// Checks if the boolean operation is trivial due to one polygon having zero contours
    /// and sets the result accordingly.
    /// </summary>
    /// <param name="subject">The subject polygon.</param>
    /// <param name="clipping">The clipping polygon.</param>
    /// <param name="operation">The boolean operation being performed.</param>
    /// <param name="result">The resulting polygon if the operation is trivial.</param>
    /// <returns>
    /// <see langword="true"/> if the operation results in a trivial case due to zero contours;
    /// otherwise, <see langword="false"/>.
    /// </returns>
    private static bool TryTrivialOperationForEmptyPolygons(
        Polygon subject,
        Polygon clipping,
        BooleanOperation operation,
        [NotNullWhen(true)] out Polygon? result)
    {
        result = null;

        if (subject.Count * clipping.Count == 0)
        {
            if (operation == BooleanOperation.Intersection)
            {
                result = [];
                return true;
            }

            if (operation == BooleanOperation.Difference)
            {
                result = BuildNormalizedPolygon(subject);
                return true;
            }

            if (operation is BooleanOperation.Union or BooleanOperation.Xor)
            {
                result = subject.Count == 0
                    ? BuildNormalizedPolygon(clipping)
                    : BuildNormalizedPolygon(subject);
                return true;
            }
        }

        return false;
    }

    /// <summary>
    /// Checks if the boolean operation is trivial due to non-overlapping bounding boxes
    /// and sets the result accordingly.
    /// </summary>
    /// <param name="subject">The subject polygon.</param>
    /// <param name="clipping">The clipping polygon.</param>
    /// <param name="subjectBB">The bounding box of the subject polygon.</param>
    /// <param name="clippingBB">The bounding box of the clipping polygon.</param>
    /// <param name="operation">The boolean operation being performed.</param>
    /// <param name="result">The resulting polygon if the operation is trivial.</param>
    /// <returns>
    /// <see langword="true"/> if the operation results in a trivial case due to non-overlapping
    /// bounding boxes; otherwise, <see langword="false"/>.
    /// </returns>
    private static bool TryTrivialOperationForNonOverlappingBoundingBoxes(
        Polygon subject,
        Polygon clipping,
        Box2 subjectBB,
        Box2 clippingBB,
        BooleanOperation operation,
        [NotNullWhen(true)] out Polygon? result)
    {
        result = null;

        if (subjectBB.Min.X > clippingBB.Max.X || clippingBB.Min.X > subjectBB.Max.X ||
            subjectBB.Min.Y > clippingBB.Max.Y || clippingBB.Min.Y > subjectBB.Max.Y)
        {
            if (operation == BooleanOperation.Intersection)
            {
                result = [];
                return true;
            }

            // The bounding boxes do not overlap
            if (operation == BooleanOperation.Difference)
            {
                result = BuildNormalizedPolygon(subject);
                return true;
            }

            if (operation is BooleanOperation.Union or BooleanOperation.Xor)
            {
                result = BuildNormalizedPolygon(subject, clipping);
                return true;
            }
        }

        return false;
    }

    private static void GetBoundsAndVertexCount(
        Polygon polygon,
        FixedPrecisionContext context,
        out Box64 bounds,
        out int vertexCount)
    {
        vertexCount = 0;

        long minX = long.MaxValue;
        long minY = long.MaxValue;
        long maxX = long.MinValue;
        long maxY = long.MinValue;

        for (int i = 0; i < polygon.Count; i++)
        {
            Contour contour = polygon[i];
            int count = contour.Count;
            if (count == 0)
            {
                continue;
            }

            vertexCount += count;

            for (int j = 0; j < count; j++)
            {
                Vertex64 point = context.Quantize(contour[j]);
                if (point.X < minX)
                {
                    minX = point.X;
                }

                if (point.X > maxX)
                {
                    maxX = point.X;
                }

                if (point.Y < minY)
                {
                    minY = point.Y;
                }

                if (point.Y > maxY)
                {
                    maxY = point.Y;
                }
            }
        }

        bounds = minX == long.MaxValue
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

    private static Polygon BuildNormalizedPolygon(Polygon polygon)
        => BuildNormalizedPolygon([polygon]);

    private static Polygon BuildNormalizedPolygon(Polygon subject, Polygon clipping)
        => BuildNormalizedPolygon([subject, clipping]);

    private static Polygon BuildNormalizedPolygon(ReadOnlySpan<Polygon> polygons)
    {
        List<ContourInfoDouble> contourInfos = [];
        for (int i = 0; i < polygons.Length; i++)
        {
            Polygon polygon = polygons[i];
            for (int j = 0; j < polygon.Count; j++)
            {
                Contour contour = polygon[j];
                if (contour.Count == 0)
                {
                    continue;
                }

                contourInfos.Add(CreateContourInfo(contour));
            }
        }

        if (contourInfos.Count == 0)
        {
            return [];
        }

        int count = contourInfos.Count;
        int[] parentIndices = new int[count];
        int[] depths = new int[count];
        Array.Fill(parentIndices, -1);

        for (int i = 0; i < count; i++)
        {
            Vertex testPoint = contourInfos[i].Vertices[0];
            double smallestContainerArea = double.PositiveInfinity;
            int parentIndex = -1;

            for (int j = 0; j < count; j++)
            {
                if (i == j)
                {
                    continue;
                }

                if (PointInContour(testPoint, contourInfos[j].Vertices))
                {
                    double area = contourInfos[j].Area;
                    if (area < smallestContainerArea)
                    {
                        smallestContainerArea = area;
                        parentIndex = j;
                    }
                }
            }

            parentIndices[i] = parentIndex;
        }

        List<int> externals = new(count);
        for (int i = 0; i < count; i++)
        {
            int depth = GetDepth(i, parentIndices);
            depths[i] = depth;

            if ((depth & 1) == 0)
            {
                externals.Add(i);
            }
        }

        externals.Sort((left, right) =>
            CompareVertices(contourInfos[left].MinVertex, contourInfos[right].MinVertex));

        Polygon result = new(count);
        bool[] added = new bool[count];
        int[] newIndices = new int[count];
        Array.Fill(newIndices, -1);

        foreach (int externalIndex in externals)
        {
            int externalContourIndex = AddContour(
                result,
                contourInfos,
                externalIndex,
                depths,
                parentIndices,
                added,
                newIndices);

            List<int> holes = [];
            for (int i = 0; i < count; i++)
            {
                if (!added[i] && parentIndices[i] == externalIndex && (depths[i] & 1) == 1)
                {
                    holes.Add(i);
                }
            }

            holes.Sort((left, right) =>
                CompareVertices(contourInfos[left].MinVertex, contourInfos[right].MinVertex));

            foreach (int holeIndex in holes)
            {
                int holeContourIndex = AddContour(
                    result,
                    contourInfos,
                    holeIndex,
                    depths,
                    parentIndices,
                    added,
                    newIndices);

                result[externalContourIndex].AddHoleIndex(holeContourIndex);
                result[holeContourIndex].ParentIndex = externalContourIndex;
            }
        }

        for (int i = 0; i < count; i++)
        {
            if (!added[i])
            {
                _ = AddContour(result, contourInfos, i, depths, parentIndices, added, newIndices);
            }
        }

        return result;
    }

    private static int CompareVertices(in Vertex left, in Vertex right)
        => left.X != right.X
            ? left.X.CompareTo(right.X)
            : left.Y.CompareTo(right.Y);

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

    private static ContourInfoDouble CreateContourInfo(Contour contour)
    {
        bool isClosed = contour.Count > 1 && contour[0] == contour[^1];
        int targetCount = isClosed ? contour.Count - 1 : contour.Count;
        List<Vertex> vertices = new(targetCount);
        for (int i = 0; i < targetCount; i++)
        {
            vertices.Add(contour[i]);
        }

        NormalizeVertices(vertices);

        Vertex minVertex = vertices[0];
        for (int i = 1; i < vertices.Count; i++)
        {
            if (CompareVertices(vertices[i], minVertex) < 0)
            {
                minVertex = vertices[i];
            }
        }

        double area = Math.Abs(GetSignedArea(vertices));

        if (isClosed)
        {
            vertices.Add(vertices[0]);
        }

        return new ContourInfoDouble(vertices, minVertex, area);
    }

    private static void NormalizeVertices(List<Vertex> vertices)
    {
        if (vertices.Count < 3)
        {
            return;
        }

        if (GetSignedArea(vertices) < 0)
        {
            vertices.Reverse();
        }

        int minIndex = 0;
        Vertex minVertex = vertices[0];
        for (int i = 1; i < vertices.Count; i++)
        {
            if (CompareVertices(vertices[i], minVertex) < 0)
            {
                minVertex = vertices[i];
                minIndex = i;
            }
        }

        if (minIndex > 0)
        {
            List<Vertex> rotated = new(vertices.Count);
            for (int i = 0; i < vertices.Count; i++)
            {
                rotated.Add(vertices[(i + minIndex) % vertices.Count]);
            }

            vertices.Clear();
            vertices.AddRange(rotated);
        }
    }

    private static double GetSignedArea(List<Vertex> vertices)
    {
        double area = 0;
        for (int i = 0; i < vertices.Count; i++)
        {
            Vertex current = vertices[i];
            Vertex next = vertices[(i + 1) % vertices.Count];
            area += Vertex.Cross(current, next);
        }

        return area;
    }

    private static bool PointInContour(in Vertex point, List<Vertex> vertices)
    {
        int count = vertices.Count;
        if (count < 3)
        {
            return false;
        }

        bool inside = false;
        Vertex previous = vertices[count - 1];

        for (int i = 0; i < count; i++)
        {
            Vertex current = vertices[i];
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
                double xIntersection = (((previous.X - current.X) * (point.Y - current.Y)) / (previous.Y - current.Y)) +
                                       current.X;
                if (point.X < xIntersection)
                {
                    inside = !inside;
                }
            }

            previous = current;
        }

        return inside;
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
                double xIntersection = ((previous.X - current.X) * (point.Y - current.Y) / (previous.Y - current.Y)) +
                                       current.X;
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
        if (Vertex.Cross(b - a, point - a) != 0)
        {
            return false;
        }

        double minX = Math.Min(a.X, b.X);
        double maxX = Math.Max(a.X, b.X);
        double minY = Math.Min(a.Y, b.Y);
        double maxY = Math.Max(a.Y, b.Y);

        return point.X >= minX && point.X <= maxX && point.Y >= minY && point.Y <= maxY;
    }

    private static bool SegmentsIntersect(
        in Vertex a1,
        in Vertex a2,
        in Vertex b1,
        in Vertex b2,
        bool inclusive = false)
    {
        // Uses cross-product tests to solve a1 + d1 * t == b1 + d2 * u.
        // cp is the denominator (cross of directions); cp == 0 means parallel/collinear.
        Vertex d1 = a2 - a1;
        Vertex d2 = b2 - b1;
        double cp = Vertex.Cross(d2, d1);
        if (cp == 0)
        {
            if (Vertex.Cross(b1 - a1, d1) != 0)
            {
                return false;
            }

            double aMinX = Math.Min(a1.X, a2.X);
            double aMaxX = Math.Max(a1.X, a2.X);
            double bMinX = Math.Min(b1.X, b2.X);
            double bMaxX = Math.Max(b1.X, b2.X);
            double aMinY = Math.Min(a1.Y, a2.Y);
            double aMaxY = Math.Max(a1.Y, a2.Y);
            double bMinY = Math.Min(b1.Y, b2.Y);
            double bMaxY = Math.Max(b1.Y, b2.Y);

            return aMaxX >= bMinX && bMaxX >= aMinX &&
                   aMaxY >= bMinY && bMaxY >= aMinY;
        }

        if (inclusive)
        {
            // Inclusive mode allows intersections at endpoints.
            double t = Vertex.Cross(a1 - b1, d2);
            if (t == 0)
            {
                return true;
            }

            if (t > 0)
            {
                if (cp < 0 || t > cp)
                {
                    // t outside [0, cp] once sign is normalized.
                    return false;
                }
            }
            else if (cp > 0 || t < cp)
            {
                return false;
            }

            t = Vertex.Cross(a1 - b1, d1);
            if (t == 0)
            {
                return true;
            }

            if (t > 0)
            {
                // t within bounds for the second segment.
                return cp > 0 && t <= cp;
            }

            return cp < 0 && t >= cp;
        }

        // Exclusive mode requires the intersection to be strictly inside both segments.
        double t2 = Vertex.Cross(a1 - b1, d2);
        if (t2 == 0)
        {
            return false;
        }

        if (t2 > 0)
        {
            if (cp < 0 || t2 >= cp)
            {
                // Reject if t2 is outside the open interval.
                return false;
            }
        }
        else if (cp > 0 || t2 <= cp)
        {
            return false;
        }

        t2 = Vertex.Cross(a1 - b1, d1);
        if (t2 == 0)
        {
            return false;
        }

        if (t2 > 0)
        {
            // Both parameters are inside open intervals.
            return cp > 0 && t2 < cp;
        }

        return cp < 0 && t2 > cp;
    }

    private static int AddContour(
        Polygon polygon,
        List<ContourInfoDouble> contourInfos,
        int contourIndex,
        int[] depths,
        int[] parentIndices,
        bool[] added,
        int[] newIndices)
    {
        List<Vertex> vertices = contourInfos[contourIndex].Vertices;
        Contour contour = new(vertices.Count);
        for (int i = 0; i < vertices.Count; i++)
        {
            contour.Add(vertices[i]);
        }

        polygon.Add(contour);
        added[contourIndex] = true;
        int newIndex = polygon.Count - 1;
        newIndices[contourIndex] = newIndex;

        contour.Depth = depths[contourIndex];
        if (depths[contourIndex] % 2 == 1)
        {
            int parentIndex = parentIndices[contourIndex];
            if (parentIndex >= 0 && newIndices[parentIndex] >= 0)
            {
                contour.ParentIndex = newIndices[parentIndex];
            }
        }

        return newIndex;
    }

    /// <summary>
    /// Processes a segment by generating sweep events for its endpoints and adding them to the event queue.
    /// </summary>
    /// <param name="contourId">The identifier of the contour to which the segment belongs.</param>
    /// <param name="s">The segment to process.</param>
    /// <param name="pt">The polygon type to which the segment belongs.</param>
    /// <param name="eventQueue">The unordered event queue to add the generated events to.</param>
    /// <param name="comparer">The comparer used to determine the order of sweep events in the queue.</param>
    private static void ProcessSegment(
        int contourId,
        in Vertex start,
        in Vertex end,
        PolygonType pt,
        List<SweepEvent> eventQueue,
        SweepEventComparer comparer,
        FixedPrecisionContext context)
    {
        Vertex64 source = context.Quantize(start);
        Vertex64 target = context.Quantize(end);
        if (source == target)
        {
            // Skip degenerate zero-length segments.
            return;
        }

        // Create sweep events for the endpoints of the segment
        SweepEvent e1 = new(source, true, pt);
        SweepEvent e2 = new(target, true, e1, pt);
        e1.OtherEvent = e2;
        e1.ContourId = e2.ContourId = contourId;
        e1.PointDouble = start;
        e2.PointDouble = end;

        // Determine which endpoint is the left endpoint
        if (comparer.Compare(e1, e2) < 0)
        {
            e2.Left = false;
        }
        else
        {
            e1.Left = false;
        }

        // Add the events to the event queue
        eventQueue.Add(e1);
        eventQueue.Add(e2);
    }

    /// <summary>
    /// Computes fields for a given sweep event.
    /// </summary>
    /// <param name="le">The sweep event to compute fields for.</param>
    /// <param name="prev">The the previous event in the status line.</param>
    /// <param name="operation">The boolean operation being performed.</param>
    private static void ComputeFields(SweepEvent le, SweepEvent? prev, BooleanOperation operation)
    {
        // Compute inOut and otherInOut fields
        if (prev == null)
        {
            le.InOut = false;
            le.OtherInOut = true;
            le.PrevInResult = null;
        }
        else if (le.PolygonType == prev.PolygonType)
        {
            // Previous line segment in sl belongs to the same polygon that "se" belongs to.
            le.InOut = !prev.InOut;
            le.OtherInOut = prev.OtherInOut;
        }
        else
        {
            // Previous line segment in sl belongs to a different polygon that "se" belongs to.
            le.InOut = !prev.OtherInOut;
            le.OtherInOut = prev.IsVertical() ? !prev.InOut : prev.InOut;
        }

        // Compute PrevInResult field
        if (prev != null)
        {
            le.PrevInResult = (!InResult(prev, operation) || prev.IsVertical())
                ? prev.PrevInResult
                : prev;
        }

        // Check if the line segment belongs to the Boolean operation
        bool inResult = InResult(le, operation);
        if (inResult)
        {
            le.ResultTransition = DetermineResultTransition(le, operation);
        }
        else
        {
            le.ResultTransition = ResultTransition.Neutral;
        }
    }

    /// <summary>
    /// Determines the result transition state for a given sweep event based on the specified boolean operation.
    /// </summary>
    /// <param name="sweepEvent">The sweep event to evaluate.</param>
    /// <param name="operation">The boolean operation being performed (e.g., Intersection, Union, XOR, Difference).</param>
    /// <returns>
    /// A <see cref="ResultTransition"/> value that represents the transition state of the event:
    /// <list type="bullet">
    /// <item><description><see cref="ResultTransition.Contributing"/> if the event contributes to the result.</description></item>
    /// <item><description><see cref="ResultTransition.NonContributing"/> if the event does not contribute to the result.</description></item>
    /// <item><description><see cref="ResultTransition.Neutral"/> if the event does not affect the transition but is part of the result.</description></item>
    /// </list>
    /// </returns>
    /// <exception cref="InvalidOperationException">Thrown if the boolean operation is invalid or unsupported.</exception>
    private static ResultTransition DetermineResultTransition(SweepEvent sweepEvent, BooleanOperation operation)
    {
        bool thisIn = !sweepEvent.InOut;
        bool thatIn = !sweepEvent.OtherInOut;
        bool isIn;

        // Determine the "in" state based on the operation
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

    /// <summary>
    /// Determines if the given sweep event belongs to the result of the boolean operation.
    /// </summary>
    /// <param name="sweepEvent">The sweep event to check.</param>
    /// <param name="operation">The boolean operation being performed.</param>
    /// <returns><see langword="true"/> if the event belongs to the result; otherwise, <see langword="false"/>.</returns>
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

    /// <summary>
    /// Determines the possible intersection of two sweep line segments.
    /// </summary>
    /// <param name="le1">The first sweep event representing a line segment.</param>
    /// <param name="le2">The second sweep event representing a line segment.</param>
    /// <param name="eventQueue">The event queue to add new events to.</param>
    /// <param name="workspace">
    /// A scratch space for temporary storage of sweep events.
    /// Must be at least 4 elements long to hold the events for the two segments and their associated other events.
    /// </param>
    /// <returns>
    /// An integer indicating the result of the intersection:
    /// <list type="bullet">
    /// <item><description>0 if no intersection or trivial intersection at endpoints.</description></item>
    /// <item><description>1 if the segments intersect at a single point.</description></item>
    /// <item><description>2 if the segments overlap and share a left endpoint.</description></item>
    /// <item><description>3 if the segments partially overlap or one includes the other.</description></item>
    /// </list>
    /// </returns>
    /// <exception cref="InvalidOperationException">
    /// Thrown when the line segments overlap but belong to the same polygon.
    /// </exception>
    private static int PossibleIntersection(
        SweepEvent le1,
        SweepEvent le2,
        StablePriorityQueue<SweepEvent, SweepEventComparer> eventQueue,
        Span<SweepEvent> workspace,
        FixedPrecisionContext context)
    {
        if (le1.OtherEvent == null || le2.OtherEvent == null)
        {
            // No intersection possible.
            return 0;
        }

        // Point intersections
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
                out Vertex64 _); // Currently unused but could be used to detect collinear overlapping segments
        }

        if (nIntersections == 0)
        {
            // No intersection
            return 0;
        }

        // Ignore intersection if it occurs at the exact left or right endpoint of both segments
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
                // Line segments intersect at an endpoint of both line segments.
                return 0;
            }
        }

        // If segments overlap and belong to the same polygon, ignore them
        if (nIntersections == 2 && le1.PolygonType == le2.PolygonType)
        {
            return 0;
        }

        // Handle a single intersection point
        SweepEventComparer comparer = eventQueue.Comparer;
        if (nIntersections == 1)
        {
            if (useFloating)
            {
                // If the intersection point is not an endpoint of le1 segment.
                if (le1.PointDouble != ip1Double && le1.OtherEvent.PointDouble != ip1Double)
                {
                    DivideSegment(le1, ip1Fixed, ip1Double, eventQueue, comparer, context);
                }

                // If the intersection point is not an endpoint of le2 segment.
                if (le2.PointDouble != ip1Double && le2.OtherEvent.PointDouble != ip1Double)
                {
                    DivideSegment(le2, ip1Fixed, ip1Double, eventQueue, comparer, context);
                }
            }
            else
            {
                // If the intersection point is not an endpoint of le1 segment.
                if (le1.Point != ip1Fixed && le1.OtherEvent.Point != ip1Fixed)
                {
                    DivideSegment(le1, ip1Fixed, new Vertex(ip1Fixed.X, ip1Fixed.Y), eventQueue, comparer, context);
                }

                // If the intersection point is not an endpoint of le2 segment.
                if (le2.Point != ip1Fixed && le2.OtherEvent.Point != ip1Fixed)
                {
                    DivideSegment(le2, ip1Fixed, new Vertex(ip1Fixed.X, ip1Fixed.Y), eventQueue, comparer, context);
                }
            }

            return 1;
        }

        // The line segments associated with le1 and le2 overlap.
        bool leftCoincide = useFloating
            ? le1.PointDouble == le2.PointDouble
            : le1.Point == le2.Point;
        bool rightCoincide = useFloating
            ? le1.OtherEvent.PointDouble == le2.OtherEvent.PointDouble
            : le1.OtherEvent.Point == le2.OtherEvent.Point;

        // Populate the events.
        // The working buffer has a length of 4, which is sufficient to hold the events
        // for the two segments and their associated other events.
        // Events are assigned in a specific order to avoid overwriting shared references.
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

            // Positions 0 and 1 contain the left events of the segments.
            // Positions 2 and 3 will contain the right events of the segments.
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
            // Only the right endpoints differ, so we use positions 0 and 1 for their sorted order.
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

            if (leftCoincide && !rightCoincide)
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
            // Since leftCoincide is false, the first two workspace slots contain distinct left events.
            splitEvent = Unsafe.Add(ref wRef, 1u);
            splitPoint = useFloating ? context.Quantize(splitEvent.PointDouble) : splitEvent.Point;
            splitPointDouble = useFloating ? splitEvent.PointDouble : new Vertex(splitPoint.X, splitPoint.Y);
            DivideSegment(Unsafe.Add(ref wRef, 0u), splitPoint, splitPointDouble, eventQueue, comparer, context);
            return 3;
        }

        // Handle general overlapping case
        // At this point: workspace[0,1] = sorted left events, workspace[2,3] = sorted right events.
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

        // One segment fully contains the other
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

    /// <summary>
    /// Divides the given segment at the specified point, creating two new segments.
    /// </summary>
    /// <param name="le">The left event representing the segment to divide.</param>
    /// <param name="p">The point at which to divide the segment.</param>
    /// <param name="eventQueue">The event queue to add the new events to.</param>
    /// <param name="comparer">The comparer used to sort the events.</param>
    private static void DivideSegment(
        SweepEvent le,
        Vertex64 p,
        Vertex pDouble,
        StablePriorityQueue<SweepEvent, SweepEventComparer> eventQueue,
        SweepEventComparer comparer,
        FixedPrecisionContext context)
    {
        if (le.OtherEvent == null)
        {
            return;
        }

        SweepEvent re = le.OtherEvent;

        // The idea is to divide the segment based on the given `inter` coordinate as follows:
        //
        //     (se_l)--------(r)(l)--------(re)
        //
        // Under normal circumstances the resulting events satisfy the conditions:
        //
        //     se_l is before r, and l is before re.
        //
        // Since the intersection point computation is bounded to the interval [se_l.x, re.x]
        // it is impossible for r/l to fall outside the interval. This leaves the corner case:
        //
        //  l.x == re.x and l.y > re.y: This corresponds to the case where the second
        //  sub-segment becomes a perfectly vertical line, and because of the bottom-to-top
        //  convention for vertical segment, the order of l and re must be swapped.
        //  In this case swapping is not a problem, because both events are in the future.
        //
        // See also: https://github.com/21re/rust-geo-booleanop/pull/11

        if (PolygonUtilities.UseFloatingScale)
        {
            if (pDouble.X == le.PointDouble.X && pDouble.Y < le.PointDouble.Y)
            {
                pDouble = new Vertex(pDouble.X.NextAfter(double.PositiveInfinity), pDouble.Y);
                p = context.Quantize(pDouble);
            }
        }
        else if (p.X == le.Point.X && p.Y < le.Point.Y)
        {
            p = new Vertex64(p.X + 1, p.Y);
            pDouble = new Vertex(p.X, p.Y);
        }

        // Create the right event for the left segment (new right endpoint)
        SweepEvent r = new(p, false, le, le.PolygonType);

        // Create the left event for the right segment (new left endpoint)
        SweepEvent l = new(p, true, re, le.PolygonType);

        r.PointDouble = pDouble;
        l.PointDouble = pDouble;

        // Assign the same contour ID to maintain connectivity
        r.ContourId = l.ContourId = le.ContourId;

        // Corner case 2 can be accounted for by swapping l / se_r
        if (comparer.Compare(l, re) > 0)
        {
            Debug.WriteLine("Rounding error detected: Adjusting left/right flags for event ordering.");
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
    /// Connects edges in the result polygon by processing the sweep events
    /// and constructing contours for the final result.
    /// </summary>
    /// <param name="sortedEvents">The sorted list of sweep events.</param>
    /// <param name="comparer">The comparer used to sort the events.</param>
    /// <returns>The resulting <see cref="Polygon"/>.</returns>
    private static Polygon ConnectEdges(List<SweepEvent> sortedEvents, SweepEventComparer comparer, FixedPrecisionContext context)
    {
        // Copy the events in the result polygon to resultEvents list
        List<SweepEvent> resultEvents = new(sortedEvents.Count);
        for (int i = 0; i < sortedEvents.Count; i++)
        {
            SweepEvent se = sortedEvents[i];
            if (se.Left && se.InResult)
            {
                resultEvents.Add(se);
            }
            else if (!se.Left && se.OtherEvent.InResult)
            {
                resultEvents.Add(se);
            }
        }

        // Due to overlapping edges, the resultEvents list may not be completely sorted
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

        bool useFloating = PolygonUtilities.UseFloatingScale;
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
            Vertex64 initial = resultEvents[i].Point;
            Vertex lastPointDouble = resultEvents[i].PointDouble;
            Vertex initialDouble = lastPointDouble;
            Vertex64 lastPoint = initial;
            contour.Add(context.Dequantize(initial));

            // Main loop to process the contour
            do
            {
                MarkProcessed(resultEvents[pos], processed, pos, contourId);
                pos = resultEvents[pos].Pos;

                MarkProcessed(resultEvents[pos], processed, pos, contourId);

                lastPoint = resultEvents[pos].Point;
                lastPointDouble = resultEvents[pos].PointDouble;
                contour.Add(context.Dequantize(lastPoint));
                pos = NextPos(pos, processed, iterationMap, out bool found);
                if (!found)
                {
                    break;
                }
            }
            while (useFloating ? resultEvents[pos].PointDouble != initialDouble : resultEvents[pos].Point != initial);

            if (useFloating ? lastPointDouble != initialDouble : lastPoint != initial)
            {
                contour.Add(context.Dequantize(initial));
            }

            result.Add(contour);
        }

        Polygon polygon = [];

        for (int i = 0; i < result.Count; i++)
        {
            Contour contour = result[i];
            if (contour.IsExternal)
            {
                // The exterior ring goes first
                polygon.Add(contour);

                // Followed by holes if any
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
        bool useFloating = PolygonUtilities.UseFloatingScale;

        int i = 0;
        while (i < data.Count)
        {
            SweepEvent xRef = data[i];
            Vertex64 xPoint = xRef.Point;
            Vertex xPointDouble = xRef.PointDouble;

            // Find index range of R events
            int rFrom = i;
            while (i < data.Count && PointsEqual(xPoint, xPointDouble, data[i], useFloating) && !data[i].Left)
            {
                i++;
            }

            int rUptoExclusive = i;

            // Find index range of L events
            int lFrom = i;
            while (i < data.Count && PointsEqual(xPoint, xPointDouble, data[i], useFloating))
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

        static bool PointsEqual(in Vertex64 fixedPoint, in Vertex pointDouble, SweepEvent sweepEvent, bool useFloating)
            => useFloating ? sweepEvent.PointDouble == pointDouble : sweepEvent.Point == fixedPoint;
    }

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
    private static Contour InitializeContourFromContext(SweepEvent sweepEvent, Polygon polygon, int contourId)
    {
        Contour contour = new();

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
                // We are outside: This contour is an exterior contour of the same depth.
                contour.ParentIndex = null;
                contour.Depth = polygon[lowerContourId].Depth;
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
    /// Finds the next unprocessed position in the result events, either forward or backward,
    /// starting from the given position.
    /// </summary>
    /// <param name="pos">The current position in the result events.</param>
    /// <param name="processed">A list indicating whether each event at the corresponding index has been processed.</param>
    /// <param name="iterationMap">A precomputed map that indicates the next position to check for unprocessed events.</param>
    /// <param name="found">A boolean indicating whether an unprocessed event was found.</param>
    /// <returns>The index of the next unprocessed position.</returns>
    /// <remarks>
    /// This method searches forward from the current position until it finds an unprocessed event with
    /// a different point or reaches the end of the list. If no such event is found, it searches backward
    /// until it finds an unprocessed event.
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

    private sealed record ContourInfoDouble(List<Vertex> Vertices, Vertex MinVertex, double Area);
}

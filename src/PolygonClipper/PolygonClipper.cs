// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Diagnostics.CodeAnalysis;

namespace PolygonClipper;

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

    /// <summary>
    /// Initializes a new instance of the <see cref="PolygonClipper"/> class.
    /// </summary>
    /// <param name="subject">The subject polygon.</param>
    /// <param name="clip">The clipping polygon.</param>
    /// <param name="operation">The operation type.</param>
    public PolygonClipper(Polygon subject, Polygon clip, BooleanOperation operation)
    {
        this.subject = subject;
        this.clipping = clip;
        this.operation = operation;
    }

    /// <summary>
    /// Computes the intersection of two polygons. The resulting polygon contains the regions that are common to both input polygons.
    /// </summary>
    /// <param name="subject">The subject polygon.</param>
    /// <param name="clip">The clipping polygon.</param>
    /// <returns>A new <see cref="Polygon"/> representing the intersection of the two polygons.</returns>
    public static Polygon Intersection(Polygon subject, Polygon clip)
    {
        PolygonClipper clipper = new(subject, clip, BooleanOperation.Intersection);
        return clipper.Run();
    }

    /// <summary>
    /// Computes the union of two polygons. The resulting polygon contains the combined regions of the two input polygons.
    /// </summary>
    /// <param name="subject">The subject polygon.</param>
    /// <param name="clip">The clipping polygon.</param>
    /// <returns>A new <see cref="Polygon"/> representing the union of the two polygons.</returns>
    public static Polygon Union(Polygon subject, Polygon clip)
    {
        PolygonClipper clipper = new(subject, clip, BooleanOperation.Union);
        return clipper.Run();
    }

    /// <summary>
    /// Computes the difference of two polygons. The resulting polygon contains the regions of the subject polygon
    /// that are not shared with the clipping polygon.
    /// </summary>
    /// <param name="subject">The subject polygon.</param>
    /// <param name="clip">The clipping polygon.</param>
    /// <returns>A new <see cref="Polygon"/> representing the difference between the two polygons.</returns>
    public static Polygon Difference(Polygon subject, Polygon clip)
    {
        PolygonClipper clipper = new(subject, clip, BooleanOperation.Difference);
        return clipper.Run();
    }

    /// <summary>
    /// Computes the symmetric difference (XOR) of two polygons. The resulting polygon contains the regions that belong
    /// to either one of the input polygons but not to their intersection.
    /// </summary>
    /// <param name="subject">The subject polygon.</param>
    /// <param name="clip">The clipping polygon.</param>
    /// <returns>A new <see cref="Polygon"/> representing the symmetric difference of the two polygons.</returns>
    public static Polygon Xor(Polygon subject, Polygon clip)
    {
        PolygonClipper clipper = new(subject, clip, BooleanOperation.Xor);
        return clipper.Run();
    }

    /// <summary>
    /// Executes the boolean operation using the sweep line algorithm.
    /// </summary>
    /// <returns>The resulting <see cref="Polygon"/>.</returns>
    public Polygon Run()
    {
        // Compute bounding boxes for optimization steps 1 and 2
        Polygon subject = this.subject;
        Polygon clipping = this.clipping;
        BooleanOperation operation = this.operation;

        // Check for trivial cases that can be resolved without sweeping
        if (TryTrivialOperationForEmptyPolygons(subject, clipping, operation, out Polygon? result))
        {
            return result;
        }

        // Process all segments in the subject polygon
        Vertex min = new(double.PositiveInfinity);
        Vertex max = new(double.NegativeInfinity);
        StablePriorityQueue<SweepEvent, SweepEventComparer> eventQueue = new(new SweepEventComparer());
        int contourId = 0;
        for (int i = 0; i < subject.ContourCount; i++)
        {
            Contour contour = subject[i];
            contourId++;
            for (int j = 0; j < contour.VertexCount - 1; j++)
            {
                ProcessSegment(contourId, contour.Segment(j), PolygonType.Subject, eventQueue, ref min, ref max);
            }
        }

        Box2 subjectBB = new(min, max);

        // Process all segments in the clipping polygon
        min = new Vertex(double.PositiveInfinity);
        max = new Vertex(double.NegativeInfinity);
        for (int i = 0; i < clipping.ContourCount; i++)
        {
            Contour contour = clipping[i];

            for (int j = 0; j < contour.VertexCount - 1; j++)
            {
                ProcessSegment(contourId, contour.Segment(j), PolygonType.Clipping, eventQueue, ref min, ref max);
            }
        }

        Box2 clippingBB = new(min, max);
        if (TryTrivialOperationForNonOverlappingBoundingBoxes(subject, clipping, subjectBB, clippingBB, operation, out result))
        {
            return result;
        }

        // Sweep line algorithm: process events in the priority queue
        List<SweepEvent> sortedEvents = [];
        StatusLine statusLine = new();
        SweepEventComparer comparer = eventQueue.Comparer;
        double subjectMaxX = subjectBB.Max.X;
        double minMaxX = Vertex.Min(subjectBB.Max, clippingBB.Max).X;

        SweepEvent? prevEvent;
        SweepEvent? nextEvent;
        int ii = 0;
        while (eventQueue.Count > 0)
        {
            SweepEvent sweepEvent = eventQueue.Dequeue();

            sortedEvents.Add(sweepEvent);
            ii++;

            // Optimization: skip further processing if intersection is impossible
            if ((operation == BooleanOperation.Intersection && sweepEvent.Point.X > minMaxX) ||
                (operation == BooleanOperation.Difference && sweepEvent.Point.X > subjectMaxX))
            {
                return ConnectEdges(sortedEvents, comparer);
            }

            if (sweepEvent.Left)
            {
                // Insert the event into the status line and get neighbors
                int it = sweepEvent.PosSL = statusLine.Insert(sweepEvent);
                prevEvent = statusLine.Prev(it);
                nextEvent = statusLine.Next(it);

                // Compute fields for the current event
                ComputeFields(sweepEvent, prevEvent, operation);

                // Check intersection with the next neighbor
                if (nextEvent != null)
                {
                    // Check intersection with the next neighbor
                    if (PossibleIntersection(sweepEvent, nextEvent, eventQueue) == 2)
                    {
                        ComputeFields(sweepEvent, prevEvent, operation);
                        ComputeFields(nextEvent, sweepEvent, operation);
                    }
                }

                // Check intersection with the previous neighbor
                if (prevEvent != null)
                {
                    // Check intersection with the previous neighbor
                    if (PossibleIntersection(prevEvent, sweepEvent, eventQueue) == 2)
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
                    _ = PossibleIntersection(prevEvent, nextEvent, eventQueue);
                }

                statusLine.RemoveAt(it);
            }
        }

        // Connect edges after processing all events
        return ConnectEdges(sortedEvents, comparer);
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

        if (subject.ContourCount * clipping.ContourCount == 0)
        {
            if (operation == BooleanOperation.Intersection)
            {
                result = new();
                return true;
            }

            if (operation == BooleanOperation.Difference)
            {
                result = subject;
                return true;
            }

            if (operation is BooleanOperation.Union or BooleanOperation.Xor)
            {
                result = subject.ContourCount == 0 ? clipping : subject;
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
                result = new();
                return true;
            }

            // The bounding boxes do not overlap
            if (operation == BooleanOperation.Difference)
            {
                result = subject;
                return true;
            }

            if (operation is BooleanOperation.Union or BooleanOperation.Xor)
            {
                result = subject;
                result.Join(clipping);
                return true;
            }
        }

        return false;
    }

    /// <summary>
    /// Processes a segment by generating sweep events for its endpoints and adding them to the event queue.
    /// </summary>
    /// <param name="contourId">The identifier of the contour to which the segment belongs.</param>
    /// <param name="s">The segment to process.</param>
    /// <param name="pt">The polygon type to which the segment belongs.</param>
    /// <param name="eventQueue">The event queue to add the generated events to.</param>
    /// <param name="min">The minimum vertex of the bounding box.</param>
    /// <param name="max">The maximum vertex of the bounding box.</param>
    private static void ProcessSegment(
        int contourId,
        Segment s,
        PolygonType pt,
        StablePriorityQueue<SweepEvent, SweepEventComparer> eventQueue,
        ref Vertex min,
        ref Vertex max)
    {
        if (s.Source == s.Target)
        {
            // Skip degenerate zero-length segments.
            return;
        }

        // Create sweep events for the endpoints of the segment
        SweepEvent e1 = new(s.Source, true, pt);
        SweepEvent e2 = new(s.Target, true, e1, pt);
        e1.OtherEvent = e2;
        e1.ContourId = e2.ContourId = contourId;

        // Determine which endpoint is the left endpoint
        if (eventQueue.Comparer.Compare(e1, e2) < 0)
        {
            e2.Left = false;
        }
        else
        {
            e1.Left = false;
        }

        min = Vertex.Min(min, s.Min);
        max = Vertex.Max(max, s.Max);

        // Add the events to the event queue
        eventQueue.Enqueue(e1);
        eventQueue.Enqueue(e2);
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
            le.OtherInOut = prev.Vertical() ? !prev.InOut : prev.InOut;
        }

        // Compute PrevInResult field
        if (prev != null)
        {
            le.PrevInResult = (!InResult(prev, operation) || prev.Vertical())
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
        StablePriorityQueue<SweepEvent, SweepEventComparer> eventQueue)
    {
        if (le1.OtherEvent == null || le2.OtherEvent == null)
        {
            // No intersection possible.
            return 0;
        }

        // Point intersections
        int nIntersections = PolygonUtilities.FindIntersection(
            le1.Segment(),
            le2.Segment(),
            out Vertex ip1,
            out Vertex _); // Currently unused but could be used to detect collinear overlapping segments

        if (nIntersections == 0)
        {
            // No intersection
            return 0;
        }

        // Ignore intersection if it occurs at the exact left or right endpoint of both segments
        if (nIntersections == 1 &&
            (le1.Point == le2.Point || le1.OtherEvent.Point == le2.OtherEvent.Point))
        {
            // Line segments intersect at an endpoint of both line segments
            return 0;
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
            // If the intersection point is not an endpoint of le1 segment.
            if (le1.Point != ip1 && le1.OtherEvent.Point != ip1)
            {
                DivideSegment(le1, ip1, eventQueue, comparer);
            }

            // If the intersection point is not an endpoint of le2 segment.
            if (le2.Point != ip1 && le2.OtherEvent.Point != ip1)
            {
                DivideSegment(le2, ip1, eventQueue, comparer);
            }

            return 1;
        }

        // The line segments associated with le1 and le2 overlap.
        // TODO: Rewrite this to avoid allocation.
        List<SweepEvent> events = new(4);
        bool leftCoincide = le1.Point == le2.Point;
        bool rightCoincide = le1.OtherEvent.Point == le2.OtherEvent.Point;

        // Populate the events
        if (!leftCoincide)
        {
            if (comparer.Compare(le1, le2) > 0)
            {
                events.Add(le2);
                events.Add(le1);
            }
            else
            {
                events.Add(le1);
                events.Add(le2);
            }
        }

        if (!rightCoincide)
        {
            if (comparer.Compare(le1.OtherEvent, le2.OtherEvent) > 0)
            {
                events.Add(le2.OtherEvent);
                events.Add(le1.OtherEvent);
            }
            else
            {
                events.Add(le1.OtherEvent);
                events.Add(le2.OtherEvent);
            }
        }

        // Handle leftCoincide case
        if (leftCoincide)
        {
            le2.EdgeType = EdgeType.NonContributing;
            le1.EdgeType = (le2.InOut == le1.InOut)
                ? EdgeType.SameTransition
                : EdgeType.DifferentTransition;

            if (leftCoincide && !rightCoincide)
            {
                DivideSegment(events[1].OtherEvent, events[0].Point, eventQueue, comparer);
            }

            return 2;
        }

        // Handle the rightCoincide case
        if (rightCoincide)
        {
            DivideSegment(events[0], events[1].Point, eventQueue, comparer);
            return 3;
        }

        // Handle general overlapping case
        if (events[0] != events[3].OtherEvent)
        {
            DivideSegment(events[0], events[1].Point, eventQueue, comparer);
            DivideSegment(events[1], events[2].Point, eventQueue, comparer);
            return 3;
        }

        // One segment fully contains the other
        DivideSegment(events[0], events[1].Point, eventQueue, comparer);
        DivideSegment(events[3].OtherEvent, events[2].Point, eventQueue, comparer);
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
        Vertex p,
        StablePriorityQueue<SweepEvent, SweepEventComparer> eventQueue,
        SweepEventComparer comparer)
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
        // it is impossible for r/l to fall outside the interval. This leaves the corner cases:
        //
        //  1. r.x == se_l.x and r.y < se_l.y: This corresponds to the case where the first
        //     sub-segment becomes a perfectly vertical line. The problem is that vertical
        //     segments always have to be processed from bottom to top consistency. The
        //     theoretically correct event order would be r first (bottom), se_l later (top).
        //     However, se_l is the event just being processed, so there is no (easy) way of
        //     processing r before se_l. The easiest solution to the problem is to avoid it,
        //     by incrementing inter.x by one ULP.
        //  2. l.x == re.x and l.y > re.y: This corresponds to the case where the second
        //     sub-segment becomes a perfectly vertical line, and because of the bottom-to-top
        //     convention for vertical segment, the order of l and re must be swapped.
        //     In this case swapping is not a problem, because both events are in the future.
        //
        // See also: https://github.com/21re/rust-geo-booleanop/pull/11

        // Prevent from corner case 1
        if (p.X == le.Point.X && p.Y < le.Point.Y)
        {
            // The files are different in the two reference repositories but both fail.
            p = new Vertex(p.X.NextAfter(double.PositiveInfinity), p.Y);
        }

        // Create the right event for the left segment (new right endpoint)
        SweepEvent r = new(p, false, le, le.PolygonType);

        // Create the left event for the right segment (new left endpoint)
        SweepEvent l = new(p, true, re, le.PolygonType);

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
    private static Polygon ConnectEdges(List<SweepEvent> sortedEvents, SweepEventComparer comparer)
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

        Polygon result = new();
        Span<bool> processed = new bool[resultEvents.Count];
        for (int i = 0; i < resultEvents.Count; i++)
        {
            if (processed[i])
            {
                continue;
            }

            int contourId = result.ContourCount;
            Contour contour = InitializeContourFromContext(resultEvents[i], result, contourId);

            int pos = i;
            Vertex initial = resultEvents[i].Point;
            contour.AddVertex(initial);

            // Main loop to process the contour
            while (true)
            {
                MarkProcessed(resultEvents[pos], processed, pos, contourId);
                pos = resultEvents[pos].Pos;

                MarkProcessed(resultEvents[pos], processed, pos, contourId);

                contour.AddVertex(resultEvents[pos].Point);
                pos = NextPos(pos, processed, iterationMap, out bool found);
                if (!found)
                {
                    break;
                }

                if (resultEvents[pos].Point == initial)
                {
                    break;
                }
            }

            result.Push(contour);
        }

        Polygon polygon = new();

        for (int i = 0; i < result.ContourCount; i++)
        {
            Contour contour = result[i];
            if (contour.IsExternal)
            {
                // The exterior ring goes first
                polygon.Push(contour);

                // Followed by holes if any
                for (int j = 0; j < contour.HoleCount; j++)
                {
                    int holeId = contour.GetHoleIndex(j);
                    polygon.Push(result[holeId]);
                }
            }
        }

        return polygon;
    }

    private static ReadOnlySpan<int> PrecomputeIterationOrder(List<SweepEvent> data)
    {
        Span<int> map = new Span<int>(new int[data.Count]);

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

                if (lowerContour.HoleOf != null)
                {
                    // The lower contour is a hole: Connect the new contour as a hole to its parent and use the same depth.
                    int parentContourId = lowerContour.HoleOf.Value;
                    polygon[parentContourId].AddHoleIndex(contourId);
                    contour.HoleOf = parentContourId;
                    contour.Depth = polygon[lowerContourId].Depth;
                }
                else
                {
                    // The lower contour is an exterior contour: Connect the new contour as a hole and increment depth.
                    polygon[lowerContourId].AddHoleIndex(contourId);
                    contour.HoleOf = lowerContourId;
                    contour.Depth = polygon[lowerContourId].Depth + 1;
                }
            }
            else
            {
                // We are outside: This contour is an exterior contour of the same depth.
                contour.HoleOf = null;
                contour.Depth = polygon[lowerContourId].Depth;
            }
        }
        else
        {
            // There is no "previous in result" event: This contour is an exterior contour with depth 0.
            contour.HoleOf = null;
            contour.Depth = 0;
        }

        return contour;
    }

    /// <summary>
    /// Finds the next unprocessed position in the result events, either forward or backward,
    /// starting from the given position.
    /// </summary>
    /// <param name="pos">The current position in the result events.</param>
    /// <param name="resultEvents">The list of sweep events representing result segments.</param>
    /// <param name="processed">A list indicating whether each event at the corresponding index has been processed.</param>
    /// <param name="originalPos">The original position to return if no unprocessed event is found.</param>
    /// <param name="iterationMap"></param>
    /// <param name="found"></param>
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
}

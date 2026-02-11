// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

#nullable disable

using System.Runtime.CompilerServices;

namespace SixLabors.PolygonClipper;

/// <summary>
/// Represents a sweep.
/// </summary>
internal sealed class SweepEvent
{
    /// <summary>
    /// Initializes a new instance of the <see cref="SweepEvent"/> class.
    /// </summary>
    /// <param name="point">The point associated with the event.</param>
    /// <param name="left">Whether the point is the left endpoint of the segment.</param>
    /// <param name="otherEvent">The event associated with the other endpoint of the segment.</param>
    /// <param name="polygonType">The polygon type to which the segment belongs.</param>
    /// <param name="edgeType">The type of the edge. Default is <see cref="EdgeType.Normal"/>.</param>
    public SweepEvent(
        Vertex64 point,
        bool left,
        SweepEvent otherEvent,
        PolygonType polygonType = PolygonType.Subject,
        EdgeType edgeType = EdgeType.Normal)
    {
        this.Point = point;
        this.PointDouble = new Vertex(point.X, point.Y);
        this.Left = left;
        this.OtherEvent = otherEvent;
        this.PolygonType = polygonType;
        this.EdgeType = edgeType;
        this.SegmentSource = point;
        this.SegmentTarget = point;
    }

    /// <summary>
    /// Initializes a new instance of the <see cref="SweepEvent"/> class.
    /// </summary>
    /// <param name="point">The point associated with the event.</param>
    /// <param name="left">Whether the point is the left endpoint of the segment.</param>
    /// <param name="polygonType">The polygon type to which the segment belongs.</param>
    public SweepEvent(Vertex64 point, bool left, PolygonType polygonType = PolygonType.Subject)
    {
        this.Point = point;
        this.PointDouble = new Vertex(point.X, point.Y);
        this.Left = left;
        this.PolygonType = polygonType;
        this.EdgeType = EdgeType.Normal;
        this.SegmentSource = point;
        this.SegmentTarget = point;
    }

    /// <summary>
    /// Initializes a new instance of the <see cref="SweepEvent"/> class.
    /// </summary>
    /// <param name="point">The point associated with the event.</param>
    /// <param name="left">Whether the point is the left endpoint of the segment.</param>
    /// <param name="contourId">The ID of the contour to which the event belongs.</param>
    public SweepEvent(Vertex64 point, bool left, int contourId)
    {
        this.Point = point;
        this.PointDouble = new Vertex(point.X, point.Y);
        this.Left = left;
        this.ContourId = contourId;
        this.PolygonType = PolygonType.Subject;
        this.EdgeType = EdgeType.Normal;
        this.SegmentSource = point;
        this.SegmentTarget = point;
    }

    /// <summary>
    /// Gets the point associated with the event.
    /// </summary>
    public Vertex64 Point { get; }

    /// <summary>
    /// Gets or sets the double-precision point associated with the event.
    /// </summary>
    public Vertex PointDouble { get; set; }

    /// <summary>
    /// Gets or sets a value indicating whether the point is the
    /// left (source) endpoint of the segment (p, other->p).
    /// </summary>
    public bool Left { get; set; }

    /// <summary>
    /// Gets or sets the ID of the contour to which the event belongs.
    /// </summary>
    public int ContourId { get; set; }

    /// <summary>
    /// Gets or sets the winding direction delta for this edge (+1 or -1).
    /// Based on the contour direction through this edge. Once set, this value
    /// is stable and does not change when segments are split.
    /// </summary>
    public int WindDx { get; set; }

    /// <summary>
    /// Gets or sets the identifier for the original contour segment.
    /// </summary>
    public int SegmentId { get; set; } = -1;

    /// <summary>
    /// Gets or sets a value indicating whether this event's point is the source (start)
    /// of the segment in the original contour direction. This is needed to compute
    /// winding contributions correctly for the positive fill rule.
    /// </summary>
    public bool IsContourSource { get; set; }

    /// <summary>
    /// Gets or sets the directed segment source point.
    /// </summary>
    public Vertex64 SegmentSource { get; set; }

    /// <summary>
    /// Gets or sets the directed segment target point.
    /// </summary>
    public Vertex64 SegmentTarget { get; set; }

    /// <summary>
    /// Gets index of the polygon to which the associated segment belongs to;
    /// </summary>
    public PolygonType PolygonType { get; }

    /// <summary>
    /// Gets or sets the type of the edge.
    /// </summary>
    public EdgeType EdgeType { get; set; }

    /// <summary>
    /// Gets or sets the event associated to the other endpoint of the segment.
    /// </summary>
    public SweepEvent OtherEvent { get; set; }

    /// <summary>
    /// Gets or sets a value indicating whether the segment (p, other->p) represent an
    /// inside-outside transition in the polygon for a vertical ray from (p.x, -infinite)
    /// that crosses the segment.
    /// </summary>
    public bool InOut { get; set; }

    /// <summary>
    /// Gets or sets a value indicating whether the inOut transition for the segment from
    /// the other polygon preceding this segment in the sweep line.
    /// </summary>
    public bool OtherInOut { get; set; }

    /// <summary>
    /// Gets or sets the sorted sweep events. Only used in "left" events.
    /// Position of the event (segment) in SL (status line).
    /// </summary>
    public int PosSL { get; set; }

    /// <summary>
    /// Gets or sets the previous segment in the sweep line  belonging to the result of the
    /// boolean operation.
    /// </summary>
    public SweepEvent PrevInResult { get; set; }

    /// <summary>
    /// Gets or sets the transition state of the event in the result.
    /// </summary>
    public ResultTransition ResultTransition { get; set; }

    /// <summary>
    /// Gets a value indicating whether the event contributes to the result.
    /// </summary>
    public bool InResult => this.ResultTransition != ResultTransition.Neutral;

    /// <summary>
    /// Gets or sets the position of the event in the sorted events.
    /// </summary>
    public int Pos { get; set; }

    /// <summary>
    /// Gets or sets a value indicating whether the event is a result in-out transition.
    /// </summary>
    public bool ResultInOut { get; set; }

    /// <summary>
    /// Gets or sets the output contour ID associated with this contour.
    /// </summary>
    public int OutputContourId { get; set; }

    /// <summary>
    /// Is the line segment (point, otherEvent->point) below point p.
    /// </summary>
    /// <param name="p">The point to check against.</param>
    /// <returns>
    /// <see langword="true"/> if the line segment is below the point; otherwise <see langword="false"/>.
    /// </returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool IsBelow(in Vertex64 p)
        => this.Left
        ? PolygonUtilities.CrossSign(this.Point, this.OtherEvent.Point, p) > 0
        : PolygonUtilities.CrossSign(this.OtherEvent.Point, this.Point, p) > 0;

    /// <summary>
    /// Is the line segment (point, otherEvent->point) above point p.
    /// </summary>
    /// <param name="p">The point to check against.</param>
    /// <returns>
    /// <see langword="true"/> if the line segment is above the point; otherwise <see langword="false"/>.
    /// </returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool IsAbove(in Vertex64 p) => !this.IsBelow(p);

    /// <summary>
    /// Is the line segment (point, otherEvent->point) a vertical line segment.
    /// </summary>
    /// <returns>
    /// <see langword="true"/> if the line segment is vertical; otherwise <see langword="false"/>.
    /// </returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool IsVertical()
    {
        if (PolygonUtilities.UseFloatingScale)
        {
            return this.PointDouble.X == this.OtherEvent.PointDouble.X;
        }

        return this.Point.X == this.OtherEvent.Point.X;
    }

    /// <summary>
    /// Determines if this sweep event comes before another sweep event.
    /// </summary>
    /// <param name="other">The other sweep event to compare with.</param>
    /// <returns>
    /// <see langword="true"/> if this event comes before the other; otherwise <see langword="false"/>.
    /// </returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool IsBefore(SweepEvent other)
    {
        if (PolygonUtilities.UseFloatingScale)
        {
            if (this.PointDouble.X != other.PointDouble.X)
            {
                return this.PointDouble.X < other.PointDouble.X;
            }

            return this.PointDouble.Y < other.PointDouble.Y;
        }

        if (this.Point.X != other.Point.X)
        {
            return this.Point.X < other.Point.X;
        }

        return this.Point.Y < other.Point.Y;
    }

    /// <summary>
    /// Returns the segment associated with the sweep event.
    /// </summary>
    /// <returns>The <see cref="Segment"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Segment GetSegment() => new(this.Point, this.OtherEvent.Point);
}

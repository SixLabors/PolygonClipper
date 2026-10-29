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
    // Live-only state is allocated and pooled with source intervals. Output
    // history needs only endpoint, transition, and predecessor data, so it must
    // not pay for source support, bundle ledgers, and active-status bookkeeping.
    private IntervalState interval;

    /// <summary>
    /// Initializes a new instance of the <see cref="SweepEvent"/> class.
    /// </summary>
    /// <param name="point">The point associated with the event.</param>
    /// <param name="left">Whether the point is the left endpoint of the segment.</param>
    /// <param name="otherEvent">The event associated with the other endpoint of the segment.</param>
    /// <param name="polygonType">The polygon type to which the segment belongs.</param>
    /// <param name="edgeType">The type of the edge. Default is <see cref="EdgeType.Normal"/>.</param>
    public SweepEvent(
        Vertex point,
        bool left,
        SweepEvent otherEvent,
        PolygonType polygonType = PolygonType.Subject,
        EdgeType edgeType = EdgeType.Normal)
    {
        this.Point = point;
        this.Left = left;
        this.OtherEvent = otherEvent;
        this.PolygonType = polygonType;
        this.EdgeType = edgeType;
    }

    /// <summary>
    /// Initializes a new instance of the <see cref="SweepEvent"/> class.
    /// </summary>
    /// <param name="point">The point associated with the event.</param>
    /// <param name="left">Whether the point is the left endpoint of the segment.</param>
    /// <param name="polygonType">The polygon type to which the segment belongs.</param>
    public SweepEvent(Vertex point, bool left, PolygonType polygonType = PolygonType.Subject)
    {
        this.Point = point;
        this.Left = left;
        this.PolygonType = polygonType;
        this.EdgeType = EdgeType.Normal;
    }

    /// <summary>
    /// Initializes a new instance of the <see cref="SweepEvent"/> class.
    /// </summary>
    /// <param name="point">The point associated with the event.</param>
    /// <param name="left">Whether the point is the left endpoint of the segment.</param>
    /// <param name="contourId">The ID of the contour to which the event belongs.</param>
    public SweepEvent(Vertex point, bool left, int contourId)
    {
        this.Point = point;
        this.Left = left;
        this.ContourId = contourId;
        this.PolygonType = PolygonType.Subject;
        this.EdgeType = EdgeType.Normal;
    }

    /// <summary>
    /// Gets or sets the point associated with the event.
    /// </summary>
    public Vertex Point { get; set; }

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
    /// Gets index of the polygon to which the associated segment belongs to;
    /// </summary>
    public PolygonType PolygonType { get; }

    /// <summary>
    /// Gets or sets the current terminal point of this live interval.
    /// </summary>
    public Vertex End
    {
        get => this.OtherEvent.Point;
        set => this.OtherEvent.Point = value;
    }

    /// <summary>
    /// Gets or sets the next source contribution endpoint.
    /// </summary>
    public Vertex SourceEnd { get => this.interval.SourceEnd; set => this.interval.SourceEnd = value; }

    /// <summary>
    /// Gets or sets the original supporting segment, preserved across rounded splits.
    /// </summary>
    public Segment Support { get => this.interval.Support; set => this.interval.Support = value; }

    /// <summary>
    /// Gets or sets the signed subject and clip contributions of this interval.
    /// </summary>
    public PolygonClipper.WindingCount Delta { get => this.interval.Delta; set => this.interval.Delta = value; }

    /// <summary>
    /// Gets or sets the winding immediately above this live interval.
    /// </summary>
    public PolygonClipper.WindingCount Above { get => this.interval.Above; set => this.interval.Above = value; }

    /// <summary>
    /// Gets or sets the endpoint changes retained for a coincident bundle.
    /// </summary>
    public PriorityQueue<PolygonClipper.WindingCount, Vertex> Ends { get => this.interval.Ends; set => this.interval.Ends = value; }

    /// <summary>
    /// Gets or sets the immutable output record for the current contributing piece.
    /// </summary>
    public SweepEvent Boundary { get => this.interval.Boundary; set => this.interval.Boundary = value; }

    /// <summary>
    /// Gets or sets the number of result intervals at or below this status entry.
    /// </summary>
    public int ResultRank { get => this.interval.ResultRank; set => this.interval.ResultRank = value; }

    /// <summary>
    /// Gets or sets the generation identifying this piece in queued endpoint records.
    /// </summary>
    public int Version { get => this.interval.Version; set => this.interval.Version = value; }

    /// <summary>
    /// Gets or sets a value indicating whether this interval is active.
    /// </summary>
    public bool Active { get => this.interval.Active; set => this.interval.Active = value; }

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
    public bool IsBelow(in Vertex p)
        => this.Left
        ? PolygonUtilities.SignedArea(this.Point, this.OtherEvent.Point, p) > 0D
        : PolygonUtilities.SignedArea(this.OtherEvent.Point, this.Point, p) > 0D;

    /// <summary>
    /// Is the line segment (point, otherEvent->point) above point p.
    /// </summary>
    /// <param name="p">The point to check against.</param>
    /// <returns>
    /// <see langword="true"/> if the line segment is above the point; otherwise <see langword="false"/>.
    /// </returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool IsAbove(in Vertex p) => !this.IsBelow(p);

    /// <summary>
    /// Is the line segment (point, otherEvent->point) a vertical line segment.
    /// </summary>
    /// <returns>
    /// <see langword="true"/> if the line segment is vertical; otherwise <see langword="false"/>.
    /// </returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool IsVertical() => this.Point.X == this.OtherEvent.Point.X;

    /// <summary>
    /// Determines if this sweep event comes before another sweep event.
    /// </summary>
    /// <param name="other">The other sweep event to compare with.</param>
    /// <returns>
    /// <see langword="true"/> if this event comes before the other; otherwise <see langword="false"/>.
    /// </returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool IsBefore(SweepEvent other) => SweepEventComparer.Instance.Compare(this.Point, other.Point) < 0;

    /// <summary>
    /// Returns the segment associated with the sweep event.
    /// </summary>
    /// <returns>The <see cref="Segment"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public Segment GetSegment() => new(this.Point, this.OtherEvent.Point);

    /// <summary>
    /// Tests incidence on the noded interval and its original supporting segment.
    /// </summary>
    /// <param name="point">The queued event point.</param>
    /// <returns>Whether this interval participates in the point event.</returns>
    public bool ContainsPoint(Vertex point)
    {
        // Rounded split endpoints belong to their interval. For other points,
        // use the original support so a rounded split cannot rotate the line
        // into a nearby source vertex. Neighbor discovery closes rounded junctions.
        return SweepEventComparer.Instance.Compare(this.Point, point) <= 0 && SweepEventComparer.Instance.Compare(point, this.End) <= 0 &&
            (point == this.Point || point == this.End ||
             PolygonUtilities.SignedArea(this.Support.Source, this.Support.Target, point) == 0);
    }

    /// <summary>
    /// Initializes a reusable directed source interval before it enters the event queue.
    /// </summary>
    /// <param name="start">The canonical starting point.</param>
    /// <param name="end">The source endpoint.</param>
    /// <param name="id">The stable interval identity.</param>
    /// <param name="delta">The signed operand contributions.</param>
    public void InitializeInterval(Vertex start, Vertex end, int id, PolygonClipper.WindingCount delta)
    {
        this.interval ??= new IntervalState();
        this.Point = start;
        this.End = end;
        this.ContourId = id;
        this.SourceEnd = end;
        this.Support = new Segment(start, end);
        this.Delta = delta;
        this.Above = default;
        this.Ends = null;
        this.Boundary = null;
        this.Version = this.ResultRank = this.PosSL = 0;
        this.Active = false;
    }

    /// <summary>
    /// Initializes an output-history record without allocating live interval state.
    /// </summary>
    /// <param name="point">The boundary endpoint.</param>
    /// <param name="left">Whether this is the canonical starting endpoint.</param>
    /// <param name="other">The paired endpoint, completed when its position is known.</param>
    /// <param name="transition">The filled-side transition of this boundary.</param>
    /// <param name="previous">The immutable boundary predecessor.</param>
    /// <param name="id">The source interval identity.</param>
    public void InitializeBoundary(
        Vertex point,
        bool left,
        SweepEvent other,
        ResultTransition transition,
        SweepEvent previous,
        int id)
    {
        this.Point = point;
        this.Left = left;
        this.OtherEvent = other;
        this.ResultTransition = transition;
        this.PrevInResult = previous;
        this.ContourId = id;
        this.OutputContourId = this.Pos = 0;
    }

    /// <summary>
    /// Stores the live interval state that is unnecessary in completed boundary history.
    /// </summary>
    private sealed class IntervalState
    {
        /// <summary>
        /// Gets or sets the next source contribution endpoint.
        /// </summary>
        public Vertex SourceEnd { get; set; }

        /// <summary>
        /// Gets or sets the original supporting segment, preserved across rounded splits.
        /// </summary>
        public Segment Support { get; set; }

        /// <summary>
        /// Gets or sets the signed subject and clip contributions of this interval.
        /// </summary>
        public PolygonClipper.WindingCount Delta { get; set; }

        /// <summary>
        /// Gets or sets the winding immediately above this live interval.
        /// </summary>
        public PolygonClipper.WindingCount Above { get; set; }

        /// <summary>
        /// Gets or sets the endpoint changes retained for a coincident bundle.
        /// </summary>
        public PriorityQueue<PolygonClipper.WindingCount, Vertex> Ends { get; set; }

        /// <summary>
        /// Gets or sets the immutable output record for the current contributing piece.
        /// </summary>
        public SweepEvent Boundary { get; set; }

        /// <summary>
        /// Gets or sets the number of result intervals at or below this status entry.
        /// </summary>
        public int ResultRank { get; set; }

        /// <summary>
        /// Gets or sets the generation identifying this piece in queued endpoint records.
        /// </summary>
        public int Version { get; set; }

        /// <summary>
        /// Gets or sets a value indicating whether this interval is active.
        /// </summary>
        public bool Active { get; set; }
    }
}

// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System.Runtime.CompilerServices;

namespace SixLabors.PolygonClipper;

#pragma warning disable SA1201 // Elements should appear in the correct order

/// <summary>
/// Generates polygonal stroke geometry for contours using AGG-style joins and caps.
/// </summary>
/// <remarks>
/// This type performs two phases:
/// <list type="number">
/// <item><description>Expand each source contour into one or two stroke-side outlines with joins/caps.</description></item>
/// <item><description>Resolve generated overlaps/self-intersections using <see cref="SelfIntersectionRemover"/> with positive fill semantics.</description></item>
/// </list>
/// The emitted contours are closed before returning.
/// </remarks>
public sealed class PolygonStroker
{
    // Numerical tolerances used while collapsing near-duplicate source points and
    // while testing near-parallel line intersections.
    private const double VertexDistanceEpsilon = 1E-14D;
    private const double IntersectionEpsilon = 1E-30D;
    private const double Pi = Math.PI;
    private const double PiMul2 = Math.PI * 2D;

    // Scratch buffers reused across contours to keep per-call allocations down.
    private ArrayBuilder<Vertex> outVertices = new(1);
    private ArrayBuilder<StrokeVertexDistance> srcVertices = new(16);

    // Streaming-state fields used by the Accumulate() state machine.
    private int closed;
    private int outVertex;
    private Status prevStatus;
    private int srcVertex;
    private Status status;
    private double strokeWidth = 0.5D;
    private double widthAbs = 0.5D;
    private double widthEps = 0.5D / 1024D;
    private int widthSign = 1;

    /// <summary>
    /// Initializes a new instance of the <see cref="PolygonStroker"/> class with the specified stroke options.
    /// </summary>
    /// <param name="options">The stroke options.</param>
    /// <exception cref="ArgumentNullException">Thrown when <paramref name="options"/> is null.</exception>
    public PolygonStroker(StrokeOptions options)
    {
        ArgumentNullException.ThrowIfNull(options);
        this.LineJoin = options.LineJoin;
        this.InnerJoin = options.InnerJoin;
        this.LineCap = options.LineCap;
        this.MiterLimit = options.MiterLimit;
        this.InnerMiterLimit = options.InnerMiterLimit;
        this.ArcDetailScale = options.ArcDetailScale;
    }

    /// <summary>
    /// Internal state machine used by <see cref="Accumulate(ref Vertex)"/> to stream stroked output vertices.
    /// </summary>
    private enum Status
    {
        /// <summary>Initial setup and input normalization.</summary>
        Initial,

        /// <summary>Ready to emit the first command for the contour.</summary>
        Ready,

        /// <summary>Emit start-cap vertices for open contours.</summary>
        Cap1,

        /// <summary>Emit end-cap vertices for open contours.</summary>
        Cap2,

        /// <summary>Emit joins for the first stroke side.</summary>
        Outline1,

        /// <summary>Switch from first side to second side for closed paths.</summary>
        CloseFirst,

        /// <summary>Emit joins for the second stroke side.</summary>
        Outline2,

        /// <summary>Flush buffered vertices from the current join/cap computation.</summary>
        OutVertices,

        /// <summary>Emit end-poly marker for first stroke side.</summary>
        EndPoly1,

        /// <summary>Emit end-poly marker for second stroke side.</summary>
        EndPoly2,

        /// <summary>Stop emitting commands.</summary>
        Stop
    }

    /// <summary>
    /// Strokes <paramref name="polygon"/> with <paramref name="width"/> using optional
    /// <paramref name="options"/> and resolves overlaps via <see cref="SelfIntersectionRemover"/>
    /// with <see cref="FillRule.Positive"/>.
    /// </summary>
    /// <param name="polygon">Input polygon to stroke.</param>
    /// <param name="width">Stroke width.</param>
    /// <param name="options">
    /// Stroke options controlling joins, caps and approximation behavior.
    /// When null, default <see cref="StrokeOptions"/> are used.
    /// </param>
    /// <returns>Self-intersection resolved stroke polygon.</returns>
    /// <exception cref="ArgumentNullException">Thrown when <paramref name="polygon"/> is null.</exception>
    public static Polygon Stroke(Polygon polygon, double width, StrokeOptions? options = null)
    {
        PolygonStroker stroker = new(options ?? new StrokeOptions())
        {
            Width = width
        };

        return stroker.Stroke(polygon);
    }

    /// <summary>
    /// Strokes <paramref name="polygon"/> using this instance's configured options and width.
    /// </summary>
    /// <param name="polygon">Input polygon to stroke.</param>
    /// <returns>Self-intersection resolved stroke polygon.</returns>
    /// <exception cref="ArgumentNullException">Thrown when <paramref name="polygon"/> is null.</exception>
    public Polygon Stroke(Polygon polygon)
    {
        ArgumentNullException.ThrowIfNull(polygon);
        if (polygon.Count == 0)
        {
            return [];
        }

        Polygon allContours = new(Math.Max(2, polygon.Count * 2));
        for (int i = 0; i < polygon.Count; i++)
        {
            Contour contour = polygon[i];

            // Treat almost-closed contours as closed so joins are generated
            // consistently at the seam.
            bool isClosed = contour.Count > 2 || IsContourClosedForEmission(contour);
            Polygon stroked = this.ProcessPathToPolygon(contour, isClosed);
            if (stroked.Count > 0)
            {
                allContours.Join(stroked);
            }
        }

        if (allContours.Count == 0)
        {
            return [];
        }

        // Positive fill normalization resolves all overlaps from join/cap emission
        // and produces final render-ready stroke contours.
        return SelfIntersectionRemover.Process(allContours, FillRule.Positive);
    }

    /// <summary>
    /// Strokes <paramref name="polygon"/> after setting the current stroke width.
    /// </summary>
    /// <param name="polygon">Input polygon to stroke.</param>
    /// <param name="width">Stroke width.</param>
    /// <returns>Self-intersection resolved stroke polygon.</returns>
    public Polygon Stroke(Polygon polygon, double width)
    {
        this.Width = width;
        return this.Stroke(polygon);
    }

    /// <summary>
    /// Gets the miter limit used to clamp outer miter joins.
    /// </summary>
    public double MiterLimit { get; }

    /// <summary>
    /// Gets the inner miter limit used to clamp joins on acute interior angles.
    /// </summary>
    public double InnerMiterLimit { get; }

    /// <summary>
    /// Gets the tessellation detail scale used for round joins and round caps.
    /// Higher values produce more vertices and smoother curves.
    /// </summary>
    public double ArcDetailScale { get; }

    /// <summary>
    /// Gets the outer line join style used for stroking corners.
    /// </summary>
    public LineJoin LineJoin { get; }

    /// <summary>
    /// Gets the line cap style used for open path ends.
    /// </summary>
    public LineCap LineCap { get; }

    /// <summary>
    /// Gets the join style used for sharp interior angles.
    /// </summary>
    public InnerJoin InnerJoin { get; }

    /// <summary>
    /// Gets or sets the stroke width.
    /// </summary>
    /// <remarks>
    /// Positive values produce conventional outward stroking. Negative values are supported
    /// and flip the side orientation while preserving magnitude.
    /// </remarks>
    public double Width
    {
        get => this.strokeWidth * 2D;
        set
        {
            this.strokeWidth = value * 0.5D;
            if (this.strokeWidth < 0D)
            {
                this.widthAbs = -this.strokeWidth;
                this.widthSign = -1;
            }
            else
            {
                this.widthAbs = this.strokeWidth;
                this.widthSign = 1;
            }

            this.widthEps = this.strokeWidth / 1024D;
        }
    }

    /// <summary>
    /// Converts a single contour into stroked polygon contours.
    /// </summary>
    /// <param name="contour">The source contour.</param>
    /// <param name="isClosed">Whether the contour should be emitted as closed.</param>
    /// <returns>The generated stroked contour set for this input contour.</returns>
    private Polygon ProcessPathToPolygon(Contour contour, bool isClosed)
    {
        ArgumentNullException.ThrowIfNull(contour);

        int pointCount = contour.Count;
        if (pointCount < 2)
        {
            return [];
        }

        bool hasExplicitClosure = pointCount > 1 && contour[0] == contour[^1];
        if (isClosed && hasExplicitClosure)
        {
            // Keep one implicit closure path in the stroker state machine.
            // Duplicate terminal vertices are re-added at final contour emission.
            pointCount--;
        }

        if (pointCount < 2)
        {
            return [];
        }

        if (pointCount == 2)
        {
            Vertex p0 = contour[0];
            Vertex p1 = contour[1];
            if (Vertex.DistanceSquared(p0, p1) <= VertexDistanceEpsilon * VertexDistanceEpsilon)
            {
                // Degenerate segment behaves like a stroked point.
                return [this.GeneratePointCap(p0.X, p0.Y)];
            }
        }

        this.Reset();
        for (int i = 0; i < pointCount; i++)
        {
            Vertex point = contour[i];
            this.Add(point.X, point.Y, PathCommand.LineTo);
        }

        if (isClosed)
        {
            this.ClosePath();
        }

        Polygon result = new(isClosed ? 2 : 1);
        this.FinishPath(result);
        return result;
    }

    /// <summary>
    /// Returns whether a contour should be treated as closed when emitting stroke geometry.
    /// </summary>
    /// <param name="contour">The contour to inspect.</param>
    /// <returns><see langword="true"/> if the contour should be treated as closed; otherwise <see langword="false"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool IsContourClosedForEmission(Contour contour)
    {
        int count = contour.Count;
        if (count < 3)
        {
            return false;
        }

        if (contour[0] == contour[^1])
        {
            return true;
        }

        Vertex delta = contour[0] - contour[^1];
        const double closeThreshold = 1E-9D;
        return delta.LengthSquared() <= closeThreshold * closeThreshold;
    }

    /// <summary>
    /// Marks the current path as closed before finishing the outline.
    /// </summary>
    private void ClosePath()
    {
        this.closed = (int)PathFlags.Close;
        this.status = Status.Initial;
    }

    /// <summary>
    /// Resets the stroker state for reuse.
    /// </summary>
    private void Reset()
    {
        // Reuse builders to avoid per-contour allocations.
        this.srcVertices.Clear();
        this.outVertices.Clear();
        this.srcVertex = 0;
        this.outVertex = 0;
        this.closed = 0;
        this.status = Status.Initial;
    }

    /// <summary>
    /// Consumes commands from <see cref="Accumulate(ref Vertex)"/> and materializes final contour lists.
    /// </summary>
    /// <param name="result">Destination polygon that receives generated contours.</param>
    private void FinishPath(Polygon result)
    {
        Vertex current = default;
        Vertex lastPoint = default;
        bool hasLastPoint = false;
        Contour? currentContour = null;
        PathCommand command;

        while (!(command = this.Accumulate(ref current)).Stop())
        {
            if (command.MoveTo())
            {
                // Start a new contour. Commit any previous contour that is already complete.
                if (currentContour is { Count: >= 4 })
                {
                    result.Add(currentContour);
                }

                currentContour = new Contour(16);
                hasLastPoint = false;
            }

            if (command.Vertex())
            {
                currentContour ??= new Contour(16);

                // Drop immediate duplicate vertices to avoid zero-length segments
                // entering the intersection-removal pass.
                if (!hasLastPoint || current != lastPoint)
                {
                    currentContour.Add(current);
                    lastPoint = current;
                    hasLastPoint = true;
                }
            }

            if (command.EndPoly())
            {
                // Ensure explicit closure at contour boundaries.
                if (currentContour is { Count: > 0 } && currentContour[0] != currentContour[^1])
                {
                    currentContour.Add(currentContour[0]);
                }

                if (currentContour is { Count: >= 4 })
                {
                    result.Add(currentContour);
                }

                currentContour = null;
                hasLastPoint = false;
            }
        }

        if (currentContour is { Count: > 0 } && currentContour[0] != currentContour[^1])
        {
            currentContour.Add(currentContour[0]);
        }

        if (currentContour is { Count: >= 4 })
        {
            result.Add(currentContour);
        }
    }

    /// <summary>
    /// Adds a path command and coordinate into the source stream.
    /// </summary>
    /// <param name="x">X coordinate.</param>
    /// <param name="y">Y coordinate.</param>
    /// <param name="cmd">Path command associated with the coordinate.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void Add(double x, double y, PathCommand cmd)
    {
        this.status = Status.Initial;
        if (cmd.MoveTo())
        {
            // MoveTo starts a new source contour.
            if (this.srcVertices.Length != 0)
            {
                this.srcVertices.RemoveLast();
            }

            this.Add(x, y);
        }
        else if (cmd.Vertex())
        {
            this.Add(x, y);
        }
        else
        {
            // Non-vertex command updates close flags.
            this.closed = cmd.GetCloseFlag();
        }
    }

    /// <summary>
    /// Appends a source vertex, collapsing trailing duplicates when needed.
    /// </summary>
    /// <param name="x">X coordinate.</param>
    /// <param name="y">Y coordinate.</param>
    /// <param name="distance">Cached edge length hint.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void Add(double x, double y, double distance = 0D)
    {
        if (this.srcVertices.Length > 1)
        {
            ref StrokeVertexDistance vd1 = ref this.srcVertices[^2];
            ref StrokeVertexDistance vd2 = ref this.srcVertices[^1];
            bool ret = vd1.Measure(vd2);
            if (!ret && this.srcVertices.Length != 0)
            {
                // If the previous segment collapses, remove the duplicate tail.
                this.srcVertices.RemoveLast();
            }
        }

        this.srcVertices.Add(new StrokeVertexDistance(x, y, distance));
    }

    /// <summary>
    /// Streams stroke output as path commands/vertices from the current source contour.
    /// </summary>
    /// <param name="point">Receives the emitted vertex when a vertex command is returned.</param>
    /// <returns>The next path command.</returns>
    private PathCommand Accumulate(ref Vertex point)
    {
        PathCommand cmd = PathCommand.LineTo;
        while (!cmd.Stop())
        {
            switch (this.status)
            {
                case Status.Initial:
                    // Normalize degenerate tail/head duplicates before any join math.
                    this.CloseVertexPath(this.closed != 0);

                    if (this.srcVertices.Length < 3)
                    {
                        // Very short contours cannot be treated as closed reliably.
                        this.closed = 0;
                    }

                    this.status = Status.Ready;
                    break;

                case Status.Ready:
                    // Require enough vertices for either open (2) or closed (3+) processing.
                    if (this.srcVertices.Length < 2 + (this.closed != 0 ? 1 : 0))
                    {
                        cmd = PathCommand.Stop;
                        break;
                    }

                    this.status = this.closed != 0 ? Status.Outline1 : Status.Cap1;
                    cmd = PathCommand.MoveTo;
                    this.srcVertex = 0;
                    this.outVertex = 0;
                    break;

                case Status.Cap1:
                    // Open path: emit start cap first.
                    this.CalcCap(ref this.srcVertices[0], ref this.srcVertices[1], this.srcVertices[0].Distance);
                    this.srcVertex = 1;
                    this.prevStatus = Status.Outline1;
                    this.status = Status.OutVertices;
                    this.outVertex = 0;
                    break;

                case Status.Cap2:
                    // Open path: emit terminal cap before reversing through side 2.
                    this.CalcCap(ref this.srcVertices[^1], ref this.srcVertices[^2], this.srcVertices[^2].Distance);
                    this.prevStatus = Status.Outline2;
                    this.status = Status.OutVertices;
                    this.outVertex = 0;
                    break;

                case Status.Outline1:
                    if (this.closed != 0)
                    {
                        if (this.srcVertex >= this.srcVertices.Length)
                        {
                            // Closed path switches to second side through an explicit endpoly.
                            this.prevStatus = Status.CloseFirst;
                            this.status = Status.EndPoly1;
                            break;
                        }
                    }
                    else if (this.srcVertex >= this.srcVertices.Length - 1)
                    {
                        this.status = Status.Cap2;
                        break;
                    }

                    // Emit join vertices for side 1 (forward traversal).
                    this.CalcJoin(
                        ref this.srcVertices[(this.srcVertex + this.srcVertices.Length - 1) % this.srcVertices.Length],
                        ref this.srcVertices[this.srcVertex],
                        ref this.srcVertices[(this.srcVertex + 1) % this.srcVertices.Length],
                        this.srcVertices[(this.srcVertex + this.srcVertices.Length - 1) % this.srcVertices.Length].Distance,
                        this.srcVertices[this.srcVertex].Distance);

                    this.srcVertex++;
                    this.prevStatus = this.status;
                    this.status = Status.OutVertices;
                    this.outVertex = 0;
                    break;

                case Status.CloseFirst:
                    // Start second side as a new contour command stream.
                    cmd = PathCommand.MoveTo;
                    this.status = Status.Outline2;
                    break;

                case Status.Outline2:
                    if (this.srcVertex <= (this.closed == 0 ? 1 : 0))
                    {
                        this.status = Status.EndPoly2;
                        this.prevStatus = Status.Stop;
                        break;
                    }

                    this.srcVertex--;

                    // Emit join vertices for side 2 (reverse traversal).
                    this.CalcJoin(
                        ref this.srcVertices[(this.srcVertex + 1) % this.srcVertices.Length],
                        ref this.srcVertices[this.srcVertex],
                        ref this.srcVertices[(this.srcVertex + this.srcVertices.Length - 1) % this.srcVertices.Length],
                        this.srcVertices[this.srcVertex].Distance,
                        this.srcVertices[(this.srcVertex + this.srcVertices.Length - 1) % this.srcVertices.Length].Distance);

                    this.prevStatus = this.status;
                    this.status = Status.OutVertices;
                    this.outVertex = 0;
                    break;

                case Status.OutVertices:
                    if (this.outVertex >= this.outVertices.Length)
                    {
                        // Re-enter previous phase once buffered join/cap points are flushed.
                        this.status = this.prevStatus;
                    }
                    else
                    {
                        point = this.outVertices[this.outVertex++];
                        return cmd;
                    }

                    break;

                case Status.EndPoly1:
                    this.status = this.prevStatus;

                    // First side is emitted counter-clockwise.
                    return PathCommand.EndPoly | (PathCommand)(PathFlags.Close | PathFlags.Ccw);

                case Status.EndPoly2:
                    this.status = this.prevStatus;

                    // Second side is emitted clockwise.
                    return PathCommand.EndPoly | (PathCommand)(PathFlags.Close | PathFlags.Cw);

                case Status.Stop:
                    cmd = PathCommand.Stop;
                    break;
            }
        }

        return cmd;
    }

    /// <summary>
    /// Removes duplicate tail/head points and optionally enforces closed-loop source topology.
    /// </summary>
    /// <param name="close">Whether closing normalization should be applied.</param>
    private void CloseVertexPath(bool close)
    {
        // Collapse duplicated trailing points while preserving a measured segment.
        while (this.srcVertices.Length > 1)
        {
            ref StrokeVertexDistance vd1 = ref this.srcVertices[^2];
            ref StrokeVertexDistance vd2 = ref this.srcVertices[^1];
            bool ret = vd1.Measure(vd2);

            if (ret)
            {
                break;
            }

            StrokeVertexDistance tail = this.srcVertices[^1];
            if (this.srcVertices.Length != 0)
            {
                this.srcVertices.RemoveLast();
            }

            if (this.srcVertices.Length != 0)
            {
                this.srcVertices.RemoveLast();
            }

            this.Add(tail.X, tail.Y, tail.Distance);
        }

        if (!close)
        {
            return;
        }

        // For closed paths, also remove zero-length seam between final and initial points.
        while (this.srcVertices.Length > 1)
        {
            ref StrokeVertexDistance vd1 = ref this.srcVertices[^1];
            ref StrokeVertexDistance vd2 = ref this.srcVertices[0];
            bool ret = vd1.Measure(vd2);

            if (ret)
            {
                break;
            }

            if (this.srcVertices.Length != 0)
            {
                this.srcVertices.RemoveLast();
            }
        }
    }

    /// <summary>
    /// Emits interpolated arc vertices between two offset vectors around a join center.
    /// </summary>
    /// <param name="x">Join center X.</param>
    /// <param name="y">Join center Y.</param>
    /// <param name="dx1">First offset vector X.</param>
    /// <param name="dy1">First offset vector Y.</param>
    /// <param name="dx2">Second offset vector X.</param>
    /// <param name="dy2">Second offset vector Y.</param>
    private void CalcArc(double x, double y, double dx1, double dy1, double dx2, double dy2)
    {
        double a1 = Math.Atan2(dy1 * this.widthSign, dx1 * this.widthSign);
        double a2 = Math.Atan2(dy2 * this.widthSign, dx2 * this.widthSign);

        // Derive angular step from arc detail scale and stroke radius.
        double da = Math.Acos(this.widthAbs / (this.widthAbs + (0.125D / this.ArcDetailScale))) * 2D;
        this.AddPoint(x + dx1, y + dy1);

        if (this.widthSign > 0)
        {
            if (a1 > a2)
            {
                a2 += PiMul2;
            }

            // Sweep forward for positive widths.
            int n = (int)((a2 - a1) / da);
            da = (a2 - a1) / (n + 1);
            a1 += da;
            for (int i = 0; i < n; i++)
            {
                this.AddPoint(x + (Math.Cos(a1) * this.strokeWidth), y + (Math.Sin(a1) * this.strokeWidth));
                a1 += da;
            }
        }
        else
        {
            if (a1 < a2)
            {
                a2 -= PiMul2;
            }

            // Sweep backward for negative widths.
            int n = (int)((a1 - a2) / da);
            da = (a1 - a2) / (n + 1);
            a1 -= da;
            for (int i = 0; i < n; i++)
            {
                this.AddPoint(x + (Math.Cos(a1) * this.strokeWidth), y + (Math.Sin(a1) * this.strokeWidth));
                a1 -= da;
            }
        }

        this.AddPoint(x + dx2, y + dy2);
    }

    /// <summary>
    /// Emits miter/revert/round join geometry, including fallback behavior when intersection is unstable.
    /// </summary>
    /// <param name="v0">Previous source vertex.</param>
    /// <param name="v1">Current source vertex.</param>
    /// <param name="v2">Next source vertex.</param>
    /// <param name="dx1">First offset vector X.</param>
    /// <param name="dy1">First offset vector Y.</param>
    /// <param name="dx2">Second offset vector X.</param>
    /// <param name="dy2">Second offset vector Y.</param>
    /// <param name="lineJoin">Requested line join mode.</param>
    /// <param name="miterLimit">Miter limit in stroke-width units.</param>
    /// <param name="bevelDistance">Distance of bevel midpoint from join center.</param>
    private void CalcMiter(
        ref StrokeVertexDistance v0,
        ref StrokeVertexDistance v1,
        ref StrokeVertexDistance v2,
        double dx1,
        double dy1,
        double dx2,
        double dy2,
        LineJoin lineJoin,
        double miterLimit,
        double bevelDistance)
    {
        double xi = v1.X;
        double yi = v1.Y;
        double intersectionDistance = 1D;
        double limit = this.widthAbs * miterLimit;
        bool miterLimitExceeded = true;
        bool intersectionFailed = true;

        // Intersect the two offset support lines to obtain the geometric miter apex.
        if (TryCalcIntersection(
                new Vertex(v0.X + dx1, v0.Y - dy1),
                new Vertex(v1.X + dx1, v1.Y - dy1),
                new Vertex(v1.X + dx2, v1.Y - dy2),
                new Vertex(v2.X + dx2, v2.Y - dy2),
                out Vertex intersection))
        {
            xi = intersection.X;
            yi = intersection.Y;
            intersectionDistance = Vertex.Distance(new Vertex(v1.X, v1.Y), intersection);
            if (intersectionDistance <= limit)
            {
                this.AddPoint(xi, yi);
                miterLimitExceeded = false;
            }

            intersectionFailed = false;
        }
        else
        {
            // If lines are parallel/near-parallel, probe a fallback candidate.
            double x2 = v1.X + dx1;
            double y2 = v1.Y - dy1;
            Vertex probe = new(x2, y2);
            if ((CrossProduct(v0, v1, probe) < 0D) ==
                (CrossProduct(v1, v2, probe) < 0D))
            {
                this.AddPoint(v1.X + dx1, v1.Y - dy1);
                miterLimitExceeded = false;
            }
        }

        if (!miterLimitExceeded)
        {
            return;
        }

        // Join-style-specific overflow behavior when the true miter exceeds limit.
        switch (lineJoin)
        {
            case LineJoin.MiterRevert:
                this.AddPoint(v1.X + dx1, v1.Y - dy1);
                this.AddPoint(v1.X + dx2, v1.Y - dy2);
                break;

            case LineJoin.MiterRound:
                this.CalcArc(v1.X, v1.Y, dx1, -dy1, dx2, -dy2);
                break;

            default:
                if (intersectionFailed)
                {
                    // No reliable apex: project a clipped bevel using local tangent/perpendicular vectors.
                    miterLimit *= this.widthSign;
                    this.AddPoint(v1.X + dx1 + (dy1 * miterLimit), v1.Y - dy1 + (dx1 * miterLimit));
                    this.AddPoint(v1.X + dx2 - (dy2 * miterLimit), v1.Y - dy2 - (dx2 * miterLimit));
                }
                else
                {
                    // Blend from bevel corners toward true intersection to honor miter limit.
                    double x1 = v1.X + dx1;
                    double y1 = v1.Y - dy1;
                    double x2 = v1.X + dx2;
                    double y2 = v1.Y - dy2;
                    intersectionDistance = (limit - bevelDistance) / (intersectionDistance - bevelDistance);
                    this.AddPoint(x1 + ((xi - x1) * intersectionDistance), y1 + ((yi - y1) * intersectionDistance));
                    this.AddPoint(x2 + ((xi - x2) * intersectionDistance), y2 + ((yi - y2) * intersectionDistance));
                }

                break;
        }
    }

    /// <summary>
    /// Emits cap geometry for an open contour endpoint.
    /// </summary>
    /// <param name="v0">Cap anchor vertex.</param>
    /// <param name="v1">Adjacent source vertex used to determine tangent direction.</param>
    /// <param name="len">Length of the incident segment.</param>
    private void CalcCap(ref StrokeVertexDistance v0, ref StrokeVertexDistance v1, double len)
    {
        this.outVertices.Clear();
        if (len < VertexDistanceEpsilon)
        {
            this.AddPoint(v0.X, v0.Y);
            this.AddPoint(v1.X, v1.Y);
            return;
        }

        double dx1 = (v1.Y - v0.Y) / len;
        double dy1 = (v1.X - v0.X) / len;
        double dx2 = 0D;
        double dy2 = 0D;

        dx1 *= this.strokeWidth;
        dy1 *= this.strokeWidth;

        if (this.LineCap != LineCap.Round)
        {
            if (this.LineCap == LineCap.Square)
            {
                // Square caps extend half-width in tangent direction.
                dx2 = dy1 * this.widthSign;
                dy2 = dx1 * this.widthSign;
            }

            this.AddPoint(v0.X - dx1 - dx2, v0.Y + dy1 - dy2);
            this.AddPoint(v0.X + dx1 - dx2, v0.Y - dy1 - dy2);
        }
        else
        {
            // Round cap emitted as half-circle arc around endpoint.
            double da = Math.Acos(this.widthAbs / (this.widthAbs + (0.125D / this.ArcDetailScale))) * 2D;
            int n = (int)(Pi / da);
            da = Pi / (n + 1);

            this.AddPoint(v0.X - dx1, v0.Y + dy1);
            if (this.widthSign > 0)
            {
                double a1 = Math.Atan2(dy1, -dx1) + da;
                for (int i = 0; i < n; i++)
                {
                    this.AddPoint(v0.X + (Math.Cos(a1) * this.strokeWidth), v0.Y + (Math.Sin(a1) * this.strokeWidth));
                    a1 += da;
                }
            }
            else
            {
                double a1 = Math.Atan2(-dy1, dx1) - da;
                for (int i = 0; i < n; i++)
                {
                    this.AddPoint(v0.X + (Math.Cos(a1) * this.strokeWidth), v0.Y + (Math.Sin(a1) * this.strokeWidth));
                    a1 -= da;
                }
            }

            this.AddPoint(v0.X + dx1, v0.Y - dy1);
        }
    }

    /// <summary>
    /// Emits join geometry for a source vertex using configured inner/outer join rules.
    /// </summary>
    /// <param name="v0">Previous source vertex.</param>
    /// <param name="v1">Current source vertex.</param>
    /// <param name="v2">Next source vertex.</param>
    /// <param name="len1">Length of segment v0-v1.</param>
    /// <param name="len2">Length of segment v1-v2.</param>
    private void CalcJoin(ref StrokeVertexDistance v0, ref StrokeVertexDistance v1, ref StrokeVertexDistance v2, double len1, double len2)
    {
        const double eps = VertexDistanceEpsilon;
        if (len1 < eps || len2 < eps)
        {
            this.outVertices.Clear();

            // Degenerate neighborhood: use best available segment direction for both offsets.
            double l1 = len1 >= eps ? len1 : len2;
            double l2 = len2 >= eps ? len2 : len1;
            double offX1 = this.strokeWidth * (v1.Y - v0.Y) / l1;
            double offY1 = this.strokeWidth * (v1.X - v0.X) / l1;
            double offX2 = this.strokeWidth * (v2.Y - v1.Y) / l2;
            double offY2 = this.strokeWidth * (v2.X - v1.X) / l2;

            this.AddPoint(v1.X + offX1, v1.Y - offY1);
            this.AddPoint(v1.X + offX2, v1.Y - offY2);
            return;
        }

        double dx1 = this.strokeWidth * (v1.Y - v0.Y) / len1;
        double dy1 = this.strokeWidth * (v1.X - v0.X) / len1;
        double dx2 = this.strokeWidth * (v2.Y - v1.Y) / len2;
        double dy2 = this.strokeWidth * (v2.X - v1.X) / len2;
        this.outVertices.Clear();

        // Cross-product sign classifies whether we are on an inner corner or outer corner
        // relative to stroke direction.
        double cp = CrossProduct(v0, v1, new Vertex(v2.X, v2.Y));
        if (Math.Abs(cp) > double.Epsilon && (cp > 0D) == (this.strokeWidth > 0D))
        {
            double limit = (len1 < len2 ? len1 : len2) / this.widthAbs;
            if (limit < this.InnerMiterLimit)
            {
                limit = this.InnerMiterLimit;
            }

            switch (this.InnerJoin)
            {
                default:
                    // Bevel-like fallback for inner corners.
                    this.AddPoint(v1.X + dx1, v1.Y - dy1);
                    this.AddPoint(v1.X + dx2, v1.Y - dy2);
                    break;

                case InnerJoin.Miter:
                    this.CalcMiter(ref v0, ref v1, ref v2, dx1, dy1, dx2, dy2, LineJoin.MiterRevert, limit, 0D);
                    break;

                case InnerJoin.Jag:
                case InnerJoin.Round:
                    // If offsets are close enough, miter produces cleaner inner-corner output.
                    cp = ((dx1 - dx2) * (dx1 - dx2)) + ((dy1 - dy2) * (dy1 - dy2));
                    if (cp < len1 * len1 && cp < len2 * len2)
                    {
                        this.CalcMiter(ref v0, ref v1, ref v2, dx1, dy1, dx2, dy2, LineJoin.MiterRevert, limit, 0D);
                    }
                    else if (this.InnerJoin == InnerJoin.Jag)
                    {
                        // Jagged inner join inserts center vertex to preserve cusp.
                        this.AddPoint(v1.X + dx1, v1.Y - dy1);
                        this.AddPoint(v1.X, v1.Y);
                        this.AddPoint(v1.X + dx2, v1.Y - dy2);
                    }
                    else
                    {
                        // Rounded inner join bridges via arc passing through the corner.
                        this.AddPoint(v1.X + dx1, v1.Y - dy1);
                        this.AddPoint(v1.X, v1.Y);
                        this.CalcArc(v1.X, v1.Y, dx2, -dy2, dx1, -dy1);
                        this.AddPoint(v1.X, v1.Y);
                        this.AddPoint(v1.X + dx2, v1.Y - dy2);
                    }

                    break;
            }
        }
        else
        {
            // Outer join path.
            double dx = (dx1 + dx2) / 2D;
            double dy = (dy1 + dy2) / 2D;
            double bevelDistance = new Vertex(dx, dy).Length();

            if (this.LineJoin is LineJoin.Round or LineJoin.Bevel &&
                this.ArcDetailScale * (this.widthAbs - bevelDistance) < this.widthEps)
            {
                // Near-collinear optimization: collapse to single intersection point when possible.
                if (TryCalcIntersection(
                        new Vertex(v0.X + dx1, v0.Y - dy1),
                        new Vertex(v1.X + dx1, v1.Y - dy1),
                        new Vertex(v1.X + dx2, v1.Y - dy2),
                        new Vertex(v2.X + dx2, v2.Y - dy2),
                        out Vertex intersection))
                {
                    dx = intersection.X;
                    dy = intersection.Y;
                    this.AddPoint(dx, dy);
                }
                else
                {
                    this.AddPoint(v1.X + dx1, v1.Y - dy1);
                }

                return;
            }

            switch (this.LineJoin)
            {
                case LineJoin.Miter:
                case LineJoin.MiterRevert:
                case LineJoin.MiterRound:
                    this.CalcMiter(ref v0, ref v1, ref v2, dx1, dy1, dx2, dy2, this.LineJoin, this.MiterLimit, bevelDistance);
                    break;

                case LineJoin.Round:
                    this.CalcArc(v1.X, v1.Y, dx1, -dy1, dx2, -dy2);
                    break;

                default:
                    this.AddPoint(v1.X + dx1, v1.Y - dy1);
                    this.AddPoint(v1.X + dx2, v1.Y - dy2);
                    break;
            }
        }
    }

    /// <summary>
    /// Appends a computed output vertex to the current join/cap vertex buffer.
    /// </summary>
    /// <param name="x">X coordinate.</param>
    /// <param name="y">Y coordinate.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void AddPoint(double x, double y) => this.outVertices.Add(new Vertex(x, y));

    /// <summary>
    /// Creates cap geometry for a single-point contour.
    /// </summary>
    /// <param name="x">Point X.</param>
    /// <param name="y">Point Y.</param>
    /// <returns>A closed contour representing the cap footprint.</returns>
    private Contour GeneratePointCap(double x, double y)
    {
        if (this.LineCap == LineCap.Round)
        {
            // Emit a full circle when a contour collapses to a point.
            double da = Math.Acos(this.widthAbs / (this.widthAbs + (0.125D / this.ArcDetailScale))) * 2D;
            int n = Math.Max(4, (int)(PiMul2 / da));
            double angleStep = PiMul2 / n;

            Contour result = new(n + 1);
            for (int i = 0; i < n; i++)
            {
                double angle = i * angleStep;
                result.Add(new Vertex(
                    x + (Math.Cos(angle) * this.strokeWidth),
                    y + (Math.Sin(angle) * this.strokeWidth)));
            }

            result.Add(result[0]);
            return result;
        }

        double w = this.strokeWidth;
        Contour square =
        [
            new Vertex(x - w, y - w),
            new Vertex(x + w, y - w),
            new Vertex(x + w, y + w),
            new Vertex(x - w, y + w),
            new Vertex(x - w, y - w),
        ];
        return square;
    }

    /// <summary>
    /// Computes the oriented area/cross-product used for turn classification.
    /// </summary>
    /// <param name="a">First segment start.</param>
    /// <param name="b">First segment end.</param>
    /// <param name="point">Third point.</param>
    /// <returns>Signed cross product value.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static double CrossProduct(in StrokeVertexDistance a, in StrokeVertexDistance b, in Vertex point)
        => ((point.X - b.X) * (b.Y - a.Y)) - ((point.Y - b.Y) * (b.X - a.X));

    /// <summary>
    /// Computes line intersection for two infinite lines defined by segment endpoints.
    /// </summary>
    /// <param name="a">First line start.</param>
    /// <param name="b">First line end.</param>
    /// <param name="c">Second line start.</param>
    /// <param name="d">Second line end.</param>
    /// <param name="intersection">Receives the intersection point when available.</param>
    /// <returns><see langword="true"/> if lines intersect robustly; otherwise <see langword="false"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool TryCalcIntersection(in Vertex a, in Vertex b, in Vertex c, in Vertex d, out Vertex intersection)
    {
        Vertex ab = b - a;
        Vertex cd = d - c;
        double denominator = Vertex.Cross(ab, cd);
        if (Math.Abs(denominator) < IntersectionEpsilon)
        {
            // Parallel or numerically unstable near-parallel lines.
            intersection = default;
            return false;
        }

        double t = Vertex.Cross(c - a, cd) / denominator;
        intersection = a + (ab * t);
        return true;
    }
}

#pragma warning restore SA1201 // Elements should appear in the correct order

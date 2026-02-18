// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System.Runtime.CompilerServices;

namespace SixLabors.PolygonClipper;

/// <summary>
/// Represents an edge that is currently active in the sweep-line.
/// </summary>
/// <remarks>
/// The sweep assumes a Y-axis-positive-down coordinate system. "Bottom" and "Top"
/// refer to the lower and upper scanline endpoints (larger and smaller Y respectively).
/// </remarks>
internal sealed class ActiveEdge
{
#pragma warning disable SA1401 // Hot sweep state uses fields to avoid accessor overhead.
    /// <summary>
    /// The lower endpoint of the edge in scanline order.
    /// </summary>
    public Vertex Bottom;

    /// <summary>
    /// The upper endpoint of the edge in scanline order.
    /// </summary>
    public Vertex Top;

    /// <summary>
    /// The X coordinate where the edge intersects the current scanline.
    /// </summary>
    public double CurrentX;

    /// <summary>
    /// The delta-X per delta-Y for the edge (its scanline slope).
    /// </summary>
    public double Dx;

    /// <summary>
    /// The winding delta contributed by this edge (+1 or -1).
    /// </summary>
    public int WindDelta;

    /// <summary>
    /// The accumulated winding count for this edge.
    /// </summary>
    public int WindCount;

    /// <summary>
    /// The output record this edge is contributing to, if any.
    /// </summary>
    public OutputRecord? OutputRecord;

    /// <summary>
    /// The previous edge in the Active Edge List (AEL).
    /// </summary>
    public ActiveEdge? PrevInAel;

    /// <summary>
    /// The next edge in the Active Edge List (AEL).
    /// </summary>
    public ActiveEdge? NextInAel;

    /// <summary>
    /// The previous edge in the Sorted Edge List (SEL).
    /// </summary>
    public ActiveEdge? PrevInSel;

    /// <summary>
    /// The next edge in the Sorted Edge List (SEL).
    /// </summary>
    public ActiveEdge? NextInSel;

    /// <summary>
    /// The temporary link used when sorting intersections.
    /// </summary>
    public ActiveEdge? Jump;

    /// <summary>
    /// The current top vertex for this edge's bound.
    /// </summary>
    public SweepVertex? VertexTop;

    /// <summary>
    /// The local minima that spawned this edge.
    /// </summary>
    public LocalMinima LocalMin;

    /// <summary>
    /// Indicates whether this edge is the left bound of its pair.
    /// </summary>
    public bool IsLeftBound;

    /// <summary>
    /// The pending join state for this edge.
    /// </summary>
    public JoinWith JoinWith;
#pragma warning restore SA1401

    /// <summary>
    /// Gets a value indicating whether this edge currently contributes to output.
    /// </summary>
    public bool IsHot => this.OutputRecord != null;

    /// <summary>
    /// Gets a value indicating whether the edge is horizontal within tolerance.
    /// </summary>
    public bool IsHorizontal => this.Top.Y == this.Bottom.Y;

    /// <summary>
    /// Gets a value indicating whether a horizontal edge is heading right.
    /// </summary>
    public bool IsHeadingRightHorizontal => double.IsNegativeInfinity(this.Dx);

    /// <summary>
    /// Gets a value indicating whether a horizontal edge is heading left.
    /// </summary>
    public bool IsHeadingLeftHorizontal => double.IsPositiveInfinity(this.Dx);

    /// <summary>
    /// Gets a value indicating whether the current top vertex is a local maxima.
    /// </summary>
    public bool IsMaxima => this.VertexTop != null && this.VertexTop.IsMaxima;

    /// <summary>
    /// Gets a value indicating whether this edge is the front edge of its output record.
    /// </summary>
    public bool IsFront => this.OutputRecord != null && this == this.OutputRecord.FrontEdge;

    /// <summary>
    /// Gets the next input vertex along the bound in the winding direction.
    /// </summary>
    public SweepVertex NextVertex => this.WindDelta > 0 ? this.VertexTop!.Next! : this.VertexTop!.Prev!;

    /// <summary>
    /// Gets the vertex two steps behind the current top, used for turn tests.
    /// </summary>
    public SweepVertex PrevPrevVertex => this.WindDelta > 0 ? this.VertexTop!.Prev!.Prev! : this.VertexTop!.Next!.Next!;

    /// <summary>
    /// Finds the previous hot edge in the AEL, if any.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public ActiveEdge? GetPrevHotEdge()
    {
        ActiveEdge? prev = this.PrevInAel;
        while (prev != null && !prev.IsHot)
        {
            prev = prev.PrevInAel;
        }

        return prev;
    }

    /// <summary>
    /// Calculates the X coordinate where this edge intersects the scanline at <paramref name="currentY" />.
    /// </summary>
    // This method sits on the hottest path in large self-intersection workloads.
    // AggressiveOptimization consistently improves codegen here versus tiered defaults.
    [MethodImpl(MethodImplOptions.AggressiveInlining | MethodImplOptions.AggressiveOptimization)]
    public static double TopX(ActiveEdge edge, double currentY)
    {
        if (currentY == edge.Top.Y || edge.Top.X == edge.Bottom.X)
        {
            return edge.Top.X;
        }

        if (currentY == edge.Bottom.Y)
        {
            return edge.Bottom.X;
        }

        return edge.Bottom.X + (edge.Dx * (currentY - edge.Bottom.Y));
    }

    /// <summary>
    /// Recomputes <see cref="Dx" /> from the current endpoints.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void UpdateDx() => this.Dx = GetDx(this.Bottom, this.Top);

    /// <summary>
    /// Computes delta-X per delta-Y, returning infinities for horizontal edges.
    /// </summary>
    private static double GetDx(Vertex pt1, Vertex pt2)
    {
        double dy = pt2.Y - pt1.Y;
        if (dy != 0)
        {
            return (pt2.X - pt1.X) / dy;
        }

        return pt2.X > pt1.X ? double.NegativeInfinity : double.PositiveInfinity;
    }
}

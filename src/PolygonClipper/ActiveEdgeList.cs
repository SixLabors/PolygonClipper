// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System.Runtime.CompilerServices;

namespace SixLabors.PolygonClipper;

/// <summary>
/// Maintains the active edge list (AEL) plus a horizontal edge stack for the sweep.
/// </summary>
/// <remarks>
/// The AEL is ordered left-to-right at the current scanline. As edges are inserted
/// and removed, this list preserves adjacency for intersection processing.
/// The horizontal stack is a lightweight LIFO queue used to process horizontal
/// bounds separately from the main sweep order.
/// </remarks>
internal sealed class ActiveEdgeList
{
    private readonly Stack<ActiveEdge> pool;
    private ActiveEdge? horizontalHead;

    /// <summary>
    /// Initializes a new instance of the <see cref="ActiveEdgeList"/> class.
    /// </summary>
    public ActiveEdgeList() => this.pool = new Stack<ActiveEdge>();

    /// <summary>
    /// Gets the head of the active edge list.
    /// </summary>
    public ActiveEdge? Head { get; private set; }

    /// <summary>
    /// Clears all active edges and returns them to the pool.
    /// </summary>
    public void ClearActiveEdges()
    {
        while (this.Head != null)
        {
            this.Remove(this.Head);
        }

        this.horizontalHead = null;
    }

    /// <summary>
    /// Resets sweep pointers without clearing the pool.
    /// </summary>
    public void Reset()
    {
        this.Head = null;
        this.horizontalHead = null;
    }

    /// <summary>
    /// Acquires a reusable active edge, allocating if the pool is empty.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public ActiveEdge Acquire()
        => this.pool.Count == 0 ? new ActiveEdge() : this.pool.Pop();

    /// <summary>
    /// Inserts an edge into the active list, maintaining left-to-right order.
    /// </summary>
    /// <param name="edge">The edge to insert.</param>
    public void InsertLeft(ActiveEdge edge)
    {
        if (this.Head == null)
        {
            edge.PrevInAel = null;
            edge.NextInAel = null;
            this.Head = edge;
            return;
        }

        if (!IsValidActiveEdgeOrder(this.Head, edge))
        {
            edge.PrevInAel = null;
            edge.NextInAel = this.Head;
            this.Head.PrevInAel = edge;
            this.Head = edge;
            return;
        }

        ActiveEdge edge2 = this.Head;
        while (edge2.NextInAel != null && IsValidActiveEdgeOrder(edge2.NextInAel, edge))
        {
            edge2 = edge2.NextInAel;
        }

        // Keep joined edges adjacent in the active list.
        if (edge2.JoinWith == JoinWith.Right)
        {
            edge2 = edge2.NextInAel!;
        }

        edge.NextInAel = edge2.NextInAel;
        if (edge2.NextInAel != null)
        {
            edge2.NextInAel.PrevInAel = edge;
        }

        edge.PrevInAel = edge2;
        edge2.NextInAel = edge;
    }

    /// <summary>
    /// Inserts a right bound edge immediately after another edge in the active list.
    /// </summary>
    /// <param name="edge">The anchor edge.</param>
    /// <param name="edge2">The edge to insert.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void InsertRight(ActiveEdge edge, ActiveEdge edge2)
    {
        edge2.NextInAel = edge.NextInAel;
        if (edge.NextInAel != null)
        {
            edge.NextInAel.PrevInAel = edge2;
        }

        edge2.PrevInAel = edge;
        edge.NextInAel = edge2;
    }

    /// <summary>
    /// Removes an edge from the active list and returns it to the pool.
    /// </summary>
    /// <param name="edge">The edge to remove.</param>
    public void Remove(ActiveEdge edge)
    {
        ActiveEdge? prev = edge.PrevInAel;
        ActiveEdge? next = edge.NextInAel;

        if (prev == null && next == null && edge != this.Head)
        {
            return;
        }

        if (prev != null)
        {
            prev.NextInAel = next;
        }
        else
        {
            this.Head = next;
        }

        if (next != null)
        {
            next.PrevInAel = prev;
        }

        this.Recycle(edge);
    }

    /// <summary>
    /// Swaps the positions of two adjacent edges in the active list.
    /// </summary>
    /// <param name="left">The left edge.</param>
    /// <param name="right">The right edge.</param>
    public void SwapPositions(ActiveEdge left, ActiveEdge right)
    {
        // Precondition: left must be immediately to the left of right.
        ActiveEdge? next = right.NextInAel;
        if (next != null)
        {
            next.PrevInAel = left;
        }

        ActiveEdge? prev = left.PrevInAel;
        if (prev != null)
        {
            prev.NextInAel = right;
        }

        right.PrevInAel = prev;
        right.NextInAel = left;
        left.PrevInAel = right;
        left.NextInAel = next;
        if (right.PrevInAel == null)
        {
            this.Head = right;
        }
    }

    /// <summary>
    /// Clears the horizontal edge stack.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void ClearHorizontalQueue() => this.horizontalHead = null;

    /// <summary>
    /// Pushes a horizontal edge onto the processing stack.
    /// </summary>
    /// <param name="edge">The horizontal edge to push.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void PushHorizontal(ActiveEdge edge)
    {
        edge.NextInSel = this.horizontalHead;
        this.horizontalHead = edge;
    }

    /// <summary>
    /// Pops the next horizontal edge to process.
    /// </summary>
    /// <param name="edge">The next horizontal edge, or <see langword="null"/>.</param>
    /// <returns><see langword="true"/> when a horizontal edge was available.</returns>
    public bool TryPopHorizontal(out ActiveEdge? edge)
    {
        edge = this.horizontalHead;
        if (edge == null)
        {
            return false;
        }

        this.horizontalHead = edge.NextInSel;
        return true;
    }

    /// <summary>
    /// Copies the active list into a sorted list and updates current X values.
    /// </summary>
    /// <param name="topY">The scanline top Y coordinate.</param>
    /// <returns>The head of the sorted list.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining | MethodImplOptions.AggressiveOptimization)]
    public ActiveEdge? CopyToSorted(double topY)
    {
        ActiveEdge? edge = this.Head;
        ActiveEdge? sortedHead = edge;
        while (edge != null)
        {
            edge.PrevInSel = edge.PrevInAel;
            edge.NextInSel = edge.NextInAel;
            edge.Jump = edge.NextInSel;

            // Joined edges can be split later during intersection processing.
            edge.CurrentX = ActiveEdge.TopX(edge, topY);

            // Defer any Y updates; intersection tests use original bounds.
            edge = edge.NextInAel;
        }

        return sortedHead;
    }

    /// <summary>
    /// Determines whether the newcomer should be inserted after the resident in the active list.
    /// </summary>
    /// <param name="resident">The current resident edge.</param>
    /// <param name="newcomer">The incoming edge to compare.</param>
    /// <returns><see langword="true"/> if the newcomer bedoubles after the resident.</returns>
    public static bool IsValidActiveEdgeOrder(ActiveEdge resident, ActiveEdge newcomer)
    {
        if (newcomer.CurrentX != resident.CurrentX)
        {
            return newcomer.CurrentX > resident.CurrentX;
        }

        // Compare turning direction: resident.Top -> newcomer.Bottom -> newcomer.Top.
        int d = PolygonUtilities.CrossSign(resident.Top, newcomer.Bottom, newcomer.Top);
        if (d != 0)
        {
            return d < 0;
        }

        // For collinear bounds, use the next turn to order them.
        if (!resident.IsMaxima && (resident.Top.Y > newcomer.Top.Y))
        {
            return PolygonUtilities.CrossSign(
                newcomer.Bottom,
                resident.Top,
                resident.NextVertex.Point) <= 0;
        }

        if (!newcomer.IsMaxima && (newcomer.Top.Y > resident.Top.Y))
        {
            return PolygonUtilities.CrossSign(
                newcomer.Bottom,
                newcomer.Top,
                newcomer.NextVertex.Point) >= 0;
        }

        double y = newcomer.Bottom.Y;
        bool newcomerIsLeft = newcomer.IsLeftBound;

        if (resident.Bottom.Y != y || resident.LocalMin.Vertex.Point.Y != y)
        {
            return newcomer.IsLeftBound;
        }

        // Only newly inserted edges reach this branch.
        if (resident.IsLeftBound != newcomerIsLeft)
        {
            return newcomerIsLeft;
        }

        if (PolygonUtilities.IsCollinear(
            resident.PrevPrevVertex.Point,
            resident.Bottom,
            resident.Top))
        {
            return true;
        }

        // Use the alternate bound turn to break the tie.
        return (PolygonUtilities.CrossSign(
            resident.PrevPrevVertex.Point,
            newcomer.Bottom,
            newcomer.PrevPrevVertex.Point) > 0) == newcomerIsLeft;
    }

    /// <summary>
    /// Resets and returns an active edge to the reuse pool.
    /// </summary>
    /// <param name="edge">The edge to recycle.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void Recycle(ActiveEdge edge)
    {
        // Clear references so pooled edges do not retain objects.
        edge.Bottom = default;
        edge.Top = default;
        edge.Dx = 0.0;
        edge.CurrentX = 0;
        edge.WindCount = 0;
        edge.OutputRecord = null;
        edge.PrevInAel = null;
        edge.NextInAel = null;
        edge.PrevInSel = null;
        edge.NextInSel = null;
        edge.Jump = null;
        edge.VertexTop = null;
        edge.LocalMin = default;
        edge.IsLeftBound = false;
        edge.JoinWith = JoinWith.None;
        this.pool.Push(edge);
    }
}

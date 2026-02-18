// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace SixLabors.PolygonClipper;

/// <summary>
/// Represents a status line for the sweep line algorithm, maintaining a sorted collection of sweep events.
/// <para>
/// Performance Characteristics:
/// - **Insertion**: O(n) in the worst case. The operation consists of:
///   1. A binary search (O(log n)) to determine the correct insertion point.
///   2. A shift operation to move subsequent elements in the list (O(k)), where k is the number of elements
///      after the insertion index. In the worst case, this can approach O(n).
/// - **Removal**: O(n) in the worst case. After finding the index of the element to remove, subsequent
///   elements in the list need to be shifted (O(k)), where k is the number of elements after the removed index.
/// - **Next/Previous Access**: O(1) after the index is known, as the list provides constant-time indexing.
/// </para>
/// The implementation ensures efficient neighbor traversal (next/previous) at O(1), making it suitable for
/// algorithms where neighboring elements are accessed frequently. The use of `BinarySearch` minimizes the cost
/// of insertion/removal compared to naive search-based approaches.
/// </summary>
[DebuggerDisplay("Count = {Count}")]
internal sealed class StatusLine
{
    private const int DefaultCapacity = 16;
    private readonly List<SweepEvent> sortedEvents;
    private readonly SegmentComparer comparer = new();

    public StatusLine()
        : this(DefaultCapacity)
    {
    }

    public StatusLine(int capacity)
        => this.sortedEvents = new List<SweepEvent>(capacity > 0 ? capacity : DefaultCapacity);

    /// <summary>
    /// Gets the number of events in the status line.
    /// </summary>
    public int Count
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => this.sortedEvents.Count;
    }

    /// <summary>
    /// Gets the minimum sweep event in the status line (first in sort order).
    /// </summary>
    public SweepEvent Min
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => this.sortedEvents[0];
    }

    /// <summary>
    /// Gets the maximum sweep event in the status line (last in sort order).
    /// </summary>
    public SweepEvent Max
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => this.sortedEvents[^1];
    }

    /// <summary>
    /// Gets the retained list capacity.
    /// </summary>
    public int RetainedCapacity => this.sortedEvents.Capacity;

    /// <summary>
    /// Gets the event at the specified index.
    /// </summary>
    /// <param name="index">The index of the event.</param>
    /// <returns>The sweep event at the given index.</returns>
    public SweepEvent this[int index]
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => this.sortedEvents[index];
    }

    /// <summary>
    /// Clears active events and ensures the desired capacity.
    /// </summary>
    /// <param name="capacity">Desired minimum capacity.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Reset(int capacity)
    {
        this.sortedEvents.Clear();
        if (capacity > this.sortedEvents.Capacity)
        {
            this.sortedEvents.EnsureCapacity(capacity);
        }
    }

    /// <summary>
    /// Adds a sweep event into the status line, maintaining sorted order.
    /// </summary>
    /// <param name="e">The sweep event to insert.</param>
    /// <returns>The index where the event was inserted.</returns>
    public int Add(SweepEvent e)
    {
        int index = this.sortedEvents.BinarySearch(e, this.comparer);
        if (index < 0)
        {
            index = ~index; // Get the correct insertion point
        }

        this.sortedEvents.Insert(index, e);
        e.PosSL = index;
        return index;
    }

    /// <summary>
    /// Removes a sweep event from the status line.
    /// </summary>
    /// <param name="index">The index of the event to remove.</param>
    /// <exception cref="ArgumentOutOfRangeException">
    /// Thrown if <paramref name="index"/> is less than 0 or greater than or equal to the number of events.
    /// </exception>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void RemoveAt(int index)
        => this.sortedEvents.RemoveAt(index);

    /// <summary>
    /// Finds the current index of a sweep event in the status line.
    /// </summary>
    /// <param name="e">The event to locate.</param>
    /// <returns>The index of the event, or -1 if it is not present.</returns>
    public int IndexOf(SweepEvent e)
    {
        List<SweepEvent> events = this.sortedEvents;
        int count = events.Count;
        int hint = e.PosSL;

        if ((uint)hint < (uint)count && ReferenceEquals(events[hint], e))
        {
            return hint;
        }

        int index = events.BinarySearch(e, this.comparer);
        if (index >= 0)
        {
            if (ReferenceEquals(events[index], e))
            {
                e.PosSL = index;
                return index;
            }

            // BinarySearch can return any comparer-equal slot. Scan local ties by reference.
            for (int i = index - 1; i >= 0 && this.comparer.Compare(events[i], e) == 0; i--)
            {
                if (ReferenceEquals(events[i], e))
                {
                    e.PosSL = i;
                    return i;
                }
            }

            for (int i = index + 1; i < count && this.comparer.Compare(events[i], e) == 0; i++)
            {
                if (ReferenceEquals(events[i], e))
                {
                    e.PosSL = i;
                    return i;
                }
            }
        }

        // Fail-safe reference lookup for correctness if comparer order is temporarily unstable.
        for (int i = 0; i < count; i++)
        {
            if (ReferenceEquals(events[i], e))
            {
                e.PosSL = i;
                return i;
            }
        }

        return -1;
    }

    /// <summary>
    /// Gets the next sweep event relative to the given index.
    /// </summary>
    /// <param name="index">The reference index.</param>
    /// <returns>The next sweep event, or <c>null</c> if none exists.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public SweepEvent? Next(int index)
    {
        if (index >= 0 && index < this.sortedEvents.Count - 1)
        {
            return this.sortedEvents[index + 1];
        }

        return null;
    }

    /// <summary>
    /// Gets the previous sweep event relative to the given index.
    /// </summary>
    /// <param name="index">The reference index.</param>
    /// <returns>The previous sweep event, or <c>null</c> if none exists.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public SweepEvent? Prev(int index)
    {
        if (index > 0 && index < this.sortedEvents.Count)
        {
            return this.sortedEvents[index - 1];
        }

        return null;
    }
}

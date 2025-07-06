// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace PolygonClipper;

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
    private SweepEvent[] events;
    private int count;
    private readonly SegmentComparer comparer = new();

    /// <summary>
    /// Initializes a new instance of the <see cref="StatusLine"/> class.
    /// </summary>
    public StatusLine()
    {
        this.events = new SweepEvent[16]; // Start with reasonable capacity
        this.count = 0;
    }

    /// <summary>
    /// Gets the number of events in the status line.
    /// </summary>
    public int Count
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => this.count;
    }

    /// <summary>
    /// Gets the event at the specified index.
    /// </summary>
    /// <param name="index">The index of the event.</param>
    /// <returns>The sweep event at the given index.</returns>
    public SweepEvent this[int index]
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => Unsafe.Add(ref MemoryMarshal.GetArrayDataReference(this.events), index);
    }

    /// <summary>
    /// Inserts a sweep event into the status line, maintaining sorted order.
    /// </summary>
    /// <param name="e">The sweep event to insert.</param>
    /// <returns>The index where the event was inserted.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public int Insert(SweepEvent e)
    {
        // Ensure capacity
        if (this.count == this.events.Length)
        {
            this.Resize();
        }

        // Binary search to find insertion point
        int insertIndex = this.BinarySearchInsertionPoint(e);

        // Fast array shift using Unsafe operations (without pointers)
        ref SweepEvent eventsRef = ref MemoryMarshal.GetArrayDataReference(this.events);

        if (insertIndex < this.count)
        {
            // Shift elements to the right using Array.Copy (optimized by runtime)
            Array.Copy(this.events, insertIndex, this.events, insertIndex + 1, this.count - insertIndex);
        }

        // Insert the new element using unsafe access
        Unsafe.Add(ref eventsRef, insertIndex) = e;
        this.count++;
        this.UpdatePositionsFrom(insertIndex);
        return insertIndex;
    }

    /// <summary>
    /// Removes a sweep event from the status line.
    /// </summary>
    /// <param name="index">The index of the event to remove.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void RemoveAt(int index)
    {
        // Shift elements to the left using Array.Copy
        if (index < this.count - 1)
        {
            Array.Copy(this.events, index + 1, this.events, index, this.count - index - 1);
        }

        // Clear the last element to avoid holding references
        ref SweepEvent eventsRef = ref MemoryMarshal.GetArrayDataReference(this.events);
        Unsafe.Add(ref eventsRef, this.count - 1) = null!;

        this.count--;
        this.UpdatePositionsFrom(index);
    }

    /// <summary>
    /// Gets the next sweep event relative to the given index.
    /// </summary>
    /// <param name="index">The reference index.</param>
    /// <returns>The next sweep event, or <c>null</c> if none exists.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public SweepEvent? Next(int index)
    {
        if ((uint)index < (uint)(this.count - 1))
        {
            return Unsafe.Add(ref MemoryMarshal.GetArrayDataReference(this.events), index + 1);
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
        if ((uint)(index - 1) < (uint)this.count)
        {
            return Unsafe.Add(ref MemoryMarshal.GetArrayDataReference(this.events), index - 1);
        }
        return null;
    }

    /// <summary>
    /// Updates position indices from the given starting index using unsafe operations.
    /// </summary>
    /// <param name="startIndex">The index to start updating from.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void UpdatePositionsFrom(int startIndex)
    {
        ref SweepEvent eventsRef = ref MemoryMarshal.GetArrayDataReference(this.events);

        for (int i = startIndex; i < this.count; i++)
        {
            Unsafe.Add(ref eventsRef, i).PosSL = i;
        }
    }

    /// <summary>
    /// Performs binary search to find the correct insertion point.
    /// </summary>
    /// <param name="item">The item to find insertion point for.</param>
    /// <returns>The index where the item should be inserted.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private int BinarySearchInsertionPoint(SweepEvent item)
    {
        ref SweepEvent eventsRef = ref MemoryMarshal.GetArrayDataReference(this.events);
        int left = 0;
        int right = this.count - 1;

        while (left <= right)
        {
            int mid = left + ((right - left) >> 1);
            int comparison = this.comparer.Compare(Unsafe.Add(ref eventsRef, mid), item);

            if (comparison < 0)
            {
                left = mid + 1;
            }
            else
            {
                right = mid - 1;
            }
        }

        return left;
    }

    /// <summary>
    /// Resizes the internal array when capacity is exceeded.
    /// </summary>
    [MethodImpl(MethodImplOptions.NoInlining)]
    private void Resize()
    {
        // TODO: Is this the best growth strategy?
        int newCapacity = this.events.Length + 16;
        SweepEvent[] newEvents = new SweepEvent[newCapacity];
        Array.Copy(this.events, newEvents, this.count);
        this.events = newEvents;
    }
}

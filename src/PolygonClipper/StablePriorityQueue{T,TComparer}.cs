// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace PolygonClipper;

/// <summary>
/// Represents a stable priority queue that maintains the order of items with the same priority.
/// </summary>
/// <typeparam name="T">The type of elements in the priority queue.</typeparam>
/// <typeparam name="TComparer">The type of comparer used to determine the priority of the elements.</typeparam>
[DebuggerDisplay("Count = {Count}")]
internal sealed class StablePriorityQueue<T, TComparer>
    where TComparer : IComparer<T>
{
    private readonly List<T> heap;

    /// <summary>
    /// Initializes a new instance of the <see cref="StablePriorityQueue{T, TComparer}"/> class with a specified comparer.
    /// </summary>
    /// <param name="comparer">The comparer to determine the priority of the elements.</param>
    /// <param name="capacity">The initial capacity of the priority queue.</param>
    public StablePriorityQueue(TComparer comparer, int capacity)
    {
        this.Comparer = comparer ?? throw new ArgumentNullException(nameof(comparer));
        this.heap = new List<T>(capacity > 0 ? capacity : 16);
    }

    /// <summary>
    /// Gets the number of elements in the priority queue.
    /// </summary>
    public int Count
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => this.heap.Count;
    }

    /// <summary>
    /// Gets the comparer used to determine the priority of the elements.
    /// </summary>
    public TComparer Comparer { get; }

    /// <summary>
    /// Adds an item to the priority queue, maintaining the heap property.
    /// </summary>
    /// <param name="item">The item to add.</param>
    public void Enqueue(T item)
    {
        List<T> data = this.heap;
        data.Add(item);
        this.Up((uint)data.Count - 1, data);
    }

    /// <summary>
    /// Removes and returns the item with the highest priority (lowest value) from the priority queue.
    /// </summary>
    /// <returns>The item with the highest priority.</returns>
    /// <exception cref="InvalidOperationException">Thrown if the priority queue is empty.</exception>
    public T Dequeue()
    {
        List<T> data = this.heap;
        int count = data.Count;
        ThrowIfEmpty(count);
        ref T dRef = ref MemoryMarshal.GetReference(CollectionsMarshal.AsSpan(data));

        int maxIndex = count - 1;
        T top = Unsafe.Add(ref dRef, 0u);
        T bottom = Unsafe.Add(ref dRef, (uint)maxIndex);
        data.RemoveAt(maxIndex);

        if (--count > 0)
        {
            Unsafe.Add(ref dRef, 0u) = bottom;
            this.Down(0u, data);
        }

        return top;
    }

    /// <summary>
    /// Returns the item with the highest priority (lowest value) without removing it.
    /// </summary>
    /// <returns>The item with the highest priority.</returns>
    /// <exception cref="InvalidOperationException">Thrown if the priority queue is empty.</exception>
    public T Peek()
    {
        ThrowIfEmpty(this.Count);
        return this.heap[0];
    }

    /// <summary>
    /// Restores the min-heap property by moving the item at the specified index upward
    /// through the heap until it is in the correct position. This is called after insertion.
    /// </summary>
    /// <param name="index">The index of the newly added item to sift upward.</param>
    /// <param name="heap">The heap to operate on.</param>
    private void Up(uint index, List<T> heap)
    {
        ref T dRef = ref MemoryMarshal.GetReference(CollectionsMarshal.AsSpan(heap));
        T item = Unsafe.Add(ref dRef, index);
        TComparer comparer = this.Comparer;

        while (index > 0)
        {
            uint parent = (index - 1u) >> 1;
            T current = Unsafe.Add(ref dRef, parent);
            if (comparer.Compare(item, current) >= 0)
            {
                break;
            }

            Unsafe.Add(ref dRef, index) = current;
            index = parent;
        }

        Unsafe.Add(ref dRef, index) = item;
    }

    /// <summary>
    /// Restores the min-heap property by moving the item at the specified index downward
    /// through the heap until it is in the correct position. This is called after removal of the root.
    /// </summary>
    /// <param name="index">The index of the item to sift downward (typically the root).</param>
    /// <param name="heap">The heap to operate on.</param>
    private void Down(uint index, List<T> heap)
    {
        Span<T> data = CollectionsMarshal.AsSpan(heap);
        ref T dRef = ref MemoryMarshal.GetReference(data);

        uint length = (uint)data.Length;
        uint halfLength = length >> 1;
        T item = Unsafe.Add(ref dRef, index);
        TComparer comparer = this.Comparer;

        while (index < halfLength)
        {
            uint bestChild = (index << 1) + 1; // Initially left child
            uint right = bestChild + 1u;

            if (right < length && comparer.Compare(Unsafe.Add(ref dRef, right), Unsafe.Add(ref dRef, bestChild)) < 0)
            {
                bestChild = right;
            }

            if (comparer.Compare(Unsafe.Add(ref dRef, bestChild), item) >= 0)
            {
                break;
            }

            Unsafe.Add(ref dRef, index) = Unsafe.Add(ref dRef, bestChild);
            index = bestChild;
        }

        Unsafe.Add(ref dRef, index) = item;
    }

    [MethodImpl(MethodImplOptions.NoInlining)]
    private static void ThrowIfEmpty(int count)
    {
        if (count == 0)
        {
            throw new InvalidOperationException("Queue is empty.");
        }
    }
}

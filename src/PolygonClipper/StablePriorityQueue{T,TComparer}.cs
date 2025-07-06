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
    private T[] heap;
    private int count;

    /// <summary>
    /// Initializes a new instance of the <see cref="StablePriorityQueue{T, TComparer}"/> class with a specified comparer and initial capacity.
    /// </summary>
    /// <param name="comparer">The comparer to determine the priority of the elements.</param>
    /// <param name="capacity">The initial capacity of the priority queue.</param>
    public StablePriorityQueue(TComparer comparer, int capacity = 4)
    {
        this.Comparer = comparer ?? throw new ArgumentNullException(nameof(comparer));
        this.heap = new T[Math.Max(capacity, 4)];
        this.count = 0;
    }

    /// <summary>
    /// Gets the number of elements in the priority queue.
    /// </summary>
    public int Count => this.count;

    /// <summary>
    /// Gets the comparer used to determine the priority of the elements.
    /// </summary>
    public TComparer Comparer { get; }

    /// <summary>
    /// Adds an item to the priority queue, maintaining the heap property.
    /// </summary>
    /// <param name="item">The item to add.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Enqueue(T item)
    {
        if (this.count == this.heap.Length)
        {
            this.Resize();
        }

        // Direct array access without bounds checking
        Unsafe.Add(ref MemoryMarshal.GetArrayDataReference(this.heap), this.count) = item;
        this.Up(this.count);
        this.count++;
    }

    /// <summary>
    /// Removes and returns the item with the highest priority (lowest value) from the priority queue.
    /// </summary>
    /// <returns>The item with the highest priority.</returns>
    /// <exception cref="InvalidOperationException">Thrown if the priority queue is empty.</exception>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public T Dequeue()
    {
        if (this.count == 0)
        {
            throw new InvalidOperationException("Queue is empty.");
        }

        ref T heapRef = ref MemoryMarshal.GetArrayDataReference(this.heap);
        T top = heapRef; // Get root element
        this.count--;

        if (this.count > 0)
        {
            // Move last element to root
            heapRef = Unsafe.Add(ref heapRef, this.count);

            // Clear the last position to avoid holding references
            Unsafe.Add(ref MemoryMarshal.GetArrayDataReference(this.heap), this.count) = default(T)!;
            this.Down(0);
        }
        else
        {
            // Clear the last remaining element
            heapRef = default(T)!;
        }

        return top;
    }

    /// <summary>
    /// Restores the heap property by moving the item at the specified index upward.
    /// </summary>
    /// <param name="index">The index of the item to move upward.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void Up(int index)
    {
        ref T heapRef = ref MemoryMarshal.GetArrayDataReference(this.heap);
        T item = Unsafe.Add(ref heapRef, index);
        TComparer comparer = this.Comparer;

        while (index > 0)
        {
            int parent = (index - 1) >> 1;
            ref T currentRef = ref Unsafe.Add(ref heapRef, parent);
            if (comparer.Compare(item, currentRef) >= 0)
            {
                break;
            }

            Unsafe.Add(ref heapRef, index) = currentRef;
            index = parent;
        }

        Unsafe.Add(ref heapRef, index) = item;
    }

    /// <summary>
    /// Restores the heap property by moving the item at the specified index downward.
    /// </summary>
    /// <param name="index">The index of the item to move downward.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void Down(int index)
    {
        ref T heapRef = ref MemoryMarshal.GetArrayDataReference(this.heap);
        int halfLength = this.count >> 1;
        T item = Unsafe.Add(ref heapRef, index);
        TComparer comparer = this.Comparer;

        while (index < halfLength)
        {
            int bestChild = (index << 1) + 1; // Initially left child
            int right = bestChild + 1;

            if (right < this.count && comparer.Compare(Unsafe.Add(ref heapRef, right), Unsafe.Add(ref heapRef, bestChild)) < 0)
            {
                bestChild = right;
            }

            ref T bestChildRef = ref Unsafe.Add(ref heapRef, bestChild);
            if (comparer.Compare(bestChildRef, item) >= 0)
            {
                break;
            }

            Unsafe.Add(ref heapRef, index) = bestChildRef;
            index = bestChild;
        }

        Unsafe.Add(ref heapRef, index) = item;
    }

    /// <summary>
    /// Resizes the internal array when capacity is exceeded.
    /// </summary>
    [MethodImpl(MethodImplOptions.NoInlining)]
    private void Resize()
    {
        int newCapacity = this.heap.Length * 2;
        Array.Resize(ref this.heap, newCapacity);
    }
}

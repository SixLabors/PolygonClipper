// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System.Collections;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace SixLabors.PolygonClipper;

/// <summary>
/// Pool-backed list of sweep events reused between clipping runs.
/// </summary>
internal sealed class SweepEventPoolList : PooledList<SweepEvent>
{
    /// <summary>
    /// Acquires a directed source interval and its paired endpoint from reusable storage.
    /// </summary>
    /// <param name="start">The canonical starting point.</param>
    /// <param name="end">The source terminal point.</param>
    /// <param name="delta">The signed operand contributions.</param>
    /// <returns>The initialized live interval.</returns>
    public SweepEvent Add(Vertex start, Vertex end, PolygonClipper.WindingCount delta)
    {
        this.TryGrow();
        SweepEvent segment = this.Items[this.Size];
        if (segment == null)
        {
            segment = new SweepEvent(start, true);
            segment.OtherEvent = new SweepEvent(end, false, segment);
            this.Items[this.Size] = segment;
        }

        segment.InitializeInterval(start, end, this.Size++, delta);
        return segment;
    }

    /// <summary>
    /// Acquires an output boundary record from the existing append-only pool.
    /// </summary>
    /// <param name="point">The boundary endpoint.</param>
    /// <param name="left">Whether this endpoint starts the canonical interval.</param>
    /// <param name="other">The paired endpoint.</param>
    /// <param name="transition">The filled-side transition.</param>
    /// <param name="previous">The fixed predecessor at interval creation.</param>
    /// <param name="id">The source interval identity.</param>
    /// <returns>The initialized history record.</returns>
    public SweepEvent Add(
        Vertex point,
        bool left,
        SweepEvent other,
        ResultTransition transition,
        SweepEvent previous,
        int id)
    {
        this.TryGrow();
        SweepEvent boundary = this.Items[this.Size] ??= new SweepEvent(point, left);
        boundary.InitializeBoundary(point, left, other, transition, previous, id);
        this.Size++;
        return boundary;
    }

    /// <summary>
    /// Orders completed boundary events for edge connection.
    /// </summary>
    /// <param name="comparer">The edge connector's event ordering.</param>
    public void Sort(SweepEventComparer comparer) => this.Items.AsSpan(0, this.Size).Sort(comparer);

    /// <summary>
    /// Gets the active event references.
    /// </summary>
    /// <returns>A span that is valid until this pool changes.</returns>
    public Span<SweepEvent> GetEventsSpan() => this.Items.AsSpan(0, this.Size);

    /// <summary>
    /// Appends references owned by another sweep pool without copying its inactive slots.
    /// </summary>
    /// <param name="source">The active references to append.</param>
    public void AddRange(SweepEventPoolList source)
    {
        this.EnsureCapacity(this.Size + source.Size);
        source.Items.AsSpan(0, source.Size).CopyTo(this.Items.AsSpan(this.Size));
        this.Size += source.Size;
    }

    /// <summary>
    /// Adds a sweep event to the active range.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void Add(SweepEvent sweepEvent)
    {
        this.TryGrow();
        this.Items[this.Size] = sweepEvent;
        this.Size++;
    }
}

/// <summary>
/// Base class for pool-backed lists with stable indexing and reuse.
/// </summary>
/// <remarks>
/// These lists are append-only during a run and reset via <see cref="Clear" /> to
/// reuse previously allocated storage and object instances. The internal array
/// can grow but never shrinks, so callers should treat <see cref="Capacity" />
/// as a long-lived pool size. Elements are only valid in the range
/// <c>[0, Count)</c>; indices remain stable for the lifetime of a run, which allows
/// pooled nodes to store indices instead of references when needed.
/// </remarks>
internal abstract class PooledList<T> : IReadOnlyList<T>
    where T : class
{
    private const int DefaultCapacity = 4;

    /// <summary>
    /// Initializes a new instance of the <see cref="PooledList{T}" /> class.
    /// </summary>
    protected PooledList() => this.Items = [];

    /// <summary>
    /// Gets the number of items that have been added during the current run.
    /// </summary>
    public int Count => this.Size;

    /// <summary>
    /// Gets the backing array used for pooled storage.
    /// </summary>
    protected T[] Items { get; private set; }

    /// <summary>
    /// Gets or sets the number of active items in the pool.
    /// </summary>
    protected int Size { get; set; }

    /// <summary>
    /// Gets the current capacity of the pooled storage.
    /// </summary>
    public int Capacity
    {
        get => this.Items.Length;
        private set
        {
            if (value <= this.Items.Length)
            {
                return;
            }

            int target = (int)BitOperations.RoundUpToPowerOf2((uint)value);
            T[] newItems = new T[target];

            // Clear resets the active count but retains reusable objects. Copy
            // those inactive slots too, including growth before the next run.
            Array.Copy(this.Items, newItems, this.Items.Length);

            this.Items = newItems;
        }
    }

    /// <summary>
    /// Gets the item at the specified index within the active range.
    /// </summary>
    public T this[int index]
    {
        get
        {
            DebugGuard.MustBeLessThan((uint)index, (uint)this.Size, nameof(index));
            return this.Items[index];
        }
    }

    /// <summary>
    /// Ensures the pool can hold at least <paramref name="capacity" /> items.
    /// </summary>
    public void EnsureCapacity(int capacity) => this.Capacity = capacity;

    /// <summary>
    /// Resets the active count to zero without clearing the backing array.
    /// </summary>
    public virtual void Clear() => this.Size = 0;

    /// <summary>
    /// Gets a struct enumerator over the active items.
    /// </summary>
    public PooledListEnumerator<T> GetEnumerator() => new(this);

    /// <inheritdoc/>
    IEnumerator<T> IEnumerable<T>.GetEnumerator() => new PooledListEnumerator<T>(this);

    /// <inheritdoc/>
    IEnumerator IEnumerable.GetEnumerator() => new PooledListEnumerator<T>(this);

    /// <summary>
    /// Grows the pool by at least one slot, doubling capacity when needed.
    /// </summary>
    protected void TryGrow()
    {
        int newSize = this.Size + 1;
        if (newSize <= this.Items.Length)
        {
            return;
        }

        int newCapacity = this.Items.Length == 0 ? DefaultCapacity : this.Items.Length * 2;
        this.Capacity = newCapacity;
    }

    /// <summary>
    /// Struct enumerator for iterating active items without allocations.
    /// </summary>
    internal struct PooledListEnumerator<TItem> : IEnumerator<TItem>
        where TItem : class
    {
        private readonly PooledList<TItem> list;
        private int index;
        private TItem? current;

        public PooledListEnumerator(PooledList<TItem> list)
        {
            this.list = list;
            this.index = 0;
            this.current = null;
        }

        public readonly TItem Current => this.current!;

        readonly object IEnumerator.Current => this.current!;

        public readonly void Dispose()
        {
        }

        public bool MoveNext()
        {
            int count = this.list.Size;
            if ((uint)this.index < (uint)count)
            {
                this.current = this.list[this.index];
                this.index++;
                return true;
            }

            return this.MoveNextRare(count);
        }

        private bool MoveNextRare(int count)
        {
            this.index = count + 1;
            this.current = null;
            return false;
        }

        public void Reset()
        {
            this.index = 0;
            this.current = null;
        }
    }
}

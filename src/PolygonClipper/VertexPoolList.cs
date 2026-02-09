// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System.Collections;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace SixLabors.PolygonClipper;

/// <summary>
/// Pool-backed list of clip vertices reused across clipping operations.
/// </summary>
internal sealed class VertexPoolList : PooledList<SweepVertex>
{
    /// <summary>
    /// Adds or reuses a clip vertex initialized with the given data.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public SweepVertex Add(Vertex point, VertexFlags flags, SweepVertex? prev)
    {
        this.TryGrow();
        SweepVertex poolVertex = this.Items[this.Size];
        if (poolVertex == null)
        {
            poolVertex = new SweepVertex(point, flags, prev);
            this.Items[this.Size] = poolVertex;
        }
        else
        {
            // Reset pooled state so linked lists are rebuilt safely.
            poolVertex.Point = point;
            poolVertex.Flags = flags;
            poolVertex.Prev = prev;
            poolVertex.Next = null;
        }

        this.Size++;
        return poolVertex;
    }
}

/// <summary>
/// Pool-backed list of output points allocated during clipping.
/// </summary>
internal sealed class OutputPointPoolList : PooledList<OutputPoint>
{
    /// <summary>
    /// Adds or reuses an output point and increments the owning record count.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public OutputPoint Add(Vertex pt, OutputRecord outputRecord)
    {
        this.TryGrow();
        OutputPoint pooledPoint = this.Items[this.Size];
        if (pooledPoint == null)
        {
            pooledPoint = new OutputPoint(pt, outputRecord);
            this.Items[this.Size] = pooledPoint;
        }
        else
        {
            pooledPoint.Point = pt;
            pooledPoint.OutputRecord = outputRecord;
            pooledPoint.Next = pooledPoint;
            pooledPoint.Prev = pooledPoint;
            pooledPoint.HorizontalSegment = null;
        }

        this.Size++;
        outputRecord.OutputPointCount++;
        return pooledPoint;
    }
}

/// <summary>
/// Pool-backed list of output records that preserves per-record state between runs.
/// </summary>
internal sealed class OutputRecordPoolList : PooledList<OutputRecord>
{
    private static readonly Contour Tombstone = [];

    /// <summary>
    /// Adds or reuses an output record with cleared state.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public OutputRecord Add()
    {
        this.TryGrow();
        OutputRecord outputRecord = this.Items[this.Size];
        if (outputRecord == null)
        {
            outputRecord = new OutputRecord();
            this.Items[this.Size] = outputRecord;
        }
        else
        {
            outputRecord.Index = 0;
            outputRecord.OutputPointCount = 0;
            outputRecord.Owner = null;
            outputRecord.FrontEdge = null;
            outputRecord.BackEdge = null;
            outputRecord.Points = null;
            outputRecord.Bounds = default;
            outputRecord.Path.Clear();
            outputRecord.Splits?.Clear();
            outputRecord.RecursiveSplit = null;
        }

        this.Size++;
        return outputRecord;
    }

    public override void Clear()
    {
        base.Clear();
        for (int i = 0; i < this.Items.Length; i++)
        {
            OutputRecord outputRecord = this.Items[i];
            if (outputRecord == null || outputRecord.Path == Tombstone)
            {
                break;
            }

            // Mark paths so pooled records are not accidentally reused without reset.
            outputRecord.Path = Tombstone;
            outputRecord.Owner = null;
            outputRecord.FrontEdge = null;
            outputRecord.BackEdge = null;
            outputRecord.Points = null;
            outputRecord.RecursiveSplit = null;
        }
    }
}

/// <summary>
/// Pool-backed list of horizontal joins used during sweep processing.
/// </summary>
internal sealed class HorizontalJoinPoolList : PooledList<HorizontalJoin>
{
    /// <summary>
    /// Adds or reuses a horizontal join entry.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public HorizontalJoin Add(OutputPoint ltor, OutputPoint rtol)
    {
        this.TryGrow();
        HorizontalJoin hJoin = this.Items[this.Size];
        if (hJoin == null)
        {
            hJoin = new HorizontalJoin(ltor, rtol);
            this.Items[this.Size] = hJoin;
        }
        else
        {
            hJoin.LeftToRight = ltor;
            hJoin.RightToLeft = rtol;
        }

        this.Size++;
        return hJoin;
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
    /// Gets or sets the current capacity of the pooled storage.
    /// </summary>
    public int Capacity
    {
        get => this.Items.Length;
        set
        {
            if (value <= this.Items.Length)
            {
                return;
            }

            int target = (int)BitOperations.RoundUpToPowerOf2((uint)value);
            T[] newItems = new T[target];
            if (this.Size > 0)
            {
                Array.Copy(this.Items, newItems, this.Size);
            }

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
            this.current = default;
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
            this.current = default;
            return false;
        }

        public void Reset()
        {
            this.index = 0;
            this.current = default;
        }
    }
}

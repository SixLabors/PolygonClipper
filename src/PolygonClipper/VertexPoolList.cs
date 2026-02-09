// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System.Collections;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace SixLabors.PolygonClipper;

/// <summary>
/// Pool-backed list of clip vertices reused across clipping operations.
/// </summary>
internal sealed class VertexPoolList : PooledList<ClipVertex>
{
    /// <summary>
    /// Adds or reuses a clip vertex initialized with the given data.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public ClipVertex Add(Vertex point, VertexFlags flags, ClipVertex? prev)
    {
        this.TryGrow();
        ClipVertex poolVertex = this.Items[this.Size];
        if (poolVertex == null)
        {
            poolVertex = new ClipVertex(point, flags, prev);
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
            OutputRecord active = this.Items[i];
            if (active == null || active.Path == Tombstone)
            {
                break;
            }

            // Mark paths so pooled records are not accidentally reused without reset.
            active.Path = Tombstone;
            active.Owner = null;
            active.FrontEdge = null;
            active.BackEdge = null;
            active.Points = null;
            active.RecursiveSplit = null;
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
internal abstract class PooledList<T> : IReadOnlyList<T>
    where T : class
{
    private const int DefaultCapacity = 4;

    protected PooledList() => this.Items = [];

    public int Count => this.Size;

    protected T[] Items { get; private set; }

    protected int Size { get; set; }

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

    public T this[int index]
    {
        get
        {
            if ((uint)index >= (uint)this.Size)
            {
                throw new ArgumentOutOfRangeException(nameof(index));
            }

            return this.Items[index];
        }
    }

    public void EnsureCapacity(int capacity) => this.Capacity = capacity;

    public virtual void Clear() => this.Size = 0;

    public PooledListEnumerator<T> GetEnumerator() => new(this);

    IEnumerator<T> IEnumerable<T>.GetEnumerator() => new PooledListEnumerator<T>(this);

    IEnumerator IEnumerable.GetEnumerator() => new PooledListEnumerator<T>(this);

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

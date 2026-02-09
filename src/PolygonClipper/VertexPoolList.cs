// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System.Collections;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace SixLabors.PolygonClipper;

internal sealed class VertexPoolList : PooledList<ClipVertex>
{
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
            poolVertex.Point = point;
            poolVertex.Flags = flags;
            poolVertex.Prev = prev;
            poolVertex.Next = null;
        }

        this.Size++;
        return poolVertex;
    }
}

internal sealed class OutPtPoolList : PooledList<OutPt>
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public OutPt Add(Vertex pt, OutRec outrec)
    {
        this.TryGrow();
        OutPt poolPt = this.Items[this.Size];
        if (poolPt == null)
        {
            poolPt = new OutPt(pt, outrec);
            this.Items[this.Size] = poolPt;
        }
        else
        {
            poolPt.Point = pt;
            poolPt.OutRec = outrec;
            poolPt.Next = poolPt;
            poolPt.Prev = poolPt;
            poolPt.Horz = null;
        }

        this.Size++;
        outrec.OutPointCount++;
        return poolPt;
    }
}

internal sealed class OutRecPoolList : PooledList<OutRec>
{
    private static readonly Contour Tombstone = [];

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public OutRec Add()
    {
        this.TryGrow();
        OutRec outRec = this.Items[this.Size];
        if (outRec == null)
        {
            outRec = new OutRec();
            this.Items[this.Size] = outRec;
        }
        else
        {
            outRec.Index = 0;
            outRec.OutPointCount = 0;
            outRec.Owner = null;
            outRec.FrontEdge = null;
            outRec.BackEdge = null;
            outRec.Points = null;
            outRec.Bounds = default;
            outRec.Path.Clear();
            outRec.PolyPath = null;
            outRec.IsOpen = false;
            outRec.Splits?.Clear();
            outRec.RecursiveSplit = null;
        }

        this.Size++;
        return outRec;
    }

    public override void Clear()
    {
        base.Clear();
        for (int i = 0; i < this.Items.Length; i++)
        {
            OutRec active = this.Items[i];
            if (active == null || active.Path == Tombstone)
            {
                break;
            }

            active.Path = Tombstone;
            active.Owner = null;
            active.FrontEdge = null;
            active.BackEdge = null;
            active.Points = null;
            active.PolyPath = null;
            active.RecursiveSplit = null;
        }
    }
}

internal sealed class HorzJoinPoolList : PooledList<HorzJoin>
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public HorzJoin Add(OutPt ltor, OutPt rtol)
    {
        this.TryGrow();
        HorzJoin hJoin = this.Items[this.Size];
        if (hJoin == null)
        {
            hJoin = new HorzJoin(ltor, rtol);
            this.Items[this.Size] = hJoin;
        }
        else
        {
            hJoin.Op1 = ltor;
            hJoin.Op2 = rtol;
        }

        this.Size++;
        return hJoin;
    }
}

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

// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System.Runtime.CompilerServices;

namespace SixLabors.PolygonClipper;

/// <summary>
/// Manages scanline ordering and local minima scheduling for the sweep line.
/// </summary>
/// <remarks>
/// This type keeps local minima in sorted order and seeds scanlines from their Y coordinates.
/// It also provides ordered scanline pop/insert operations used during the sweep.
/// </remarks>
internal sealed class ScanlineSchedule
{
    private static readonly LocalMinimaComparer LocalMinimaComparerInstance = new();

    private ArrayBuilder<LocalMinima> localMinima;
    private readonly List<double> scanlines;
    private int localMinimaIndex;
    private bool isLocalMinimaSorted;

    /// <summary>
    /// Initializes a new instance of the <see cref="ScanlineSchedule"/> class.
    /// </summary>
    public ScanlineSchedule()
    {
        this.localMinima = new ArrayBuilder<LocalMinima>(16);
        this.scanlines = [];
    }

    /// <summary>
    /// Gets the number of registered local minima.
    /// </summary>
    public int LocalMinimaCount => this.localMinima.Length;

    /// <summary>
    /// Gets a retained-capacity score used to decide pooling reuse.
    /// </summary>
    public int RetainedCapacityScore => this.localMinima.Capacity + this.scanlines.Capacity;

    /// <summary>
    /// Adds a local minima to the schedule.
    /// </summary>
    /// <param name="localMinima">The minima to append.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void AddLocalMinima(in LocalMinima localMinima)
    {
        this.localMinima.Add(localMinima);
        this.isLocalMinimaSorted = false;
    }

    /// <summary>
    /// Marks the local minima list as unsorted.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void MarkDirty() => this.isLocalMinimaSorted = false;

    /// <summary>
    /// Clears all minima and scanline state.
    /// </summary>
    public void Clear()
    {
        this.localMinima.Clear();
        this.scanlines.Clear();
        this.localMinimaIndex = 0;
        this.isLocalMinimaSorted = false;
    }

    /// <summary>
    /// Clears scanlines while keeping local minima intact.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void ClearScanlines() => this.scanlines.Clear();

    /// <summary>
    /// Sorts minima (if needed) and seeds the scanline list.
    /// </summary>
    public void Reset()
    {
        if (!this.isLocalMinimaSorted)
        {
            this.localMinima.Sort(LocalMinimaComparerInstance);
            this.isLocalMinimaSorted = true;
        }

        this.scanlines.Clear();
        int localMinimaCount = this.localMinima.Length;
        this.scanlines.EnsureCapacity(localMinimaCount);
        for (int i = localMinimaCount - 1; i >= 0; i--)
        {
            this.scanlines.Add(this.localMinima[i].Vertex.Point.Y);
        }

        this.localMinimaIndex = 0;
    }

    /// <summary>
    /// Determines whether the next local minima is on the given scanline.
    /// </summary>
    /// <param name="y">The scanline Y coordinate.</param>
    /// <returns><see langword="true"/> if a minima exists at this Y.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool HasLocalMinimaAtY(double y)
        => this.localMinimaIndex < this.localMinima.Length &&
           this.localMinima[this.localMinimaIndex].Vertex.Point.Y == y;

    /// <summary>
    /// Pops the next local minima from the schedule.
    /// </summary>
    /// <returns>The next local minima.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public LocalMinima PopLocalMinima() => this.localMinima[this.localMinimaIndex++];

    /// <summary>
    /// Inserts a scanline value into the ordered list.
    /// </summary>
    /// <param name="y">The scanline Y coordinate.</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void InsertScanline(double y)
    {
        int index = this.scanlines.BinarySearch(y);
        if (index >= 0)
        {
            return;
        }

        index = ~index;
        this.scanlines.Insert(index, y);
    }

    /// <summary>
    /// Pops the next scanline from the schedule.
    /// </summary>
    /// <param name="y">The popped scanline value.</param>
    /// <returns><see langword="true"/> when a scanline was available.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool TryPopScanline(out double y)
    {
        int count = this.scanlines.Count - 1;
        if (count < 0)
        {
            y = 0;
            return false;
        }

        y = this.scanlines[count];
        this.scanlines.RemoveAt(count--);
        while (count >= 0 && y == this.scanlines[count])
        {
            this.scanlines.RemoveAt(count--);
        }

        return true;
    }
}

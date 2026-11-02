// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System.Runtime.CompilerServices;

namespace SixLabors.PolygonClipper;

/// <summary>
/// Compares queued endpoints with the shared sweep event order.
/// </summary>
internal readonly struct QueueEntryComparer : IComparer<QueueEntry>
{
    /// <inheritdoc/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public int Compare(QueueEntry x, QueueEntry y)
    {
        // A value-type comparer lets the JIT inline heap comparisons.
        // Keep the ordering in SweepEventComparer so all callers use the same rules.
        return SweepEventComparer.Instance.Compare(x, y);
    }
}

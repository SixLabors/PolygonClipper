// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System.Collections;

namespace SixLabors.PolygonClipper;

/// <summary>
/// Compares two <see cref="SweepEvent"/> instances for sorting in the event queue.
/// </summary>
internal sealed class SweepEventComparer : IComparer<SweepEvent>, IComparer<QueueEntry>, IComparer<Vertex>, IComparer
{
    /// <summary>
    /// The shared comparer for point, queue, and boundary event order.
    /// </summary>
    public static readonly SweepEventComparer Instance = new();

    /// <inheritdoc/>
    public int Compare(Vertex x, Vertex y)
    {
        int order = x.X.CompareTo(y.X);
        return order == 0 ? x.Y.CompareTo(y.Y) : order;
    }

    /// <inheritdoc/>
    public int Compare(SweepEvent? x, SweepEvent? y)
    {
        if (ReferenceEquals(x, y))
        {
            return 0;
        }

        if (x == null)
        {
            return -1;
        }

        if (y == null)
        {
            return 1;
        }

        int pointOrder = this.Compare(x.Point, y.Point);
        if (pointOrder != 0)
        {
            return pointOrder;
        }

        // Compare left vs. right endpoint
        if (x.Left != y.Left)
        {
            return x.Left ? 1 : -1;
        }

        // Compare collinearity using signed area
        double area = PolygonUtilities.SignedArea(x.Point, x.OtherEvent.Point, y.OtherEvent.Point);
        if (area != 0)
        {
            return x.IsBelow(y.OtherEvent.Point) ? -1 : 1;
        }

        // Compare by polygon type: subject polygons have higher priority
        int order = ((int)x.PolygonType).CompareTo((int)y.PolygonType);
        if (order == 0)
        {
            // Collinear rays can have different lengths. Order their other
            // endpoints consistently rather than declaring each event smaller.
            order = x.OtherEvent.Point.X.CompareTo(y.OtherEvent.Point.X);
            if (order == 0)
            {
                order = x.OtherEvent.Point.Y.CompareTo(y.OtherEvent.Point.Y);
            }

            // At either endpoint, the shorter collinear segment comes first.
            if (!x.Left)
            {
                order = -order;
            }
        }

        return order;
    }

    /// <inheritdoc/>
    public int Compare(QueueEntry x, QueueEntry y)
    {
        int order = this.Compare(x.Point, y.Point);
        if (order == 0)
        {
            order = x.EdgeIndex.CompareTo(y.EdgeIndex);
        }

        if (order == 0)
        {
            order = x.Version.CompareTo(y.Version);
        }

        return order == 0 ? x.IsStart.CompareTo(y.IsStart) : order;
    }

    /// <inheritdoc/>
    public int Compare(object? x, object? y)
    {
        if (x is SweepEvent a && y is SweepEvent b)
        {
            return this.Compare(a, b);
        }

        throw new ArgumentException("Both arguments must be of type SweepEvent.", nameof(x));
    }
}

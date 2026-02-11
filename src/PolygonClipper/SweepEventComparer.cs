// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System.Collections;

namespace SixLabors.PolygonClipper;

/// <summary>
/// Compares two <see cref="SweepEvent"/> instances for sorting in the event queue.
/// </summary>
internal sealed class SweepEventComparer : IComparer<SweepEvent>, IComparer
{
    /// <inheritdoc/>
    public int Compare(SweepEvent? x, SweepEvent? y)
    {
        if (x == null)
        {
            return -1;
        }

        if (y == null)
        {
            return 1;
        }

        Vertex xPoint = x.Point;
        Vertex yPoint = y.Point;

        // Compare by x-coordinate
        if (xPoint.X > yPoint.X)
        {
            return 1;
        }

        if (xPoint.X < yPoint.X)
        {
            return -1;
        }

        // Compare by y-coordinate when x-coordinates are the same
        if (xPoint.Y != yPoint.Y)
        {
            return xPoint.Y > yPoint.Y ? 1 : -1;
        }

        // Compare left vs. right endpoint
        if (x.Left != y.Left)
        {
            return x.Left ? 1 : -1;
        }

        Vertex xOtherPoint = x.OtherEvent.Point;
        Vertex yOtherPoint = y.OtherEvent.Point;

        // Compare collinearity using signed area
        double area = PolygonUtilities.SignedArea(xPoint, xOtherPoint, yOtherPoint);
        if (area == 0D)
        {
            // Compare by polygon type: subject polygons have higher priority
            return x.PolygonType != PolygonType.Subject && y.PolygonType == PolygonType.Subject ? 1 : -1;
        }

        bool isBelow = x.Left ? area > 0D : area < 0D;
        return isBelow ? -1 : 1;
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

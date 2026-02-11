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

        return CompareEvents(x, y);
    }

    internal static int CompareEvents(SweepEvent x, SweepEvent y)
    {
        if (PolygonUtilities.UseFloatingScale)
        {
            Vertex xPoint = x.PointDouble;
            Vertex yPoint = y.PointDouble;

            if (xPoint.X > yPoint.X)
            {
                return 1;
            }

            if (xPoint.X < yPoint.X)
            {
                return -1;
            }

            if (xPoint.Y != yPoint.Y)
            {
                return xPoint.Y > yPoint.Y ? 1 : -1;
            }

            if (x.Left != y.Left)
            {
                return x.Left ? 1 : -1;
            }

            Vertex xOtherPoint = x.OtherEvent.PointDouble;
            Vertex yOtherPoint = y.OtherEvent.PointDouble;

            double areaValue = ((xPoint.X - yOtherPoint.X) * (xOtherPoint.Y - yOtherPoint.Y)) -
                               ((xOtherPoint.X - yOtherPoint.X) * (xPoint.Y - yOtherPoint.Y));
            if (areaValue == 0D)
            {
                return x.PolygonType != PolygonType.Subject && y.PolygonType == PolygonType.Subject ? 1 : -1;
            }

            bool isBelowValue = x.Left ? areaValue > 0D : areaValue < 0D;
            return isBelowValue ? -1 : 1;
        }

        Vertex64 xPointFixed = x.Point;
        Vertex64 yPointFixed = y.Point;
        Vertex64 xOtherPointFixed = x.OtherEvent.Point;
        Vertex64 yOtherPointFixed = y.OtherEvent.Point;

        // Compare by x-coordinate
        if (xPointFixed.X > yPointFixed.X)
        {
            return 1;
        }

        if (xPointFixed.X < yPointFixed.X)
        {
            return -1;
        }

        // Compare by y-coordinate when x-coordinates are the same
        if (xPointFixed.Y != yPointFixed.Y)
        {
            return xPointFixed.Y > yPointFixed.Y ? 1 : -1;
        }

        // Compare left vs. right endpoint
        if (x.Left != y.Left)
        {
            return x.Left ? 1 : -1;
        }

        // Compare collinearity using signed area
        int area = PolygonUtilities.CrossSign(xPointFixed, xOtherPointFixed, yOtherPointFixed);
        if (area == 0)
        {
            // Compare by polygon type: subject polygons have higher priority
            return x.PolygonType != PolygonType.Subject && y.PolygonType == PolygonType.Subject ? 1 : -1;
        }

        bool isBelow = x.Left ? area > 0 : area < 0;
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

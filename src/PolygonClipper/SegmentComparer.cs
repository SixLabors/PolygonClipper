// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System.Collections;
using System.Runtime.CompilerServices;

namespace SixLabors.PolygonClipper;

/// <summary>
/// Allows the comparison of segments for sorting.
/// </summary>
internal sealed class SegmentComparer : IComparer<SweepEvent>, IComparer
{
    /// <inheritdoc/>
    public int Compare(SweepEvent? x, SweepEvent? y)
    {
        // If the events are the same, return 0 (no order difference)
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

        bool inversed = !x.IsBefore(y);
        SweepEvent perhapsInversedX = inversed ? y : x;
        SweepEvent perhapsInversedY = inversed ? x : y;

        if (PolygonUtilities.UseFloatingScale)
        {
            Vertex xPointDouble = perhapsInversedX.PointDouble;
            Vertex yPointDouble = perhapsInversedY.PointDouble;
            Vertex xOtherDouble = perhapsInversedX.OtherEvent.PointDouble;
            Vertex yOtherDouble = perhapsInversedY.OtherEvent.PointDouble;

            double area1Double = ((xPointDouble.X - yPointDouble.X) * (xOtherDouble.Y - yPointDouble.Y)) -
                                 ((xOtherDouble.X - yPointDouble.X) * (xPointDouble.Y - yPointDouble.Y));
            double area2Double = ((xPointDouble.X - yOtherDouble.X) * (xOtherDouble.Y - yOtherDouble.Y)) -
                                 ((xOtherDouble.X - yOtherDouble.X) * (xPointDouble.Y - yOtherDouble.Y));

            if (area1Double != 0D || area2Double != 0D)
            {
                if (xPointDouble == yPointDouble)
                {
                    bool isBelow = perhapsInversedX.Left ? area2Double > 0D : area2Double < 0D;
                    return LessIf(isBelow, inversed);
                }

                if (xPointDouble.X == yPointDouble.X)
                {
                    return LessIf(xPointDouble.Y < yPointDouble.Y, inversed);
                }

                if ((area1Double > 0D) == (area2Double > 0D))
                {
                    return LessIf(area1Double > 0D, inversed);
                }

                if (area1Double == 0D)
                {
                    return LessIf(area2Double > 0D, inversed);
                }

                int interResult = PolygonUtilities.FindIntersectionDouble(
                    xPointDouble,
                    xOtherDouble,
                    yPointDouble,
                    yOtherDouble,
                    out Vertex pi0,
                    out Vertex _);

                if (interResult == 0)
                {
                    return LessIf(area1Double > 0D, inversed);
                }

                if (interResult == 1)
                {
                    if (pi0 == yPointDouble)
                    {
                        return LessIf(area2Double > 0D, inversed);
                    }

                    return LessIf(area1Double > 0D, inversed);
                }
            }

            if (perhapsInversedX.PolygonType == perhapsInversedY.PolygonType)
            {
                if (xPointDouble == yPointDouble)
                {
                    return LessIf(perhapsInversedX.ContourId < perhapsInversedY.ContourId, inversed);
                }

                return LessIf(true, inversed);
            }

            return LessIf(perhapsInversedX.PolygonType == PolygonType.Subject, inversed);
        }

        Vertex64 xPoint = perhapsInversedX.Point;
        Vertex64 yPoint = perhapsInversedY.Point;
        Vertex64 xOtherPoint = perhapsInversedX.OtherEvent.Point;
        Vertex64 yOtherPoint = perhapsInversedY.OtherEvent.Point;

        // Check if the segments are collinear by comparing their signed areas
        int area1 = PolygonUtilities.CrossSign(xPoint, xOtherPoint, yPoint);
        int area2 = PolygonUtilities.CrossSign(xPoint, xOtherPoint, yOtherPoint);

        if (area1 != 0 || area2 != 0)
        {
            // Segments are not collinear
            // If they share their left endpoint, use the right endpoint to sort
            if (xPoint == yPoint)
            {
                bool isBelow = perhapsInversedX.Left ? area2 > 0 : area2 < 0;
                return LessIf(isBelow, inversed);
            }

            // Different left endpoints: use the y-coordinate to sort if x-coordinates are the same
            if (xPoint.X == yPoint.X)
            {
                return LessIf(xPoint.Y < yPoint.Y, inversed);
            }

            // If `x` and `y` lie on the same side of the reference segment,
            // no intersection check is necessary.
            if ((area1 > 0) == (area2 > 0))
            {
                return LessIf(area1 > 0, inversed);
            }

            // If `x` lies on the reference segment, compare based on `y`.
            if (area1 == 0)
            {
                return LessIf(area2 > 0, inversed);
            }

            // Form segments from the events.
            Segment seg0 = new(xPoint, xOtherPoint);
            Segment seg1 = new(yPoint, yOtherPoint);

            // Call the provided intersection method.
            int interResult = PolygonUtilities.FindIntersection(seg0, seg1, out Vertex64 pi0, out Vertex64 _);
            if (interResult == 0)
            {
                // No unique intersection found: decide based on area1.
                return LessIf(area1 > 0, inversed);
            }

            if (interResult == 1)
            {
                // Unique intersection found.
                if (pi0 == perhapsInversedY.Point)
                {
                    return LessIf(area2 > 0, inversed);
                }

                return LessIf(area1 > 0, inversed);
            }

            // If interResult is neither 0 nor 1, fall through to collinear logic.
        }

        // Collinear branch - mimicking the Rust logic:
        if (perhapsInversedX.PolygonType == perhapsInversedY.PolygonType)
        {
            // Both segments belong to the same polygon.
            if (xPoint == yPoint)
            {
                // When left endpoints are identical, order by contour id.
                return LessIf(perhapsInversedX.ContourId < perhapsInversedY.ContourId, inversed);
            }

            // If left endpoints differ, the Rust version simply returns "less" (i.e. the one inserted earlier).
            // Here we mimic that by always returning -1.
            return LessIf(true, inversed);
        }

        // Segments are collinear but belong to different polygons.
        return LessIf(perhapsInversedX.PolygonType == PolygonType.Subject, inversed);
    }

    /// <inheritdoc/>
    public int Compare(object? x, object? y)
    {
        if (x == null)
        {
            return -1;
        }

        if (y == null)
        {
            return 1;
        }

        if (x is SweepEvent a && y is SweepEvent b)
        {
            return this.Compare(a, b);
        }

        throw new ArgumentException("Both arguments must be of type SweepEvent.", nameof(x));
    }

    /// <summary>
    /// Converts a boolean comparison result to an ordering value.
    /// Returns -1 if the condition is true, 1 if false.
    /// </summary>
    /// <param name="condition">The boolean condition to evaluate.</param>
    /// <param name="inversed">Should the result be inversed.</param>
    /// <returns>-1 if condition is true, 1 if false.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static int LessIf(bool condition, bool inversed = false) => condition ^ inversed ? -1 : 1;
}

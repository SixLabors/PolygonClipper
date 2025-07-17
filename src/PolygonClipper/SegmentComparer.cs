// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System.Collections;
using System.Runtime.CompilerServices;

namespace PolygonClipper;

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

        SweepEvent perhapsInversedX, perhapsInversedY;
        bool inversed;

        if (x.IsBefore(y))
        {
            perhapsInversedX = x;
            perhapsInversedY = y;
            inversed = false;
        }
        else
        {
            perhapsInversedX = y;
            perhapsInversedY = x;
            inversed = true;
        }

        // Check if the segments are collinear by comparing their signed areas
        double area1 = PolygonUtilities.SignedArea(perhapsInversedX.Point, perhapsInversedX.OtherEvent.Point, perhapsInversedY.Point);
        double area2 = PolygonUtilities.SignedArea(perhapsInversedX.Point, perhapsInversedX.OtherEvent.Point, perhapsInversedY.OtherEvent.Point);

        if (area1 != 0 || area2 != 0)
        {
            // Segments are not collinear
            // If they share their left endpoint, use the right endpoint to sort
            if (perhapsInversedX.Point == perhapsInversedY.Point)
            {
                return LessIf(perhapsInversedX.Below(perhapsInversedY.OtherEvent.Point), inversed);
            }

            // Different left endpoints: use the y-coordinate to sort if x-coordinates are the same
            if (perhapsInversedX.Point.X == perhapsInversedY.Point.X)
            {
                return LessIf(perhapsInversedX.Point.Y < perhapsInversedY.Point.Y, inversed);
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
            Segment seg0 = new(perhapsInversedX.Point, perhapsInversedX.OtherEvent.Point);
            Segment seg1 = new(perhapsInversedY.Point, perhapsInversedY.OtherEvent.Point);

            // Call the provided intersection method.
            int interResult = PolygonUtilities.FindIntersection(seg0, seg1, out Vertex pi0, out Vertex _);

            if (interResult == 0)
            {
                // No unique intersection found: decide based on area1.
                return LessIf(area1 > 0, inversed);
            }
            else if (interResult == 1)
            {
                // Unique intersection found.
                if (pi0 == y.Point)
                {
                    return LessIf(area2 > 0, inversed);
                }

                return LessIf(area1 > 0, inversed);
            }

            // If interResult is neither 0 nor 1, fall through to collinear logic.
        }

        // Collinear branch – mimicking the Rust logic:
        if (perhapsInversedX.PolygonType == perhapsInversedY.PolygonType)
        {
            // Both segments belong to the same polygon.
            if (perhapsInversedX.Point == perhapsInversedY.Point)
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

// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System;
using System.Collections;
using System.Collections.Generic;

namespace PolygonClipper;

/// <summary>
/// Allows the comparison of segments for sorting.
/// </summary>
internal sealed class SegmentComparer : IComparer<SweepEvent>, IComparer
{
    private readonly SweepEventComparer eventComparer = new();

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

        // Check if the segments are collinear by comparing their signed areas
        double area1 = PolygonUtilities.SignedArea(x.Point, x.OtherEvent.Point, y.Point);
        double area2 = PolygonUtilities.SignedArea(x.Point, x.OtherEvent.Point, y.OtherEvent.Point);

        if (area1 != 0 || area2 != 0)
        {
            // Segments are not collinear
            // If they share their left endpoint, use the right endpoint to sort
            if (x.Point == y.Point)
            {
                return x.Below(y.OtherEvent.Point) ? -1 : 1;
            }

            // Different left endpoints: use the y-coordinate to sort if x-coordinates are the same
            if (x.Point.X == y.Point.X)
            {
                return x.Point.Y < y.Point.Y ? -1 : 1;
            }

            // If `x` and `y` lie on the same side of the reference segment,
            // no intersection check is necessary.
            if ((area1 > 0) == (area2 > 0))
            {
                return area1 > 0 ? -1 : 1;
            }

            // If `x` lies on the reference segment, compare based on `y`.
            if (area1 == 0)
            {
                return area2 > 0 ? -1 : 1;
            }

            // Form segments from the events.
            Segment seg0 = new(x.Point, x.OtherEvent.Point);
            Segment seg1 = new(y.Point, y.OtherEvent.Point);

            // Call the provided intersection method.
            int interResult = PolygonUtilities.FindIntersection(seg0, seg1, out Vertex pi0, out Vertex _);

            if (interResult == 0)
            {
                // No unique intersection found: decide based on area1.
                return (area1 > 0) ? -1 : 1;
            }
            else if (interResult == 1)
            {
                // Unique intersection found.
                if (pi0 == y.Point)
                {
                    return (area2 > 0) ? -1 : 1;
                }

                return (area1 > 0) ? -1 : 1;
            }

            // If interResult is neither 0 nor 1, fall through to collinear logic.
        }

        // Collinear branch – mimicking the Rust logic:
        if (x.PolygonType == y.PolygonType)
        {
            // Both segments belong to the same polygon.
            if (x.Point == y.Point)
            {
                // When left endpoints are identical, order by contour id.
                return (x.ContourId < y.ContourId) ? -1 : 1;
            }

            // If left endpoints differ, the Rust version simply returns "less" (i.e. the one inserted earlier).
            // Here we mimic that by always returning -1.
            return -1;
        }

        // Segments are collinear but belong to different polygons.
        return (x.PolygonType == PolygonType.Subject) ? -1 : 1;
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
}

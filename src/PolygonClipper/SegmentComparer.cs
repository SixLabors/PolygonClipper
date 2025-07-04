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

        SweepEvent se_old_l, se_new_l;
        Func<bool, int> less_if;

        if (x.IsBefore(y))
        {
            se_old_l = x;
            se_new_l = y;
            less_if = Helper.LessIf;
        }
        else
        {
            se_old_l = y;
            se_new_l = x;
            less_if = Helper.LessIfInversed;
        }

        // Check if the segments are collinear by comparing their signed areas
        double area1 = PolygonUtilities.SignedArea(se_old_l.Point, se_old_l.OtherEvent.Point, se_new_l.Point);
        double area2 = PolygonUtilities.SignedArea(se_old_l.Point, se_old_l.OtherEvent.Point, se_new_l.OtherEvent.Point);

        if (area1 != 0 || area2 != 0)
        {
            // Segments are not collinear
            // If they share their left endpoint, use the right endpoint to sort
            if (se_old_l.Point == se_new_l.Point)
            {
                return less_if(se_old_l.Below(se_new_l.OtherEvent.Point));
            }

            // Different left endpoints: use the y-coordinate to sort if x-coordinates are the same
            if (se_old_l.Point.X == se_new_l.Point.X)
            {
                return less_if(se_old_l.Point.Y < se_new_l.Point.Y);
            }

            // If `x` and `y` lie on the same side of the reference segment,
            // no intersection check is necessary.
            if ((area1 > 0) == (area2 > 0))
            {
                return less_if(area1 > 0);
            }

            // If `x` lies on the reference segment, compare based on `y`.
            if (area1 == 0)
            {
                return less_if(area2 > 0);
            }

            // Form segments from the events.
            Segment seg0 = new(se_old_l.Point, se_old_l.OtherEvent.Point);
            Segment seg1 = new(se_new_l.Point, se_new_l.OtherEvent.Point);

            // Call the provided intersection method.
            int interResult = PolygonUtilities.FindIntersection(seg0, seg1, out Vertex pi0, out Vertex _);

            if (interResult == 0)
            {
                // No unique intersection found: decide based on area1.
                return less_if(area1 > 0);
            }
            else if (interResult == 1)
            {
                // Unique intersection found.
                if (pi0 == y.Point)
                {
                    return less_if(area2 > 0);
                }

                return less_if(area1 > 0);
            }

            // If interResult is neither 0 nor 1, fall through to collinear logic.
        }

        // Collinear branch – mimicking the Rust logic:
        if (se_old_l.PolygonType == se_new_l.PolygonType)
        {
            // Both segments belong to the same polygon.
            if (se_old_l.Point == se_new_l.Point)
            {
                // When left endpoints are identical, order by contour id.
                return less_if(se_old_l.ContourId < se_new_l.ContourId);
            }

            // If left endpoints differ, the Rust version simply returns "less" (i.e. the one inserted earlier).
            // Here we mimic that by always returning -1.
            return less_if(true);
        }

        // Segments are collinear but belong to different polygons.
        return less_if(se_old_l.PolygonType == PolygonType.Subject);
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

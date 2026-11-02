// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

namespace SixLabors.PolygonClipper;

/// <summary>
/// Expands or contracts the filled area of closed polygons.
/// </summary>
public static class PolygonOffsetter
{
    /// <summary>
    /// Expands or contracts the filled area of closed contours by the specified distance.
    /// </summary>
    /// <param name="polygon">Closed source contours. Their vertices and directions are not modified.</param>
    /// <param name="delta">A finite distance. Positive expands the area; negative contracts it.</param>
    /// <param name="fillRule">The rule that determines the source filled area.</param>
    /// <param name="options">The join style, miter limit and arc detail.</param>
    /// <returns>
    /// Closed boundaries with their full containment hierarchy. Exterior contours and islands are counterclockwise.
    /// Holes are clockwise. Collapsed areas disappear. Zero distance returns a detached, resolved copy.
    /// </returns>
    /// <exception cref="ArgumentNullException">The polygon or options are null.</exception>
    /// <exception cref="ArgumentException">The polygon contains an open contour.</exception>
    /// <exception cref="ArgumentOutOfRangeException">The distance is not finite or the fill rule is invalid.</exception>
    public static Polygon Offset(Polygon polygon, double delta, FillRule fillRule, OffsetOptions options)
    {
        Guard.NotNull(polygon, nameof(polygon));
        Guard.NotNull(options, nameof(options));
        Guard.MustBeBetweenOrEqualTo(delta, double.MinValue, double.MaxValue, nameof(delta));

        for (int i = 0; i < polygon.Count; i++)
        {
            Guard.IsTrue(polygon[i].IsClosed, nameof(polygon), "Offset requires closed contours. Use Stroke for open paths.");
        }

        // Resolve the requested filled area before outlining it. Raw overlaps and self-crossings
        // are not independent boundaries and must not introduce internal offset seams.
        Polygon area = PolygonClipper.Normalize(polygon, fillRule);
        if (delta == 0 || area.Count == 0)
        {
            return area;
        }

        if (area[0].HoleCount == area.Count - 1)
        {
            // One exterior with its direct holes needs no component merge.
            return OffsetComponent(area, delta, options);
        }

        Polygon combined = new(area.Count);
        for (int contourIndex = 0; contourIndex < area.Count; contourIndex++)
        {
            Contour exterior = area[contourIndex];
            if ((exterior.Depth & 1) != 0)
            {
                continue;
            }

            Polygon component = new(exterior.HoleCount + 1) { exterior };
            for (int i = 0; i < exterior.HoleCount; i++)
            {
                component.Add(area[exterior.GetHoleIndex(i)]);
            }

            Polygon offset = OffsetComponent(component, delta, options);
            for (int i = 0; i < offset.Count; i++)
            {
                combined.Add(offset[i]);
            }
        }

        // A collapsed component can have negative winding outside its source boundary.
        // Resolve that winding before it can cancel a retained area in another component.
        // The final union merges expansions and restores hierarchy between separate components.
        return PolygonClipper.Normalize(combined, FillRule.NonZero);
    }

    /// <summary>
    /// Offsets one resolved exterior and its direct holes without changing their coordinates.
    /// </summary>
    /// <param name="component">One nonempty filled component with its exterior first.</param>
    /// <param name="delta">The validated nonzero signed distance.</param>
    /// <param name="options">The join and arc settings.</param>
    /// <returns>The resolved offset component, which can split or disappear.</returns>
    private static Polygon OffsetComponent(Polygon component, double delta, OffsetOptions options)
    {
        if (delta < 0)
        {
            Contour exterior = component[0];
            Box2 bounds = exterior.GetBoundingBox();
            ExtendedPrecisionDouble distance = new(-delta);
            ExtendedPrecisionDouble halfWidth = ExtendedPrecisionDouble.Difference(bounds.Max.X * 0.5D, bounds.Min.X * 0.5D);
            ExtendedPrecisionDouble halfHeight = ExtendedPrecisionDouble.Difference(bounds.Max.Y * 0.5D, bounds.Min.Y * 0.5D);

            // Retain the subtraction residual so a rounded box width cannot erase a surviving thin area.
            if ((distance - halfWidth).Value >= 0 || (distance - halfHeight).Value >= 0)
            {
                // A convex erosion must remain at least this distance from every supporting edge.
                // It cannot have area once that distance reaches half a bounding-box dimension.
                // Test convexity only for that case. This avoids constructing a dense collapsed walk.
                bool convex = true;
                for (int i = 0; i < exterior.Count; i++)
                {
                    if (PolygonUtilities.SignedArea(exterior[i], exterior[(i + 1) % exterior.Count], exterior[(i + 2) % exterior.Count]) < 0)
                    {
                        convex = false;
                        break;
                    }
                }

                if (convex)
                {
                    return [];
                }
            }
        }

        Polygon boundaries = PolygonStroker.OffsetBoundaries(component, delta, options);

        if (boundaries.Count == 1 && TryGetConvexStart(boundaries[0], out int start))
        {
            // A strictly convex counterclockwise walk already bounds exactly the
            // Positive-filled result. Match Martinez's leftmost starting vertex
            // and orientation without sending this resolved boundary through another sweep.
            Contour boundary = boundaries[0];
            if (start != 0)
            {
                boundary.RotateVertices(start);
            }

            boundary.SetCounterClockwise();
            return boundaries;
        }

        // Source winding is 0 or 1. Band coverage is nonnegative, including overlapping strips.
        // The displaced walk has source winding plus band coverage for expansion, or minus it
        // for contraction. Positive filling selects the union or the uncovered source area.
        // The source walk cancels in both expressions, so it need not enter the final sweep.
        return PolygonClipper.Normalize(boundaries, FillRule.Positive);
    }

    /// <summary>
    /// Checks for a simple, strictly convex counterclockwise walk and finds its first sweep vertex.
    /// </summary>
    /// <param name="contour">A closed displaced walk with at least three vertices.</param>
    /// <param name="start">The index of the lexicographically smallest vertex.</param>
    /// <returns>Whether every turn is positive and the edge directions make exactly one full turn.</returns>
    private static bool TryGetConvexStart(Contour contour, out int start)
    {
        ReadOnlySpan<Vertex> points = contour.AsVertexSpan();
        Vertex previous = points[^1];
        Vertex current = points[0];
        bool previousUpper = current.Y > previous.Y || (current.Y == previous.Y && current.X > previous.X);
        int turns = 0;
        start = 0;
        for (int i = 0; i < points.Length; i++)
        {
            Vertex next = points[i + 1 == points.Length ? 0 : i + 1];
            double turn = PolygonUtilities.SignedArea(previous, current, next);
            if (!double.IsFinite(current.X) || !double.IsFinite(current.Y) ||
                !double.IsFinite(turn) || turn <= 0)
            {
                return false;
            }

            // Positive turns restrict each direction change to less than half a turn.
            // Crossing from the lower half-plane to the upper half-plane counts a full
            // direction wrap. One wrap proves convexity; stars and repeated walks wrap more often.
            // Endpoint comparisons classify the half-plane without overflowing edge subtraction.
            bool upper = next.Y > current.Y || (next.Y == current.Y && next.X > current.X);
            if (!previousUpper && upper && ++turns > 1)
            {
                return false;
            }

            if (SweepEventComparer.Instance.Compare(current, points[start]) < 0)
            {
                start = i;
            }

            previousUpper = upper;
            previous = current;
            current = next;
        }

        return turns == 1;
    }
}

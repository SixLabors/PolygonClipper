// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System;
using System.Diagnostics.CodeAnalysis;
using System.Runtime.CompilerServices;

namespace PolygonClipper;

internal static class PolygonUtilities
{
    /// <summary>
    /// Returns the signed area of a triangle.
    /// </summary>
    /// <param name="p0">The first point.</param>
    /// <param name="p1">The second point.</param>
    /// <param name="p2">The third point.</param>
    /// <returns>The <see cref="double"/> area.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static double SignedArea2(Vertex p0, Vertex p1, Vertex p2)
        => ((p0.X - p2.X) * (p1.Y - p2.Y)) - ((p1.X - p2.X) * (p0.Y - p2.Y));

    /// <summary>
    /// Computes the robust signed area (actually twice the area) of the triangle defined by the three vertices.
    /// The sign indicates the orientation (positive for counterclockwise, negative for clockwise).
    /// </summary>
    /// <param name="p0">The first point.</param>
    /// <param name="p1">The second point.</param>
    /// <param name="p2">The third point.</param>
    /// <returns>The <see cref="double"/> area.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static double SignedArea(Vertex p0, Vertex p1, Vertex p2)
    {
        // Fast computation in double precision:
        double det = ((p1.X - p0.X) * (p2.Y - p0.Y)) - ((p1.Y - p0.Y) * (p2.X - p0.X));

        // If the determinant is clearly non-zero, return it.
        const double tolerance = 1e-12;
        if (Math.Abs(det) > tolerance)
        {
            return det;
        }

        // If the value is near zero, recompute using higher-precision arithmetic.
        decimal ax = (decimal)p0.X, ay = (decimal)p0.Y;
        decimal bx = (decimal)p1.X, by = (decimal)p1.Y;
        decimal cx = (decimal)p2.X, cy = (decimal)p2.Y;
        decimal detDec = ((bx - ax) * (cy - ay)) - ((by - ay) * (cx - ax));

        return (double)detDec;
    }

    /// <summary>
    /// Finds the intersection of two line segments, constraining results to their intersection bounding box.
    /// </summary>
    /// <param name="seg0">The first segment.</param>
    /// <param name="seg1">The second segment.</param>
    /// <param name="pi0">The first intersection point.</param>
    /// <param name="pi1">The second intersection point (if overlap occurs).</param>
    /// <returns>
    /// An <see cref="int"/> indicating the number of intersection points:
    /// - Returns 0 if there is no intersection.
    /// - Returns 1 if the segments intersect at a single point.
    /// - Returns 2 if the segments overlap.
    /// </returns>
    public static int FindIntersection(Segment seg0, Segment seg1, out Vertex pi0, out Vertex pi1)
    {
        pi0 = default;
        pi1 = default;

        if (!TryGetIntersectionBoundingBox(seg0.Source, seg0.Target, seg1.Source, seg1.Target, out Box2? bbox))
        {
            return 0;
        }

        int interResult = FindIntersectionImpl(seg0, seg1, out pi0, out pi1);

        if (interResult == 1)
        {
            pi0 = ConstrainToBoundingBox(pi0, bbox.Value);
        }
        else if (interResult == 2)
        {
            pi0 = ConstrainToBoundingBox(pi0, bbox.Value);
            pi1 = ConstrainToBoundingBox(pi1, bbox.Value);
        }

        return interResult;
    }

    /// <summary>
    /// Finds the intersection of two line segments.
    /// </summary>
    /// <param name="seg0">The first line segment.</param>
    /// <param name="seg1">The second line segment.</param>
    /// <param name="pi0">
    /// The first intersection point (if any). If the segments intersect at a single point, this will contain the intersection point.
    /// If the segments overlap, this will contain the start of the overlapping segment.
    /// </param>
    /// <param name="pi1">
    /// The second intersection point (if any). If the segments overlap, this will contain the end of the overlapping segment.
    /// </param>
    /// <returns>
    /// An <see cref="int"/> indicating the number of intersection points:
    /// - Returns 0 if there is no intersection.
    /// - Returns 1 if the segments intersect at a single point.
    /// - Returns 2 if the segments overlap.
    /// </returns>
    private static int FindIntersectionImpl(Segment seg0, Segment seg1, out Vertex pi0, out Vertex pi1)
    {
        pi0 = default;
        pi1 = default;

        Vertex a1 = seg0.Source;
        Vertex a2 = seg1.Source;

        Vertex va = seg0.Target - a1;
        Vertex vb = seg1.Target - a2;
        Vertex e = a2 - a1;

        double kross = Vertex.Cross(va, vb);
        double sqrKross = kross * kross;
        double sqrLenA = Vertex.Dot(va, va);

        if (sqrKross > 0)
        {
            // Lines of the segments are not parallel
            double s = Vertex.Cross(e, vb) / kross;
            if (s < 0 || s > 1)
            {
                return 0;
            }

            double t = Vertex.Cross(e, va) / kross;
            if (t < 0 || t > 1)
            {
                return 0;
            }

            // If s or t is exactly 0 or 1, the intersection is on an endpoint
            if (s == 0 || s == 1)
            {
                // On an endpoint of line segment a
                pi0 = MidPoint(a1, s, va);
                return 1;
            }

            if (t == 0 || t == 1)
            {
                // On an endpoint of line segment b
                pi0 = MidPoint(a2, t, vb);
                return 1;
            }

            // Intersection of lines is a point on each segment
            pi0 = a1 + (s * va);
            return 1;
        }

        // Lines are parallel; check if they are collinear
        kross = Vertex.Cross(e, va);
        sqrKross = kross * kross;
        if (sqrKross > 0)
        {
            // Lines of the segments are different
            return 0;
        }

        // Segments are collinear, check for overlap
        double sa = Vertex.Dot(va, e) / sqrLenA;
        double sb = sa + (Vertex.Dot(va, vb) / sqrLenA);
        double smin = Math.Min(sa, sb);
        double smax = Math.Max(sa, sb);

        if (smin <= 1 && smax >= 0)
        {
            if (smin == 1)
            {
                pi0 = MidPoint(a1, smin, va);
                return 1;
            }

            if (smax == 0)
            {
                pi0 = MidPoint(a1, smax, va);
                return 1;
            }

            pi0 = MidPoint(a1, Math.Max(smin, 0), va);
            pi1 = MidPoint(a1, Math.Min(smax, 1), va);
            return 2;
        }

        return 0;
    }

    /// <summary>
    /// Computes the bounding box of the intersection area of two line segments.
    /// </summary>
    /// <param name="a1">The first point of the first segment.</param>
    /// <param name="a2">The second point of the first segment.</param>
    /// <param name="b1">The first point of the second segment.</param>
    /// <param name="b2">The second point of the second segment.</param>
    /// <param name="result">The intersection bounding box if one exists, otherwise null.</param>
    /// <returns>
    /// <see langword="true"/> if the segments intersect; otherwise, <see langword="false"/>.
    /// </returns>
    private static bool TryGetIntersectionBoundingBox(Vertex a1, Vertex a2, Vertex b1, Vertex b2, [NotNullWhen(true)] out Box2? result)
    {
        Vertex minA = Vertex.Min(a1, a2);
        Vertex maxA = Vertex.Max(a1, a2);
        Vertex minB = Vertex.Min(b1, b2);
        Vertex maxB = Vertex.Max(b1, b2);

        Vertex interMin = Vertex.Max(minA, minB);
        Vertex interMax = Vertex.Min(maxA, maxB);

        if (interMin.X <= interMax.X && interMin.Y <= interMax.Y)
        {
            result = new Box2(interMin, interMax);
            return true;
        }

        result = null;
        return false;
    }

    /// <summary>
    /// Constrains a point to the given bounding box.
    /// </summary>
    /// <param name="p">The point to constrain.</param>
    /// <param name="bbox">The bounding box.</param>
    /// <returns>The constrained point.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vertex ConstrainToBoundingBox(Vertex p, Box2 bbox)
        => Vertex.Min(Vertex.Max(p, bbox.Min), bbox.Max);

    /// <summary>
    /// Computes the point at a given fractional distance along a directed line segment.
    /// </summary>
    /// <param name="p">The starting vertex of the segment.</param>
    /// <param name="s">The scalar factor representing the fractional distance along the segment.</param>
    /// <param name="d">The direction vector of the segment.</param>
    /// <returns>The interpolated vertex at the given fractional distance.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vertex MidPoint(Vertex p, double s, Vertex d) => p + (s * d);
}

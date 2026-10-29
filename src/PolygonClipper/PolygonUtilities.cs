// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System.Diagnostics.CodeAnalysis;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace SixLabors.PolygonClipper;

/// <summary>
/// Provides utility methods for performing geometric calculations related to polygons, such as calculating signed areas
/// and finding intersections of line segments.
/// </summary>
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
    public static double SignedArea(in Vertex p0, in Vertex p1, in Vertex p2)
    {
        Vertex a = p0 - p2;
        Vertex b = p1 - p2;
        double first = a.X * b.Y;
        double second = a.Y * b.X;
        double area = first - second;

        // The orientation filter includes rounding in coordinate differences
        // and products. Recover those differences only when cancellation can
        // obscure the sign. The bound selects precision; it does not merge points.
        const double errorFactor = 3.3306690738754716E-16D;
        if (first == 0 || second == 0 || !double.IsFinite(area) ||
            Math.Abs(area) > errorFactor * (Math.Abs(first) + Math.Abs(second)))
        {
            return area;
        }

        ExtendedPrecisionDouble ax = ExtendedPrecisionDouble.Difference(p0.X, p2.X);
        ExtendedPrecisionDouble ay = ExtendedPrecisionDouble.Difference(p0.Y, p2.Y);
        ExtendedPrecisionDouble bx = ExtendedPrecisionDouble.Difference(p1.X, p2.X);
        ExtendedPrecisionDouble by = ExtendedPrecisionDouble.Difference(p1.Y, p2.Y);
        return ((ax * by) - (ay * bx)).Value;
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
    public static int FindIntersection(in Segment seg0, in Segment seg1, out Vertex pi0, out Vertex pi1)
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
    private static int FindIntersectionImpl(in Segment seg0, in Segment seg1, out Vertex pi0, out Vertex pi1)
    {
        pi0 = default;
        pi1 = default;

        Vertex a1 = seg0.Source;
        Vertex a2 = seg1.Source;

        Vertex va = seg0.Target - a1;
        Vertex vb = seg1.Target - a2;
        Vertex e = a2 - a1;

        double kross = Vertex.Cross(va, vb);
        double sqrLenA = Vertex.Dot(va, va);

        // Squaring a representable non-zero cross product can underflow to zero,
        // incorrectly sending small crossing segments into the collinear branch.
        if (kross != 0)
        {
            // Lines of the segments are not parallel
            // Retain subtraction, determinant, and division roundoff until the
            // final coordinate is rounded. Independent segment pairs meeting at
            // one junction must construct the same point for event closure.
            ExtendedPrecisionDouble ax = ExtendedPrecisionDouble.Difference(seg0.Target.X, a1.X);
            ExtendedPrecisionDouble ay = ExtendedPrecisionDouble.Difference(seg0.Target.Y, a1.Y);
            ExtendedPrecisionDouble bx = ExtendedPrecisionDouble.Difference(seg1.Target.X, a2.X);
            ExtendedPrecisionDouble by = ExtendedPrecisionDouble.Difference(seg1.Target.Y, a2.Y);
            ExtendedPrecisionDouble ex = ExtendedPrecisionDouble.Difference(a2.X, a1.X);
            ExtendedPrecisionDouble ey = ExtendedPrecisionDouble.Difference(a2.Y, a1.Y);
            ExtendedPrecisionDouble denominator = (ax * by) - (ay * bx);
            ExtendedPrecisionDouble numerator = (ex * by) - (ey * bx);
            ExtendedPrecisionDouble otherNumerator = (ex * ay) - (ey * ax);
            double start = numerator.Value;
            double end = (numerator - denominator).Value;
            double otherStart = otherNumerator.Value;
            double otherEnd = (otherNumerator - denominator).Value;
            bool positive = denominator.Value > 0;
            if (positive ? start < 0 || end > 0 || otherStart < 0 || otherEnd > 0
                         : start > 0 || end < 0 || otherStart > 0 || otherEnd < 0)
            {
                return 0;
            }

            // A rounded quotient can equal one for an interior crossing.
            // Preserve a source endpoint only when the retained numerator
            // actually equals zero or the denominator.
            if (start == 0 || end == 0)
            {
                pi0 = start == 0 ? a1 : seg0.Target;
                return 1;
            }

            if (otherStart == 0 || otherEnd == 0)
            {
                pi0 = otherStart == 0 ? a2 : seg1.Target;
                return 1;
            }

            ExtendedPrecisionDouble parameter = numerator / denominator;
            ExtendedPrecisionDouble intersectionX = parameter * ax;
            ExtendedPrecisionDouble intersectionY = parameter * ay;
            intersectionX += new ExtendedPrecisionDouble(a1.X);
            intersectionY += new ExtendedPrecisionDouble(a1.Y);
            pi0 = new Vertex(intersectionX.Value, intersectionY.Value);
            return 1;
        }

        // Lines are parallel; check if they are collinear
        kross = Vertex.Cross(e, va);
        if (kross != 0)
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
                pi0 = seg0.Target;
                return 1;
            }

            if (smax == 0)
            {
                pi0 = a1;
                return 1;
            }

            // Overlap endpoints are existing source vertices. Reconstructing
            // them through a parameter can move a shared vertex by one ULP.
            pi0 = smin <= 0 ? a1 : sa <= sb ? a2 : seg1.Target;
            pi1 = smax >= 1 ? seg0.Target : sa >= sb ? a2 : seg1.Target;
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
    private static bool TryGetIntersectionBoundingBox(
        in Vertex a1,
        in Vertex a2,
        in Vertex b1,
        in Vertex b2,
        [NotNullWhen(true)] out Box2? result)
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
    private static Vertex ConstrainToBoundingBox(in Vertex p, in Box2 bbox)
        => Vertex.Min(Vertex.Max(p, bbox.Min), bbox.Max);

    /// <summary>
    /// Computes the point at a given fractional distance adouble a directed line segment.
    /// </summary>
    /// <param name="p">The starting vertex of the segment.</param>
    /// <param name="s">The scalar factor representing the fractional distance adouble the segment.</param>
    /// <param name="d">The direction vector of the segment.</param>
    /// <returns>The interpolated vertex at the given fractional distance.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vertex MidPoint(in Vertex p, double s, in Vertex d) => p + (s * d);

    /// <summary>
    /// Returns the dot product of the vectors AB and BC.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static double Dot(in Vertex a, in Vertex b, in Vertex c)
        => Vertex.Dot(b - a, c - b);

    /// <summary>
    /// Returns the cross product of the vectors AB and BC.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static double Cross(in Vertex a, in Vertex b, in Vertex c)
        => Vertex.Cross(b - a, c - b);

    /// <summary>
    /// Returns the sign of the cross product of the vectors AB and BC.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int CrossSign(in Vertex a, in Vertex b, in Vertex c)
    {
        double crossValueInt = Cross(a, b, c);
        if (crossValueInt == 0)
        {
            return 0;
        }

        return crossValueInt > 0 ? 1 : -1;
    }

    /// <summary>
    /// Returns true when three vertices are collinear.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool IsCollinear(in Vertex a, in Vertex shared, in Vertex b)
        => CrossSign(a, shared, b) == 0;

    /// <summary>
    /// Computes the signed area of a contour.
    /// </summary>
    public static double Area(List<Vertex> path) => Area(CollectionsMarshal.AsSpan(path));

    /// <summary>
    /// Computes the signed area of a contour.
    /// </summary>
    /// <param name="path">The contour vertices.</param>
    /// <returns>The signed area.</returns>
    public static double Area(ReadOnlySpan<Vertex> path)
    {
        int count = path.Length;
        if (count < 3)
        {
            return 0D;
        }

        double area = 0;

        // Translation does not change area. Subtract a local origin before the
        // products so a small ring far from zero does not lose its area or sign
        // when large absolute-coordinate products cancel.
        Vertex origin = path[0];
        Vertex prev = path[count - 1] - origin;
        for (int i = 0; i < count; i++)
        {
            Vertex current = path[i] - origin;
            area += Vertex.Cross(prev, current);
            prev = current;
        }

        return area * 0.5D;
    }

    /// <summary>
    /// Computes the squared perpendicular distance from a point to a line segment.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static double PerpendicularDistanceSquared(in Vertex point, in Vertex line1, in Vertex line2)
    {
        Vertex toPoint = point - line1;
        Vertex direction = line2 - line1;
        double lengthSquared = Vertex.Dot(direction, direction);
        if (lengthSquared == 0D)
        {
            return 0D;
        }

        double cross = Vertex.Cross(toPoint, direction);
        return (cross * cross) / lengthSquared;
    }

    /// <summary>
    /// Finds the intersection of two line segments, including endpoints.
    /// </summary>
    public static bool TryGetLineIntersection(
        in Vertex a1,
        in Vertex a2,
        in Vertex b1,
        in Vertex b2,
        out Vertex intersection)
    {
        double dy1 = a2.Y - a1.Y;
        double dx1 = a2.X - a1.X;
        double dy2 = b2.Y - b1.Y;
        double dx2 = b2.X - b1.X;
        double det = (dy1 * dx2) - (dy2 * dx1);
        if (det == 0D)
        {
            intersection = default;
            return false;
        }

        double t = (((a1.X - b1.X) * dy2) - ((a1.Y - b1.Y) * dx2)) / det;
        if (t <= 0D)
        {
            intersection = a1;
            return true;
        }

        if (t >= 1D)
        {
            intersection = a2;
            return true;
        }

        intersection = new Vertex(a1.X + (t * dx1), a1.Y + (t * dy1));

        return true;
    }

    /// <summary>
    /// Projects a point onto a segment and returns the closest point.
    /// </summary>
    public static Vertex ClosestPointOnSegment(in Vertex point, in Vertex seg1, in Vertex seg2)
    {
        if (seg1 == seg2)
        {
            return seg1;
        }

        double dx = seg2.X - seg1.X;
        double dy = seg2.Y - seg1.Y;
        double q = (((point.X - seg1.X) * dx) + ((point.Y - seg1.Y) * dy)) / ((dx * dx) + (dy * dy));

        // Clamp to segment bounds so we always return the closest point on the finite segment.
        q = Math.Clamp(q, 0D, 1D);
        return new Vertex(seg1.X + (q * dx), seg1.Y + (q * dy));
    }

    /// <summary>
    /// Returns true when two segments intersect.
    /// </summary>
    public static bool SegmentsIntersect(in Vertex a1, in Vertex a2, in Vertex b1, in Vertex b2, bool inclusive = false)
    {
        // Uses cross-product tests to solve a1 + d1 * t == b1 + d2 * u.
        // cp is the denominator (cross of directions); cp == 0 means parallel/collinear.
        Vertex d1 = a2 - a1;
        Vertex d2 = b2 - b1;
        double cp = Vertex.Cross(d2, d1);
        if (cp == 0)
        {
            return false;
        }

        if (inclusive)
        {
            // Inclusive mode allows intersections at endpoints.
            double t = Vertex.Cross(a1 - b1, d2);
            if (t == 0)
            {
                return true;
            }

            if (t > 0)
            {
                if (cp < 0 || t > cp)
                {
                    // t outside [0, cp] once sign is normalized.
                    return false;
                }
            }
            else if (cp > 0 || t < cp)
            {
                return false;
            }

            t = Vertex.Cross(a1 - b1, d1);
            if (t == 0)
            {
                return true;
            }

            if (t > 0)
            {
                // t within bounds for the second segment.
                return cp > 0 && t <= cp;
            }

            return cp < 0 && t >= cp;
        }

        // Exclusive mode requires the intersection to be strictly inside both segments.
        double t2 = Vertex.Cross(a1 - b1, d2);
        if (t2 == 0)
        {
            return false;
        }

        if (t2 > 0)
        {
            if (cp < 0 || t2 >= cp)
            {
                // Reject if t2 is outside the open interval.
                return false;
            }
        }
        else if (cp > 0 || t2 <= cp)
        {
            return false;
        }

        t2 = Vertex.Cross(a1 - b1, d1);
        if (t2 == 0)
        {
            return false;
        }

        if (t2 > 0)
        {
            // Both parameters are inside open intervals.
            return cp > 0 && t2 < cp;
        }

        return cp < 0 && t2 > cp;
    }

    /// <summary>
    /// Computes the bounding box of a contour.
    /// </summary>
    public static Box2 GetBounds(List<Vertex> path)
    {
        if (path.Count == 0)
        {
            return default;
        }

        double minX = double.MaxValue;
        double minY = double.MaxValue;
        double maxX = double.MinValue;
        double maxY = double.MinValue;

        for (int i = 0; i < path.Count; i++)
        {
            Vertex pt = path[i];
            if (pt.X < minX)
            {
                minX = pt.X;
            }

            if (pt.X > maxX)
            {
                maxX = pt.X;
            }

            if (pt.Y < minY)
            {
                minY = pt.Y;
            }

            if (pt.Y > maxY)
            {
                maxY = pt.Y;
            }
        }

        if (minX == double.MaxValue)
        {
            return default;
        }

        return new Box2(new Vertex(minX, minY), new Vertex(maxX, maxY));
    }

    /// <summary>
    /// Returns the midpoint of a contour's bounding box.
    /// </summary>
    private static Vertex GetBoundsMidPoint(List<Vertex> path) => GetBounds(path).MidPoint();

    /// <summary>
    /// Determines whether a point is inside a contour.
    /// </summary>
    public static PointInPolygonResult PointInPolygon(in Vertex point, List<Vertex> polygon)
        => PointInPolygon(point, CollectionsMarshal.AsSpan(polygon));

    /// <summary>
    /// Determines whether a point is inside a contour.
    /// </summary>
    /// <param name="point">The point to test.</param>
    /// <param name="polygon">The contour vertices.</param>
    /// <returns>The point's position relative to the contour.</returns>
    public static PointInPolygonResult PointInPolygon(in Vertex point, ReadOnlySpan<Vertex> polygon)
    {
        int len = polygon.Length;
        int start = 0;
        if (len < 3)
        {
            return PointInPolygonResult.Outside;
        }

        while (start < len && polygon[start].Y == point.Y)
        {
            start++;
        }

        if (start == len)
        {
            return PointInPolygonResult.Outside;
        }

        bool isAbove = polygon[start].Y < point.Y;
        bool startingAbove = isAbove;
        int val = 0;
        int i = start + 1;
        int end = len;
        while (true)
        {
            if (i == end)
            {
                if (end == 0 || start == 0)
                {
                    break;
                }

                end = start;
                i = 0;
            }

            if (isAbove)
            {
                while (i < end && polygon[i].Y < point.Y)
                {
                    i++;
                }
            }
            else
            {
                while (i < end && polygon[i].Y > point.Y)
                {
                    i++;
                }
            }

            if (i == end)
            {
                continue;
            }

            Vertex curr = polygon[i];
            Vertex prev = i > 0 ? polygon[i - 1] : polygon[len - 1];

            if (curr.Y == point.Y)
            {
                if (curr.X == point.X ||
                    (curr.Y == prev.Y && ((point.X < prev.X) != (point.X < curr.X))))
                {
                    return PointInPolygonResult.On;
                }

                i++;
                if (i == start)
                {
                    break;
                }

                continue;
            }

            if (point.X < curr.X && point.X < prev.X)
            {
                // no-op
            }
            else if (point.X > prev.X && point.X > curr.X)
            {
                val = 1 - val;
            }
            else
            {
                int cps2 = CrossSign(prev, curr, point);
                if (cps2 == 0)
                {
                    return PointInPolygonResult.On;
                }

                if ((cps2 < 0) == isAbove)
                {
                    val = 1 - val;
                }
            }

            isAbove = !isAbove;
            i++;
        }

        if (isAbove == startingAbove)
        {
            return val == 0 ? PointInPolygonResult.Outside : PointInPolygonResult.Inside;
        }

        if (i == len)
        {
            i = 0;
        }

        int cps = i == 0
            ? CrossSign(polygon[len - 1], polygon[0], point)
            : CrossSign(polygon[i - 1], polygon[i], point);

        if (cps == 0)
        {
            return PointInPolygonResult.On;
        }

        if ((cps < 0) == isAbove)
        {
            val = 1 - val;
        }

        return val == 0 ? PointInPolygonResult.Outside : PointInPolygonResult.Inside;
    }

    /// <summary>
    /// Returns true if the outer contour contains the inner contour.
    /// </summary>
    private static bool PathContainsPath(List<Vertex> inner, List<Vertex> outer)
    {
        PointInPolygonResult pip = PointInPolygonResult.On;
        for (int i = 0; i < inner.Count; i++)
        {
            switch (PointInPolygon(inner[i], outer))
            {
                case PointInPolygonResult.Outside:
                    if (pip == PointInPolygonResult.Outside)
                    {
                        return false;
                    }

                    pip = PointInPolygonResult.Outside;
                    break;
                case PointInPolygonResult.Inside:
                    if (pip == PointInPolygonResult.Inside)
                    {
                        return true;
                    }

                    pip = PointInPolygonResult.Inside;
                    break;
                default:
                    break;
            }
        }

        Vertex midpoint = GetBoundsMidPoint(inner);
        return PointInPolygon(midpoint, outer) != PointInPolygonResult.Outside;
    }

    /// <summary>
    /// Returns true if the outer contour contains the inner contour.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool Path2ContainsPath1(List<Vertex> inner, List<Vertex> outer) => PathContainsPath(inner, outer);

    /// <summary>
    /// Finds the intersection of two line segments, constraining results to their intersection bounding box.
    /// </summary>
    /// <param name="a1">The first point of the first segment.</param>
    /// <param name="a2">The second point of the first segment.</param>
    /// <param name="b1">The first point of the second segment.</param>
    /// <param name="b2">The second point of the second segment.</param>
    /// <param name="pi0">The first intersection point.</param>
    /// <param name="pi1">The second intersection point (if overlap occurs).</param>
    /// <returns>
    /// An <see cref="int"/> indicating the number of intersection points:
    /// - Returns 0 if there is no intersection.
    /// - Returns 1 if the segments intersect at a single point.
    /// - Returns 2 if the segments overlap.
    /// </returns>
    public static int FindIntersection(in Vertex a1, in Vertex a2, in Vertex b1, in Vertex b2, out Vertex pi0, out Vertex pi1)
        => FindIntersection(new Segment(a1, a2), new Segment(b1, b2), out pi0, out pi1);
}

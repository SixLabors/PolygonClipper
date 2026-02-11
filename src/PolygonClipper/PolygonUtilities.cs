// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System.Numerics;
using System.Runtime.CompilerServices;

namespace SixLabors.PolygonClipper;

internal static class PolygonUtilities
{
    private const double DoubleExactIntLimit = 9007199254740992D; // 2^53
    [ThreadStatic]
    private static double invScale;

    [ThreadStatic]
    private static double scale;

    internal static void SetFloatingScale(double inverseScale)
    {
        invScale = inverseScale;
        scale = inverseScale == 0D ? 0D : 1D / inverseScale;
    }

    internal static void ClearFloatingScale()
    {
        invScale = 0D;
        scale = 0D;
    }

    internal static bool UseFloatingScale
        => invScale != 0D;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static double ToDouble(long value)
        => value * invScale;
    /// <summary>
    /// Returns the dot product of the vectors AB and BC.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Int128 Dot(in Vertex64 a, in Vertex64 b, in Vertex64 c)
    {
        Vertex64 ab = b - a;
        Vertex64 bc = c - b;
        return Vertex64.Dot(ab, bc);
    }

    /// <summary>
    /// Returns the cross product of the vectors AB and BC.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Int128 Cross(in Vertex64 a, in Vertex64 b, in Vertex64 c)
        => Vertex64.Cross(b - a, c - b);

    /// <summary>
    /// Returns the sign of the cross product of the vectors AB and BC.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int CrossSign(in Vertex64 a, in Vertex64 b, in Vertex64 c)
    {
        if (invScale != 0D)
        {
            double ax = a.X * invScale;
            double ay = a.Y * invScale;
            double bx = b.X * invScale;
            double by = b.Y * invScale;
            double cx = c.X * invScale;
            double cy = c.Y * invScale;
            double crossValue = ((bx - ax) * (cy - by)) - ((by - ay) * (cx - bx));
            if (crossValue == 0D)
            {
                return 0;
            }

            return crossValue > 0D ? 1 : -1;
        }

        Int128 crossValueInt = Cross(a, b, c);
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
    public static bool IsCollinear(in Vertex64 a, in Vertex64 shared, in Vertex64 b)
        => CrossSign(a, shared, b) == 0;

    /// <summary>
    /// Returns the signed area of a triangle.
    /// </summary>
    /// <param name="p0">The first point.</param>
    /// <param name="p1">The second point.</param>
    /// <param name="p2">The third point.</param>
    /// <returns>The <see cref="double"/> area.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static double SignedArea(in Vertex64 p0, in Vertex64 p1, in Vertex64 p2)
    {
        if (invScale != 0D)
        {
            double p0x = p0.X * invScale;
            double p0y = p0.Y * invScale;
            double p1x = p1.X * invScale;
            double p1y = p1.Y * invScale;
            double p2x = p2.X * invScale;
            double p2y = p2.Y * invScale;
            return ((p0x - p2x) * (p1y - p2y)) - ((p1x - p2x) * (p0y - p2y));
        }

        return (double)Vertex64.Cross(p0 - p2, p1 - p2);
    }

    /// <summary>
    /// Computes the signed area of a contour.
    /// </summary>
    public static double Area(List<Vertex64> path)
    {
        int count = path.Count;
        if (count < 3)
        {
            return 0D;
        }

        Int128 area = 0;
        Vertex64 prev = path[count - 1];
        for (int i = 0; i < count; i++)
        {
            Vertex64 current = path[i];
            area += (Int128)(prev.Y + current.Y) * (prev.X - current.X);
            prev = current;
        }

        return (double)area * 0.5D;
    }

    /// <summary>
    /// Computes the squared perpendicular distance from a point to a line segment.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static double PerpendicularDistanceSquared(in Vertex64 point, in Vertex64 line1, in Vertex64 line2)
    {
        if (invScale != 0D)
        {
            double px = point.X * invScale;
            double py = point.Y * invScale;
            double l1x = line1.X * invScale;
            double l1y = line1.Y * invScale;
            double l2x = line2.X * invScale;
            double l2y = line2.Y * invScale;

            double dx = l2x - l1x;
            double dy = l2y - l1y;
            double lenSq = (dx * dx) + (dy * dy);
            if (lenSq == 0D)
            {
                return 0D;
            }

            double cx = px - l1x;
            double cy = py - l1y;
            double crossValue = (cx * dy) - (cy * dx);
            return (crossValue * crossValue) / lenSq;
        }

        Vertex64 toPoint = point - line1;
        Vertex64 direction = line2 - line1;
        double lengthSquared = (double)Vertex64.Dot(direction, direction);
        if (lengthSquared == 0D)
        {
            return 0D;
        }

        double cross = (double)Vertex64.Cross(toPoint, direction);
        return (cross * cross) / lengthSquared;
    }

    /// <summary>
    /// Finds the intersection of two line segments, including endpoints.
    /// </summary>
    public static bool TryGetLineIntersection(
        in Vertex64 a1,
        in Vertex64 a2,
        in Vertex64 b1,
        in Vertex64 b2,
        out Vertex64 intersection)
    {
        if (invScale != 0D)
        {
            double ax1 = a1.X * invScale;
            double ay1 = a1.Y * invScale;
            double ax2 = a2.X * invScale;
            double ay2 = a2.Y * invScale;
            double bx1 = b1.X * invScale;
            double by1 = b1.Y * invScale;
            double bx2 = b2.X * invScale;
            double by2 = b2.Y * invScale;

            double d1x = ax2 - ax1;
            double d1y = ay2 - ay1;
            double d2x = bx2 - bx1;
            double d2y = by2 - by1;
            double detValue = (d2x * d1y) - (d2y * d1x);
            if (detValue == 0D)
            {
                intersection = default;
                return false;
            }

            double tValue = ((ax1 - bx1) * d2y - (ay1 - by1) * d2x) / detValue;
            double ix;
            double iy;
            if (tValue <= 0D)
            {
                ix = ax1;
                iy = ay1;
            }
            else if (tValue >= 1D)
            {
                ix = ax2;
                iy = ay2;
            }
            else
            {
                ix = ax1 + (tValue * d1x);
                iy = ay1 + (tValue * d1y);
            }

            intersection = new Vertex64(
                RoundToLong(ix * scale),
                RoundToLong(iy * scale));
            return true;
        }

        Vertex64 d1 = a2 - a1;
        Vertex64 d2 = b2 - b1;
        Int128 det = Vertex64.Cross(d2, d1);
        if (det == 0)
        {
            intersection = default;
            return false;
        }

        Int128 tNumerator = Vertex64.Cross(a1 - b1, d2);
        if (det < 0)
        {
            det = -det;
            tNumerator = -tNumerator;
        }

        if (tNumerator <= 0)
        {
            intersection = a1;
            return true;
        }

        if (tNumerator >= det)
        {
            intersection = a2;
            return true;
        }

        double t = DivideWithScale(tNumerator, det);
        intersection = new Vertex64(
            RoundToLong(a1.X + (t * d1.X)),
            RoundToLong(a1.Y + (t * d1.Y)));

        return true;
    }

    /// <summary>
    /// Projects a point onto a segment and returns the closest point.
    /// </summary>
    public static Vertex64 ClosestPointOnSegment(in Vertex64 point, in Vertex64 seg1, in Vertex64 seg2)
    {
        if (invScale != 0D)
        {
            double px = point.X * invScale;
            double py = point.Y * invScale;
            double s1x = seg1.X * invScale;
            double s1y = seg1.Y * invScale;
            double s2x = seg2.X * invScale;
            double s2y = seg2.Y * invScale;

            if (s1x == s2x && s1y == s2y)
            {
                return seg1;
            }

            double dx = s2x - s1x;
            double dy = s2y - s1y;
            double qValue = ((px - s1x) * dx + (py - s1y) * dy) / ((dx * dx) + (dy * dy));
            qValue = Math.Clamp(qValue, 0D, 1D);
            double ix = s1x + (qValue * dx);
            double iy = s1y + (qValue * dy);
            return new Vertex64(
                RoundToLong(ix * scale),
                RoundToLong(iy * scale));
        }

        if (seg1 == seg2)
        {
            return seg1;
        }

        Vertex64 direction = seg2 - seg1;
        double lengthSquared = (double)Vertex64.Dot(direction, direction);
        double q = (double)Vertex64.Dot(point - seg1, direction) / lengthSquared;
        Math.Clamp(q, 0, 1);
        return new Vertex64(
            RoundToLong(seg1.X + (q * direction.X)),
            RoundToLong(seg1.Y + (q * direction.Y)));
    }

    /// <summary>
    /// Returns true when two segments intersect.
    /// </summary>
    public static bool SegmentsIntersect(in Vertex64 a1, in Vertex64 a2, in Vertex64 b1, in Vertex64 b2, bool inclusive = false)
    {
        // Uses cross-product tests to solve a1 + d1 * t == b1 + d2 * u.
        // cp is the denominator (cross of directions); cp == 0 means parallel/collinear.
        Vertex64 d1 = a2 - a1;
        Vertex64 d2 = b2 - b1;
        Int128 cp = Vertex64.Cross(d2, d1);
        if (cp == 0)
        {
            return false;
        }

        if (inclusive)
        {
            // Inclusive mode allows intersections at endpoints.
            Int128 t = Vertex64.Cross(a1 - b1, d2);
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

            t = Vertex64.Cross(a1 - b1, d1);
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
        Int128 t2 = Vertex64.Cross(a1 - b1, d2);
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

        t2 = Vertex64.Cross(a1 - b1, d1);
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
    public static Box64 GetBounds(List<Vertex64> path)
    {
        if (path.Count == 0)
        {
            return default;
        }

        long minX = long.MaxValue;
        long minY = long.MaxValue;
        long maxX = long.MinValue;
        long maxY = long.MinValue;

        for (int i = 0; i < path.Count; i++)
        {
            Vertex64 pt = path[i];
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

        if (minX == long.MaxValue)
        {
            return default;
        }

        return new Box64(new Vertex64(minX, minY), new Vertex64(maxX, maxY));
    }

    /// <summary>
    /// Returns the midpoint of a contour's bounding box.
    /// </summary>
    private static Vertex64 GetBoundsMidPoint(List<Vertex64> path) => GetBounds(path).MidPoint();

    /// <summary>
    /// Determines whether a point is inside a contour.
    /// </summary>
    public static PointInPolygonResult PointInPolygon(in Vertex64 point, List<Vertex64> polygon)
    {
        int len = polygon.Count;
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

            Vertex64 curr = polygon[i];
            Vertex64 prev = i > 0 ? polygon[i - 1] : polygon[len - 1];

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
    private static bool PathContainsPath(List<Vertex64> inner, List<Vertex64> outer)
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

        Vertex64 midpoint = GetBoundsMidPoint(inner);
        return PointInPolygon(midpoint, outer) != PointInPolygonResult.Outside;
    }

    /// <summary>
    /// Returns true if the outer contour contains the inner contour.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool Path2ContainsPath1(List<Vertex64> inner, List<Vertex64> outer) => PathContainsPath(inner, outer);

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
    public static int FindIntersection(in Segment seg0, in Segment seg1, out Vertex64 pi0, out Vertex64 pi1)
    {
        pi0 = default;
        pi1 = default;

        if (invScale != 0D)
        {
            return FindIntersectionFloating(seg0, seg1, out pi0, out pi1);
        }

        if (!TryGetIntersectionBoundingBox(seg0.Source, seg0.Target, seg1.Source, seg1.Target, out Box64 bbox))
        {
            return 0;
        }

        int interResult = FindIntersectionImpl(seg0, seg1, out pi0, out pi1);

        if (interResult == 1)
        {
            pi0 = ConstrainToBoundingBox(pi0, bbox);
        }
        else if (interResult == 2)
        {
            pi0 = ConstrainToBoundingBox(pi0, bbox);
            pi1 = ConstrainToBoundingBox(pi1, bbox);
        }

        return interResult;
    }

    internal static int FindIntersectionDouble(in Segment seg0, in Segment seg1, out Vertex pi0, out Vertex pi1)
    {
        pi0 = default;
        pi1 = default;

        Vertex a1 = Dequantize(seg0.Source);
        Vertex a2 = Dequantize(seg0.Target);
        Vertex b1 = Dequantize(seg1.Source);
        Vertex b2 = Dequantize(seg1.Target);

        if (!TryGetIntersectionBoundingBoxDouble(a1, a2, b1, b2, out Box2 bbox))
        {
            return 0;
        }

        int interResult = FindIntersectionImplDouble(a1, a2, b1, b2, out pi0, out pi1);
        if (interResult == 1)
        {
            pi0 = ConstrainToBoundingBoxDouble(pi0, bbox);
        }
        else if (interResult == 2)
        {
            pi0 = ConstrainToBoundingBoxDouble(pi0, bbox);
            pi1 = ConstrainToBoundingBoxDouble(pi1, bbox);
        }

        return interResult;
    }

    internal static int FindIntersectionDouble(
        in Vertex a1,
        in Vertex a2,
        in Vertex b1,
        in Vertex b2,
        out Vertex pi0,
        out Vertex pi1)
    {
        pi0 = default;
        pi1 = default;

        if (!TryGetIntersectionBoundingBoxDouble(a1, a2, b1, b2, out Box2 bbox))
        {
            return 0;
        }

        int interResult = FindIntersectionImplDouble(a1, a2, b1, b2, out pi0, out pi1);
        if (interResult == 1)
        {
            pi0 = ConstrainToBoundingBoxDouble(pi0, bbox);
        }
        else if (interResult == 2)
        {
            pi0 = ConstrainToBoundingBoxDouble(pi0, bbox);
            pi1 = ConstrainToBoundingBoxDouble(pi1, bbox);
        }

        return interResult;
    }

    private static int FindIntersectionFloating(in Segment seg0, in Segment seg1, out Vertex64 pi0, out Vertex64 pi1)
    {
        pi0 = default;
        pi1 = default;

        Vertex a1 = Dequantize(seg0.Source);
        Vertex a2 = Dequantize(seg0.Target);
        Vertex b1 = Dequantize(seg1.Source);
        Vertex b2 = Dequantize(seg1.Target);

        if (!TryGetIntersectionBoundingBoxDouble(a1, a2, b1, b2, out Box2 bbox))
        {
            return 0;
        }

        int interResult = FindIntersectionImplDouble(a1, a2, b1, b2, out Vertex dpi0, out Vertex dpi1);

        if (interResult == 1)
        {
            dpi0 = ConstrainToBoundingBoxDouble(dpi0, bbox);
        }
        else if (interResult == 2)
        {
            dpi0 = ConstrainToBoundingBoxDouble(dpi0, bbox);
            dpi1 = ConstrainToBoundingBoxDouble(dpi1, bbox);
        }

        pi0 = Quantize(dpi0);
        pi1 = Quantize(dpi1);
        return interResult;
    }

    private static int FindIntersectionImplDouble(
        in Vertex a1,
        in Vertex a2,
        in Vertex b1,
        in Vertex b2,
        out Vertex pi0,
        out Vertex pi1)
    {
        pi0 = default;
        pi1 = default;

        Vertex va = a2 - a1;
        Vertex vb = b2 - b1;
        Vertex e = b1 - a1;

        double kross = Vertex.Cross(va, vb);
        double sqrKross = kross * kross;
        double sqrLenA = Vertex.Dot(va, va);

        if (sqrKross > 0D)
        {
            double s = Vertex.Cross(e, vb) / kross;
            if (s is < 0D or > 1D)
            {
                return 0;
            }

            double t = Vertex.Cross(e, va) / kross;
            if (t is < 0D or > 1D)
            {
                return 0;
            }

            if (s is 0D or 1D)
            {
                pi0 = a1 + (s * va);
                return 1;
            }

            if (t is 0D or 1D)
            {
                pi0 = b1 + (t * vb);
                return 1;
            }

            pi0 = a1 + (s * va);
            return 1;
        }

        kross = Vertex.Cross(e, va);
        sqrKross = kross * kross;
        if (sqrKross > 0D)
        {
            return 0;
        }

        double sa = Vertex.Dot(va, e) / sqrLenA;
        double sb = sa + (Vertex.Dot(va, vb) / sqrLenA);
        double smin = Math.Min(sa, sb);
        double smax = Math.Max(sa, sb);

        if (smin <= 1D && smax >= 0D)
        {
            if (smin == 1D)
            {
                pi0 = a1 + (smin * va);
                return 1;
            }

            if (smax == 0D)
            {
                pi0 = a1 + (smax * va);
                return 1;
            }

            pi0 = a1 + (Math.Max(smin, 0D) * va);
            pi1 = a1 + (Math.Min(smax, 1D) * va);
            return 2;
        }

        return 0;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vertex Dequantize(in Vertex64 value)
        => new(value.X * invScale, value.Y * invScale);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static Vertex64 Quantize(in Vertex value)
        => new(
            RoundToLong(value.X * scale),
            RoundToLong(value.Y * scale));

    private static bool TryGetIntersectionBoundingBoxDouble(
        in Vertex a1,
        in Vertex a2,
        in Vertex b1,
        in Vertex b2,
        out Box2 result)
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

        result = default;
        return false;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vertex ConstrainToBoundingBoxDouble(in Vertex p, in Box2 bbox)
        => Vertex.Min(Vertex.Max(p, bbox.Min), bbox.Max);

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
    private static int FindIntersectionImpl(in Segment seg0, in Segment seg1, out Vertex64 pi0, out Vertex64 pi1)
    {
        pi0 = default;
        pi1 = default;

        Vertex64 a1 = seg0.Source;
        Vertex64 a2 = seg1.Source;

        Vertex64 va = seg0.Target - a1;
        Vertex64 vb = seg1.Target - a2;
        Vertex64 e = a2 - a1;

        Int128 kross = Vertex64.Cross(va, vb);

        if (kross != 0)
        {
            // Lines of the segments are not parallel
            Int128 numS = Vertex64.Cross(e, vb);
            Int128 numT = Vertex64.Cross(e, va);

            if (kross > 0)
            {
                if (numS < 0 || numS > kross || numT < 0 || numT > kross)
                {
                    return 0;
                }
            }
            else
            {
                if (numS > 0 || numS < kross || numT > 0 || numT < kross)
                {
                    return 0;
                }
            }

            if (numS == 0)
            {
                pi0 = a1;
                return 1;
            }

            if (numS == kross)
            {
                pi0 = seg0.Target;
                return 1;
            }

            if (numT == 0)
            {
                pi0 = a2;
                return 1;
            }

            if (numT == kross)
            {
                pi0 = seg1.Target;
                return 1;
            }

            double s = DivideWithScale(numS, kross);
            pi0 = MidPoint(a1, s, va);

            return 1;
        }

        // Lines are parallel; check if they are collinear
        kross = Vertex64.Cross(e, va);
        if (kross != 0)
        {
            // Lines of the segments are different
            return 0;
        }

        // Segments are collinear, check for overlap using exact ratios.
        Int128 denom = Vertex64.Dot(va, va);
        if (denom == 0)
        {
            return 0;
        }

        Int128 saNum = Vertex64.Dot(va, e);
        Int128 sbNum = saNum + Vertex64.Dot(va, vb);
        Int128 sminNum = saNum < sbNum ? saNum : sbNum;
        Int128 smaxNum = saNum > sbNum ? saNum : sbNum;

        if (sminNum <= denom && smaxNum >= 0)
        {
            if (sminNum == denom)
            {
                pi0 = seg0.Target;
                return 1;
            }

            if (smaxNum == 0)
            {
                pi0 = a1;
                return 1;
            }

            Int128 startNum = sminNum < 0 ? 0 : sminNum;
            Int128 endNum = smaxNum > denom ? denom : smaxNum;
            double s0 = DivideWithScale(startNum, denom);
            double s1 = DivideWithScale(endNum, denom);
            pi0 = MidPoint(a1, s0, va);
            pi1 = MidPoint(a1, s1, va);
            return pi0 == pi1 ? 1 : 2;
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
        in Vertex64 a1,
        in Vertex64 a2,
        in Vertex64 b1,
        in Vertex64 b2,
        out Box64 result)
    {
        Vertex64 minA = Vertex64.Min(a1, a2);
        Vertex64 maxA = Vertex64.Max(a1, a2);
        Vertex64 minB = Vertex64.Min(b1, b2);
        Vertex64 maxB = Vertex64.Max(b1, b2);

        Vertex64 interMin = Vertex64.Max(minA, minB);
        Vertex64 interMax = Vertex64.Min(maxA, maxB);

        if (interMin.X <= interMax.X && interMin.Y <= interMax.Y)
        {
            result = new Box64(interMin, interMax);
            return true;
        }

        result = default;
        return false;
    }

    /// <summary>
    /// Constrains a point to the given bounding box.
    /// </summary>
    /// <param name="p">The point to constrain.</param>
    /// <param name="bbox">The bounding box.</param>
    /// <returns>The constrained point.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vertex64 ConstrainToBoundingBox(in Vertex64 p, in Box64 bbox)
        => Vertex64.Min(Vertex64.Max(p, bbox.Min), bbox.Max);

    /// <summary>
    /// Computes the point at a given fractional distance along a directed line segment.
    /// </summary>
    /// <param name="p">The starting vertex of the segment.</param>
    /// <param name="s">The scalar factor representing the fractional distance along the segment.</param>
    /// <param name="d">The direction vector of the segment.</param>
    /// <returns>The interpolated vertex at the given fractional distance.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vertex64 MidPoint(in Vertex64 p, double s, in Vertex64 d)
        => new(
            RoundToLong(p.X + (s * d.X)),
            RoundToLong(p.Y + (s * d.Y)));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static long RoundToLong(double value)
    {
        if (value <= -DoubleExactIntLimit || value >= DoubleExactIntLimit)
        {
            return (long)Math.Round((decimal)value, MidpointRounding.AwayFromZero);
        }

        return (long)Math.Round(value, MidpointRounding.AwayFromZero);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static double DivideWithScale(Int128 numerator, Int128 denominator)
    {
        if (denominator == 0)
        {
            return double.NaN;
        }

        UInt128 absNumerator = (UInt128)(numerator < 0 ? -numerator : numerator);
        UInt128 absDenominator = (UInt128)(denominator < 0 ? -denominator : denominator);
        int maxBits = Math.Max(GetBitLength(absNumerator), GetBitLength(absDenominator));
        int shift = maxBits > 53 ? maxBits - 53 : 0;

        if (shift > 0)
        {
            numerator >>= shift;
            denominator >>= shift;
        }

        return (double)numerator / (double)denominator;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static int GetBitLength(UInt128 value)
    {
        if (value == 0)
        {
            return 0;
        }

        ulong high = (ulong)(value >> 64);
        if (high != 0)
        {
            return 64 + BitOperations.Log2(high) + 1;
        }

        ulong low = (ulong)value;
        return BitOperations.Log2(low) + 1;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static int GetBitLength(long value)
    {
        ulong absValue = (ulong)(value < 0 ? -value : value);
        if (absValue == 0)
        {
            return 0;
        }

        return BitOperations.Log2(absValue) + 1;
    }


}

// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System.Runtime.CompilerServices;

namespace SixLabors.PolygonClipper;

internal static class ClipGeometry
{
    internal const double FloatingPointTolerance = 1e-12;
    internal const double CoordinateEpsilon = 1e-6;
    internal const double PointEqualityTolerance = 0.5 * CoordinateEpsilon;
    internal const double ClosePointTolerance = 2 * CoordinateEpsilon;
    internal const double SmallAreaTolerance = CoordinateEpsilon * CoordinateEpsilon;
    internal const double SmallAreaTolerance2 = 2 * SmallAreaTolerance;
    internal const double JoinDistanceSquared = 0.25 * SmallAreaTolerance;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool IsAlmostZero(double value) => Math.Abs(value) <= FloatingPointTolerance;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool PointEquals(in Vertex a, in Vertex b)
        => Math.Abs(a.X - b.X) <= PointEqualityTolerance &&
           Math.Abs(a.Y - b.Y) <= PointEqualityTolerance;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static double Dot(Vertex a, Vertex b, Vertex c)
        => ((b.X - a.X) * (c.X - b.X)) + ((b.Y - a.Y) * (c.Y - b.Y));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static double DotProduct(Vertex a, Vertex b, Vertex c) => Dot(a, b, c);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static double Cross(Vertex a, Vertex b, Vertex c)
        => ((b.X - a.X) * (c.Y - b.Y)) - ((b.Y - a.Y) * (c.X - b.X));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int CrossSign(Vertex a, Vertex b, Vertex c)
    {
        double cross = Cross(a, b, c);
        if (IsAlmostZero(cross))
        {
            return 0;
        }

        return cross > 0 ? 1 : -1;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int CrossProductSign(Vertex a, Vertex b, Vertex c) => CrossSign(a, b, c);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool IsCollinear(Vertex a, Vertex shared, Vertex b)
        => IsAlmostZero(Cross(a, shared, b));

    public static bool TryGetLineIntersection(Vertex a1, Vertex a2, Vertex b1, Vertex b2, out Vertex intersection)
    {
        double dy1 = a2.Y - a1.Y;
        double dx1 = a2.X - a1.X;
        double dy2 = b2.Y - b1.Y;
        double dx2 = b2.X - b1.X;
        double det = (dy1 * dx2) - (dy2 * dx1);
        if (IsAlmostZero(det))
        {
            intersection = default;
            return false;
        }

        double t = (((a1.X - b1.X) * dy2) - ((a1.Y - b1.Y) * dx2)) / det;
        if (t <= 0D || IsAlmostZero(t))
        {
            intersection = a1;
        }
        else if (t >= 1D || IsAlmostZero(t - 1D))
        {
            intersection = a2;
        }
        else
        {
            intersection = new Vertex(a1.X + (t * dx1), a1.Y + (t * dy1));
        }

        return true;
    }

    public static Vertex ClosestPointOnSegment(Vertex point, Vertex seg1, Vertex seg2)
    {
        if (IsAlmostZero(seg1.X - seg2.X) && IsAlmostZero(seg1.Y - seg2.Y))
        {
            return seg1;
        }

        double dx = seg2.X - seg1.X;
        double dy = seg2.Y - seg1.Y;
        double q = (((point.X - seg1.X) * dx) + ((point.Y - seg1.Y) * dy)) / ((dx * dx) + (dy * dy));
        if (q < 0D)
        {
            q = 0D;
        }
        else if (q > 1D)
        {
            q = 1D;
        }

        return new Vertex(seg1.X + (q * dx), seg1.Y + (q * dy));
    }

    public static bool SegmentsIntersect(Vertex a1, Vertex a2, Vertex b1, Vertex b2, bool inclusive = false)
    {
        double dy1 = a2.Y - a1.Y;
        double dx1 = a2.X - a1.X;
        double dy2 = b2.Y - b1.Y;
        double dx2 = b2.X - b1.X;
        double cp = (dy1 * dx2) - (dy2 * dx1);
        if (IsAlmostZero(cp))
        {
            return false;
        }

        if (inclusive)
        {
            double t = ((a1.X - b1.X) * dy2) - ((a1.Y - b1.Y) * dx2);
            if (IsAlmostZero(t))
            {
                return true;
            }

            if (t > 0D)
            {
                if (cp < 0D || t > cp)
                {
                    return false;
                }
            }
            else if (cp > 0D || t < cp)
            {
                return false;
            }

            t = ((a1.X - b1.X) * dy1) - ((a1.Y - b1.Y) * dx1);
            if (IsAlmostZero(t))
            {
                return true;
            }

            if (t > 0D)
            {
                return cp > 0D && t <= cp;
            }

            return cp < 0D && t >= cp;
        }

        double t2 = ((a1.X - b1.X) * dy2) - ((a1.Y - b1.Y) * dx2);
        if (IsAlmostZero(t2))
        {
            return false;
        }

        if (t2 > 0D)
        {
            if (cp < 0D || t2 >= cp)
            {
                return false;
            }
        }
        else if (cp > 0D || t2 <= cp)
        {
            return false;
        }

        t2 = ((a1.X - b1.X) * dy1) - ((a1.Y - b1.Y) * dx1);
        if (IsAlmostZero(t2))
        {
            return false;
        }

        if (t2 > 0D)
        {
            return cp > 0D && t2 < cp;
        }

        return cp < 0D && t2 > cp;
    }

    public static ClipBounds GetBounds(Contour path)
    {
        if (path.Count == 0)
        {
            return default(ClipBounds);
        }

        ClipBounds result = ClipMath.InvalidBounds;
        for (int i = 0; i < path.Count; i++)
        {
            Vertex pt = path[i];
            if (pt.X < result.Left)
            {
                result.Left = pt.X;
            }

            if (pt.X > result.Right)
            {
                result.Right = pt.X;
            }

            if (pt.Y < result.Top)
            {
                result.Top = pt.Y;
            }

            if (pt.Y > result.Bottom)
            {
                result.Bottom = pt.Y;
            }
        }

        return Math.Abs(result.Left - double.MaxValue) < FloatingPointTolerance ? default(ClipBounds) : result;
    }

    public static Vertex GetBoundsMidPoint(Contour path) => GetBounds(path).MidPoint();

    public static ClipperPointInPolygonResult PointInPolygon(Vertex point, Contour polygon)
    {
        int len = polygon.Count;
        int start = 0;
        if (len < 3)
        {
            return ClipperPointInPolygonResult.IsOutside;
        }

        while (start < len && IsAlmostZero(polygon[start].Y - point.Y))
        {
            start++;
        }

        if (start == len)
        {
            return ClipperPointInPolygonResult.IsOutside;
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

            if (IsAlmostZero(curr.Y - point.Y))
            {
                if (IsAlmostZero(curr.X - point.X) ||
                    (IsAlmostZero(curr.Y - prev.Y) && ((point.X < prev.X) != (point.X < curr.X))))
                {
                    return ClipperPointInPolygonResult.IsOn;
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
                    return ClipperPointInPolygonResult.IsOn;
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
            return val == 0 ? ClipperPointInPolygonResult.IsOutside : ClipperPointInPolygonResult.IsInside;
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
            return ClipperPointInPolygonResult.IsOn;
        }

        if ((cps < 0) == isAbove)
        {
            val = 1 - val;
        }

        return val == 0 ? ClipperPointInPolygonResult.IsOutside : ClipperPointInPolygonResult.IsInside;
    }

    public static bool PathContainsPath(Contour inner, Contour outer)
    {
        ClipperPointInPolygonResult pip = ClipperPointInPolygonResult.IsOn;
        for (int i = 0; i < inner.Count; i++)
        {
            switch (PointInPolygon(inner[i], outer))
            {
                case ClipperPointInPolygonResult.IsOutside:
                    if (pip == ClipperPointInPolygonResult.IsOutside)
                    {
                        return false;
                    }

                    pip = ClipperPointInPolygonResult.IsOutside;
                    break;
                case ClipperPointInPolygonResult.IsInside:
                    if (pip == ClipperPointInPolygonResult.IsInside)
                    {
                        return true;
                    }

                    pip = ClipperPointInPolygonResult.IsInside;
                    break;
                default:
                    break;
            }
        }

        Vertex midpoint = GetBoundsMidPoint(inner);
        return PointInPolygon(midpoint, outer) != ClipperPointInPolygonResult.IsOutside;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool Path2ContainsPath1(Contour inner, Contour outer) => PathContainsPath(inner, outer);
}

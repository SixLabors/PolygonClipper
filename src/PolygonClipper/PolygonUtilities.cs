// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System.Diagnostics.CodeAnalysis;
using System.Runtime.CompilerServices;

namespace SixLabors.PolygonClipper;

internal static class PolygonUtilities
{
    // Sorts vertices by X then Y so we can build deterministic vertex orderings
    // in trivial/short-circuit polygon results.
    private static readonly Comparison<Vertex> LexicographicVertexComparison = (left, right)
        => left.X != right.X
            ? left.X.CompareTo(right.X)
            : left.Y.CompareTo(right.Y);

    internal const double FloatingPointTolerance = 1e-12;
    internal const double CoordinateEpsilon = 1e-6;
    internal const double PointEqualityTolerance = 0.5 * CoordinateEpsilon;
    internal const double ClosePointTolerance = 2 * CoordinateEpsilon;
    internal const double SmallAreaTolerance = CoordinateEpsilon * CoordinateEpsilon;
    internal const double SmallAreaTolerance2 = 2 * SmallAreaTolerance;
    internal const double JoinDistanceSquared = 0.25 * SmallAreaTolerance;

    /// <summary>
    /// Determines whether a value is almost zero.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool IsAlmostZero(double value)
        => Math.Abs(value) <= FloatingPointTolerance;

    /// <summary>
    /// Compares two vertices for near-equality.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool PointEquals(in Vertex a, in Vertex b)
    {
        Vertex delta = Vertex.Abs(a - b);
        return delta.X <= PointEqualityTolerance && delta.Y <= PointEqualityTolerance;
    }

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
        double cross = Cross(a, b, c);
        if (IsAlmostZero(cross))
        {
            return 0;
        }

        return cross > 0 ? 1 : -1;
    }

    /// <summary>
    /// Returns true when three vertices are collinear.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool IsCollinear(in Vertex a, in Vertex shared, in Vertex b)
        => IsAlmostZero(Cross(a, shared, b));

    /// <summary>
    /// Returns the signed area of a triangle.
    /// </summary>
    /// <param name="p0">The first point.</param>
    /// <param name="p1">The second point.</param>
    /// <param name="p2">The third point.</param>
    /// <returns>The <see cref="double"/> area.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static double SignedArea(in Vertex p0, in Vertex p1, in Vertex p2)
        => ((p0.X - p2.X) * (p1.Y - p2.Y)) - ((p1.X - p2.X) * (p0.Y - p2.Y));

    /// <summary>
    /// Computes the signed area of a contour.
    /// </summary>
    public static double Area(Contour path)
    {
        int count = path.Count;
        if (count < 3)
        {
            return 0D;
        }

        double area = 0D;
        Vertex prev = path[count - 1];
        for (int i = 0; i < count; i++)
        {
            Vertex current = path[i];
            area += (prev.Y + current.Y) * (prev.X - current.X);
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
        Vertex d1 = a2 - a1;
        Vertex d2 = b2 - b1;
        double det = Vertex.Cross(d2, d1);
        if (det == 0)
        {
            intersection = default;
            return false;
        }

        double t = Vertex.Cross(a1 - b1, d2) / det;
        intersection = t switch
        {
            <= 0D => a1,
            >= 1D => a2,
            _ => a1 + (t * d1)
        };

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

        Vertex direction = seg2 - seg1;
        double q = Vertex.Dot(point - seg1, direction) / Vertex.Dot(direction, direction);
        Math.Clamp(q, 0, 1);
        return seg1 + (q * direction);
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

            if (t > 0D)
            {
                if (cp < 0D || t > cp)
                {
                    // t outside [0, cp] once sign is normalized.
                    return false;
                }
            }
            else if (cp > 0D || t < cp)
            {
                return false;
            }

            t = Vertex.Cross(a1 - b1, d1);
            if (t == 0)
            {
                return true;
            }

            if (t > 0D)
            {
                // t within bounds for the second segment.
                return cp > 0D && t <= cp;
            }

            return cp < 0D && t >= cp;
        }

        // Exclusive mode requires the intersection to be strictly inside both segments.
        double t2 = Vertex.Cross(a1 - b1, d2);
        if (t2 == 0)
        {
            return false;
        }

        if (t2 > 0D)
        {
            if (cp < 0D || t2 >= cp)
            {
                // Reject if t2 is outside the open interval.
                return false;
            }
        }
        else if (cp > 0D || t2 <= cp)
        {
            return false;
        }

        t2 = Vertex.Cross(a1 - b1, d1);
        if (t2 == 0)
        {
            return false;
        }

        if (t2 > 0D)
        {
            // Both parameters are inside open intervals.
            return cp > 0D && t2 < cp;
        }

        return cp < 0D && t2 > cp;
    }

    /// <summary>
    /// Computes the bounding box of a contour.
    /// </summary>
    public static Box2 GetBounds(Contour path)
    {
        if (path.Count == 0)
        {
            return default;
        }

        double minX = double.MaxValue;
        double minY = double.MaxValue;
        double maxX = -double.MaxValue;
        double maxY = -double.MaxValue;

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

        if (Math.Abs(minX - double.MaxValue) < FloatingPointTolerance)
        {
            return default;
        }

        return new Box2(new Vertex(minX, minY), new Vertex(maxX, maxY));
    }

    /// <summary>
    /// Returns the midpoint of a contour's bounding box.
    /// </summary>
    private static Vertex GetBoundsMidPoint(Contour path) => GetBounds(path).MidPoint();

    /// <summary>
    /// Determines whether a point is inside a contour.
    /// </summary>
    public static PointInPolygonResult PointInPolygon(in Vertex point, Contour polygon)
    {
        int len = polygon.Count;
        int start = 0;
        if (len < 3)
        {
            return PointInPolygonResult.Outside;
        }

        while (start < len && IsAlmostZero(polygon[start].Y - point.Y))
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

            if (IsAlmostZero(curr.Y - point.Y))
            {
                if (IsAlmostZero(curr.X - point.X) ||
                    (IsAlmostZero(curr.Y - prev.Y) && ((point.X < prev.X) != (point.X < curr.X))))
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
    private static bool PathContainsPath(Contour inner, Contour outer)
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
    public static bool Path2ContainsPath1(Contour inner, Contour outer) => PathContainsPath(inner, outer);

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
        double sqrKross = kross * kross;
        double sqrLenA = Vertex.Dot(va, va);

        if (sqrKross > 0)
        {
            // Lines of the segments are not parallel
            double s = Vertex.Cross(e, vb) / kross;
            if (s is < 0 or > 1)
            {
                return 0;
            }

            double t = Vertex.Cross(e, va) / kross;
            if (t is < 0 or > 1)
            {
                return 0;
            }

            // If s or t is exactly 0 or 1, the intersection is on an endpoint
            if (s is 0 or 1)
            {
                // On an endpoint of line segment a
                pi0 = MidPoint(a1, s, va);
                return 1;
            }

            if (t is 0 or 1)
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
    /// Computes the point at a given fractional distance along a directed line segment.
    /// </summary>
    /// <param name="p">The starting vertex of the segment.</param>
    /// <param name="s">The scalar factor representing the fractional distance along the segment.</param>
    /// <param name="d">The direction vector of the segment.</param>
    /// <returns>The interpolated vertex at the given fractional distance.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static Vertex MidPoint(in Vertex p, double s, in Vertex d) => p + (s * d);

    /// <summary>
    /// Builds a normalized polygon with deterministic contour ordering/vertex ordering.
    /// Used for trivial/short-circuit operations to match the sweep output ordering.
    /// </summary>
    /// <param name="polygon">The polygon to normalize.</param>
    /// <returns>The normalized polygon.</returns>
    public static Polygon BuildNormalizedPolygon(Polygon polygon)
        => BuildNormalizedPolygon([polygon]);

    /// <summary>
    /// Builds a normalized polygon from the union of subject and clipping polygons.
    /// Used for trivial/short-circuit operations to match the sweep output ordering.
    /// </summary>
    /// <param name="subject">The subject polygon.</param>
    /// <param name="clipping">The clipping polygon.</param>
    /// <returns>The normalized polygon.</returns>
    public static Polygon BuildNormalizedPolygon(Polygon subject, Polygon clipping)
        => BuildNormalizedPolygon([subject, clipping]);

    /// <summary>
    /// Builds a normalized polygon from one or more polygons by:
    /// - normalizing vertex orientation/rotation per contour,
    /// - computing nesting (parent/child) via containment tests,
    /// - ordering exterior contours first, then their holes.
    /// </summary>
    /// <param name="polygons">The polygons to normalize.</param>
    /// <returns>The normalized polygon.</returns>
    private static Polygon BuildNormalizedPolygon(ReadOnlySpan<Polygon> polygons)
    {
        // Flatten all contours into a working list so we can normalize and re-parent them
        // independent of the input polygon ordering.
        List<ContourInfo> contourInfos = [];
        for (int i = 0; i < polygons.Length; i++)
        {
            Polygon polygon = polygons[i];
            for (int j = 0; j < polygon.Count; j++)
            {
                Contour contour = polygon[j];
                if (contour.Count == 0)
                {
                    continue;
                }

                // Normalize each contour so comparisons across input polygons are deterministic.
                contourInfos.Add(CreateContourInfo(contour));
            }
        }

        if (contourInfos.Count == 0)
        {
            return [];
        }

        int count = contourInfos.Count;
        int[] parentIndices = new int[count];
        int[] depths = new int[count];
        Array.Fill(parentIndices, -1);

        // Establish containment relationships by finding the smallest-area contour
        // that contains a test point from each contour.
        for (int i = 0; i < count; i++)
        {
            Vertex testPoint = contourInfos[i].Vertices[0];
            double smallestContainerArea = double.PositiveInfinity;
            int parentIndex = -1;

            for (int j = 0; j < count; j++)
            {
                if (i == j)
                {
                    continue;
                }

                if (PointInContour(testPoint, contourInfos[j].Vertices))
                {
                    double area = contourInfos[j].Area;
                    if (area < smallestContainerArea)
                    {
                        smallestContainerArea = area;
                        parentIndex = j;
                    }
                }
            }

            parentIndices[i] = parentIndex;
        }

        // Compute the nesting depth for each contour (0 = exterior, 1 = hole, etc.).
        // Exteriors are the even-depth contours.
        List<int> externals = new(count);
        for (int i = 0; i < count; i++)
        {
            int depth = GetDepth(i, parentIndices);
            depths[i] = depth;

            if ((depth & 1) == 0)
            {
                externals.Add(i);
            }
        }

        // Sort exteriors by their minimum vertex so their order is deterministic.
        externals.Sort((left, right) =>
            LexicographicVertexComparison(contourInfos[left].MinVertex, contourInfos[right].MinVertex));

        Polygon result = new(count);
        bool[] added = new bool[count];
        int[] newIndices = new int[count];
        Array.Fill(newIndices, -1);

        foreach (int externalIndex in externals)
        {
            // Add the exterior contour first.
            int externalContourIndex = AddContour(
                result,
                contourInfos,
                externalIndex,
                depths,
                parentIndices,
                added,
                newIndices);

            // Collect direct holes of this exterior and order them deterministically.
            List<int> holes = [];
            for (int i = 0; i < count; i++)
            {
                if (!added[i] && parentIndices[i] == externalIndex && (depths[i] & 1) == 1)
                {
                    holes.Add(i);
                }
            }

            holes.Sort((left, right) =>
                LexicographicVertexComparison(contourInfos[left].MinVertex, contourInfos[right].MinVertex));

            // Add holes immediately after their exterior so the contour ordering matches
            // the sweep output (exterior first, then its holes).
            foreach (int holeIndex in holes)
            {
                int holeContourIndex = AddContour(
                    result,
                    contourInfos,
                    holeIndex,
                    depths,
                    parentIndices,
                    added,
                    newIndices);

                result[externalContourIndex].AddHoleIndex(holeContourIndex);
                result[holeContourIndex].ParentIndex = externalContourIndex;
            }
        }

        // Any remaining contours (should be none, but safe for malformed input).
        for (int i = 0; i < count; i++)
        {
            if (!added[i])
            {
                _ = AddContour(result, contourInfos, i, depths, parentIndices, added, newIndices);
            }
        }

        return result;
    }

    /// <summary>
    /// Returns true if any segment from <paramref name="subject"/> intersects any segment from
    /// <paramref name="clipping"/>.
    /// </summary>
    /// <param name="subject">The subject polygon.</param>
    /// <param name="clipping">The clipping polygon.</param>
    /// <returns>True if any pair of segments intersects.</returns>
    public static bool PolygonsHaveIntersection(Polygon subject, Polygon clipping)
    {
        for (int i = 0; i < subject.Count; i++)
        {
            Contour subjectContour = subject[i];
            for (int j = 0; j < subjectContour.Count; j++)
            {
                Segment subjectSegment = subjectContour.GetSegment(j);
                if (subjectSegment.IsDegenerate())
                {
                    continue;
                }

                for (int k = 0; k < clipping.Count; k++)
                {
                    Contour clippingContour = clipping[k];
                    for (int l = 0; l < clippingContour.Count; l++)
                    {
                        Segment clippingSegment = clippingContour.GetSegment(l);
                        if (clippingSegment.IsDegenerate())
                        {
                            continue;
                        }

                        if (FindIntersection(subjectSegment, clippingSegment, out _, out _) != 0)
                        {
                            return true;
                        }
                    }
                }
            }
        }

        return false;
    }

    /// <summary>
    /// Returns any vertex from the polygon (first vertex of the first non-empty contour).
    /// </summary>
    /// <param name="polygon">The polygon to search.</param>
    /// <param name="vertex">The found vertex, if any.</param>
    /// <returns>True if a vertex was found.</returns>
    public static bool TryGetAnyVertex(Polygon polygon, out Vertex vertex)
    {
        for (int i = 0; i < polygon.Count; i++)
        {
            Contour contour = polygon[i];
            if (contour.Count > 0)
            {
                vertex = contour[0];
                return true;
            }
        }

        vertex = default;
        return false;
    }

    /// <summary>
    /// Determines whether a point is inside a polygon, respecting hole information when present.
    /// </summary>
    /// <param name="point">The test point.</param>
    /// <param name="polygon">The polygon to test.</param>
    /// <returns>True if the point is inside the polygon.</returns>
    public static bool ContainsPoint(in Vertex point, Polygon polygon)
    {
        if (polygon.Count == 0)
        {
            return false;
        }

        // Detect whether hole metadata is available to apply external/hole rules.
        bool hasHoleInfo = false;
        for (int i = 0; i < polygon.Count; i++)
        {
            Contour contour = polygon[i];
            if (contour.HoleCount > 0 || contour.ParentIndex != null)
            {
                hasHoleInfo = true;
                break;
            }
        }

        if (hasHoleInfo)
        {
            for (int i = 0; i < polygon.Count; i++)
            {
                Contour contour = polygon[i];
                if (!contour.IsExternal)
                {
                    continue;
                }

                // If the point is inside an exterior ring, ensure it is not inside any hole
                // that belongs to that exterior.
                if (PointInContour(point, contour))
                {
                    for (int j = 0; j < contour.HoleCount; j++)
                    {
                        if (PointInContour(point, polygon[contour.GetHoleIndex(j)]))
                        {
                            return false;
                        }
                    }

                    return true;
                }
            }

            return false;
        }

        // Without hole metadata, use XOR across all contours to infer containment.
        bool inside = false;
        for (int i = 0; i < polygon.Count; i++)
        {
            if (PointInContour(point, polygon[i]))
            {
                inside = !inside;
            }
        }

        return inside;
    }

    /// <summary>
    /// Ray-casting point-in-polygon test against a list of vertices.
    /// </summary>
    private static bool PointInContour(in Vertex point, List<Vertex> vertices)
    {
        int count = vertices.Count;
        if (count < 3)
        {
            return false;
        }

        bool inside = false;
        Vertex previous = vertices[count - 1];

        for (int i = 0; i < count; i++)
        {
            Vertex current = vertices[i];
            if (current == previous)
            {
                previous = current;
                continue;
            }

            if (IsPointOnSegment(point, previous, current))
            {
                return true;
            }

            bool intersects = (current.Y > point.Y) != (previous.Y > point.Y);
            if (intersects)
            {
                double xIntersection = ((previous.X - current.X) * (point.Y - current.Y) / (previous.Y - current.Y)) +
                                       current.X;
                if (point.X < xIntersection)
                {
                    inside = !inside;
                }
            }

            previous = current;
        }

        return inside;
    }

    /// <summary>
    /// Ray-casting point-in-polygon test against a contour.
    /// </summary>
    private static bool PointInContour(in Vertex point, Contour contour)
    {
        int count = contour.Count;
        if (count < 3)
        {
            return false;
        }

        bool inside = false;
        Vertex previous = contour[count - 1];

        for (int i = 0; i < count; i++)
        {
            Vertex current = contour[i];
            if (current == previous)
            {
                previous = current;
                continue;
            }

            if (IsPointOnSegment(point, previous, current))
            {
                return true;
            }

            bool intersects = (current.Y > point.Y) != (previous.Y > point.Y);
            if (intersects)
            {
                double xIntersection = ((previous.X - current.X) * (point.Y - current.Y) / (previous.Y - current.Y)) +
                                       current.X;
                if (point.X < xIntersection)
                {
                    inside = !inside;
                }
            }

            previous = current;
        }

        return inside;
    }

    /// <summary>
    /// Checks whether a point lies exactly on a line segment.
    /// </summary>
    private static bool IsPointOnSegment(in Vertex point, in Vertex a, in Vertex b)
    {
        if (SignedArea(a, b, point) != 0D)
        {
            return false;
        }

        double minX = Math.Min(a.X, b.X);
        double maxX = Math.Max(a.X, b.X);
        double minY = Math.Min(a.Y, b.Y);
        double maxY = Math.Max(a.Y, b.Y);

        return point.X >= minX && point.X <= maxX && point.Y >= minY && point.Y <= maxY;
    }

    /// <summary>
    /// Computes nesting depth by walking parent links.
    /// </summary>
    private static int GetDepth(int index, int[] parentIndices)
    {
        int depth = 0;
        int current = parentIndices[index];
        while (current >= 0)
        {
            depth++;
            current = parentIndices[current];
        }

        return depth;
    }

    /// <summary>
    /// Builds a normalized contour snapshot used for ordering and containment checks.
    /// </summary>
    private static ContourInfo CreateContourInfo(Contour contour)
    {
        // GeoJSON inputs are typically closed; drop the duplicate closing vertex for normalization,
        // then re-add it afterward to preserve contour closure.
        bool isClosed = contour.Count > 1 && contour[0] == contour[^1];
        int targetCount = isClosed ? contour.Count - 1 : contour.Count;
        List<Vertex> vertices = new(targetCount);
        for (int i = 0; i < targetCount; i++)
        {
            vertices.Add(contour[i]);
        }

        // Normalize orientation and rotate so the minimum vertex is first.
        NormalizeVertices(vertices);

        // Track the minimum vertex for stable ordering.
        Vertex minVertex = vertices[0];
        for (int i = 1; i < vertices.Count; i++)
        {
            if (LexicographicVertexComparison(vertices[i], minVertex) < 0)
            {
                minVertex = vertices[i];
            }
        }

        // Area is used to select the smallest containing contour.
        double area = Math.Abs(GetSignedArea(vertices));

        // Restore closure if needed.
        if (isClosed)
        {
            vertices.Add(vertices[0]);
        }

        return new ContourInfo(vertices, minVertex, area);
    }

    /// <summary>
    /// Normalizes a contour's orientation to counterclockwise and rotates so the minimum vertex
    /// is the first entry. This gives deterministic vertex ordering.
    /// </summary>
    private static void NormalizeVertices(List<Vertex> vertices)
    {
        if (vertices.Count < 3)
        {
            return;
        }

        // Ensure counterclockwise orientation.
        if (GetSignedArea(vertices) < 0)
        {
            vertices.Reverse();
        }

        // Rotate so the minimum vertex is first.
        int minIndex = 0;
        Vertex minVertex = vertices[0];
        for (int i = 1; i < vertices.Count; i++)
        {
            if (LexicographicVertexComparison(vertices[i], minVertex) < 0)
            {
                minVertex = vertices[i];
                minIndex = i;
            }
        }

        if (minIndex > 0)
        {
            List<Vertex> rotated = new(vertices.Count);
            for (int i = 0; i < vertices.Count; i++)
            {
                rotated.Add(vertices[(i + minIndex) % vertices.Count]);
            }

            vertices.Clear();
            vertices.AddRange(rotated);
        }
    }

    /// <summary>
    /// Computes the signed area of a polygonal ring.
    /// </summary>
    private static double GetSignedArea(List<Vertex> vertices)
    {
        double area = 0D;
        for (int i = 0; i < vertices.Count; i++)
        {
            Vertex current = vertices[i];
            Vertex next = vertices[(i + 1) % vertices.Count];
            area += Vertex.Cross(current, next);
        }

        return area;
    }

    /// <summary>
    /// Adds a contour to the output polygon and updates new index mappings.
    /// </summary>
    private static int AddContour(
        Polygon polygon,
        List<ContourInfo> contourInfos,
        int contourIndex,
        int[] depths,
        int[] parentIndices,
        bool[] added,
        int[] newIndices)
    {
        // Copy normalized vertices into a new contour instance.
        Contour contour = new();
        foreach (Vertex vertex in contourInfos[contourIndex].Vertices)
        {
            contour.Add(vertex);
        }

        polygon.Add(contour);
        added[contourIndex] = true;
        int newIndex = polygon.Count - 1;
        newIndices[contourIndex] = newIndex;

        // Store depth and parent references using the new index mapping.
        contour.Depth = depths[contourIndex];
        if (depths[contourIndex] % 2 == 1)
        {
            int parentIndex = parentIndices[contourIndex];
            if (parentIndex >= 0 && newIndices[parentIndex] >= 0)
            {
                contour.ParentIndex = newIndices[parentIndex];
            }
        }

        return newIndex;
    }

    private sealed record ContourInfo(List<Vertex> Vertices, Vertex MinVertex, double Area);
}

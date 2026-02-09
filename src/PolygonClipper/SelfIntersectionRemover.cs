// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System.Runtime.CompilerServices;

namespace SixLabors.PolygonClipper;

/// <summary>
/// Provides functionality to remove self-intersections from polygons using the positive
/// fill rule (non-zero winding) and a sweep line algorithm.
/// </summary>
/// <remarks>
/// <para>
/// This class implements a sweep line algorithm that applies the positive fill rule
/// to resolve self-intersections within a polygon. A region is considered "inside"
/// if its winding number is greater than zero.
/// </para>
/// <para>
/// The algorithm works in three phases:
/// </para>
/// <list type="number">
/// <item><description>
/// <b>Intersection Detection:</b> Uses a sweep line to find all points where segments
/// intersect each other (both self-intersections and cross-contour intersections).
/// </description></item>
/// <item><description>
/// <b>Segment Splitting:</b> Divides segments at intersection points, creating new
/// vertices where crossings occur.
/// </description></item>
/// <item><description>
/// <b>Boundary Extraction:</b> Computes winding numbers and keeps only edges that
/// form the boundary between filled (winding &gt; 0) and unfilled (winding &lt;= 0) regions.
/// </description></item>
/// </list>
/// </remarks>
internal static class SelfIntersectionRemover
{
    // Per-thread clipper instance to reuse pools without cross-thread contention.
    [ThreadStatic]
    private static SelfIntersectionUnionClipper? cachedClipper;

    // Cached subject list to avoid per-call list allocations.
    [ThreadStatic]
    private static List<Contour>? cachedSubject;

    // Cached subject paths to reuse list instances across calls.
    [ThreadStatic]
    private static List<Contour>? cachedSubjectPaths;

    /// <summary>
    /// Processes a polygon to remove self-intersections using the positive fill rule.
    /// </summary>
    /// <param name="polygon">The polygon to process.</param>
    /// <returns>
    /// A new <see cref="Polygon"/> with self-intersections resolved. Regions with
    /// positive winding number are considered filled.
    /// </returns>
    public static Polygon Process(Polygon polygon)
    {
        if (polygon.Count == 0)
        {
            return [];
        }

        // Normalize orientation so the union uses positive fill semantics consistently.
        EnsureClosedContours(polygon);
        Polygon prepared = PreparePositiveFillInput(polygon);
        return UnionWithClipper(prepared);
    }

    /// <summary>
    /// Ensures contour orientation matches the positive fill rule (externals counterclockwise, holes clockwise).
    /// </summary>
    /// <param name="polygon">The polygon to prepare.</param>
    /// <returns>The original polygon when no changes are needed, otherwise a reoriented copy.</returns>
    private static Polygon PreparePositiveFillInput(Polygon polygon)
    {
        int count = polygon.Count;
        if (count <= 1)
        {
            return polygon;
        }

        int[] parentIndices = new int[count];
        Array.Fill(parentIndices, -1);

        bool hasHierarchy = false;
        for (int i = 0; i < count; i++)
        {
            Contour contour = polygon[i];
            if (contour.ParentIndex == null && contour.HoleCount <= 0)
            {
                continue;
            }

            hasHierarchy = true;
            break;
        }

        if (hasHierarchy)
        {
            for (int i = 0; i < count; i++)
            {
                parentIndices[i] = polygon[i].ParentIndex ?? -1;
            }
        }
        else
        {
            Box2[] bounds = new Box2[count];
            double[] absAreas = new double[count];
            for (int i = 0; i < count; i++)
            {
                Contour contour = polygon[i];
                bounds[i] = contour.GetBoundingBox();
                absAreas[i] = Math.Abs(GetSignedArea(contour));

                if (HasSelfIntersection(contour))
                {
                    // Avoid reorienting inputs that are already self-intersecting.
                    return polygon;
                }
            }

            for (int i = 0; i < count; i++)
            {
                for (int j = i + 1; j < count; j++)
                {
                    if (ContoursIntersect(polygon[i], polygon[j], bounds[i], bounds[j]))
                    {
                        // Contours overlap; keep original orientation to avoid changing fill behavior.
                        return polygon;
                    }
                }
            }

            for (int i = 0; i < count; i++)
            {
                Contour contour = polygon[i];
                if (contour.Count == 0)
                {
                    continue;
                }

                // Use a stable vertex sample for containment checks.
                Vertex testPoint = GetContourTestPoint(contour);
                double smallestArea = double.PositiveInfinity;
                int parentIndex = -1;

                for (int j = 0; j < count; j++)
                {
                    if (i == j || !bounds[j].Contains(testPoint))
                    {
                        continue;
                    }

                    if (PolygonUtilities.PointInPolygon(testPoint, polygon[j]) != ClipperPointInPolygonResult.IsInside)
                    {
                        continue;
                    }

                    if (absAreas[j] < smallestArea)
                    {
                        smallestArea = absAreas[j];
                        parentIndex = j;
                    }
                }

                parentIndices[i] = parentIndex;
            }
        }

        int[] depths = new int[count];
        bool needsCopy = false;
        for (int i = 0; i < count; i++)
        {
            int depth = GetDepth(i, parentIndices);
            depths[i] = depth;

            Contour contour = polygon[i];
            if (contour.Count == 0)
            {
                continue;
            }

            bool shouldBeCounterClockwise = (depth & 1) == 0;
            if (contour.IsCounterClockwise() != shouldBeCounterClockwise)
            {
                needsCopy = true;
            }
        }

        if (!needsCopy)
        {
            return polygon;
        }

        Polygon oriented = new(count);
        for (int i = 0; i < count; i++)
        {
            Contour source = polygon[i];
            Contour copy = new(source.Count);
            for (int j = 0; j < source.Count; j++)
            {
                copy.Add(source[j]);
            }

            if ((depths[i] & 1) == 0)
            {
                copy.SetCounterClockwise();
            }
            else
            {
                copy.SetClockwise();
            }

            oriented.Add(copy);
        }

        return oriented;
    }

    /// <summary>
    /// Ensures all contours are explicitly closed by repeating the first vertex at the end.
    /// </summary>
    /// <param name="polygon">The polygon to normalize.</param>
    private static void EnsureClosedContours(Polygon polygon)
    {
        for (int i = 0; i < polygon.Count; i++)
        {
            Contour contour = polygon[i];
            if (contour.Count > 0 && contour[0] != contour[^1])
            {
                contour.Add(contour[0]);
            }
        }
    }

    /// <summary>
    /// Executes a union using the internal clipper with a fill rule selected from the input orientation.
    /// </summary>
    /// <param name="polygon">The polygon to union.</param>
    /// <returns>A polygon containing the unioned contours.</returns>
    private static Polygon UnionWithClipper(Polygon polygon)
    {
        List<Contour> subject = BuildSubjectPaths(polygon);

        GetLowestPathInfo(subject, out int lowestPathIdx, out bool isNegArea);
        bool pathsReversed = lowestPathIdx >= 0 && isNegArea;
        ClipperFillRule fillRule = pathsReversed ? ClipperFillRule.Negative : ClipperFillRule.Positive;

        SelfIntersectionUnionClipper clipper = cachedClipper ??= new SelfIntersectionUnionClipper();
        clipper.Clear();
        clipper.PreserveCollinear = false;
        clipper.ReverseSolution = pathsReversed;
        clipper.AddSubject(subject);

        Polygon result = new(polygon.Count);
        clipper.Execute(fillRule, result);
        return result;
    }

    /// <summary>
    /// Builds reusable subject paths from a polygon, minimizing allocations across calls.
    /// </summary>
    /// <param name="polygon">The polygon to convert.</param>
    /// <returns>A cached list of <see cref="Contour"/> instances ready for clipping.</returns>
    private static List<Contour> BuildSubjectPaths(Polygon polygon)
    {
        List<Contour> subject = cachedSubject ??= new List<Contour>(polygon.Count);
        subject.Clear();
        subject.EnsureCapacity(polygon.Count);

        List<Contour> pathPool = cachedSubjectPaths ??= new List<Contour>(polygon.Count);
        for (int i = 0; i < polygon.Count; i++)
        {
            Contour contour = polygon[i];
            Contour path = GetPooledPath(pathPool, i, contour.Count);
            for (int j = 0; j < contour.Count; j++)
            {
                Vertex vertex = contour[j];
                path.Add(new Vertex(vertex.X, -vertex.Y));
            }

            subject.Add(path);
        }

        return subject;
    }

    /// <summary>
    /// Retrieves a reusable <see cref="Contour"/> from the pool, sizing it as needed.
    /// </summary>
    /// <param name="pathPool">The pool of reusable paths.</param>
    /// <param name="index">The index of the path to reuse.</param>
    /// <param name="capacity">The required capacity for the path.</param>
    /// <returns>A cleared <see cref="Contour"/> ready for use.</returns>
    private static Contour GetPooledPath(List<Contour> pathPool, int index, int capacity)
    {
        if (index < pathPool.Count)
        {
            Contour path = pathPool[index];
            path.Clear();
            path.EnsureVertexCapacity(capacity);

            return path;
        }

        Contour newPath = new(capacity);
        pathPool.Add(newPath);
        return newPath;
    }

    /// <summary>
    /// Determines the lowest point across all paths and whether its contour area is negative.
    /// </summary>
    /// <param name="paths">The paths to examine.</param>
    /// <param name="lowestPathIdx">The index of the path containing the lowest point.</param>
    /// <param name="isNegArea">True when the lowest path has negative area.</param>
    private static void GetLowestPathInfo(List<Contour> paths, out int lowestPathIdx, out bool isNegArea)
    {
        lowestPathIdx = -1;
        isNegArea = false;

        if (paths.Count == 0)
        {
            return;
        }

        Vertex lowestPoint = default;
        bool hasPoint = false;

        for (int i = 0; i < paths.Count; i++)
        {
            Contour path = paths[i];
            if (path.Count == 0)
            {
                continue;
            }

            Vertex candidate = GetLowestPoint(path);
            if (!hasPoint || candidate.Y > lowestPoint.Y ||
                (candidate.Y == lowestPoint.Y && candidate.X < lowestPoint.X))
            {
                lowestPoint = candidate;
                lowestPathIdx = i;
                hasPoint = true;
            }
        }

        if (lowestPathIdx >= 0)
        {
            isNegArea = PolygonUtilities.Area(paths[lowestPathIdx]) < 0;
        }
    }

    /// <summary>
    /// Finds the lowest point (highest Y, then lowest X) in the specified path.
    /// </summary>
    /// <param name="path">The path to scan.</param>
    /// <returns>The lowest vertex.</returns>
    private static Vertex GetLowestPoint(Contour path)
    {
        Vertex lowest = path[0];
        for (int i = 1; i < path.Count; i++)
        {
            Vertex candidate = path[i];
            if (candidate.Y > lowest.Y || (candidate.Y == lowest.Y && candidate.X < lowest.X))
            {
                lowest = candidate;
            }
        }

        return lowest;
    }

    /// <summary>
    /// Computes the nesting depth for a contour relative to its parents.
    /// </summary>
    /// <param name="index">The index of the contour.</param>
    /// <param name="parentIndices">The parent lookup table.</param>
    /// <returns>The zero-based depth.</returns>
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
    /// Selects a representative vertex from a contour for containment tests.
    /// </summary>
    /// <param name="contour">The contour to examine.</param>
    /// <returns>A vertex guaranteed not to be the duplicated closing point.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vertex GetContourTestPoint(Contour contour)
    {
        if (contour.Count == 0)
        {
            return default;
        }

        Vertex first = contour[0];
        if (contour.Count > 1 && first == contour[^1])
        {
            return contour[1];
        }

        return first;
    }

    /// <summary>
    /// Calculates the signed area of a contour. Positive values indicate counterclockwise orientation.
    /// </summary>
    /// <param name="contour">The contour to measure.</param>
    /// <returns>The signed area in square units.</returns>
    private static double GetSignedArea(Contour contour)
    {
        double area = 0D;
        for (int i = 0; i < contour.Count; i++)
        {
            Vertex current = contour[i];
            Vertex next = contour[(i + 1) % contour.Count];
            area += Vertex.Cross(current, next);
        }

        return area * 0.5D;
    }

    private static bool HasSelfIntersection(Contour contour)
    {
        int segmentCount = contour.Count - 1;
        if (segmentCount < 3)
        {
            return false;
        }

        for (int i = 0; i < segmentCount; i++)
        {
            Segment segmentA = new(contour[i], contour[i + 1]);
            if (segmentA.IsDegenerate())
            {
                continue;
            }

            for (int j = i + 1; j < segmentCount; j++)
            {
                if (j == i || j == i + 1 || (i == 0 && j == segmentCount - 1))
                {
                    continue;
                }

                Segment segmentB = new(contour[j], contour[j + 1]);
                if (segmentB.IsDegenerate())
                {
                    continue;
                }

                if (PolygonUtilities.FindIntersection(segmentA, segmentB, out _, out _) != 0)
                {
                    return true;
                }
            }
        }

        return false;
    }

    private static bool ContoursIntersect(Contour left, Contour right, in Box2 leftBounds, in Box2 rightBounds)
    {
        if (!leftBounds.Intersects(rightBounds))
        {
            return false;
        }

        int leftCount = left.Count - 1;
        int rightCount = right.Count - 1;

        for (int i = 0; i < leftCount; i++)
        {
            Segment leftSegment = new(left[i], left[i + 1]);
            if (leftSegment.IsDegenerate())
            {
                continue;
            }

            for (int j = 0; j < rightCount; j++)
            {
                Segment rightSegment = new(right[j], right[j + 1]);
                if (rightSegment.IsDegenerate())
                {
                    continue;
                }

                if (PolygonUtilities.FindIntersection(leftSegment, rightSegment, out _, out _) != 0)
                {
                    return true;
                }
            }
        }

        return false;
    }
}

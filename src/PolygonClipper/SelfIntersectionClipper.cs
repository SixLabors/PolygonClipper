// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

namespace SixLabors.PolygonClipper;

internal static class SelfIntersectionClipper
{
    // Per-thread clipper instance to reuse pools without cross-thread contention.
    [ThreadStatic]
    private static UnionClipper? cachedClipper;

    // Cached subject list to avoid per-call list allocations.
    [ThreadStatic]
    private static List<Contour>? cachedSubject;

    // Cached subject paths to reuse list instances across calls.
    [ThreadStatic]
    private static List<Contour>? cachedSubjectPaths;

    public static List<Contour> Union(Polygon polygon)
    {
        List<Contour> subject = BuildSubjectPaths(polygon);

        GetLowestPathInfo(subject, out int lowestPathIdx, out bool isNegArea);
        bool pathsReversed = lowestPathIdx >= 0 && isNegArea;
        ClipperFillRule fillRule = pathsReversed ? ClipperFillRule.Negative : ClipperFillRule.Positive;

        UnionClipper clipper = cachedClipper ??= new UnionClipper();
        clipper.Clear();
        clipper.PreserveCollinear = false;
        clipper.ReverseSolution = pathsReversed;
        clipper.AddSubject(subject);

        List<Contour> contours = new(polygon.Count);
        clipper.Execute(ClipperOperation.Union, fillRule, contours);
        return contours;
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
}

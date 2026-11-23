// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using Clipper2Lib;

namespace SixLabors.PolygonClipper.Benchmarks;

/// <summary>
/// Converts closed benchmark geometry to Clipper2 paths.
/// </summary>
internal static class BenchmarkGeometry
{
    /// <summary>
    /// Copies contour vertices without changing their direction or closure.
    /// </summary>
    /// <param name="polygon">The closed source or result contours.</param>
    /// <returns>The matching Clipper2 paths.</returns>
    public static PathsD ToPaths(Polygon polygon)
    {
        PathsD paths = new(polygon.Count);
        foreach (Contour contour in polygon)
        {
            PathD path = new(contour.Count);
            foreach (Vertex vertex in contour)
            {
                path.Add(new PointD(vertex.X, vertex.Y));
            }

            paths.Add(path);
        }

        return paths;
    }
}

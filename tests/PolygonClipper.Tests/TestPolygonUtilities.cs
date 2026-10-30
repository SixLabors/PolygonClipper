// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using Clipper2Lib;
using GeoJson.Feature;
using GeoJson.Geometry;
using GeoPolygon = GeoJson.Geometry.Polygon;

namespace SixLabors.PolygonClipper.Tests;

internal static class TestPolygonUtilities
{
    /// <summary>
    /// Checks boundary connections without requiring the same contour grouping.
    /// </summary>
    /// <param name="actual">The actual boundary paths.</param>
    /// <param name="expected">The reference boundary paths.</param>
    /// <returns>Whether each junction and segment has one matching reference item.</returns>
    /// <remarks>
    /// Matches junctions by distance, then checks every undirected segment.
    /// Use a separate area or point check when coordinate rounding can change the filled area.
    /// </remarks>
    public static bool HaveEquivalentBoundaryGraphs(IEnumerable<PathD> actual, IEnumerable<PathD> expected)
    {
        (List<Vertex> actualNodes, Dictionary<(int Start, int End), int> actualEdges) = BuildBoundaryGraph(actual);
        (List<Vertex> expectedNodes, Dictionary<(int Start, int End), int> expectedEdges) = BuildBoundaryGraph(expected);
        if (actualNodes.Count != expectedNodes.Count || actualEdges.Count != expectedEdges.Count)
        {
            return false;
        }

        List<(Vertex Point, int Index)> ordered = new(expectedNodes.Count);
        for (int i = 0; i < expectedNodes.Count; i++)
        {
            ordered.Add((expectedNodes[i], i));
        }

        IComparer<(Vertex Point, int Index)> comparer = Comparer<(Vertex Point, int Index)>.Create(
            static (first, second) => first.Point.X.CompareTo(second.Point.X));
        ordered.Sort(comparer);

        int[] mapping = new int[actualNodes.Count];
        HashSet<int> assigned = [];
        for (int i = 0; i < actualNodes.Count; i++)
        {
            int index = FindNearestBoundaryNode(actualNodes[i], ordered, comparer);
            if (!assigned.Add(index))
            {
                // Two junctions cannot use one reference junction. That would
                // hide a lost connection or a change caused by coordinate rounding.
                return false;
            }

            mapping[i] = index;
        }

        foreach (((int start, int end), int count) in actualEdges)
        {
            int first = mapping[start];
            int second = mapping[end];
            (int, int) edge = first < second ? (first, second) : (second, first);
            if (!expectedEdges.TryGetValue(edge, out int expectedCount) || count != expectedCount)
            {
                return false;
            }
        }

        return true;
    }

    public static (Polygon Subject, Polygon Clipping) BuildPolygon(FeatureCollection data)
    {
        IGeometryObject subjectGeometry = data.Features[0].Geometry;
        IGeometryObject clippingGeometry = data.Features[1].Geometry;

        Polygon subject = ConvertToPolygon(subjectGeometry);
        Polygon clipping = ConvertToPolygon(clippingGeometry);

        return (subject, clipping);
    }

    /// <summary>
    /// Converts GeoJSON coordinates and preserves exterior and hole indices.
    /// </summary>
    /// <param name="geometry">The Polygon or MultiPolygon geometry.</param>
    /// <returns>The contours with their GeoJSON hierarchy.</returns>
    public static Polygon ConvertToPolygon(IGeometryObject geometry)
    {
        if (geometry is GeoPolygon geoJsonPolygon)
        {
            // Convert GeoJSON Polygon to our Polygon type
            Polygon polygon = [];
            foreach (LineString ring in geoJsonPolygon.Coordinates)
            {
                Contour contour = [];
                foreach (IPosition xy in ring.Coordinates)
                {
                    contour.Add(new Vertex(xy.Longitude, xy.Latitude));
                }

                if (polygon.Count > 0)
                {
                    // Later rings in this GeoJSON Polygon belong to its first ring.
                    contour.ParentIndex = 0;
                    contour.Depth = 1;
                    polygon[0].AddHoleIndex(polygon.Count);
                }

                polygon.Add(contour);

                if (!ring.IsClosed())
                {
                    contour.Add(contour[0]);
                }
            }

            return polygon;
        }
        else if (geometry is MultiPolygon geoJsonMultiPolygon)
        {
            // Convert GeoJSON MultiPolygon to our Polygon type
            Polygon polygon = [];
            foreach (GeoPolygon geoPolygon in geoJsonMultiPolygon.Coordinates)
            {
                int exteriorIndex = polygon.Count;
                foreach (LineString ring in geoPolygon.Coordinates)
                {
                    Contour contour = [];
                    foreach (IPosition xy in ring.Coordinates)
                    {
                        contour.Add(new Vertex(xy.Longitude, xy.Latitude));
                    }

                    if (polygon.Count > exteriorIndex)
                    {
                        // Each component has its own exterior; keep global contour indices.
                        contour.ParentIndex = exteriorIndex;
                        contour.Depth = 1;
                        polygon[exteriorIndex].AddHoleIndex(polygon.Count);
                    }

                    polygon.Add(contour);

                    if (!ring.IsClosed())
                    {
                        contour.Add(contour[0]);
                    }
                }
            }

            return polygon;
        }

        throw new InvalidOperationException("Unsupported geometry type.");
    }

    /// <summary>
    /// Collects boundary junctions and counts segments between them.
    /// </summary>
    /// <param name="paths">The closed boundary paths.</param>
    /// <returns>The junction positions and segment counts.</returns>
    private static (List<Vertex> Nodes, Dictionary<(int Start, int End), int> Edges) BuildBoundaryGraph(IEnumerable<PathD> paths)
    {
        List<Vertex> nodes = [];
        Dictionary<Vertex, int> indices = [];
        Dictionary<(int Start, int End), int> edges = [];
        foreach (PathD path in paths)
        {
            for (int i = 0; i < path.Count; i++)
            {
                PointD point = path[i];
                PointD next = path[(i + 1) % path.Count];
                Vertex first = new(point.x, point.y);
                Vertex second = new(next.x, next.y);
                if (first == second)
                {
                    // A repeated closing vertex adds no segment.
                    continue;
                }

                int start = GetBoundaryNodeIndex(first, indices, nodes);
                int end = GetBoundaryNodeIndex(second, indices, nodes);
                (int, int) edge = start < end ? (start, end) : (end, start);
                edges[edge] = edges.GetValueOrDefault(edge) + 1;
            }
        }

        return (nodes, edges);
    }

    /// <summary>
    /// Gets the shared index for a boundary junction.
    /// </summary>
    /// <param name="point">The junction position.</param>
    /// <param name="indices">The positions already indexed.</param>
    /// <param name="nodes">The indexed junctions.</param>
    /// <returns>The junction index.</returns>
    private static int GetBoundaryNodeIndex(Vertex point, Dictionary<Vertex, int> indices, List<Vertex> nodes)
    {
        if (!indices.TryGetValue(point, out int index))
        {
            index = nodes.Count;
            indices.Add(point, index);
            nodes.Add(point);
        }

        return index;
    }

    /// <summary>
    /// Finds the closest reference junction without a fixed distance threshold.
    /// </summary>
    /// <param name="point">The actual junction.</param>
    /// <param name="ordered">The reference junctions sorted by X.</param>
    /// <param name="comparer">The X-coordinate comparer used for sorting.</param>
    /// <returns>The closest junction's original index.</returns>
    private static int FindNearestBoundaryNode(
        Vertex point,
        List<(Vertex Point, int Index)> ordered,
        IComparer<(Vertex Point, int Index)> comparer)
    {
        int position = ordered.BinarySearch((point, 0), comparer);
        int right = position < 0 ? ~position : position;
        int left = right - 1;
        double bestDistance = double.PositiveInfinity;
        int bestIndex = -1;
        while (left >= 0 || right < ordered.Count)
        {
            double leftX = left >= 0 ? ordered[left].Point.X - point.X : double.PositiveInfinity;
            double rightX = right < ordered.Count ? ordered[right].Point.X - point.X : double.PositiveInfinity;
            double leftDistance = leftX * leftX;
            double rightDistance = rightX * rightX;
            if (Math.Min(leftDistance, rightDistance) > bestDistance)
            {
                // A greater X distance alone rules out all later candidates.
                break;
            }

            (Vertex candidate, int index) = leftDistance <= rightDistance ? ordered[left--] : ordered[right++];
            double distance = Vertex.DistanceSquared(point, candidate);
            if (distance < bestDistance)
            {
                bestDistance = distance;
                bestIndex = index;
            }
        }

        return bestIndex;
    }
}

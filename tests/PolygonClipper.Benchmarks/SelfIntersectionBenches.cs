// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using BenchmarkDotNet.Attributes;
using Clipper2Lib;

namespace SixLabors.PolygonClipper.Benchmarks;

/// <summary>
/// Benchmarks the performance of self-intersection removal for star polygons.
/// </summary>
[MemoryDiagnoser]
[OperationsPerSecond]
public class SelfIntersectionBenches
{
    private Polygon polygon;
    private PathsD clipperSubject;
    private Clipper2Lib.FillRule clipperFillRule;
    private bool clipperReverseSolution;
    private const int ClipperPrecision = 6;

    [Params(101, 301, 1001)]
    public int VertexCount { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        this.polygon = BuildStarPolygon(this.VertexCount, 100d);
        this.clipperSubject = BuildClipperSubject(this.polygon);
        GetLowestPathInfo(this.clipperSubject, out int lowestPathIdx, out bool isNegArea);
        this.clipperReverseSolution = lowestPathIdx >= 0 && isNegArea;
        this.clipperFillRule = this.clipperReverseSolution ? Clipper2Lib.FillRule.Negative : Clipper2Lib.FillRule.Positive;
    }

    [Benchmark]
    public Polygon RemoveSelfIntersections()
        => PolygonClipper.RemoveSelfIntersections(this.polygon);

    [Benchmark(Baseline = true)]
    public PolyTreeD Clipper2Union()
    {
        // Match Clipper2's single-polygon union with a positive fill rule.
        ClipperD clipper = new(ClipperPrecision)
        {
            PreserveCollinear = false,
            ReverseSolution = this.clipperReverseSolution
        };

        clipper.AddSubject(this.clipperSubject);
        PolyTreeD solution = [];
        clipper.Execute(ClipType.Union, this.clipperFillRule, solution);
        return solution;
    }

    private static Polygon BuildStarPolygon(int vertexCount, double radius)
    {
        if (vertexCount < 5 || (vertexCount & 1) == 0)
        {
            throw new ArgumentOutOfRangeException(nameof(vertexCount), "Vertex count must be an odd number >= 5.");
        }

        int step = (vertexCount - 1) / 2;
        Contour contour = new(vertexCount);

        for (int i = 0; i < vertexCount; i++)
        {
            int index = (i * step) % vertexCount;
            double angle = (index * Math.PI * 2d) / vertexCount;
            contour.Add(new Vertex(Math.Cos(angle) * radius, Math.Sin(angle) * radius));
        }

        contour.Add(contour[0]);

        Polygon polygon = [contour];
        return polygon;
    }

    private static PathsD BuildClipperSubject(Polygon polygon)
    {
        PathsD subject = new(polygon.Count);
        for (int i = 0; i < polygon.Count; i++)
        {
            Contour contour = polygon[i];
            PathD path = new(contour.Count);
            for (int j = 0; j < contour.Count; j++)
            {
                Vertex vertex = contour[j];
                path.Add(new PointD(vertex.X, vertex.Y));
            }

            subject.Add(path);
        }

        return subject;
    }

    private static void GetLowestPathInfo(PathsD paths, out int lowestPathIdx, out bool isNegArea)
    {
        lowestPathIdx = -1;
        isNegArea = false;

        if (paths.Count == 0)
        {
            return;
        }

        PointD lowestPoint = default;
        bool hasPoint = false;

        for (int i = 0; i < paths.Count; i++)
        {
            PathD path = paths[i];
            if (path.Count == 0)
            {
                continue;
            }

            PointD candidate = GetLowestPoint(path);
            if (!hasPoint || candidate.y > lowestPoint.y ||
                (candidate.y == lowestPoint.y && candidate.x < lowestPoint.x))
            {
                lowestPoint = candidate;
                lowestPathIdx = i;
                hasPoint = true;
            }
        }

        if (lowestPathIdx >= 0)
        {
            isNegArea = Clipper.Area(paths[lowestPathIdx]) < 0;
        }
    }

    private static PointD GetLowestPoint(PathD path)
    {
        PointD lowest = path[0];
        for (int i = 1; i < path.Count; i++)
        {
            PointD candidate = path[i];
            if (candidate.y > lowest.y || (candidate.y == lowest.y && candidate.x < lowest.x))
            {
                lowest = candidate;
            }
        }

        return lowest;
    }
}

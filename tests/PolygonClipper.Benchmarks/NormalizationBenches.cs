// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using BenchmarkDotNet.Attributes;
using Clipper2Lib;

namespace SixLabors.PolygonClipper.Benchmarks;

/*
 * Comparison results: 30 October 2026.
 * Windows 11. AMD Ryzen AI Max+ 395. .NET 10.0.14. BenchmarkDotNet 0.15.8.
 * V2 build: 4f814c3 with the orientation-bound formula. release/1.x: cb06839. Clipper2: 4be4655.
 * Mean: milliseconds per operation. Op/s: operations per second. B/op: allocated bytes per operation.
 * Input preparation occurs before measurement. ClipperD uses precision 6.
 * Each case uses 12 measured iterations, a 250 ms iteration target, and one launch. All samples remain included.
 * All comparisons below use Positive cleanup on one star. Warmup: 10 iterations.
 * release/1.x uses its Clipper2-derived cleanup. V2 uses its shared Martinez sweep.
 *
 * Vertices | Engine      | Mean (ms)  | Op/s     | B/op
 * ---------+-------------+------------+----------+---------
 * 101      | V2          | 1.751403   | 570.971  | 8648
 * 101      | release/1.x | 0.516666   | 1935.486 | 13760
 * 101      | Clipper2    | 0.449719   | 2223.613 | 210728
 * 301      | V2          | 17.035385  | 58.701   | 33272
 * 301      | release/1.x | 5.714182   | 175.003  | 40032
 * 301      | Clipper2    | 5.731402   | 174.477  | 2352348
 * 1001     | V2          | 205.407133 | 4.868    | 66064
 * 1001     | release/1.x | 84.452671  | 11.841   | 17449540
 * 1001     | Clipper2    | 76.949340  | 12.996   | 17525094
 */

/// <summary>
/// Benchmarks normalization performance for star polygons.
/// </summary>
[MemoryDiagnoser]
[OperationsPerSecond]
public class NormalizationBenches
{
    private Polygon polygon;
    private PathsD clipperSubject;
    private const int ClipperPrecision = 6;

    [Params(101, 301, 1001)]
    public int VertexCount { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        this.polygon = BuildStarPolygon(this.VertexCount, 100d);
        this.clipperSubject = BuildClipperSubject(this.polygon);
        GetLowestPathInfo(this.clipperSubject, out int lowestPathIdx, out bool isNegArea);
        if (lowestPathIdx >= 0 && isNegArea)
        {
            ReversePaths(this.clipperSubject);
        }
    }

    [Benchmark]
    public Polygon Normalize()
        => PolygonClipper.Normalize(this.polygon, FillRule.Positive);

    [Benchmark(Baseline = true)]
    public PolyTreeD Clipper2Union()
    {
        ClipperD clipper = new(ClipperPrecision)
        {
            PreserveCollinear = false,
            ReverseSolution = false
        };

        clipper.AddSubject(this.clipperSubject);
        PolyTreeD solution = [];
        clipper.Execute(ClipType.Union, Clipper2Lib.FillRule.Positive, solution);
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

    private static void ReversePaths(PathsD paths)
    {
        for (int i = 0; i < paths.Count; i++)
        {
            paths[i].Reverse();
        }
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

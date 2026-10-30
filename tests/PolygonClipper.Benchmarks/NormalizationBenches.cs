// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using BenchmarkDotNet.Attributes;
using Clipper2Lib;

namespace SixLabors.PolygonClipper.Benchmarks;

/*
BenchmarkDotNet v0.15.8, Windows 11 (10.0.26300.9818)
AMD RYZEN AI MAX+ 395 w/ Radeon 8060S 3.00GHz, 1 CPU, 32 logical and 16 physical cores
.NET SDK 11.0.100-preview.7.26381.103
  [Host]   : .NET 10.0.14 (10.0.14, 10.0.1426.52326), X64 RyuJIT x86-64-v4
  ShortRun : .NET 10.0.14 (10.0.14, 10.0.1426.52326), X64 RyuJIT x86-64-v4

Job=ShortRun  IterationCount=3  LaunchCount=1
WarmupCount=3

| Method        | VertexCount | Mean         | Error       | StdDev      | Op/s      | Ratio | RatioSD | Gen0      | Gen1      | Gen2      | Allocated   | Alloc Ratio |
|-------------- |------------ |-------------:|------------:|------------:|----------:|------:|--------:|----------:|----------:|----------:|------------:|------------:|
| Normalize     | 101         |   1,828.4 us |    229.6 us |    12.58 us |   546.932 |  3.64 |    0.08 |         - |         - |         - |     3.52 KB |        0.02 |
| Clipper2Union | 101         |     502.5 us |    223.6 us |    12.26 us | 1,989.877 |  1.00 |    0.03 |   12.2070 |    3.9063 |         - |   205.76 KB |        1.00 |
|               |             |              |             |             |           |       |         |           |           |           |             |             |
| Normalize     | 301         |  19,279.5 us |  6,815.6 us |   373.58 us |    51.869 |  3.20 |    0.06 |         - |         - |         - |    10.57 KB |       0.005 |
| Clipper2Union | 301         |   6,027.2 us |    676.2 us |    37.06 us |   165.916 |  1.00 |    0.01 |  492.1875 |  492.1875 |  492.1875 |  2297.18 KB |       1.000 |
|               |             |              |             |             |           |       |         |           |           |           |             |             |
| Normalize     | 1001        | 203,995.7 us | 30,961.7 us | 1,697.11 us |     4.902 |  2.54 |    0.07 |         - |         - |         - |    48.99 KB |       0.003 |
| Clipper2Union | 1001        |  80,238.6 us | 44,367.1 us | 2,431.91 us |    12.463 |  1.00 |    0.04 | 1857.1429 | 1857.1429 | 1857.1429 | 17115.08 KB |       1.000 |
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

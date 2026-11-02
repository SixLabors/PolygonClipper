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

| Method            | VertexCount | Mean         | Error       | StdDev      | Op/s      | Ratio | RatioSD | Gen0      | Gen1      | Gen2      | Allocated  | Alloc Ratio |
|------------------ |------------ |-------------:|------------:|------------:|----------:|------:|--------:|----------:|----------:|----------:|-----------:|------------:|
| Normalize         | 101         |   1,695.4 us |    251.4 us |    13.78 us |   589.827 |  3.68 |    0.10 |         - |         - |         - |    3.41 KB |        0.02 |
| Clipper2Normalize | 101         |     460.9 us |    253.8 us |    13.91 us | 2,169.784 |  1.00 |    0.04 |   12.2070 |    3.9063 |         - |  205.76 KB |        1.00 |
|                   |             |              |             |             |           |       |         |           |           |           |            |             |
| Normalize         | 301         |  16,385.1 us |  5,075.3 us |   278.19 us |    61.031 |  2.80 |    0.04 |         - |         - |         - |    9.66 KB |       0.004 |
| Clipper2Normalize | 301         |   5,860.5 us |    765.2 us |    41.94 us |   170.633 |  1.00 |    0.01 |  492.1875 |  492.1875 |  492.1875 | 2297.18 KB |       1.000 |
|                   |             |              |             |             |           |       |         |           |           |           |            |             |
| Normalize         | 1001        | 197,339.6 us | 76,838.2 us | 4,211.76 us |     5.067 |  2.51 |    0.09 |         - |         - |         - |   31.54 KB |       0.002 |
| Clipper2Normalize | 1001        |  78,746.5 us | 55,733.0 us | 3,054.92 us |    12.699 |  1.00 |    0.05 | 1857.1429 | 1857.1429 | 1857.1429 | 17115.1 KB |       1.000 |
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

    /// <summary>
    /// Gets or sets the number of distinct vertices in each star.
    /// </summary>
    [Params(101, 301, 1001)]
    public int VertexCount { get; set; }

    /// <summary>
    /// Builds the input geometry for both engines.
    /// </summary>
    [GlobalSetup]
    public void Setup()
    {
        this.polygon = BuildStarPolygon(this.VertexCount, 100d);
        this.clipperSubject = BenchmarkGeometry.ToPaths(this.polygon);
    }

    /// <summary>
    /// Resolves the positive-filled star with Martinez.
    /// </summary>
    /// <returns>The resolved polygon.</returns>
    [Benchmark]
    public Polygon Normalize()
        => PolygonClipper.Normalize(this.polygon, FillRule.Positive);

    /// <summary>
    /// Resolves the same positive-filled star with Clipper2.
    /// </summary>
    /// <returns>The resolved contour tree.</returns>
    [Benchmark(Baseline = true)]
    public object Clipper2Normalize()
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

    /// <summary>
    /// Builds a star whose nonadjacent edges cross.
    /// </summary>
    /// <param name="vertexCount">The odd number of distinct vertices.</param>
    /// <param name="radius">The distance from the center to each vertex.</param>
    /// <returns>The closed star contour.</returns>
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
}

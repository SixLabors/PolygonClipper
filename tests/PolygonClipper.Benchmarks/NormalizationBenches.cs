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

| Method            | VertexCount | Mean         | Error         | StdDev      | Op/s      | Ratio | RatioSD | Gen0      | Gen1      | Gen2      | Allocated   | Alloc Ratio |
|------------------ |------------ |-------------:|--------------:|------------:|----------:|------:|--------:|----------:|----------:|----------:|------------:|------------:|
| Normalize         | 101         |   1,702.8 us |     124.77 us |     6.84 us |   587.273 |  3.73 |    0.02 |         - |         - |         - |     3.41 KB |        0.02 |
| Clipper2Normalize | 101         |     456.9 us |      53.14 us |     2.91 us | 2,188.465 |  1.00 |    0.01 |   12.2070 |    3.9063 |         - |   205.76 KB |        1.00 |
|                   |             |              |               |             |           |       |         |           |           |           |             |             |
| Normalize         | 301         |  15,951.5 us |     452.26 us |    24.79 us |    62.690 |  2.72 |    0.02 |         - |         - |         - |    10.46 KB |       0.005 |
| Clipper2Normalize | 301         |   5,870.9 us |     710.59 us |    38.95 us |   170.333 |  1.00 |    0.01 |  492.1875 |  492.1875 |  492.1875 |  2297.18 KB |       1.000 |
|                   |             |              |               |             |           |       |         |           |           |           |             |             |
| Normalize         | 1001        | 199,441.3 us | 112,517.17 us | 6,167.45 us |     5.014 |  2.31 |    0.10 |         - |         - |         - |    48.88 KB |       0.003 |
| Clipper2Normalize | 1001        |  86,335.0 us |  57,758.23 us | 3,165.92 us |    11.583 |  1.00 |    0.05 | 1833.3333 | 1833.3333 | 1833.3333 | 17114.61 KB |       1.000 |
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
    public PolyTreeD Clipper2Normalize()
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

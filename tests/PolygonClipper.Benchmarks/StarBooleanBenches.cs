// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using BenchmarkDotNet.Attributes;
using BenchmarkDotNet.Configs;
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

| Method               | Categories   | VertexCount | Mean           | Error          | StdDev        | Op/s     | Ratio | RatioSD | Gen0       | Gen1       | Gen2      | Allocated    | Alloc Ratio |
|--------------------- |------------- |------------ |---------------:|---------------:|--------------:|---------:|------:|--------:|-----------:|-----------:|----------:|-------------:|------------:|
| Difference           | Difference   | 101         |       8.592 ms |      2.6134 ms |     0.1433 ms | 116.3822 |  1.39 |    0.04 |    62.5000 |    15.6250 |         - |   1217.61 KB |        0.30 |
| Clipper2Difference   | Difference   | 101         |       6.195 ms |      2.7946 ms |     0.1532 ms | 161.4186 |  1.00 |    0.03 |   328.1250 |   320.3125 |  164.0625 |   4082.33 KB |        1.00 |
|                      |              |             |                |                |               |          |       |         |            |            |           |              |             |
| Difference           | Difference   | 301         |     138.097 ms |     55.6052 ms |     3.0479 ms |   7.2413 |  1.15 |    0.06 |  2500.0000 |  2250.0000 |  750.0000 |  41170.04 KB |        1.16 |
| Clipper2Difference   | Difference   | 301         |     120.159 ms |    126.1519 ms |     6.9148 ms |   8.3223 |  1.00 |    0.07 |  3000.0000 |  3000.0000 | 1600.0000 |  35389.65 KB |        1.00 |
|                      |              |             |                |                |               |          |       |         |            |            |           |              |             |
| Difference           | Difference   | 1001        |   2,090.871 ms |    352.1612 ms |    19.3031 ms |   0.4783 |  0.56 |    0.01 | 28000.0000 | 25000.0000 | 4000.0000 | 473410.95 KB |        1.18 |
| Clipper2Difference   | Difference   | 1001        |   3,732.178 ms |  1,137.1280 ms |    62.3298 ms |   0.2679 |  1.00 |    0.02 | 25000.0000 | 24000.0000 | 5000.0000 | 399665.55 KB |        1.00 |
|                      |              |             |                |                |               |          |       |         |            |            |           |              |             |
| Intersection         | Intersection | 101         |       8.622 ms |      1.5939 ms |     0.0874 ms | 115.9833 |  1.38 |    0.03 |    93.7500 |          - |         - |   1626.57 KB |        0.33 |
| Clipper2Intersection | Intersection | 101         |       6.233 ms |      2.6067 ms |     0.1429 ms | 160.4281 |  1.00 |    0.03 |   328.1250 |   320.3125 |  164.0625 |   4962.32 KB |        1.00 |
|                      |              |             |                |                |               |          |       |         |            |            |           |              |             |
| Intersection         | Intersection | 301         |     173.414 ms |     68.4030 ms |     3.7494 ms |   5.7665 |  1.07 |    0.02 |  3500.0000 |  3250.0000 | 1000.0000 |  51374.25 KB |        1.18 |
| Clipper2Intersection | Intersection | 301         |     162.692 ms |     30.7326 ms |     1.6846 ms |   6.1466 |  1.00 |    0.01 |  3500.0000 |  3500.0000 | 1500.0000 |  43405.66 KB |        1.00 |
|                      |              |             |                |                |               |          |       |         |            |            |           |              |             |
| Intersection         | Intersection | 1001        |   2,371.434 ms |    588.3850 ms |    32.2514 ms |   0.4217 |  0.46 |    0.03 | 32000.0000 | 29000.0000 | 4000.0000 | 614610.15 KB |        1.22 |
| Clipper2Intersection | Intersection | 1001        |   5,125.881 ms |  7,195.6056 ms |   394.4154 ms |   0.1951 |  1.00 |    0.09 | 29000.0000 | 28000.0000 | 6000.0000 | 501897.55 KB |        1.00 |
|                      |              |             |                |                |               |          |       |         |            |            |           |              |             |
| Union                | Union        | 101         |       8.066 ms |      0.2893 ms |     0.0159 ms | 123.9813 |  0.20 |    0.00 |    46.8750 |    15.6250 |         - |    930.56 KB |        0.17 |
| Clipper2Union        | Union        | 101         |      40.169 ms |     15.9411 ms |     0.8738 ms |  24.8950 |  1.00 |    0.03 |   384.6154 |   230.7692 |   76.9231 |   5327.27 KB |        1.00 |
|                      |              |             |                |                |               |          |       |         |            |            |           |              |             |
| Union                | Union        | 301         |     127.845 ms |     93.5855 ms |     5.1297 ms |   7.8220 |  0.06 |    0.00 |  2000.0000 |  1800.0000 |  600.0000 |  30663.72 KB |        0.77 |
| Clipper2Union        | Union        | 301         |   2,227.592 ms |  1,380.0858 ms |    75.6472 ms |   0.4489 |  1.00 |    0.04 |  4000.0000 |  4000.0000 | 3000.0000 |  39612.54 KB |        1.00 |
|                      |              |             |                |                |               |          |       |         |            |            |           |              |             |
| Union                | Union        | 1001        |   1,663.617 ms |    317.9582 ms |    17.4284 ms |   0.6011 | 0.007 |    0.00 | 20000.0000 | 18000.0000 | 3000.0000 |  361171.7 KB |        0.95 |
| Clipper2Union        | Union        | 1001        | 240,792.163 ms | 71,896.3248 ms | 3,940.8802 ms |   0.0042 | 1.000 |    0.02 | 23000.0000 | 21000.0000 | 6000.0000 | 378279.99 KB |        1.00 |
|                      |              |             |                |                |               |          |       |         |            |            |           |              |             |
| Xor                  | Xor          | 101         |      16.579 ms |     15.8643 ms |     0.8696 ms |  60.3162 |  1.33 |    0.07 |   968.7500 |   937.5000 |  468.7500 |  10455.01 KB |        1.26 |
| Clipper2Xor          | Xor          | 101         |      12.510 ms |      7.3404 ms |     0.4024 ms |  79.9386 |  1.00 |    0.04 |   609.3750 |   609.3750 |  375.0000 |   8327.71 KB |        1.00 |
|                      |              |             |                |                |               |          |       |         |            |            |           |              |             |
| Xor                  | Xor          | 301         |     234.583 ms |     73.9136 ms |     4.0515 ms |   4.2629 |  0.80 |    0.02 |  5000.0000 |  4000.0000 | 1000.0000 |  90769.98 KB |        1.27 |
| Clipper2Xor          | Xor          | 301         |     293.448 ms |     77.4884 ms |     4.2474 ms |   3.4078 |  1.00 |    0.02 |  5000.0000 |  5000.0000 | 2000.0000 |  71681.67 KB |        1.00 |
|                      |              |             |                |                |               |          |       |         |            |            |           |              |             |
| Xor                  | Xor          | 1001        |   3,112.260 ms |    227.6820 ms |    12.4800 ms |   0.3213 |  0.26 |    0.02 | 54000.0000 | 47000.0000 | 5000.0000 | 948257.05 KB |        1.27 |
| Clipper2Xor          | Xor          | 1001        |  11,932.729 ms | 17,927.6550 ms |   982.6753 ms |   0.0838 |  1.00 |    0.10 | 45000.0000 | 44000.0000 | 7000.0000 | 744375.98 KB |        1.00 |
*/

/// <summary>
/// Benchmarks PolygonClipper boolean operations using two overlapping star polygons.
/// </summary>
[MemoryDiagnoser]
[OperationsPerSecond]
[GroupBenchmarksBy(BenchmarkLogicalGroupRule.ByCategory)]
[CategoriesColumn]
public class StarBooleanBenches
{
    private const int ClipperPrecision = 6;
    private Polygon subject;
    private Polygon clipping;
    private PathsD clipperSubject;
    private PathsD clipperClipping;

    /// <summary>
    /// Gets or sets the number of distinct vertices in each star.
    /// </summary>
    [Params(101, 301, 1001)]
    public int VertexCount { get; set; }

    /// <summary>
    /// Builds the same star inputs for both engines.
    /// </summary>
    [GlobalSetup]
    public void Setup()
    {
        this.subject = BuildStarPolygon(this.VertexCount, 100D, 0D, 0D);
        this.clipping = BuildStarPolygon(this.VertexCount, 100D, 35D, 22D);
        this.clipperSubject = BenchmarkGeometry.ToPaths(this.subject);
        this.clipperClipping = BenchmarkGeometry.ToPaths(this.clipping);
    }

    /// <summary>
    /// Unions the source stars with even-odd filling.
    /// </summary>
    /// <returns>The resolved polygon.</returns>
    [Benchmark]
    [BenchmarkCategory("Union")]
    public Polygon Union()
        => PolygonClipper.Union(this.subject, this.clipping, FillRule.EvenOdd);

    /// <summary>
    /// Performs the same union operation and fill rule with Clipper2.
    /// </summary>
    /// <returns>The resolved contour tree.</returns>
    [Benchmark(Baseline = true)]
    [BenchmarkCategory("Union")]
    public PolyTreeD Clipper2Union()
    {
        ClipperD clipper = CreateClipper();
        clipper.AddSubject(this.clipperSubject);
        clipper.AddClip(this.clipperClipping);
        PolyTreeD solution = [];
        clipper.Execute(ClipType.Union, Clipper2Lib.FillRule.EvenOdd, solution);
        return solution;
    }

    /// <summary>
    /// Intersects the source stars with even-odd filling.
    /// </summary>
    /// <returns>The resolved polygon.</returns>
    [Benchmark]
    [BenchmarkCategory("Intersection")]
    public Polygon Intersection()
        => PolygonClipper.Intersection(this.subject, this.clipping, FillRule.EvenOdd);

    /// <summary>
    /// Performs the same intersection operation and fill rule with Clipper2.
    /// </summary>
    /// <returns>The resolved contour tree.</returns>
    [Benchmark(Baseline = true)]
    [BenchmarkCategory("Intersection")]
    public PolyTreeD Clipper2Intersection()
    {
        ClipperD clipper = CreateClipper();
        clipper.AddSubject(this.clipperSubject);
        clipper.AddClip(this.clipperClipping);
        PolyTreeD solution = [];
        clipper.Execute(ClipType.Intersection, Clipper2Lib.FillRule.EvenOdd, solution);
        return solution;
    }

    /// <summary>
    /// Subtracts the clipping star from the subject star with even-odd filling.
    /// </summary>
    /// <returns>The resolved polygon.</returns>
    [Benchmark]
    [BenchmarkCategory("Difference")]
    public Polygon Difference()
        => PolygonClipper.Difference(this.subject, this.clipping, FillRule.EvenOdd);

    /// <summary>
    /// Performs the same difference operation and fill rule with Clipper2.
    /// </summary>
    /// <returns>The resolved contour tree.</returns>
    [Benchmark(Baseline = true)]
    [BenchmarkCategory("Difference")]
    public PolyTreeD Clipper2Difference()
    {
        ClipperD clipper = CreateClipper();
        clipper.AddSubject(this.clipperSubject);
        clipper.AddClip(this.clipperClipping);
        PolyTreeD solution = [];
        clipper.Execute(ClipType.Difference, Clipper2Lib.FillRule.EvenOdd, solution);
        return solution;
    }

    /// <summary>
    /// Finds the symmetric difference of the source stars with even-odd filling.
    /// </summary>
    /// <returns>The resolved polygon.</returns>
    [Benchmark]
    [BenchmarkCategory("Xor")]
    public Polygon Xor()
        => PolygonClipper.Xor(this.subject, this.clipping, FillRule.EvenOdd);

    /// <summary>
    /// Performs the same xor operation and fill rule with Clipper2.
    /// </summary>
    /// <returns>The resolved contour tree.</returns>
    [Benchmark(Baseline = true)]
    [BenchmarkCategory("Xor")]
    public PolyTreeD Clipper2Xor()
    {
        ClipperD clipper = CreateClipper();
        clipper.AddSubject(this.clipperSubject);
        clipper.AddClip(this.clipperClipping);
        PolyTreeD solution = [];
        clipper.Execute(ClipType.Xor, Clipper2Lib.FillRule.EvenOdd, solution);
        return solution;
    }

    /// <summary>
    /// Creates a Clipper2 engine with matching output direction and collinear handling.
    /// </summary>
    /// <returns>The clipping engine.</returns>
    private static ClipperD CreateClipper()
        => new(ClipperPrecision)
        {
            PreserveCollinear = false,
            ReverseSolution = false
        };

    /// <summary>
    /// Builds a translated star whose nonadjacent edges cross.
    /// </summary>
    /// <param name="vertexCount">The odd number of distinct vertices.</param>
    /// <param name="radius">The distance from the center to each vertex.</param>
    /// <param name="centerX">The center X coordinate.</param>
    /// <param name="centerY">The center Y coordinate.</param>
    /// <returns>The closed star contour.</returns>
    private static Polygon BuildStarPolygon(int vertexCount, double radius, double centerX, double centerY)
    {
        if (vertexCount < 5 || (vertexCount & 1) == 0)
        {
            throw new ArgumentOutOfRangeException(nameof(vertexCount), "Vertex count must be an odd number >= 5.");
        }

        int step = (vertexCount - 1) / 2;
        Contour contour = new(vertexCount + 1);
        for (int i = 0; i < vertexCount; i++)
        {
            int index = (i * step) % vertexCount;
            double angle = (index * Math.PI * 2D) / vertexCount;
            double x = centerX + (Math.Cos(angle) * radius);
            double y = centerY + (Math.Sin(angle) * radius);
            contour.Add(new Vertex(x, y));
        }

        contour.Add(contour[0]);
        Polygon polygon = [contour];
        return polygon;
    }
}

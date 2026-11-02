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
| Difference           | Difference   | 101         |       8.593 ms |      1.0394 ms |     0.0570 ms | 116.3708 |  1.35 |    0.08 |    78.1250 |    31.2500 |         - |   1316.74 KB |        0.32 |
| Clipper2Difference   | Difference   | 101         |       6.392 ms |      8.5489 ms |     0.4686 ms | 156.4422 |  1.00 |    0.09 |   328.1250 |   320.3125 |  164.0625 |   4082.33 KB |        1.00 |
|                      |              |             |                |                |               |          |       |         |            |            |           |              |             |
| Difference           | Difference   | 301         |     130.095 ms |     99.1253 ms |     5.4334 ms |   7.6867 |  1.00 |    0.04 |  3000.0000 |  2800.0000 | 1000.0000 |  42073.83 KB |        1.19 |
| Clipper2Difference   | Difference   | 301         |     130.746 ms |     53.8317 ms |     2.9507 ms |   7.6484 |  1.00 |    0.03 |  3000.0000 |  3000.0000 | 1600.0000 |  35390.13 KB |        1.00 |
|                      |              |             |                |                |               |          |       |         |            |            |           |              |             |
| Difference           | Difference   | 1001        |   2,125.882 ms |    673.9883 ms |    36.9436 ms |   0.4704 |  0.57 |    0.02 | 28000.0000 | 25000.0000 | 4000.0000 | 484677.92 KB |        1.21 |
| Clipper2Difference   | Difference   | 1001        |   3,726.175 ms |  1,879.7932 ms |   103.0378 ms |   0.2684 |  1.00 |    0.03 | 25000.0000 | 24000.0000 | 5000.0000 | 399665.54 KB |        1.00 |
|                      |              |             |                |                |               |          |       |         |            |            |           |              |             |
| Intersection         | Intersection | 101         |       9.145 ms |      0.9038 ms |     0.0495 ms | 109.3527 |  1.53 |    0.04 |    93.7500 |    46.8750 |         - |   1755.73 KB |        0.35 |
| Clipper2Intersection | Intersection | 101         |       5.968 ms |      3.5492 ms |     0.1945 ms | 167.5699 |  1.00 |    0.04 |   328.1250 |   320.3125 |  164.0625 |   4962.32 KB |        1.00 |
|                      |              |             |                |                |               |          |       |         |            |            |           |              |             |
| Intersection         | Intersection | 301         |     157.319 ms |     79.0129 ms |     4.3310 ms |   6.3565 |  1.09 |    0.04 |  3250.0000 |  3000.0000 | 1000.0000 |  52551.37 KB |        1.21 |
| Clipper2Intersection | Intersection | 301         |     144.021 ms |     66.3565 ms |     3.6372 ms |   6.9434 |  1.00 |    0.03 |  3500.0000 |  3500.0000 | 1500.0000 |  43405.64 KB |        1.00 |
|                      |              |             |                |                |               |          |       |         |            |            |           |              |             |
| Intersection         | Intersection | 1001        |   2,282.190 ms |    388.6867 ms |    21.3052 ms |   0.4382 |  0.52 |    0.01 | 33000.0000 | 30000.0000 | 4000.0000 | 627672.06 KB |        1.25 |
| Clipper2Intersection | Intersection | 1001        |   4,390.261 ms |    894.3474 ms |    49.0222 ms |   0.2278 |  1.00 |    0.01 | 29000.0000 | 28000.0000 | 6000.0000 | 501898.72 KB |        1.00 |
|                      |              |             |                |                |               |          |       |         |            |            |           |              |             |
| Union                | Union        | 101         |       8.655 ms |      2.8097 ms |     0.1540 ms | 115.5335 |  0.21 |    0.00 |    46.8750 |    15.6250 |         - |    978.11 KB |        0.18 |
| Clipper2Union        | Union        | 101         |      40.964 ms |      4.0998 ms |     0.2247 ms |  24.4115 |  1.00 |    0.01 |   384.6154 |   230.7692 |   76.9231 |   5326.67 KB |        1.00 |
|                      |              |             |                |                |               |          |       |         |            |            |           |              |             |
| Union                | Union        | 301         |     128.309 ms |     63.5969 ms |     3.4860 ms |   7.7937 |  0.06 |    0.00 |  2250.0000 |  2000.0000 |  750.0000 |  31126.62 KB |        0.79 |
| Clipper2Union        | Union        | 301         |   2,173.310 ms |  1,079.3527 ms |    59.1630 ms |   0.4601 |  1.00 |    0.03 |  4000.0000 |  4000.0000 | 3000.0000 |  39617.97 KB |        1.00 |
|                      |              |             |                |                |               |          |       |         |            |            |           |              |             |
| Union                | Union        | 1001        |   1,734.050 ms |    566.3988 ms |    31.0462 ms |   0.5767 | 0.007 |    0.00 | 20000.0000 | 18000.0000 | 3000.0000 |  366522.3 KB |        0.97 |
| Clipper2Union        | Union        | 1001        | 234,883.294 ms | 49,429.5458 ms | 2,709.4002 ms |   0.0043 | 1.000 |    0.01 | 23000.0000 | 21000.0000 | 6000.0000 | 378275.68 KB |        1.00 |
|                      |              |             |                |                |               |          |       |         |            |            |           |              |             |
| Xor                  | Xor          | 101         |      18.371 ms |     15.0344 ms |     0.8241 ms |  54.4343 |  1.41 |    0.09 |  1000.0000 |   968.7500 |  500.0000 |  10669.71 KB |        1.28 |
| Clipper2Xor          | Xor          | 101         |      13.032 ms |     13.6608 ms |     0.7488 ms |  76.7356 |  1.00 |    0.07 |   609.3750 |   609.3750 |  375.0000 |   8327.78 KB |        1.00 |
|                      |              |             |                |                |               |          |       |         |            |            |           |              |             |
| Xor                  | Xor          | 301         |     235.357 ms |    110.2588 ms |     6.0437 ms |   4.2489 |  0.84 |    0.02 |  5000.0000 |  4000.0000 | 1000.0000 |  92684.45 KB |        1.29 |
| Clipper2Xor          | Xor          | 301         |     279.997 ms |     39.8076 ms |     2.1820 ms |   3.5715 |  1.00 |    0.01 |  5000.0000 |  5000.0000 | 2000.0000 |  71681.65 KB |        1.00 |
|                      |              |             |                |                |               |          |       |         |            |            |           |              |             |
| Xor                  | Xor          | 1001        |   3,251.201 ms |  1,311.9581 ms |    71.9129 ms |   0.3076 |  0.32 |    0.01 | 54000.0000 | 47000.0000 | 5000.0000 | 969498.71 KB |        1.30 |
| Clipper2Xor          | Xor          | 1001        |  10,123.423 ms |  1,516.1091 ms |    83.1031 ms |   0.0988 |  1.00 |    0.01 | 45000.0000 | 44000.0000 | 7000.0000 | 744382.44 KB |        1.00 |
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
    public object Clipper2Union()
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
    public object Clipper2Intersection()
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
    public object Clipper2Difference()
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
    public object Clipper2Xor()
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

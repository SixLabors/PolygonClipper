// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using BenchmarkDotNet.Attributes;
using Clipper2Lib;
using GeoJson.Feature;
using SixLabors.PolygonClipper.Tests;

namespace SixLabors.PolygonClipper.Benchmarks;

/*
BenchmarkDotNet v0.15.8, Windows 11 (10.0.26300.9818)
AMD RYZEN AI MAX+ 395 w/ Radeon 8060S 3.00GHz, 1 CPU, 32 logical and 16 physical cores
.NET SDK 11.0.100-preview.7.26381.103
  [Host]   : .NET 10.0.14 (10.0.14, 10.0.1426.52326), X64 RyuJIT x86-64-v4
  ShortRun : .NET 10.0.14 (10.0.14, 10.0.1426.52326), X64 RyuJIT x86-64-v4

Job=ShortRun  IterationCount=3  LaunchCount=1
WarmupCount=3

| Method        | File                 | FillRule | Mean          | Error          | StdDev      | Op/s       | Ratio | RatioSD | Gen0     | Gen1     | Gen2    | Allocated | Alloc Ratio |
|-------------- |--------------------- |--------- |--------------:|---------------:|------------:|-----------:|------:|--------:|---------:|---------:|--------:|----------:|------------:|
| Union         | asia.geojson         | EvenOdd  | 10,873.490 us |  2,805.9234 us | 153.8021 us |      91.97 |  0.69 |    0.03 |  15.6250 |        - |       - |  463688 B |        0.07 |
| Clipper2Union | asia.geojson         | EvenOdd  | 15,826.802 us | 13,961.0760 us | 765.2537 us |      63.18 |  1.00 |    0.06 | 437.5000 | 406.2500 | 93.7500 | 6613050 B |        1.00 |
|               |                      |          |               |                |             |            |       |         |          |          |         |           |             |
| Union         | asia.geojson         | NonZero  | 11,579.924 us |  8,157.2822 us | 447.1282 us |      86.36 |  0.78 |    0.04 |  15.6250 |        - |       - |  463688 B |        0.07 |
| Clipper2Union | asia.geojson         | NonZero  | 14,908.794 us | 11,245.5674 us | 616.4075 us |      67.07 |  1.00 |    0.05 | 437.5000 | 406.2500 | 93.7500 | 6613050 B |        1.00 |
|               |                      |          |               |                |             |            |       |         |          |          |         |           |             |
| Union         | hole_hole.geojson    | EvenOdd  |      5.475 us |      0.3199 us |   0.0175 us | 182,655.67 |  2.18 |    0.02 |   0.0916 |        - |       - |    1632 B |        0.16 |
| Clipper2Union | hole_hole.geojson    | EvenOdd  |      2.508 us |      0.4731 us |   0.0259 us | 398,689.33 |  1.00 |    0.01 |   0.5951 |   0.0114 |       - |    9992 B |        1.00 |
|               |                      |          |               |                |             |            |       |         |          |          |         |           |             |
| Union         | hole_hole.geojson    | NonZero  |      4.839 us |      0.5636 us |   0.0309 us | 206,639.50 |  2.44 |    0.03 |   0.0458 |        - |       - |     872 B |        0.12 |
| Clipper2Union | hole_hole.geojson    | NonZero  |      1.980 us |      0.4235 us |   0.0232 us | 505,151.22 |  1.00 |    0.01 |   0.4463 |   0.0076 |       - |    7512 B |        1.00 |
|               |                      |          |               |                |             |            |       |         |          |          |         |           |             |
| Union         | state(...)ojson [21] | EvenOdd  |    733.801 us |    252.2293 us |  13.8255 us |   1,362.77 |  2.32 |    0.04 |        - |        - |       - |   16136 B |        0.05 |
| Clipper2Union | state(...)ojson [21] | EvenOdd  |    315.624 us |     36.5730 us |   2.0047 us |   3,168.33 |  1.00 |    0.01 |  20.5078 |   6.8359 |       - |  348984 B |        1.00 |
|               |                      |          |               |                |             |            |       |         |          |          |         |           |             |
| Union         | state(...)ojson [21] | NonZero  |    737.601 us |     70.9301 us |   3.8879 us |   1,355.75 |  2.36 |    0.07 |        - |        - |       - |   16136 B |        0.05 |
| Clipper2Union | state(...)ojson [21] | NonZero  |    312.442 us |    182.0525 us |   9.9789 us |   3,200.59 |  1.00 |    0.04 |  20.5078 |   6.8359 |       - |  348984 B |        1.00 |
*/

/// <summary>
/// Benchmarks the performance of <c>PolygonClipper</c> for polygon union operations using a fixed input dataset.
/// </summary>
[MemoryDiagnoser]
[OperationsPerSecond]
public class Benches
{
    private Polygon subject;
    private Polygon clipping;
    private PathsD clipperSubject = null!;
    private PathsD clipperClipping = null!;
    private const int ClipperPrecision = 6;

    /// <summary>
    /// Gets or sets the input geometry file.
    /// </summary>
    [Params("hole_hole.geojson", "states_source.geojson", "asia.geojson")]
    public string File { get; set; }

    /// <summary>
    /// Gets or sets the fill rule used by both engines.
    /// </summary>
    [Params(FillRule.EvenOdd, FillRule.NonZero)]
    public FillRule FillRule { get; set; }

    /// <summary>
    /// Loads the input polygons for both engines.
    /// </summary>
    [GlobalSetup]
    public void Setup()
    {
        FeatureCollection collection = TestData.Benchmarks.GetFeatureCollection(this.File);
        (Polygon subject, Polygon clipping) = TestPolygonUtilities.BuildPolygon(collection);
        this.subject = subject;
        this.clipping = clipping;

        this.clipperSubject = BenchmarkGeometry.ToPaths(subject);
        this.clipperClipping = BenchmarkGeometry.ToPaths(clipping);
    }

    /// <summary>
    /// Unions the source polygons with the selected fill rule.
    /// </summary>
    /// <returns>The resolved polygon.</returns>
    [Benchmark]
    public Polygon Union() => PolygonClipper.Union(this.subject, this.clipping, this.FillRule);

    /// <summary>
    /// Unions the same polygons and fill rule with Clipper2.
    /// </summary>
    /// <returns>The resolved contour tree.</returns>
    [Benchmark(Baseline = true)]
    public object Clipper2Union()
    {
        ClipperD clipper = new(ClipperPrecision)
        {
            PreserveCollinear = false,
            ReverseSolution = false
        };

        clipper.AddSubject(this.clipperSubject);
        clipper.AddClip(this.clipperClipping);
        PolyTreeD solution = [];
        clipper.Execute(ClipType.Union, (Clipper2Lib.FillRule)this.FillRule, solution);
        return solution;
    }
}

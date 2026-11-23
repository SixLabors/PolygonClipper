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

| Method        | File                 | FillRule | Mean          | Error          | StdDev        | Op/s       | Ratio | RatioSD | Gen0     | Gen1     | Gen2     | Allocated | Alloc Ratio |
|-------------- |--------------------- |--------- |--------------:|---------------:|--------------:|-----------:|------:|--------:|---------:|---------:|---------:|----------:|------------:|
| Union         | asia.geojson         | EvenOdd  | 10,308.052 us |  2,417.9597 us |   132.5365 us |      97.01 |  0.70 |    0.03 |  15.6250 |        - |        - |  455752 B |        0.07 |
| Clipper2Union | asia.geojson         | EvenOdd  | 14,730.047 us | 13,378.4588 us |   733.3185 us |      67.89 |  1.00 |    0.06 | 484.3750 | 468.7500 | 140.6250 | 6613312 B |        1.00 |
|               |                      |          |               |                |               |            |       |         |          |          |          |           |             |
| Union         | asia.geojson         | NonZero  | 10,082.858 us |  2,377.1639 us |   130.3004 us |      99.18 |  0.67 |    0.14 |  15.6250 |        - |        - |  455752 B |        0.07 |
| Clipper2Union | asia.geojson         | NonZero  | 15,738.498 us | 64,525.5472 us | 3,536.8630 us |      63.54 |  1.04 |    0.30 | 375.0000 | 343.7500 | 187.5000 | 6613021 B |        1.00 |
|               |                      |          |               |                |               |            |       |         |          |          |          |           |             |
| Union         | hole_hole.geojson    | EvenOdd  |      5.080 us |      2.2882 us |     0.1254 us | 196,852.80 |  1.98 |    0.07 |   0.0763 |        - |        - |    1304 B |        0.13 |
| Clipper2Union | hole_hole.geojson    | EvenOdd  |      2.562 us |      1.3565 us |     0.0744 us | 390,384.23 |  1.00 |    0.04 |   0.5951 |   0.0114 |        - |    9992 B |        1.00 |
|               |                      |          |               |                |               |            |       |         |          |          |          |           |             |
| Union         | hole_hole.geojson    | NonZero  |      4.533 us |      2.4151 us |     0.1324 us | 220,607.24 |  2.16 |    0.07 |   0.0305 |        - |        - |     616 B |        0.08 |
| Clipper2Union | hole_hole.geojson    | NonZero  |      2.095 us |      0.8690 us |     0.0476 us | 477,409.56 |  1.00 |    0.03 |   0.4463 |   0.0076 |        - |    7512 B |        1.00 |
|               |                      |          |               |                |               |            |       |         |          |          |          |           |             |
| Union         | state(...)ojson [21] | EvenOdd  |    703.498 us |    382.3907 us |    20.9601 us |   1,421.47 |  2.31 |    0.10 |        - |        - |        - |   15826 B |        0.05 |
| Clipper2Union | state(...)ojson [21] | EvenOdd  |    304.315 us |    215.0055 us |    11.7852 us |   3,286.06 |  1.00 |    0.05 |  20.5078 |   9.7656 |        - |  348984 B |        1.00 |
|               |                      |          |               |                |               |            |       |         |          |          |          |           |             |
| Union         | state(...)ojson [21] | NonZero  |    678.031 us |    291.4692 us |    15.9764 us |   1,474.86 |  2.25 |    0.09 |        - |        - |        - |   15826 B |        0.05 |
| Clipper2Union | state(...)ojson [21] | NonZero  |    301.061 us |    228.4132 us |    12.5201 us |   3,321.59 |  1.00 |    0.05 |  20.5078 |   9.7656 |        - |  348984 B |        1.00 |
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
    public PolyTreeD Clipper2Union()
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

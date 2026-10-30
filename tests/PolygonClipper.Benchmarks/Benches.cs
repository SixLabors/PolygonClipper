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

| Method                | File                 | Mean          | Error         | StdDev      | Op/s       | Gen0     | Gen1     | Gen2     | Allocated |
|---------------------- |--------------------- |--------------:|--------------:|------------:|-----------:|---------:|---------:|---------:|----------:|
| PolygonClipperEvenOdd | asia.geojson         | 11,200.902 us | 2,124.6733 us | 116.4605 us |      89.28 |  15.6250 |        - |        - |  463688 B |
| PolygonClipperNonZero | asia.geojson         | 11,388.093 us | 7,892.3194 us | 432.6047 us |      87.81 |  15.6250 |        - |        - |  463688 B |
| Clipper2Union         | asia.geojson         | 14,425.834 us | 7,891.2400 us | 432.5455 us |      69.32 | 468.7500 | 437.5000 | 125.0000 | 6613210 B |
| PolygonClipperEvenOdd | hole_hole.geojson    |      5.639 us |     0.6195 us |   0.0340 us | 177,347.74 |   0.0916 |        - |        - |    1632 B |
| PolygonClipperNonZero | hole_hole.geojson    |      5.105 us |     4.3337 us |   0.2375 us | 195,880.72 |   0.0458 |        - |        - |     872 B |
| Clipper2Union         | hole_hole.geojson    |      2.775 us |     1.8953 us |   0.1039 us | 360,372.84 |   0.5951 |   0.0114 |        - |    9992 B |
| PolygonClipperEvenOdd | state(...)ojson [21] |    738.178 us |   115.4451 us |   6.3279 us |   1,354.69 |        - |        - |        - |   16136 B |
| PolygonClipperNonZero | state(...)ojson [21] |    777.143 us |   449.2819 us |  24.6267 us |   1,286.76 |        - |        - |        - |   16136 B |
| Clipper2Union         | state(...)ojson [21] |    331.071 us |   326.0528 us |  17.8721 us |   3,020.50 |  20.5078 |   9.7656 |        - |  348984 B |
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

    [Params("hole_hole.geojson", "states_source.geojson", "asia.geojson")]
    public string File { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        FeatureCollection collection = TestData.Benchmarks.GetFeatureCollection(this.File);
        (Polygon subject, Polygon clipping) = TestPolygonUtilities.BuildPolygon(collection);
        this.subject = subject;
        this.clipping = clipping;

        this.clipperSubject = BuildClipperPaths(subject);
        this.clipperClipping = BuildClipperPaths(clipping);
    }

    /// <summary>
    /// Measures even-odd union on the raw input contours.
    /// </summary>
    /// <returns>The clipped geometry.</returns>
    [Benchmark]
    public Polygon PolygonClipperEvenOdd() => SixLabors.PolygonClipper.PolygonClipper.Union(this.subject, this.clipping, FillRule.EvenOdd);

    /// <summary>
    /// Measures non-zero clipping without a separate normalization operation.
    /// </summary>
    /// <returns>The clipped geometry.</returns>
    [Benchmark]
    public Polygon PolygonClipperNonZero() => SixLabors.PolygonClipper.PolygonClipper.Union(this.subject, this.clipping, FillRule.NonZero);

    [Benchmark]
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
        clipper.Execute(ClipType.Union, Clipper2Lib.FillRule.EvenOdd, solution);
        return solution;
    }

    private static PathsD BuildClipperPaths(Polygon polygon)
    {
        PathsD paths = new(polygon.Count);
        for (int i = 0; i < polygon.Count; i++)
        {
            Contour contour = polygon[i];
            PathD path = new(contour.Count);
            for (int j = 0; j < contour.Count; j++)
            {
                Vertex vertex = contour[j];
                path.Add(new PointD(vertex.X, vertex.Y));
            }

            paths.Add(path);
        }

        return paths;
    }
}

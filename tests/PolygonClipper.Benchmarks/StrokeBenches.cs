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

| Method                              | VertexCount | Mean            | Error         | StdDev        | Op/s        | Ratio | RatioSD | Gen0      | Gen1      | Gen2      | Allocated   | Alloc Ratio |
|------------------------------------ |------------ |----------------:|--------------:|--------------:|------------:|------:|--------:|----------:|----------:|----------:|------------:|------------:|
| PolygonStrokerProcessPolygonAndClip | 101         |        86.35 us |      10.28 us |      0.564 us | 11,581.2627 |  0.01 |    0.00 |    8.7891 |         - |         - |   144.85 KB |        0.08 |
| Clipper2InflatePaths                | 101         |     6,711.66 us |   6,421.82 us |    352.002 us |    148.9944 |  1.00 |    0.06 |  117.1875 |  117.1875 |  117.1875 |  1804.44 KB |        1.00 |
|                                     |             |                 |               |               |             |       |         |           |           |           |             |             |
| PolygonStrokerProcessPolygonAndClip | 301         |       263.81 us |      58.59 us |      3.211 us |  3,790.6037 | 0.004 |    0.00 |   27.3438 |   13.1836 |         - |   448.83 KB |        0.07 |
| Clipper2InflatePaths                | 301         |    66,415.27 us |  48,388.21 us |  2,652.321 us |     15.0568 | 1.001 |    0.05 |  625.0000 |  500.0000 |  500.0000 |  6144.33 KB |        1.00 |
|                                     |             |                 |               |               |             |       |         |           |           |           |             |             |
| PolygonStrokerProcessPolygonAndClip | 1001        |     1,199.12 us |     178.09 us |      9.762 us |    833.9431 | 0.000 |    0.00 |  498.0469 |  498.0469 |  498.0469 |  1921.08 KB |        0.06 |
| Clipper2InflatePaths                | 1001        | 2,556,662.97 us | 375,556.20 us | 20,585.503 us |      0.3911 | 1.000 |    0.01 | 3000.0000 | 2000.0000 | 2000.0000 | 32250.45 KB |        1.00 |
*/

/// <summary>
/// Benchmarks stroke generation and clipping against Clipper2 path inflation.
/// </summary>
[MemoryDiagnoser]
[OperationsPerSecond]
public class StrokeBenches
{
    private const int ClipperPrecision = 6;
    private const double StrokeWidth = 12D;
    private Polygon polygon;
    private PathsD clipperPaths;
    private StrokeOptions strokeOptions;

    [Params(101, 301, 1001)]
    public int VertexCount { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        this.polygon = BuildCompoundGearPolygon(this.VertexCount);
        this.clipperPaths = BuildClipperPaths(this.polygon);
        this.strokeOptions = new StrokeOptions()
        {
            LineJoin = LineJoin.Round,
            LineCap = LineCap.Round,
        };
    }

    [Benchmark]
    public Polygon PolygonStrokerProcessPolygonAndClip()
        => PolygonStroker.Stroke(this.polygon, StrokeWidth, this.strokeOptions);

    [Benchmark(Baseline = true)]
    public PathsD Clipper2InflatePaths()
        => Clipper.InflatePaths(
            this.clipperPaths,
            StrokeWidth * 0.5D,
            JoinType.Round,
            EndType.Round,
            4D,
            ClipperPrecision);

    private static Polygon BuildCompoundGearPolygon(int toothCount)
    {
        if (toothCount < 3)
        {
            throw new ArgumentOutOfRangeException(nameof(toothCount), "Tooth count must be >= 3.");
        }

        Contour outer = BuildGearContour(toothCount, 120D, 104D, 0D, 0D, clockwise: false);
        Contour inner = BuildGearContour(toothCount, 62D, 50D, 0D, 0D, clockwise: true);
        return [outer, inner];
    }

    private static Contour BuildGearContour(
        int toothCount,
        double outerRadius,
        double innerRadius,
        double centerX,
        double centerY,
        bool clockwise)
    {
        int vertexCount = toothCount * 2;
        double angleStep = Math.PI / toothCount;
        Contour contour = new(vertexCount + 1);

        for (int i = 0; i < vertexCount; i++)
        {
            double radius = (i & 1) == 0 ? outerRadius : innerRadius;
            double angle = i * angleStep;
            contour.Add(
                new Vertex(
                    centerX + (Math.Cos(angle) * radius),
                    centerY + (Math.Sin(angle) * radius)));
        }

        contour.Add(contour[0]);
        bool shouldBeCounterClockwise = !clockwise;
        if (contour.IsCounterClockwise() != shouldBeCounterClockwise)
        {
            contour.Reverse();
        }

        return contour;
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

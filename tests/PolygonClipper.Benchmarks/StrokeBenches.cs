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

| Method         | ToothCount | Mean       | Error       | StdDev    | Op/s    | Ratio | RatioSD | Gen0      | Gen1      | Gen2      | Allocated   | Alloc Ratio |
|--------------- |----------- |-----------:|------------:|----------:|--------:|------:|--------:|----------:|----------:|----------:|------------:|------------:|
| Stroke         | 101        |   3.908 ms |   0.1493 ms | 0.0082 ms | 255.888 |  1.41 |    0.05 |    7.8125 |         - |         - |   166.28 KB |        0.18 |
| Clipper2Stroke | 101        |   2.780 ms |   1.9662 ms | 0.1078 ms | 359.756 |  1.00 |    0.05 |   54.6875 |   31.2500 |         - |   943.66 KB |        1.00 |
|                |            |            |             |           |         |       |         |           |           |           |             |             |
| Stroke         | 301        |  32.695 ms |  33.5114 ms | 1.8369 ms |  30.586 |  1.16 |    0.06 |         - |         - |         - |   487.37 KB |        0.18 |
| Clipper2Stroke | 301        |  28.300 ms |   5.9455 ms | 0.3259 ms |  35.335 |  1.00 |    0.01 |  187.5000 |  125.0000 |   31.2500 |  2700.68 KB |        1.00 |
|                |            |            |             |           |         |       |         |           |           |           |             |             |
| Stroke         | 1001       | 354.823 ms |  29.7121 ms | 1.6286 ms |   2.818 |  0.38 |    0.00 | 1000.0000 | 1000.0000 | 1000.0000 | 20570.89 KB |        1.36 |
| Clipper2Stroke | 1001       | 934.864 ms | 134.0522 ms | 7.3479 ms |   1.070 |  1.00 |    0.01 | 1000.0000 | 1000.0000 | 1000.0000 | 15092.64 KB |        1.00 |
*/

/// <summary>
/// Compares resolved strokes of closed gear contours.
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

    /// <summary>
    /// Gets or sets the number of teeth per gear. Each contour has twice this many vertices.
    /// </summary>
    [Params(101, 301, 1001)]
    public int ToothCount { get; set; }

    /// <summary>
    /// Builds the input geometry for both engines.
    /// </summary>
    [GlobalSetup]
    public void Setup()
    {
        this.polygon = BuildCompoundGearPolygon(this.ToothCount);
        this.clipperPaths = BenchmarkGeometry.ToPaths(this.polygon);
        this.strokeOptions = new StrokeOptions()
        {
            LineJoin = LineJoin.Round,
            NormalizeOutput = true,
        };
    }

    /// <summary>
    /// Strokes both closed gears and resolves overlaps.
    /// </summary>
    /// <returns>The resolved stroke polygon.</returns>
    [Benchmark]
    public Polygon Stroke()
        => PolygonStroker.Stroke(this.polygon, StrokeWidth, this.strokeOptions);

    /// <summary>
    /// Strokes the same closed gears with Clipper2 and the same arc error.
    /// </summary>
    /// <returns>The resolved stroke paths.</returns>
    [Benchmark(Baseline = true)]
    public object Clipper2Stroke()
        => Clipper.InflatePaths(
            this.clipperPaths,
            StrokeWidth * 0.5D,
            JoinType.Round,
            EndType.Joined,
            4D,
            ClipperPrecision,
            arcTolerance: 0.125D);

    /// <summary>
    /// Builds two closed gear contours with opposite directions.
    /// </summary>
    /// <param name="toothCount">The number of teeth in each gear.</param>
    /// <returns>The two source contours.</returns>
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

    /// <summary>
    /// Builds a gear by alternating the tip and root radii.
    /// </summary>
    /// <param name="toothCount">The number of teeth.</param>
    /// <param name="outerRadius">The radius of each tooth tip.</param>
    /// <param name="innerRadius">The radius of each tooth root.</param>
    /// <param name="centerX">The center X coordinate.</param>
    /// <param name="centerY">The center Y coordinate.</param>
    /// <param name="clockwise">Whether vertices run clockwise.</param>
    /// <returns>The closed gear contour.</returns>
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
}

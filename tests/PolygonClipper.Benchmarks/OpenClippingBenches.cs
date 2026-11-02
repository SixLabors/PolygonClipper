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

| Method               | PathCount | VerticesPerPath | Transpose | Mean         | Error         | StdDev      | Op/s      | Ratio | RatioSD | Gen0     | Gen1     | Gen2     | Allocated  | Alloc Ratio |
|--------------------- |---------- |---------------- |---------- |-------------:|--------------:|------------:|----------:|------:|--------:|---------:|---------:|---------:|-----------:|------------:|
| Intersection         | 10        | 2               | False     |     9.883 us |     3.8132 us |   0.2090 us | 101,188.0 |  3.92 |    0.09 |   0.1526 |        - |        - |    2.66 KB |        0.20 |
| Clipper2Intersection | 10        | 2               | False     |     2.519 us |     0.6361 us |   0.0349 us | 396,913.1 |  1.00 |    0.02 |   0.8163 |   0.0267 |        - |   13.36 KB |        1.00 |
|                      |           |                 |           |              |               |             |           |       |         |          |          |          |            |             |
| Intersection         | 10        | 2               | True      |    10.748 us |     2.3662 us |   0.1297 us |  93,036.3 |  4.25 |    0.16 |   0.1526 |        - |        - |    2.66 KB |        0.18 |
| Clipper2Intersection | 10        | 2               | True      |     2.531 us |     1.8811 us |   0.1031 us | 395,167.2 |  1.00 |    0.05 |   0.9117 |   0.0343 |        - |   14.91 KB |        1.00 |
|                      |           |                 |           |              |               |             |           |       |         |          |          |          |            |             |
| Intersection         | 10        | 13              | False     |    26.421 us |     5.0646 us |   0.2776 us |  37,849.3 |  2.77 |    0.05 |   0.3967 |        - |        - |    6.76 KB |        0.16 |
| Clipper2Intersection | 10        | 13              | False     |     9.543 us |     3.2125 us |   0.1761 us | 104,784.4 |  1.00 |    0.02 |   2.5940 |   0.2747 |        - |   42.57 KB |        1.00 |
|                      |           |                 |           |              |               |             |           |       |         |          |          |          |            |             |
| Intersection         | 10        | 13              | True      |    33.000 us |     2.9610 us |   0.1623 us |  30,303.4 |  5.75 |    0.19 |   0.3662 |        - |        - |    6.76 KB |        0.21 |
| Clipper2Intersection | 10        | 13              | True      |     5.748 us |     3.9759 us |   0.2179 us | 173,965.4 |  1.00 |    0.05 |   1.9608 |   0.1678 |        - |   32.04 KB |        1.00 |
|                      |           |                 |           |              |               |             |           |       |         |          |          |          |            |             |
| Intersection         | 1000      | 2               | False     | 1,078.401 us |   254.6082 us |  13.9559 us |     927.3 |  5.15 |    0.08 |  15.6250 |   5.8594 |        - |  257.89 KB |        0.24 |
| Clipper2Intersection | 1000      | 2               | False     |   209.518 us |    46.4339 us |   2.5452 us |   4,772.9 |  1.00 |    0.01 |  65.4297 |  50.2930 |        - | 1072.85 KB |        1.00 |
|                      |           |                 |           |              |               |             |           |       |         |          |          |          |            |             |
| Intersection         | 1000      | 2               | True      | 1,251.839 us |   465.4236 us |  25.5114 us |     798.8 |  1.98 |    0.06 |  15.6250 |   5.8594 |        - |  257.89 KB |        0.21 |
| Clipper2Intersection | 1000      | 2               | True      |   631.272 us |   285.8368 us |  15.6677 us |   1,584.1 |  1.00 |    0.03 |  76.1719 |  65.4297 |        - | 1244.98 KB |        1.00 |
|                      |           |                 |           |              |               |             |           |       |         |          |          |          |            |             |
| Intersection         | 1000      | 13              | False     | 4,641.406 us | 1,638.6524 us |  89.8201 us |     215.5 |  2.65 |    0.06 |  31.2500 |   7.8125 |        - |  597.73 KB |        0.16 |
| Clipper2Intersection | 1000      | 13              | False     | 1,751.292 us |   608.8764 us |  33.3746 us |     571.0 |  1.00 |    0.02 | 248.0469 | 246.0938 | 123.0469 |  3808.3 KB |        1.00 |
|                      |           |                 |           |              |               |             |           |       |         |          |          |          |            |             |
| Intersection         | 1000      | 13              | True      | 5,604.583 us | 6,479.1650 us | 355.1449 us |     178.4 |  3.65 |    0.26 |  31.2500 |   7.8125 |        - |  597.73 KB |        0.20 |
| Clipper2Intersection | 1000      | 13              | True      | 1,539.851 us | 1,446.5691 us |  79.2913 us |     649.4 |  1.00 |    0.06 | 166.0156 | 164.0625 |  82.0313 | 2931.87 KB |        1.00 |
*/

/// <summary>
/// Compares open line clipping against the same closed rectangle and hole.
/// </summary>
[MemoryDiagnoser]
[OperationsPerSecond]
public class OpenClippingBenches
{
    private Polygon subject = null!;
    private Polygon clip = null!;
    private PathsD openSubject = null!;
    private PathsD closedClip = null!;

    /// <summary>
    /// Gets or sets the number of independent source paths.
    /// </summary>
    [Params(10, 1000)]
    public int PathCount { get; set; }

    /// <summary>
    /// Gets or sets the number of vertices in each source path.
    /// </summary>
    [Params(2, 13)]
    public int VerticesPerPath { get; set; }

    /// <summary>
    /// Gets or sets whether both operands exchange their x and y coordinates.
    /// </summary>
    [Params(false, true)]
    public bool Transpose { get; set; }

    /// <summary>
    /// Builds the open paths and closed mask for both engines.
    /// </summary>
    [GlobalSetup]
    public void Setup()
    {
        this.subject = new(this.PathCount);
        this.openSubject = new(this.PathCount);
        for (int i = 0; i < this.PathCount; i++)
        {
            Contour contour = new(this.VerticesPerPath) { IsClosed = false };
            PathD path = new(this.VerticesPerPath);
            for (int j = 0; j < this.VerticesPerPath; j++)
            {
                double x = -100D + (1200D * j / (this.VerticesPerPath - 1));
                double y = (i * 100D) + 25D + (this.VerticesPerPath == 2 ? 0D : (j & 1) * 25D);
                if (this.Transpose)
                {
                    (x, y) = (y, x);
                }

                contour.Add(new Vertex(x, y));
                path.Add(new PointD(x, y));
            }

            this.subject.Add(contour);
            this.openSubject.Add(path);
        }

        double height = this.PathCount * 100D;
        Contour exterior = [new(0, 0), new(1000, 0), new(1000, height), new(0, height)];
        Contour hole = [new(300, height * 0.25D), new(300, height * 0.75D), new(700, height * 0.75D), new(700, height * 0.25D)];
        this.clip = new(2);
        this.closedClip = new(2);
        foreach (Contour contour in new[] { exterior, hole })
        {
            PathD path = new(contour.Count);
            Contour transformed = new(contour.Count);
            foreach (Vertex vertex in contour)
            {
                Vertex point = this.Transpose ? new(vertex.Y, vertex.X) : vertex;
                path.Add(new PointD(point.X, point.Y));
                transformed.Add(point);
            }

            this.closedClip.Add(path);
            this.clip.Add(transformed);
        }
    }

    /// <summary>
    /// Clips the original open paths with the closed mask using Martinez.
    /// </summary>
    /// <returns>The open result contours.</returns>
    [Benchmark]
    public Polygon Intersection() => PolygonClipper.Intersection(this.subject, this.clip, FillRule.NonZero);

    /// <summary>
    /// Clips the same original paths with the same closed mask using Clipper2.
    /// </summary>
    /// <returns>The open result paths.</returns>
    [Benchmark(Baseline = true)]
    public PathsD Clipper2Intersection()
    {
        ClipperD engine = new(6);
        engine.AddOpenSubject(this.openSubject);
        engine.AddClip(this.closedClip);
        PathsD closed = [];
        PathsD open = [];
        engine.Execute(ClipType.Intersection, Clipper2Lib.FillRule.NonZero, closed, open);
        return open;
    }
}

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

| Method               | PathCount | VerticesPerPath | Transpose | Mean         | Error        | StdDev      | Op/s      | Ratio | RatioSD | Gen0     | Gen1     | Gen2     | Allocated  | Alloc Ratio |
|--------------------- |---------- |---------------- |---------- |-------------:|-------------:|------------:|----------:|------:|--------:|---------:|---------:|---------:|-----------:|------------:|
| Intersection         | 10        | 2               | False     |    10.904 us |     9.838 us |   0.5393 us |  91,709.9 |  3.87 |    0.21 |   0.1678 |        - |        - |    2.84 KB |        0.21 |
| Clipper2Intersection | 10        | 2               | False     |     2.823 us |     1.937 us |   0.1062 us | 354,259.3 |  1.00 |    0.05 |   0.8163 |   0.0267 |        - |   13.36 KB |        1.00 |
|                      |           |                 |           |              |              |             |           |       |         |          |          |          |            |             |
| Intersection         | 10        | 2               | True      |    10.301 us |     2.189 us |   0.1200 us |  97,079.2 |  4.13 |    0.09 |   0.1678 |        - |        - |    2.84 KB |        0.19 |
| Clipper2Intersection | 10        | 2               | True      |     2.495 us |     1.087 us |   0.0596 us | 400,776.5 |  1.00 |    0.03 |   0.9117 |   0.0343 |        - |   14.91 KB |        1.00 |
|                      |           |                 |           |              |              |             |           |       |         |          |          |          |            |             |
| Intersection         | 10        | 13              | False     |    28.831 us |     1.610 us |   0.0883 us |  34,684.6 |  2.87 |    0.02 |   0.3967 |        - |        - |    6.94 KB |        0.16 |
| Clipper2Intersection | 10        | 13              | False     |    10.062 us |     1.545 us |   0.0847 us |  99,387.5 |  1.00 |    0.01 |   2.5940 |   0.2747 |        - |   42.57 KB |        1.00 |
|                      |           |                 |           |              |              |             |           |       |         |          |          |          |            |             |
| Intersection         | 10        | 13              | True      |    41.346 us |    18.318 us |   1.0041 us |  24,186.3 |  7.46 |    0.18 |   0.3662 |        - |        - |    6.94 KB |        0.22 |
| Clipper2Intersection | 10        | 13              | True      |     5.542 us |     1.384 us |   0.0758 us | 180,453.8 |  1.00 |    0.02 |   1.9608 |   0.1678 |        - |   32.04 KB |        1.00 |
|                      |           |                 |           |              |              |             |           |       |         |          |          |          |            |             |
| Intersection         | 1000      | 2               | False     | 1,136.246 us |   158.943 us |   8.7122 us |     880.1 |  5.68 |    0.13 |  15.6250 |   5.8594 |        - |  258.07 KB |        0.24 |
| Clipper2Intersection | 1000      | 2               | False     |   200.069 us |    93.368 us |   5.1178 us |   4,998.3 |  1.00 |    0.03 |  65.4297 |  50.2930 |        - | 1072.85 KB |        1.00 |
|                      |           |                 |           |              |              |             |           |       |         |          |          |          |            |             |
| Intersection         | 1000      | 2               | True      | 1,264.030 us |    91.072 us |   4.9920 us |     791.1 |  1.80 |    0.17 |  15.6250 |   5.8594 |        - |  258.07 KB |        0.21 |
| Clipper2Intersection | 1000      | 2               | True      |   708.218 us | 1,343.360 us |  73.6341 us |   1,412.0 |  1.01 |    0.13 |  76.1719 |  65.4297 |        - | 1244.98 KB |        1.00 |
|                      |           |                 |           |              |              |             |           |       |         |          |          |          |            |             |
| Intersection         | 1000      | 13              | False     | 4,938.736 us |   178.069 us |   9.7606 us |     202.5 |  1.23 |    0.10 |  31.2500 |   7.8125 |        - |  597.91 KB |        0.16 |
| Clipper2Intersection | 1000      | 13              | False     | 4,027.525 us | 6,991.955 us | 383.2526 us |     248.3 |  1.01 |    0.11 | 316.4063 | 312.5000 | 109.3750 | 3808.37 KB |        1.00 |
|                      |           |                 |           |              |              |             |           |       |         |          |          |          |            |             |
| Intersection         | 1000      | 13              | True      | 5,410.344 us | 3,778.677 us | 207.1221 us |     184.8 |  1.85 |    0.18 |  31.2500 |   7.8125 |        - |  597.91 KB |        0.20 |
| Clipper2Intersection | 1000      | 13              | True      | 2,948.340 us | 5,395.866 us | 295.7656 us |     339.2 |  1.01 |    0.13 | 234.3750 | 230.4688 |  74.2188 | 2931.89 KB |        1.00 |
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
    public object Intersection() => PolygonClipper.Intersection(this.subject, this.clip, FillRule.NonZero);

    /// <summary>
    /// Clips the same original paths with the same closed mask using Clipper2.
    /// </summary>
    /// <returns>The open result paths.</returns>
    [Benchmark(Baseline = true)]
    public object Clipper2Intersection()
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

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

| Method         | VertexCount | Delta | Mean           | Error           | StdDev        | Op/s        | Ratio | RatioSD | Gen0      | Gen1      | Gen2      | Allocated  | Alloc Ratio |
|--------------- |------------ |------ |---------------:|----------------:|--------------:|------------:|------:|--------:|----------:|----------:|----------:|-----------:|------------:|
| Offset         | 16          | -1200 |       2.403 us |       0.7705 us |     0.0422 us | 416,069.248 |  0.07 |    0.00 |    0.0305 |         - |         - |      560 B |        0.02 |
| Clipper2Offset | 16          | -1200 |      36.459 us |      55.4619 us |     3.0401 us |  27,427.985 |  1.00 |    0.10 |    1.3428 |         - |         - |    23376 B |        1.00 |
|                |             |       |                |                 |               |             |       |         |           |           |           |            |             |
| Offset         | 16          | -900  |      56.001 us |      49.8951 us |     2.7349 us |  17,856.887 |  4.20 |    0.80 |    0.1831 |         - |         - |     3072 B |        0.15 |
| Clipper2Offset | 16          | -900  |      13.825 us |      61.8652 us |     3.3910 us |  72,334.519 |  1.04 |    0.30 |    1.2207 |    0.0305 |         - |    20664 B |        1.00 |
|                |             |       |                |                 |               |             |       |         |           |           |           |            |             |
| Offset         | 16          | -100  |       3.765 us |       0.5495 us |     0.0301 us | 265,629.040 |  0.98 |    0.03 |    0.0572 |         - |         - |      984 B |        0.08 |
| Clipper2Offset | 16          | -100  |       3.846 us |       1.9876 us |     0.1089 us | 260,010.493 |  1.00 |    0.03 |    0.7248 |    0.0076 |         - |    12176 B |        1.00 |
|                |             |       |                |                 |               |             |       |         |           |           |           |            |             |
| Offset         | 16          | -1    |       4.728 us |       5.7869 us |     0.3172 us | 211,520.252 |  1.04 |    0.09 |    0.0572 |         - |         - |      984 B |        0.08 |
| Clipper2Offset | 16          | -1    |       4.546 us |       6.4522 us |     0.3537 us | 219,990.806 |  1.00 |    0.10 |    0.7248 |    0.0114 |         - |    12176 B |        1.00 |
|                |             |       |                |                 |               |             |       |         |           |           |           |            |             |
| Offset         | 16          | 1     |       4.026 us |       0.5961 us |     0.0327 us | 248,368.898 |  2.62 |    0.32 |    0.0534 |         - |         - |      984 B |        0.16 |
| Clipper2Offset | 16          | 1     |       1.558 us |       4.2528 us |     0.2331 us | 641,854.604 |  1.01 |    0.18 |    0.3738 |    0.0038 |         - |     6264 B |        1.00 |
|                |             |       |                |                 |               |             |       |         |           |           |           |            |             |
| Offset         | 16          | 100   |       4.123 us |       0.4013 us |     0.0220 us | 242,522.207 |  2.87 |    0.02 |    0.0534 |         - |         - |      984 B |        0.16 |
| Clipper2Offset | 16          | 100   |       1.438 us |       0.1523 us |     0.0083 us | 695,541.890 |  1.00 |    0.01 |    0.3738 |    0.0038 |         - |     6264 B |        1.00 |
|                |             |       |                |                 |               |             |       |         |           |           |           |            |             |
| Offset         | 16          | 1200  |       4.145 us |       4.5489 us |     0.2493 us | 241,229.810 |  3.02 |    0.19 |    0.0534 |         - |         - |      984 B |        0.16 |
| Clipper2Offset | 16          | 1200  |       1.372 us |       0.9723 us |     0.0533 us | 728,650.422 |  1.00 |    0.05 |    0.3738 |    0.0038 |         - |     6264 B |        1.00 |
|                |             |       |                |                 |               |             |       |         |           |           |           |            |             |
| Offset         | 256         | -1200 |      43.989 us |      13.4638 us |     0.7380 us |  22,733.036 | 0.003 |    0.00 |    0.2441 |         - |         - |     4400 B |       0.002 |
| Clipper2Offset | 256         | -1200 |  16,134.432 us |   2,791.2480 us |   152.9977 us |      61.979 | 1.000 |    0.01 |  468.7500 |  468.7500 |  468.7500 |  2313486 B |       1.000 |
|                |             |       |                |                 |               |             |       |         |           |           |           |            |             |
| Offset         | 256         | -900  |   2,361.832 us |     262.1388 us |    14.3687 us |     423.400 |  3.23 |    0.02 |         - |         - |         - |    37632 B |        0.18 |
| Clipper2Offset | 256         | -900  |     732.110 us |      24.1240 us |     1.3223 us |   1,365.916 |  1.00 |    0.00 |   12.6953 |    3.9063 |         - |   214040 B |        1.00 |
|                |             |       |                |                 |               |             |       |         |           |           |           |            |             |
| Offset         | 256         | -100  |      63.996 us |      19.3250 us |     1.0593 us |  15,625.878 |  0.49 |    0.01 |    0.4883 |         - |         - |     8664 B |        0.05 |
| Clipper2Offset | 256         | -100  |     130.294 us |       6.0478 us |     0.3315 us |   7,674.949 |  1.00 |    0.00 |    9.5215 |    1.9531 |         - |   162640 B |        1.00 |
|                |             |       |                |                 |               |             |       |         |           |           |           |            |             |
| Offset         | 256         | -1    |      62.656 us |       3.7711 us |     0.2067 us |  15,960.129 |  0.97 |    0.03 |    0.4883 |         - |         - |     8664 B |        0.06 |
| Clipper2Offset | 256         | -1    |      64.429 us |      41.1809 us |     2.2573 us |  15,521.011 |  1.00 |    0.04 |    8.9722 |    1.9531 |         - |   150536 B |        1.00 |
|                |             |       |                |                 |               |             |       |         |           |           |           |            |             |
| Offset         | 256         | 1     |      66.962 us |       9.0239 us |     0.4946 us |  14,933.742 |  4.35 |    0.11 |    0.4883 |         - |         - |     8664 B |        0.13 |
| Clipper2Offset | 256         | 1     |      15.408 us |       7.8896 us |     0.4325 us |  64,900.325 |  1.00 |    0.03 |    4.0588 |    0.4272 |         - |    67896 B |        1.00 |
|                |             |       |                |                 |               |             |       |         |           |           |           |            |             |
| Offset         | 256         | 100   |      65.022 us |       4.8034 us |     0.2633 us |  15,379.462 |  3.97 |    0.17 |    0.4883 |         - |         - |     8664 B |        0.13 |
| Clipper2Offset | 256         | 100   |      16.406 us |      14.6723 us |     0.8042 us |  60,951.676 |  1.00 |    0.06 |    4.0588 |    0.4272 |         - |    67896 B |        1.00 |
|                |             |       |                |                 |               |             |       |         |           |           |           |            |             |
| Offset         | 256         | 1200  |      73.638 us |      84.0497 us |     4.6070 us |  13,579.929 |  4.24 |    0.56 |    0.4883 |         - |         - |     8664 B |        0.13 |
| Clipper2Offset | 256         | 1200  |      17.596 us |      47.1671 us |     2.5854 us |  56,831.787 |  1.01 |    0.18 |    4.0588 |    0.4272 |         - |    67896 B |        1.00 |
|                |             |       |                |                 |               |             |       |         |           |           |           |            |             |
| Offset         | 1024        | -1200 |     209.607 us |     406.6169 us |    22.2880 us |   4,770.836 | 0.001 |    0.00 |    0.9766 |         - |         - |    16688 B |       0.000 |
| Clipper2Offset | 1024        | -1200 | 328,268.167 us | 142,160.3136 us | 7,792.2865 us |       3.046 | 1.000 |    0.03 | 2000.0000 | 2000.0000 | 2000.0000 | 34406024 B |       1.000 |
|                |             |       |                |                 |               |             |       |         |           |           |           |            |             |
| Offset         | 1024        | -900  |  10,390.567 us |     697.6087 us |    38.2383 us |      96.241 |  0.93 |    0.01 |         - |         - |         - |   148224 B |        0.19 |
| Clipper2Offset | 1024        | -900  |  11,164.283 us |   1,175.7563 us |    64.4472 us |      89.571 |  1.00 |    0.01 |   46.8750 |   15.6250 |         - |   790344 B |        1.00 |
|                |             |       |                |                 |               |             |       |         |           |           |           |            |             |
| Offset         | 1024        | -100  |     276.123 us |      36.4375 us |     1.9973 us |   3,621.579 |  0.20 |    0.00 |    1.9531 |         - |         - |    33240 B |        0.05 |
| Clipper2Offset | 1024        | -100  |   1,401.647 us |      77.3652 us |     4.2406 us |     713.446 |  1.00 |    0.00 |   37.1094 |   11.7188 |         - |   643472 B |        1.00 |
|                |             |       |                |                 |               |             |       |         |           |           |           |            |             |
| Offset         | 1024        | -1    |     268.599 us |      27.7142 us |     1.5191 us |   3,723.028 |  0.96 |    0.01 |    1.9531 |         - |         - |    33240 B |        0.06 |
| Clipper2Offset | 1024        | -1    |     281.066 us |      74.0769 us |     4.0604 us |   3,557.883 |  1.00 |    0.02 |   35.1563 |   20.5078 |         - |   593456 B |        1.00 |
|                |             |       |                |                 |               |             |       |         |           |           |           |            |             |
| Offset         | 1024        | 1     |     276.471 us |      18.1653 us |     0.9957 us |   3,617.020 |  4.38 |    0.29 |    1.9531 |         - |         - |    33240 B |        0.13 |
| Clipper2Offset | 1024        | 1     |      63.401 us |      93.7135 us |     5.1368 us |  15,772.592 |  1.00 |    0.10 |   15.8081 |    5.8594 |         - |   264600 B |        1.00 |
|                |             |       |                |                 |               |             |       |         |           |           |           |            |             |
| Offset         | 1024        | 100   |     284.974 us |      75.0911 us |     4.1160 us |   3,509.086 |  4.90 |    0.06 |    1.9531 |         - |         - |    33240 B |        0.13 |
| Clipper2Offset | 1024        | 100   |      58.213 us |       5.3678 us |     0.2942 us |  17,178.212 |  1.00 |    0.01 |   15.8081 |    5.8594 |         - |   264600 B |        1.00 |
|                |             |       |                |                 |               |             |       |         |           |           |           |            |             |
| Offset         | 1024        | 1200  |     284.830 us |      32.9884 us |     1.8082 us |   3,510.863 |  4.79 |    0.05 |    1.9531 |         - |         - |    33240 B |        0.13 |
| Clipper2Offset | 1024        | 1200  |      59.499 us |      11.5245 us |     0.6317 us |  16,806.922 |  1.00 |    0.01 |   15.8081 |    5.8594 |         - |   264600 B |        1.00 |
*/

/// <summary>
/// Compares complete polygon offsets on equivalent closed geometry.
/// </summary>
[MemoryDiagnoser]
[OperationsPerSecond]
public class OffsetBenches
{
    private const int ClipperPrecision = 8;
    private Polygon polygon;
    private PathsD paths;
    private OffsetOptions options;

    /// <summary>
    /// Gets or sets the number of source vertices.
    /// </summary>
    [Params(16, 256, 1024)]
    public int VertexCount { get; set; }

    /// <summary>
    /// Gets or sets the signed offset distance.
    /// </summary>
    [Params(-1200D, -900D, -100D, -1D, 1D, 100D, 1200D)]
    public double Delta { get; set; }

    /// <summary>
    /// Builds the regular polygon for both engines.
    /// </summary>
    [GlobalSetup]
    public void Setup()
    {
        Contour contour = new(this.VertexCount);
        PathD path = new(this.VertexCount);
        for (int i = 0; i < this.VertexCount; i++)
        {
            double angle = i * Math.PI * 2 / this.VertexCount;
            Vertex vertex = new(1000 * Math.Cos(angle), 1000 * Math.Sin(angle));
            contour.Add(vertex);
            path.Add(new(vertex.X, vertex.Y));
        }

        this.polygon = [contour];
        this.paths = [path];
        this.options = new() { LineJoin = LineJoin.Miter };
    }

    /// <summary>
    /// Offsets the source through its fill rule, boundary construction and Martinez resolution.
    /// </summary>
    /// <returns>The resolved offset polygon.</returns>
    [Benchmark]
    public Polygon Offset() => PolygonOffsetter.Offset(this.polygon, this.Delta, FillRule.NonZero, this.options);

    /// <summary>
    /// Offsets the same valid source with Clipper2 miter joins.
    /// </summary>
    /// <returns>The offset paths.</returns>
    [Benchmark(Baseline = true)]
    public PathsD Clipper2Offset()
        => Clipper.InflatePaths(this.paths, this.Delta, JoinType.Miter, EndType.Polygon, 4, ClipperPrecision);
}

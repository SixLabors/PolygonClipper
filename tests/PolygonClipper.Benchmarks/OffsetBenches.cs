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

| Method         | VertexCount | Delta | Mean           | Error           | StdDev         | Op/s        | Ratio | RatioSD | Gen0      | Gen1      | Gen2      | Allocated  | Alloc Ratio |
|--------------- |------------ |------ |---------------:|----------------:|---------------:|------------:|------:|--------:|----------:|----------:|----------:|-----------:|------------:|
| Offset         | 16          | -1200 |       2.468 us |       0.3326 us |      0.0182 us | 405,178.385 |  0.08 |    0.00 |    0.0343 |         - |         - |      616 B |        0.03 |
| Clipper2Offset | 16          | -1200 |      29.521 us |       4.0812 us |      0.2237 us |  33,874.633 |  1.00 |    0.01 |    1.3733 |    0.0305 |         - |    23376 B |        1.00 |
|                |             |       |                |                 |                |             |       |         |           |           |           |            |             |
| Offset         | 16          | -900  |      57.562 us |      31.7352 us |      1.7395 us |  17,372.449 |  4.41 |    0.40 |    0.1831 |         - |         - |     3184 B |        0.15 |
| Clipper2Offset | 16          | -900  |      13.131 us |      24.1340 us |      1.3229 us |  76,157.684 |  1.01 |    0.12 |    1.2207 |    0.0305 |         - |    20664 B |        1.00 |
|                |             |       |                |                 |                |             |       |         |           |           |           |            |             |
| Offset         | 16          | -100  |       6.204 us |       4.0512 us |      0.2221 us | 161,177.093 |  1.58 |    0.09 |    0.0916 |         - |         - |     1600 B |        0.13 |
| Clipper2Offset | 16          | -100  |       3.924 us |       4.0858 us |      0.2240 us | 254,843.936 |  1.00 |    0.07 |    0.7248 |    0.0114 |         - |    12176 B |        1.00 |
|                |             |       |                |                 |                |             |       |         |           |           |           |            |             |
| Offset         | 16          | -1    |       6.655 us |       1.1062 us |      0.0606 us | 150,259.247 |  1.73 |    0.15 |    0.0916 |         - |         - |     1600 B |        0.13 |
| Clipper2Offset | 16          | -1    |       3.878 us |       7.2758 us |      0.3988 us | 257,884.953 |  1.01 |    0.13 |    0.7248 |    0.0114 |         - |    12176 B |        1.00 |
|                |             |       |                |                 |                |             |       |         |           |           |           |            |             |
| Offset         | 16          | 1     |       6.678 us |       2.2652 us |      0.1242 us | 149,753.146 |  4.68 |    0.59 |    0.0916 |         - |         - |     1600 B |        0.26 |
| Clipper2Offset | 16          | 1     |       1.450 us |       4.1261 us |      0.2262 us | 689,619.764 |  1.02 |    0.19 |    0.3738 |    0.0038 |         - |     6264 B |        1.00 |
|                |             |       |                |                 |                |             |       |         |           |           |           |            |             |
| Offset         | 16          | 100   |       6.501 us |       4.1664 us |      0.2284 us | 153,826.681 |  5.02 |    0.32 |    0.0916 |         - |         - |     1600 B |        0.26 |
| Clipper2Offset | 16          | 100   |       1.299 us |       1.5966 us |      0.0875 us | 769,606.026 |  1.00 |    0.08 |    0.3738 |    0.0038 |         - |     6264 B |        1.00 |
|                |             |       |                |                 |                |             |       |         |           |           |           |            |             |
| Offset         | 16          | 1200  |       6.076 us |       2.4293 us |      0.1332 us | 164,583.210 |  4.76 |    0.10 |    0.0916 |         - |         - |     1600 B |        0.26 |
| Clipper2Offset | 16          | 1200  |       1.276 us |       0.2702 us |      0.0148 us | 783,515.559 |  1.00 |    0.01 |    0.3738 |    0.0038 |         - |     6264 B |        1.00 |
|                |             |       |                |                 |                |             |       |         |           |           |           |            |             |
| Offset         | 256         | -1200 |      44.279 us |       3.5438 us |      0.1942 us |  22,584.136 | 0.003 |    0.00 |    0.2441 |         - |         - |     4457 B |       0.002 |
| Clipper2Offset | 256         | -1200 |  15,788.872 us |   2,011.9164 us |    110.2799 us |      63.336 | 1.000 |    0.01 |  468.7500 |  468.7500 |  468.7500 |  2313486 B |       1.000 |
|                |             |       |                |                 |                |             |       |         |           |           |           |            |             |
| Offset         | 256         | -900  |   2,314.089 us |     923.7301 us |     50.6328 us |     432.135 |  3.34 |    0.07 |         - |         - |         - |    37786 B |        0.18 |
| Clipper2Offset | 256         | -900  |     692.685 us |      97.9071 us |      5.3666 us |   1,443.657 |  1.00 |    0.01 |   12.6953 |    3.9063 |         - |   214040 B |        1.00 |
|                |             |       |                |                 |                |             |       |         |           |           |           |            |             |
| Offset         | 256         | -100  |      99.391 us |       5.6818 us |      0.3114 us |  10,061.287 |  0.80 |    0.01 |    0.7324 |         - |         - |    13120 B |        0.08 |
| Clipper2Offset | 256         | -100  |     123.854 us |      26.3756 us |      1.4457 us |   8,074.024 |  1.00 |    0.01 |    9.6436 |    1.9531 |         - |   162640 B |        1.00 |
|                |             |       |                |                 |                |             |       |         |           |           |           |            |             |
| Offset         | 256         | -1    |     100.865 us |       5.5870 us |      0.3062 us |   9,914.212 |  1.71 |    0.01 |    0.7324 |         - |         - |    13120 B |        0.09 |
| Clipper2Offset | 256         | -1    |      59.068 us |       6.9002 us |      0.3782 us |  16,929.521 |  1.00 |    0.01 |    8.9722 |    1.9531 |         - |   150536 B |        1.00 |
|                |             |       |                |                 |                |             |       |         |           |           |           |            |             |
| Offset         | 256         | 1     |     103.380 us |       4.4748 us |      0.2453 us |   9,673.071 |  7.19 |    0.12 |    0.7324 |         - |         - |    13120 B |        0.19 |
| Clipper2Offset | 256         | 1     |      14.380 us |       5.0707 us |      0.2779 us |  69,540.046 |  1.00 |    0.02 |    4.0588 |    0.4272 |         - |    67896 B |        1.00 |
|                |             |       |                |                 |                |             |       |         |           |           |           |            |             |
| Offset         | 256         | 100   |     100.712 us |      11.0057 us |      0.6033 us |   9,929.330 |  6.95 |    0.12 |    0.7324 |         - |         - |    13120 B |        0.19 |
| Clipper2Offset | 256         | 100   |      14.492 us |       4.9685 us |      0.2723 us |  69,003.320 |  1.00 |    0.02 |    4.0588 |    0.4272 |         - |    67896 B |        1.00 |
|                |             |       |                |                 |                |             |       |         |           |           |           |            |             |
| Offset         | 256         | 1200  |     102.491 us |      16.3031 us |      0.8936 us |   9,757.000 |  7.05 |    0.07 |    0.7324 |         - |         - |    13120 B |        0.19 |
| Clipper2Offset | 256         | 1200  |      14.531 us |       2.2596 us |      0.1239 us |  68,817.537 |  1.00 |    0.01 |    4.0588 |    0.4272 |         - |    67896 B |        1.00 |
|                |             |       |                |                 |                |             |       |         |           |           |           |            |             |
| Offset         | 1024        | -1200 |     195.474 us |     171.1595 us |      9.3818 us |   5,115.769 | 0.001 |    0.00 |    0.9766 |         - |         - |    16744 B |       0.000 |
| Clipper2Offset | 1024        | -1200 | 323,053.700 us | 309,818.4386 us | 16,982.1941 us |       3.095 | 1.002 |    0.06 | 2000.0000 | 2000.0000 | 2000.0000 | 34406024 B |       1.000 |
|                |             |       |                |                 |                |             |       |         |           |           |           |            |             |
| Offset         | 1024        | -900  |  11,477.235 us |   8,978.6788 us |    492.1517 us |      87.129 |  0.94 |    0.05 |         - |         - |         - |   148336 B |        0.19 |
| Clipper2Offset | 1024        | -900  |  12,293.630 us |  11,688.8019 us |    640.7027 us |      81.343 |  1.00 |    0.06 |   46.8750 |   15.6250 |         - |   790344 B |        1.00 |
|                |             |       |                |                 |                |             |       |         |           |           |           |            |             |
| Offset         | 1024        | -100  |     503.410 us |     373.1325 us |     20.4527 us |   1,986.452 |  0.36 |    0.02 |    2.9297 |         - |         - |    50005 B |        0.08 |
| Clipper2Offset | 1024        | -100  |   1,388.242 us |   1,137.2492 us |     62.3365 us |     720.335 |  1.00 |    0.06 |   37.1094 |   11.7188 |         - |   643472 B |        1.00 |
|                |             |       |                |                 |                |             |       |         |           |           |           |            |             |
| Offset         | 1024        | -1    |     524.284 us |     516.7832 us |     28.3266 us |   1,907.365 |  1.88 |    0.16 |    2.9297 |         - |         - |    50026 B |        0.08 |
| Clipper2Offset | 1024        | -1    |     279.539 us |     420.7071 us |     23.0604 us |   3,577.313 |  1.00 |    0.10 |   35.1563 |   20.5078 |         - |   593456 B |        1.00 |
|                |             |       |                |                 |                |             |       |         |           |           |           |            |             |
| Offset         | 1024        | 1     |     542.421 us |     224.6571 us |     12.3142 us |   1,843.586 |  8.87 |    0.46 |    2.9297 |         - |         - |    50026 B |        0.19 |
| Clipper2Offset | 1024        | 1     |      61.299 us |      61.8332 us |      3.3893 us |  16,313.446 |  1.00 |    0.07 |   15.7471 |    5.7373 |         - |   264600 B |        1.00 |
|                |             |       |                |                 |                |             |       |         |           |           |           |            |             |
| Offset         | 1024        | 100   |     543.999 us |     637.9292 us |     34.9671 us |   1,838.237 |  7.22 |    1.46 |    2.9297 |         - |         - |    50026 B |        0.19 |
| Clipper2Offset | 1024        | 100   |      78.080 us |     330.4798 us |     18.1147 us |  12,807.297 |  1.04 |    0.29 |   15.8081 |    5.8594 |         - |   264600 B |        1.00 |
|                |             |       |                |                 |                |             |       |         |           |           |           |            |             |
| Offset         | 1024        | 1200  |     691.062 us |     587.1478 us |     32.1836 us |   1,447.049 |  9.34 |    1.47 |    2.9297 |         - |         - |    50026 B |        0.19 |
| Clipper2Offset | 1024        | 1200  |      75.458 us |     228.7613 us |     12.5392 us |  13,252.362 |  1.02 |    0.21 |   15.7471 |    5.7373 |         - |   264600 B |        1.00 |
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
    public object Clipper2Offset()
        => Clipper.InflatePaths(this.paths, this.Delta, JoinType.Miter, EndType.Polygon, 4, ClipperPrecision);
}

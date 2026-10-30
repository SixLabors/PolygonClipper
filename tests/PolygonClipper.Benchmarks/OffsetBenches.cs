// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using BenchmarkDotNet.Attributes;
using Clipper2Lib;
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

| Method         | VertexCount | Delta | Mean           | Error           | StdDev        | Op/s        | Ratio | RatioSD | Gen0      | Gen1      | Gen2      | Allocated  | Alloc Ratio |
|--------------- |------------ |------ |---------------:|----------------:|--------------:|------------:|------:|--------:|----------:|----------:|----------:|-----------:|------------:|
| Offset         | 16          | -1200 |       3.187 us |       1.1629 us |     0.0637 us | 313,729.508 |  0.09 |    0.00 |    0.0420 |         - |         - |      712 B |        0.03 |
| Clipper2Offset | 16          | -1200 |      33.982 us |      16.4081 us |     0.8994 us |  29,426.923 |  1.00 |    0.03 |    1.3733 |    0.0305 |         - |    23376 B |        1.00 |
|                |             |       |                |                 |               |             |       |         |           |           |           |            |             |
| Offset         | 16          | -1    |       6.414 us |       0.8044 us |     0.0441 us | 155,897.311 |  1.64 |    0.02 |    0.1068 |         - |         - |     1792 B |        0.15 |
| Clipper2Offset | 16          | -1    |       3.919 us |       0.9182 us |     0.0503 us | 255,160.744 |  1.00 |    0.02 |    0.7248 |    0.0114 |         - |    12176 B |        1.00 |
|                |             |       |                |                 |               |             |       |         |           |           |           |            |             |
| Offset         | 16          | 1     |       6.444 us |       0.8902 us |     0.0488 us | 155,181.218 |  4.69 |    0.07 |    0.1068 |         - |         - |     1792 B |        0.29 |
| Clipper2Offset | 16          | 1     |       1.373 us |       0.4167 us |     0.0228 us | 728,208.963 |  1.00 |    0.02 |    0.3738 |    0.0038 |         - |     6264 B |        1.00 |
|                |             |       |                |                 |               |             |       |         |           |           |           |            |             |
| Offset         | 256         | -1200 |      47.463 us |      10.8460 us |     0.5945 us |  21,068.890 | 0.003 |    0.00 |    0.2441 |         - |         - |     4553 B |       0.002 |
| Clipper2Offset | 256         | -1200 |  17,393.549 us |   2,751.6011 us |   150.8245 us |      57.493 | 1.000 |    0.01 |  468.7500 |  468.7500 |  468.7500 |  2313643 B |       1.000 |
|                |             |       |                |                 |               |             |       |         |           |           |           |            |             |
| Offset         | 256         | -1    |     112.811 us |      61.9141 us |     3.3937 us |   8,864.413 |  1.57 |    0.13 |    0.7324 |         - |         - |    13313 B |        0.09 |
| Clipper2Offset | 256         | -1    |      72.041 us |     123.9203 us |     6.7925 us |  13,880.950 |  1.01 |    0.11 |    8.9111 |    1.9531 |         - |   150536 B |        1.00 |
|                |             |       |                |                 |               |             |       |         |           |           |           |            |             |
| Offset         | 256         | 1     |     122.703 us |     117.8423 us |     6.4593 us |   8,149.775 |  7.00 |    1.07 |    0.7324 |         - |         - |    13313 B |        0.20 |
| Clipper2Offset | 256         | 1     |      17.908 us |      60.4415 us |     3.3130 us |  55,841.318 |  1.02 |    0.22 |    4.0588 |    0.4272 |         - |    67896 B |        1.00 |
|                |             |       |                |                 |               |             |       |         |           |           |           |            |             |
| Offset         | 1024        | -1200 |     213.038 us |      96.3788 us |     5.2828 us |   4,694.003 | 0.001 |    0.00 |    0.9766 |         - |         - |    16851 B |       0.000 |
| Clipper2Offset | 1024        | -1200 | 307,856.250 us | 111,568.5614 us | 6,115.4493 us |       3.248 | 1.000 |    0.02 | 1000.0000 | 1000.0000 | 1000.0000 | 34406036 B |       1.000 |
|                |             |       |                |                 |               |             |       |         |           |           |           |            |             |
| Offset         | 1024        | -1    |     544.064 us |      71.5710 us |     3.9230 us |   1,838.020 |  1.91 |    0.06 |    2.9297 |         - |         - |    50218 B |        0.08 |
| Clipper2Offset | 1024        | -1    |     284.680 us |     167.3285 us |     9.1718 us |   3,512.721 |  1.00 |    0.04 |   35.1563 |   20.5078 |         - |   593456 B |        1.00 |
|                |             |       |                |                 |               |             |       |         |           |           |           |            |             |
| Offset         | 1024        | 1     |     570.951 us |     484.5199 us |    26.5582 us |   1,751.463 |  9.28 |    0.50 |    2.9297 |         - |         - |    50218 B |        0.19 |
| Clipper2Offset | 1024        | 1     |      61.601 us |      47.6335 us |     2.6110 us |  16,233.583 |  1.00 |    0.05 |   15.8081 |    5.8594 |         - |   264600 B |        1.00 |
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
    [Params(-1200D, -1D, 1D)]
    public double Delta { get; set; }

    /// <summary>
    /// Builds the source and checks topology and coordinates before measurement.
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

        Polygon actual = this.Offset();
        PathsD expected = this.Clipper2Offset();

        // The edge-to-center distance of a regular polygon is radius * cos(pi / vertexCount).
        int contourCount = this.Delta <= -1000 * Math.Cos(Math.PI / this.VertexCount) ? 0 : 1;
        if (actual.Count != contourCount || expected.Count != contourCount)
        {
            throw new InvalidOperationException("The offset boundary count differs from the regular polygon's collapse distance.");
        }

        if (contourCount == 0)
        {
            return;
        }

        PathsD actualPaths = [new PathD(actual[0].Select(vertex => new PointD(vertex.X, vertex.Y)))];
        if (!TestPolygonUtilities.HaveEquivalentBoundaryGraphs(actualPaths, expected))
        {
            throw new InvalidOperationException("The offset boundary connections differ.");
        }

        // Clipper2 rounds input and output coordinates. Check agreement within one millionth
        // of a source unit as well as topology, rather than comparing timings for different work.
        foreach (Vertex vertex in actual[0])
        {
            double nearest = double.PositiveInfinity;
            PathD boundary = expected[0];
            for (int i = 0; i < boundary.Count; i++)
            {
                PointD first = boundary[i];
                PointD second = boundary[(i + 1) % boundary.Count];
                Vertex closest = PolygonUtilities.ClosestPointOnSegment(vertex, new(first.x, first.y), new(second.x, second.y));
                nearest = Math.Min(nearest, Vertex.DistanceSquared(vertex, closest));
            }

            if (nearest > 1E-12D)
            {
                throw new InvalidOperationException($"Offset vertex {vertex} differs from Clipper2 by {Math.Sqrt(nearest):R}, exceeding 1E-6.");
            }
        }
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

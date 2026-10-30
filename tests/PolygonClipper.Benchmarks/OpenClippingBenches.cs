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

| Method                     | PathCount | VerticesPerPath | Transpose | Mean         | Error         | StdDev      | Op/s      | Ratio | RatioSD | Gen0     | Gen1     | Gen2     | Allocated  | Alloc Ratio |
|--------------------------- |---------- |---------------- |---------- |-------------:|--------------:|------------:|----------:|------:|--------:|---------:|---------:|---------:|-----------:|------------:|
| PolygonClipperIntersection | 10        | 2               | False     |    10.041 us |      6.363 us |   0.3488 us |  99,591.4 |  3.84 |    0.17 |   0.1678 |        - |        - |    2.84 KB |        0.21 |
| Clipper2Intersection       | 10        | 2               | False     |     2.617 us |      1.762 us |   0.0966 us | 382,076.6 |  1.00 |    0.05 |   0.8163 |   0.0267 |        - |   13.36 KB |        1.00 |
|                            |           |                 |           |              |               |             |           |       |         |          |          |          |            |             |
| PolygonClipperIntersection | 10        | 2               | True      |    11.112 us |     31.236 us |   1.7122 us |  89,995.8 |  4.11 |    0.57 |   0.1678 |        - |        - |    2.84 KB |        0.19 |
| Clipper2Intersection       | 10        | 2               | True      |     2.706 us |      2.357 us |   0.1292 us | 369,564.4 |  1.00 |    0.06 |   0.9117 |   0.0343 |        - |   14.91 KB |        1.00 |
|                            |           |                 |           |              |               |             |           |       |         |          |          |          |            |             |
| PolygonClipperIntersection | 10        | 13              | False     |    29.653 us |     14.059 us |   0.7706 us |  33,723.5 |  2.98 |    0.08 |   0.3967 |        - |        - |    6.94 KB |        0.16 |
| Clipper2Intersection       | 10        | 13              | False     |     9.945 us |      2.376 us |   0.1302 us | 100,556.0 |  1.00 |    0.02 |   2.5940 |   0.2747 |        - |   42.57 KB |        1.00 |
|                            |           |                 |           |              |               |             |           |       |         |          |          |          |            |             |
| PolygonClipperIntersection | 10        | 13              | True      |    35.715 us |     12.425 us |   0.6811 us |  27,999.5 |  5.94 |    0.17 |   0.3662 |        - |        - |    6.94 KB |        0.22 |
| Clipper2Intersection       | 10        | 13              | True      |     6.019 us |      2.905 us |   0.1592 us | 166,136.7 |  1.00 |    0.03 |   1.9608 |   0.1678 |        - |   32.04 KB |        1.00 |
|                            |           |                 |           |              |               |             |           |       |         |          |          |          |            |             |
| PolygonClipperIntersection | 1000      | 2               | False     | 1,202.917 us |  1,173.842 us |  64.3422 us |     831.3 |  5.81 |    0.27 |  15.6250 |   5.8594 |        - |  258.07 KB |        0.24 |
| Clipper2Intersection       | 1000      | 2               | False     |   207.058 us |     27.796 us |   1.5236 us |   4,829.6 |  1.00 |    0.01 |  65.4297 |  50.2930 |        - | 1072.85 KB |        1.00 |
|                            |           |                 |           |              |               |             |           |       |         |          |          |          |            |             |
| PolygonClipperIntersection | 1000      | 2               | True      | 1,340.080 us |    949.095 us |  52.0231 us |     746.2 |  2.13 |    0.12 |  15.6250 |   5.8594 |        - |  258.07 KB |        0.21 |
| Clipper2Intersection       | 1000      | 2               | True      |   629.687 us |    598.724 us |  32.8181 us |   1,588.1 |  1.00 |    0.06 |  76.1719 |  65.4297 |        - | 1244.98 KB |        1.00 |
|                            |           |                 |           |              |               |             |           |       |         |          |          |          |            |             |
| PolygonClipperIntersection | 1000      | 13              | False     | 5,080.375 us |  1,062.886 us |  58.2604 us |     196.8 |  1.41 |    0.03 |  31.2500 |   7.8125 |        - |  597.91 KB |        0.16 |
| Clipper2Intersection       | 1000      | 13              | False     | 3,600.080 us |  1,497.421 us |  82.0787 us |     277.8 |  1.00 |    0.03 | 316.4063 | 312.5000 | 109.3750 | 3808.37 KB |        1.00 |
|                            |           |                 |           |              |               |             |           |       |         |          |          |          |            |             |
| PolygonClipperIntersection | 1000      | 13              | True      | 5,290.140 us |    385.407 us |  21.1255 us |     189.0 |  1.62 |    0.26 |  31.2500 |   7.8125 |        - |  597.91 KB |        0.20 |
| Clipper2Intersection       | 1000      | 13              | True      | 3,353.168 us | 12,912.942 us | 707.8019 us |     298.2 |  1.03 |    0.25 | 234.3750 | 230.4688 |  74.2188 | 2931.89 KB |        1.00 |
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
    /// Builds equal input geometry and checks exact output edges before measurement.
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

        Polygon result = (Polygon)this.PolygonClipperIntersection();
        PathsD expected = (PathsD)this.Clipper2Intersection();
        List<(double X1, double Y1, double X2, double Y2)> actualEdges = [];
        foreach (Contour contour in result)
        {
            if (contour.IsClosed)
            {
                throw new InvalidOperationException("Open-subject intersection returned an area contour.");
            }

            for (int i = 1; i < contour.Count; i++)
            {
                AddEdge(actualEdges, contour[i - 1].X, contour[i - 1].Y, contour[i].X, contour[i].Y);
            }
        }

        List<(double X1, double Y1, double X2, double Y2)> expectedEdges = [];
        foreach (PathD path in expected)
        {
            for (int i = 1; i < path.Count; i++)
            {
                AddEdge(expectedEdges, path[i - 1].x, path[i - 1].y, path[i].x, path[i].y);
            }
        }

        actualEdges.Sort();
        expectedEdges.Sort();
        if (!actualEdges.SequenceEqual(expectedEdges))
        {
            throw new InvalidOperationException("The benchmark inputs did not produce equivalent open output edges.");
        }
    }

    /// <summary>
    /// Clips the original open paths with the closed mask using Martinez.
    /// </summary>
    /// <returns>The open result contours.</returns>
    [Benchmark]
    public object PolygonClipperIntersection() => PolygonClipper.Intersection(this.subject, this.clip, FillRule.NonZero);

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

    /// <summary>
    /// Adds a direction-independent edge without rounding coordinates or changing subdivisions.
    /// </summary>
    /// <param name="edges">The collected output edges.</param>
    /// <param name="x1">The first endpoint's x coordinate.</param>
    /// <param name="y1">The first endpoint's y coordinate.</param>
    /// <param name="x2">The second endpoint's x coordinate.</param>
    /// <param name="y2">The second endpoint's y coordinate.</param>
    private static void AddEdge(List<(double X1, double Y1, double X2, double Y2)> edges, double x1, double y1, double x2, double y2)
    {
        if (x1 < x2 || (x1 == x2 && y1 <= y2))
        {
            edges.Add((x1, y1, x2, y2));
        }
        else
        {
            edges.Add((x2, y2, x1, y1));
        }
    }
}

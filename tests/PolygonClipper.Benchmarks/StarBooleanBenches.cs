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

| Method                     | VertexCount | Mean         | Error        | StdDev      | Op/s     | Gen0       | Gen1       | Gen2      | Allocated     |
|--------------------------- |------------ |-------------:|-------------:|------------:|---------:|-----------:|-----------:|----------:|--------------:|
| PolygonClipperUnion        | 101         |    10.241 ms |    14.189 ms |   0.7778 ms |  97.6464 |    46.8750 |    15.6250 |         - |     984.86 KB |
| PolygonClipperIntersection | 101         |     9.785 ms |     3.148 ms |   0.1726 ms | 102.2006 |    93.7500 |    46.8750 |         - |    1769.23 KB |
| PolygonClipperDifference   | 101         |     8.865 ms |     1.617 ms |   0.0886 ms | 112.8081 |    78.1250 |    15.6250 |         - |    1330.24 KB |
| PolygonClipperXor          | 101         |    19.532 ms |     7.799 ms |   0.4275 ms |  51.1991 |  1312.5000 |  1312.5000 |  843.7500 |   11968.01 KB |
| PolygonClipperUnion        | 301         |   122.200 ms |   128.921 ms |   7.0666 ms |   8.1833 |  2600.0000 |  2600.0000 | 1200.0000 |   33225.89 KB |
| PolygonClipperIntersection | 301         |   170.792 ms |   109.231 ms |   5.9873 ms |   5.8551 |  3333.3333 |  3333.3333 | 1333.3333 |   57243.84 KB |
| PolygonClipperDifference   | 301         |   156.373 ms |   170.076 ms |   9.3224 ms |   6.3950 |  3250.0000 |  3250.0000 | 1500.0000 |   48985.99 KB |
| PolygonClipperXor          | 301         |   277.201 ms |    61.909 ms |   3.3934 ms |   3.6075 |  6000.0000 |  5500.0000 | 2000.0000 |  106509.15 KB |
| PolygonClipperUnion        | 1001        | 1,853.955 ms | 1,062.212 ms |  58.2234 ms |   0.5394 | 21000.0000 | 20000.0000 | 5000.0000 |  421820.51 KB |
| PolygonClipperIntersection | 1001        | 2,498.312 ms |   270.235 ms |  14.8125 ms |   0.4003 | 34000.0000 | 32000.0000 | 6000.0000 |  738265.43 KB |
| PolygonClipperDifference   | 1001        | 2,083.018 ms |   226.483 ms |  12.4143 ms |   0.4801 | 28000.0000 | 26000.0000 | 5000.0000 |  539974.69 KB |
| PolygonClipperXor          | 1001        | 3,337.877 ms | 3,338.490 ms | 182.9939 ms |   0.2996 | 54000.0000 | 48000.0000 | 6000.0000 | 1080092.26 KB |
*/

/// <summary>
/// Benchmarks PolygonClipper boolean operations using two overlapping star polygons.
/// </summary>
[MemoryDiagnoser]
[OperationsPerSecond]
public class StarBooleanBenches
{
    private const int ClipperPrecision = 6;
    private Polygon subject;
    private Polygon clipping;
    private PathsD clipperSubject;
    private PathsD clipperClipping;

    [Params(101, 301, 1001)]
    public int VertexCount { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        this.subject = BuildStarPolygon(this.VertexCount, 100D, 0D, 0D);
        this.clipping = BuildStarPolygon(this.VertexCount, 100D, 35D, 22D);
        this.clipperSubject = BuildClipperPaths(this.subject);
        this.clipperClipping = BuildClipperPaths(this.clipping);
    }

    [Benchmark]
    public Polygon PolygonClipperUnion()
        => PolygonClipper.Union(this.subject, this.clipping, FillRule.EvenOdd);

    //[Benchmark]
    //public PolyTreeD Clipper2Union()
    //{
    //    ClipperD clipper = CreateClipper();
    //    clipper.AddSubject(this.clipperSubject);
    //    clipper.AddClip(this.clipperClipping);
    //    PolyTreeD solution = [];
    //    clipper.Execute(ClipType.Union, Clipper2Lib.FillRule.EvenOdd, solution);
    //    return solution;
    //}

    [Benchmark]
    public Polygon PolygonClipperIntersection()
        => PolygonClipper.Intersection(this.subject, this.clipping, FillRule.EvenOdd);

    //[Benchmark]
    //public PolyTreeD Clipper2Intersection()
    //{
    //    ClipperD clipper = CreateClipper();
    //    clipper.AddSubject(this.clipperSubject);
    //    clipper.AddClip(this.clipperClipping);
    //    PolyTreeD solution = [];
    //    clipper.Execute(ClipType.Intersection, Clipper2Lib.FillRule.EvenOdd, solution);
    //    return solution;
    //}

    [Benchmark]
    public Polygon PolygonClipperDifference()
        => PolygonClipper.Difference(this.subject, this.clipping, FillRule.EvenOdd);

    //[Benchmark]
    //public PolyTreeD Clipper2Difference()
    //{
    //    ClipperD clipper = CreateClipper();
    //    clipper.AddSubject(this.clipperSubject);
    //    clipper.AddClip(this.clipperClipping);
    //    PolyTreeD solution = [];
    //    clipper.Execute(ClipType.Difference, Clipper2Lib.FillRule.EvenOdd, solution);
    //    return solution;
    //}

    [Benchmark]
    public Polygon PolygonClipperXor()
        => PolygonClipper.Xor(this.subject, this.clipping, FillRule.EvenOdd);

    //[Benchmark]
    //public PolyTreeD Clipper2Xor()
    //{
    //    ClipperD clipper = CreateClipper();
    //    clipper.AddSubject(this.clipperSubject);
    //    clipper.AddClip(this.clipperClipping);
    //    PolyTreeD solution = [];
    //    clipper.Execute(ClipType.Xor, Clipper2Lib.FillRule.EvenOdd, solution);
    //    return solution;
    //}

    private static ClipperD CreateClipper()
        => new(ClipperPrecision)
        {
            PreserveCollinear = false,
            ReverseSolution = false
        };

    private static Polygon BuildStarPolygon(int vertexCount, double radius, double centerX, double centerY)
    {
        if (vertexCount < 5 || (vertexCount & 1) == 0)
        {
            throw new ArgumentOutOfRangeException(nameof(vertexCount), "Vertex count must be an odd number >= 5.");
        }

        int step = (vertexCount - 1) / 2;
        Contour contour = new(vertexCount + 1);
        for (int i = 0; i < vertexCount; i++)
        {
            int index = (i * step) % vertexCount;
            double angle = (index * Math.PI * 2D) / vertexCount;
            double x = centerX + (Math.Cos(angle) * radius);
            double y = centerY + (Math.Sin(angle) * radius);
            contour.Add(new Vertex(x, y));
        }

        contour.Add(contour[0]);
        Polygon polygon = [contour];
        return polygon;
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

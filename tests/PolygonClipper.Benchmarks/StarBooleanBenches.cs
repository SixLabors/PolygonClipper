// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using BenchmarkDotNet.Attributes;
using Clipper2Lib;

namespace SixLabors.PolygonClipper.Benchmarks;

/*
 * Comparison results: 30 October 2026.
 * Windows 11. AMD Ryzen AI Max+ 395. .NET 10.0.14. BenchmarkDotNet 0.15.8.
 * V2 build: 4f814c3 with the orientation-bound formula. release/1.x: cb06839. Clipper2: 4be4655.
 * Mean: milliseconds per operation. Op/s: operations per second. B/op: allocated bytes per operation.
 * Input preparation occurs before measurement. ClipperD uses precision 6.
 * Each case uses 12 measured iterations, a 250 ms iteration target, and one launch. All samples remain included.
 * Rust: rust-geo-booleanop 0.3.2 at b9bbdbb. rustc 1.98.0. Release build. geo-types 0.7.20.
 * Rust timings include the native call and output destruction. Rust supports EvenOdd only.
 * The .NET counters measure managed bytes. A separate Rust build counts requested native bytes.
 * The comparison harness measures the fill rules and engines shown below on the same pair of stars.
 * Warmup: 10 iterations, except the four variable 101-vertex cases, which use a repeat with 30 iterations.
 * Those repeats cover Union with EvenOdd and NonZero, and Intersection and Difference with EvenOdd.
 * Every supported engine uses the same longer warmup in each repeated case.
 * release/1.x NonZero includes its required input preparation.
 * Both dense-star EvenOdd unions pass the boundary checks despite different contour grouping.
 *
 * Vertices | Fill rule | Operation    | Engine      | Mean (ms)   | Op/s    | B/op
 * ---------+-----------+--------------+-------------+-------------+---------+----------
 * 101      | EvenOdd   | Union        | V2          | 9.531152    | 104.919 | 1221184
 * 101      | EvenOdd   | Union        | release/1.x | 33.957776   | 29.448  | 10058513
 * 101      | EvenOdd   | Union        | Clipper2    | 37.183475   | 26.894  | 5454480
 * 101      | EvenOdd   | Union        | Rust        | 28.678877   | 34.869  | 12025868
 * 101      | EvenOdd   | Intersection | V2          | 10.124404   | 98.771  | 1979952
 * 101      | EvenOdd   | Intersection | release/1.x | 35.579118   | 28.106  | 11013441
 * 101      | EvenOdd   | Intersection | Clipper2    | 6.114008    | 163.559 | 5081410
 * 101      | EvenOdd   | Intersection | Rust        | 29.568188   | 33.820  | 14123564
 * 101      | EvenOdd   | Difference   | V2          | 9.295369    | 107.580 | 1539000
 * 101      | EvenOdd   | Difference   | release/1.x | 34.142293   | 29.289  | 10492458
 * 101      | EvenOdd   | Difference   | Clipper2    | 6.015985    | 166.224 | 4180303
 * 101      | EvenOdd   | Difference   | Rust        | 30.183794   | 33.130  | 13713340
 * 101      | EvenOdd   | Xor          | V2          | 23.045433   | 43.393  | 10778162
 * 101      | EvenOdd   | Xor          | release/1.x | 33.714825   | 29.661  | 12728260
 * 101      | EvenOdd   | Xor          | Clipper2    | 10.867327   | 92.019  | 8527564
 * 101      | EvenOdd   | Xor          | Rust        | 35.518577   | 28.154  | 18811516
 * 101      | NonZero   | Union        | V2          | 7.799769    | 128.209 | 213480
 * 101      | NonZero   | Union        | release/1.x | 4.803016    | 208.202 | 1125464
 * 101      | NonZero   | Union        | Clipper2    | 3.329548    | 300.341 | 874861
 * 301      | EvenOdd   | Union        | V2          | 113.486163  | 8.812   | 34139832
 * 301      | EvenOdd   | Union        | release/1.x | 445.814992  | 2.243   | 88329168
 * 301      | EvenOdd   | Union        | Clipper2    | 1942.372475 | 0.515   | 40570064
 * 301      | EvenOdd   | Union        | Rust        | 419.326683  | 2.385   | 104351676
 * 301      | EvenOdd   | Intersection | V2          | 143.969358  | 6.946   | 55822008
 * 301      | EvenOdd   | Intersection | release/1.x | 495.890125  | 2.017   | 96940976
 * 301      | EvenOdd   | Intersection | Clipper2    | 123.051400  | 8.127   | 44447544
 * 301      | EvenOdd   | Intersection | Rust        | 468.113050  | 2.136   | 121501628
 * 301      | EvenOdd   | Difference   | V2          | 137.709375  | 7.262   | 45124380
 * 301      | EvenOdd   | Difference   | release/1.x | 459.630592  | 2.176   | 92145616
 * 301      | EvenOdd   | Difference   | Clipper2    | 103.656529  | 9.647   | 36238448
 * 301      | EvenOdd   | Difference   | Rust        | 440.715792  | 2.269   | 117719484
 * 301      | EvenOdd   | Xor          | V2          | 265.359167  | 3.768   | 94318776
 * 301      | EvenOdd   | Xor          | release/1.x | 560.481942  | 1.784   | 111897144
 * 301      | EvenOdd   | Xor          | Clipper2    | 266.766542  | 3.749   | 73402336
 * 301      | EvenOdd   | Xor          | Rust        | 555.358192  | 1.801   | 159997324
 * 301      | NonZero   | Union        | V2          | 72.203506   | 13.850  | 1879008
 * 301      | NonZero   | Union        | release/1.x | 50.096682   | 19.961  | 11411998
 * 301      | NonZero   | Union        | Clipper2    | 72.272604   | 13.837  | 6609810
 * 1001     | NonZero   | Union        | V2          | 923.958408  | 1.082   | 63130176
 * 1001     | NonZero   | Union        | release/1.x | 852.718975  | 1.173   | 151387784
 * 1001     | NonZero   | Union        | Clipper2    | 2850.236992 | 0.351   | 61347952
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

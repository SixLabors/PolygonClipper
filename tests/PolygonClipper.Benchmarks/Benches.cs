// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using BenchmarkDotNet.Attributes;
using Clipper2Lib;
using GeoJson.Feature;
using SixLabors.PolygonClipper.Tests;

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
 * All comparisons below use union on the same source coordinates.
 * Warmup: 10 iterations. release/1.x NonZero includes its required input preparation.
 * The release/1.x NonZero hole_hole output fails the geometry check and has no comparison timing.
 *
 * Input         | Fill rule | Engine      | Mean (ms) | Op/s       | B/op
 * --------------+-----------+-------------+-----------+------------+---------
 * hole_hole     | EvenOdd   | V2          | 0.005698  | 175501.974 | 2560
 * hole_hole     | EvenOdd   | release/1.x | 0.013171  | 75925.110  | 9440
 * hole_hole     | EvenOdd   | Clipper2    | 0.002463  | 406026.381 | 9992
 * hole_hole     | EvenOdd   | Rust        | 0.013409  | 74578.548  | 13948
 * hole_hole     | NonZero   | V2          | 0.005199  | 192359.257 | 1392
 * hole_hole     | NonZero   | Clipper2    | 0.001927  | 519056.802 | 7512
 * states_source | EvenOdd   | V2          | 0.753519  | 1327.106   | 36936
 * states_source | EvenOdd   | release/1.x | 1.205269  | 829.691    | 454496
 * states_source | EvenOdd   | Clipper2    | 0.296078  | 3377.492   | 348984
 * states_source | EvenOdd   | Rust        | 1.193274  | 838.031    | 806340
 * states_source | NonZero   | V2          | 0.786269  | 1271.830   | 36936
 * states_source | NonZero   | release/1.x | 1.968205  | 508.077    | 584856
 * states_source | NonZero   | Clipper2    | 0.295386  | 3385.403   | 348984
 * asia          | EvenOdd   | V2          | 12.277375 | 81.451     | 1189296
 * asia          | EvenOdd   | release/1.x | 18.382942 | 54.398     | 7527048
 * asia          | EvenOdd   | Clipper2    | 13.100940 | 76.330     | 6612934
 * asia          | EvenOdd   | Rust        | 18.389607 | 54.379     | 11034172
 * asia          | NonZero   | V2          | 12.439713 | 80.388     | 1189296
 * asia          | NonZero   | release/1.x | 30.984123 | 32.275     | 9062752
 * asia          | NonZero   | Clipper2    | 13.507377 | 74.034     | 6613557
 */

/// <summary>
/// Benchmarks the performance of <c>PolygonClipper</c> for polygon union operations using a fixed input dataset.
/// </summary>
[MemoryDiagnoser]
[OperationsPerSecond]
public class Benches
{
    private Polygon subject;
    private Polygon clipping;
    private PathsD clipperSubject = null!;
    private PathsD clipperClipping = null!;
    private const int ClipperPrecision = 6;

    [Params("hole_hole.geojson", "states_source.geojson", "asia.geojson")]
    public string File { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        FeatureCollection collection = TestData.Benchmarks.GetFeatureCollection(this.File);
        (Polygon subject, Polygon clipping) = TestPolygonUtilities.BuildPolygon(collection);
        this.subject = subject;
        this.clipping = clipping;

        this.clipperSubject = BuildClipperPaths(subject);
        this.clipperClipping = BuildClipperPaths(clipping);
    }

    /// <summary>
    /// Measures even-odd union on the raw input contours.
    /// </summary>
    /// <returns>The clipped geometry.</returns>
    [Benchmark]
    public Polygon PolygonClipperEvenOdd() => SixLabors.PolygonClipper.PolygonClipper.Union(this.subject, this.clipping, FillRule.EvenOdd);

    /// <summary>
    /// Measures non-zero clipping without a separate normalization operation.
    /// </summary>
    /// <returns>The clipped geometry.</returns>
    [Benchmark]
    public Polygon PolygonClipperNonZero() => SixLabors.PolygonClipper.PolygonClipper.Union(this.subject, this.clipping, FillRule.NonZero);

    [Benchmark]
    public PolyTreeD Clipper2Union()
    {
        ClipperD clipper = new(ClipperPrecision)
        {
            PreserveCollinear = false,
            ReverseSolution = false
        };

        clipper.AddSubject(this.clipperSubject);
        clipper.AddClip(this.clipperClipping);
        PolyTreeD solution = [];
        clipper.Execute(ClipType.Union, Clipper2Lib.FillRule.EvenOdd, solution);
        return solution;
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

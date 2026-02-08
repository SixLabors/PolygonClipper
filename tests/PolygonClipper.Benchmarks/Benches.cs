// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using BenchmarkDotNet.Attributes;
using Clipper2Lib;
using GeoJson.Feature;
using SixLabors.PolygonClipper.Tests;

namespace SixLabors.PolygonClipper.Benchmarks;

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

    [Benchmark]
    public Polygon PolygonClipper() => SixLabors.PolygonClipper.PolygonClipper.Union(this.subject, this.clipping);

    [Benchmark]
    public PathsD Clipper2Union()
    {
        ClipperD clipper = new(ClipperPrecision)
        {
            PreserveCollinear = false,
            ReverseSolution = false
        };

        clipper.AddSubject(this.clipperSubject);
        clipper.AddClip(this.clipperClipping);
        PathsD solution = [];
        clipper.Execute(ClipType.Union, FillRule.EvenOdd, solution);
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

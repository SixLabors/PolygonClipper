// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

#nullable disable

using BenchmarkDotNet.Attributes;
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

    [Params("hole_hole.geojson", "states_source.geojson", "asia.geojson")]
    public string File { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        FeatureCollection collection = TestData.Benchmarks.GetFeatureCollection(this.File);
        (Polygon subject, Polygon clipping) = TestPolygonUtilities.BuildPolygon(collection);
        this.subject = subject;
        this.clipping = clipping;
    }

    [Benchmark]
    public Polygon PolygonClipper() => global::SixLabors.PolygonClipper.PolygonClipper.Union(this.subject, this.clipping);
}

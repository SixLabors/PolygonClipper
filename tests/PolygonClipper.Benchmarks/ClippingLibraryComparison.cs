// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

#nullable disable

using BenchmarkDotNet.Attributes;
using Clipper2Lib;
using GeoJSON.Text.Feature;
using PolygonClipper.Tests;
using PolygonClipper.Tests.TestCases;

namespace PolygonClipper.Benchmarks;

/// <summary>
/// <para>
/// Benchmarks the performance of <c>PolygonClipper</c> against <c>Clipper2</c>
/// for polygon union operations using a fixed input dataset.
/// </para>
/// <para>
/// Note: Clipper2 produces an incorrect result for the test case used in this benchmark.
/// As such, the Clipper2 timing is included for reference only and should not be considered
/// a valid correctness comparison. This benchmark is intended to evaluate the performance
/// of <c>PolygonClipper</c> in isolation.
/// </para>
/// </summary>
[MemoryDiagnoser]
public class ClippingLibraryComparison
{
    private static readonly FeatureCollection Data = TestData.Generic.GetFeatureCollection("issue71.geojson");
    private Polygon subject;
    private Polygon clipping;

    private PathsD subject2;
    private PathsD clipping2;

    [GlobalSetup]
    public void Setup()
    {
        (Polygon subject, Polygon clipping) = TestPolygonUtilities.BuildPolygon(Data);
        this.subject = subject;
        this.clipping = clipping;

        (PathsD subject2, PathsD clipping2) = TestPolygonUtilities.BuildClipper2Polygon(Data);
        this.subject2 = subject2;
        this.clipping2 = clipping2;
    }

    [Benchmark]
    public Polygon PolygonClipper() => global::PolygonClipper.PolygonClipper.Union(this.subject, this.clipping);

    [Benchmark(Baseline = true)]
    public PathsD Clipper2()
    {
        PathsD solution = [];
        ClipperD clipper2 = new();
        clipper2.AddSubject(this.subject2);
        clipper2.AddClip(this.clipping2);
        clipper2.Execute(ClipType.Union, FillRule.Positive, solution);
        return solution;
    }
}

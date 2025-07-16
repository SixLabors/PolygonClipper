// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using Clipper2Lib;
using GeoJSON.Text.Feature;
using PolygonClipper.Tests.TestCases;
using Xunit;

namespace PolygonClipper.Tests;

public class TestPolygonUtilitiesTests
{
    private static readonly FeatureCollection Data = TestData.Generic.GetFeatureCollection("issue71.geojson");

    private static readonly (Polygon subject, Polygon clipping) Polygons = TestPolygonUtilities.BuildPolygon(Data);

    [Fact]
    public void ConvertToPolygon_ValidGeometry_ReturnsPolygon()
    {
        (Polygon subject, Polygon clipping) = TestPolygonUtilities.BuildPolygon(Data);

        Assert.NotNull(subject);
        Assert.NotNull(clipping);
        Assert.IsType<Polygon>(subject);
        Assert.IsType<Polygon>(clipping);

        Assert.Equal(2, subject.ContourCount);
        Assert.Equal(122, subject[0].VertexCount);
        Assert.Equal(9, subject[1].VertexCount);

        Assert.Equal(1, clipping.ContourCount);
        Assert.Equal(12, clipping[0].VertexCount);
    }

    [Fact]
    public void ConvertToClipper2Polygon_ValidGeometry_ReturnsPathsD()
    {
        (PathsD subject, PathsD clipping) = TestPolygonUtilities.BuildClipper2Polygon(Data);

        Assert.NotNull(subject);
        Assert.NotNull(clipping);
        Assert.IsType<PathsD>(subject);
        Assert.IsType<PathsD>(clipping);

        Assert.Equal(2, subject.Count);
        Assert.Equal(122, subject[0].Count);
        Assert.Equal(9, subject[1].Count);

        Assert.Single(clipping);
        Assert.Equal(12, clipping[0].Count);
    }

    [Fact]
    public void PolygonClipper_Union_ValidPolygons_ReturnsCorrectResult()
    {
        //(Polygon subject, Polygon clipping) = TestPolygonUtilities.BuildPolygon(Data);

        Polygon solution = PolygonClipper.Union(Polygons.subject, Polygons.clipping);
        Assert.NotNull(solution);

        Assert.Equal(2, solution.ContourCount);
        Assert.Equal(122, solution[0].VertexCount);
        Assert.Equal(9, solution[1].VertexCount);
    }

    [Fact]
    public void Clipper2_Union_ValidPolygons_ReturnsCorrectResult()
    {
        (PathsD subject, PathsD clipping) = TestPolygonUtilities.BuildClipper2Polygon(Data);

        PathsD solution = [];
        ClipperD clipper2 = new();
        clipper2.AddSubject(subject);
        clipper2.AddClip(clipping);
        Assert.True(clipper2.Execute(ClipType.Union, FillRule.Positive, solution));

        // Clipper2 does not duplicate the first vertex of the each contour
        // for closed polygons, so we need to adjust the expected counts.
        Assert.Equal(2, solution.Count);
        Assert.Equal(121, solution[0].Count);
        Assert.Equal(8, solution[1].Count);
    }
}

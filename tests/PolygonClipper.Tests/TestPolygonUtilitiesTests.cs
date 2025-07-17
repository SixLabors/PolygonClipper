// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using GeoJSON.Text.Feature;
using PolygonClipper.Tests.TestCases;

namespace PolygonClipper.Tests;

public class TestPolygonUtilitiesTests
{
    private static readonly FeatureCollection Data = TestData.Generic.GetFeatureCollection("issue71.geojson");

    /// <summary>
    /// Static to allow profiling in Rider.
    /// </summary>
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
    public void PolygonClipper_Union_Profile_Test()
    {
        Polygon solution = PolygonClipper.Union(Polygons.subject, Polygons.clipping);
        Assert.NotNull(solution);

        Assert.Equal(2, solution.ContourCount);
        Assert.Equal(122, solution[0].VertexCount);
        Assert.Equal(9, solution[1].VertexCount);
    }
}

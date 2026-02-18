// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using GeoJson.Feature;

namespace SixLabors.PolygonClipper.Tests;

public class TestPolygonUtilitiesTests
{
    private static readonly FeatureCollection Data = TestData.Generic.GetFeatureCollection("issue71.geojson");

    /// <summary>
    /// Static to allow profiling in Rider.
    /// </summary>
    private static readonly (Polygon Subject, Polygon Clipping) Polygons = TestPolygonUtilities.BuildPolygon(Data);

    [Fact]
    public void ConvertToPolygon_ValidGeometry_ReturnsPolygon()
    {
        (Polygon subject, Polygon clipping) = TestPolygonUtilities.BuildPolygon(Data);

        Assert.NotNull(subject);
        Assert.NotNull(clipping);
        Assert.IsType<Polygon>(subject);
        Assert.IsType<Polygon>(clipping);

        Assert.Equal(2, subject.Count);
        Assert.Equal(122, subject[0].Count);
        Assert.Equal(9, subject[1].Count);

        Assert.Equal(1, clipping.Count);
        Assert.Equal(12, clipping[0].Count);
    }

    [Fact]
    public void PolygonClipper_Union_Profile_Test()
    {
        Polygon solution = PolygonClipper.Union(Polygons.Subject, Polygons.Clipping);
        Assert.NotNull(solution);

        Assert.Equal(2, solution.Count);
        Assert.Equal(122, solution[0].Count);
        Assert.Equal(9, solution[1].Count);
    }
}

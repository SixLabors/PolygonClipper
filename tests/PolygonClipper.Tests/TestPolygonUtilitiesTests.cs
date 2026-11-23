// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using Clipper2Lib;
using GeoJson.Feature;

namespace SixLabors.PolygonClipper.Tests;

public class TestPolygonUtilitiesTests
{
    private static readonly FeatureCollection Data = TestData.Generic.GetFeatureCollection("issue71.geojson");

    /// <summary>
    /// Static to allow profiling in Rider.
    /// </summary>
    private static readonly (Polygon Subject, Polygon Clipping) Polygons = TestPolygonUtilities.BuildPolygon(Data);

    /// <summary>
    /// Checks that contour grouping does not hide a change in boundary connections.
    /// </summary>
    /// <param name="changeConnection">Whether to change the connections between the same junctions.</param>
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void BoundaryGraphs_CheckConnectionsAtSharedVertex(bool changeConnection)
    {
        // Both triangles meet at (2, 2). One contour can follow both boundaries.
        PathsD separate =
        [
            [new PointD(0, 0), new PointD(2, 0), new PointD(2, 2)],
            [new PointD(2, 2), new PointD(4, 2), new PointD(4, 4)]
        ];

        PathD joined =
        [
            new PointD(0, 0), new PointD(2, 0), new PointD(2, 2),
            new PointD(4, 2), new PointD(4, 4), new PointD(2, 2)
        ];

        if (changeConnection)
        {
            // Keep the junctions and segment count, but connect different pairs.
            (joined[1], joined[3]) = (joined[3], joined[1]);
        }

        Assert.Equal(!changeConnection, TestPolygonUtilities.HaveEquivalentBoundaryGraphs([joined], separate));
    }

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
        Polygon solution = PolygonClipper.Union(Polygons.Subject, Polygons.Clipping, FillRule.EvenOdd);
        Assert.NotNull(solution);

        Assert.Equal(2, solution.Count);
        Assert.Equal(121, solution[0].Count);
        Assert.Equal(8, solution[1].Count);
    }
}

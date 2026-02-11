// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

namespace SixLabors.PolygonClipper.Tests;

public class PolygonUtilitiesTests
{
    [Fact]
    public void AnalyticalSignedArea()
    {
        // Assert negative area
        Assert.Equal(-1D, PolygonUtilities.SignedArea(new Vertex64(0, 0), new Vertex64(0, 1), new Vertex64(1, 1)));

        // Assert positive area
        Assert.Equal(1D, PolygonUtilities.SignedArea(new Vertex64(0, 1), new Vertex64(0, 0), new Vertex64(1, 0)));

        // Assert collinear points, 0 area
        Assert.Equal(0D, PolygonUtilities.SignedArea(new Vertex64(0, 0), new Vertex64(1, 1), new Vertex64(2, 2)));

        // Assert point on segment
        Assert.Equal(0D, PolygonUtilities.SignedArea(new Vertex64(-1, 0), new Vertex64(2, 3), new Vertex64(0, 1)));

        // Assert point on segment (order reversed)
        Assert.Equal(0D, PolygonUtilities.SignedArea(new Vertex64(2, 3), new Vertex64(-1, 0), new Vertex64(0, 1)));
    }
}

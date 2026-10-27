// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

namespace SixLabors.PolygonClipper.Tests;

public class PolygonUtilitiesTests
{
    [Fact]
    public void FindIntersection_TinyCrossingSegments_ReturnsUniqueCrossing()
    {
        double size = Math.ScaleB(1D, -300);
        Segment first = new(new Vertex(0, 0), new Vertex(size, size));
        Segment second = new(new Vertex(0, size * 0.75D), new Vertex(size, size * 0.75D));

        int count = PolygonUtilities.FindIntersection(first, second, out Vertex intersection, out _);

        Assert.Equal(1, count);
        Assert.Equal(new Vertex(size * 0.75D, size * 0.75D), intersection);

        count = PolygonUtilities.FindIntersection(first.Source, first.Target, second.Source, second.Target, out intersection, out _);

        Assert.Equal(1, count);
        Assert.Equal(new Vertex(size * 0.75D, size * 0.75D), intersection);
    }

    [Fact]
    public void AnalyticalSignedArea()
    {
        // Assert negative area
        Assert.Equal(-1D, PolygonUtilities.SignedArea(new Vertex(0, 0), new Vertex(0, 1), new Vertex(1, 1)));

        // Assert positive area
        Assert.Equal(1D, PolygonUtilities.SignedArea(new Vertex(0, 1), new Vertex(0, 0), new Vertex(1, 0)));

        // Assert collinear points, 0 area
        Assert.Equal(0D, PolygonUtilities.SignedArea(new Vertex(0, 0), new Vertex(1, 1), new Vertex(2, 2)));

        // Assert point on segment
        Assert.Equal(0D, PolygonUtilities.SignedArea(new Vertex(-1, 0), new Vertex(2, 3), new Vertex(0, 1)));

        // Assert point on segment (order reversed)
        Assert.Equal(0D, PolygonUtilities.SignedArea(new Vertex(2, 3), new Vertex(-1, 0), new Vertex(0, 1)));
    }
}

// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

namespace SixLabors.PolygonClipper.Tests;

public class PolygonUtilitiesTests
{
    /// <summary>
    /// Checks a determinant whose rounded products are equal but whose exact difference is one.
    /// </summary>
    [Fact]
    public void Cross_NearlyParallelIntegerVectors_PreservesUnitDeterminant()
    {
        const double n = 134217728D;
        Vertex first = new(n, n + 1);
        Vertex second = new(n - 1, n);

        // n*n - (n+1)*(n-1) is exactly one. Reversing the operands changes its sign.
        Assert.Equal(1D, Vertex.Cross(first, second));
        Assert.Equal(-1D, Vertex.Cross(second, first));
        Assert.Equal(0D, Vertex.Cross(first, first));
    }

    /// <summary>
    /// Checks two nearly parallel segments that both pass through the origin.
    /// </summary>
    [Fact]
    public void FindIntersection_CancelledProducts_ReturnsOrigin()
    {
        const double n = 134217728D;
        Segment first = new(new Vertex(-n, -n - 1), new Vertex(n, n + 1));
        Segment second = new(new Vertex(1 - n, -n), new Vertex(n - 1, n));

        int count = PolygonUtilities.FindIntersection(first, second, out Vertex intersection, out _);

        Assert.Equal(1, count);
        Assert.Equal(new Vertex(0, 0), intersection);
        count = PolygonUtilities.FindIntersection(first.Source, first.Target, second.Source, second.Target, out intersection, out _);
        Assert.Equal(1, count);
        Assert.Equal(new Vertex(0, 0), intersection);
    }

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

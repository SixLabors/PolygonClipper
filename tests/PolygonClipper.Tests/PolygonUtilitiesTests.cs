// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

namespace SixLabors.PolygonClipper.Tests;

public class PolygonUtilitiesTests
{
    /// <summary>
    /// Checks a point against a diagonal segment in both operand orders.
    /// </summary>
    /// <param name="x">The point's x coordinate.</param>
    /// <param name="y">The point's y coordinate.</param>
    /// <param name="expected">Whether the point lies on the segment.</param>
    [Theory]
    [InlineData(1D, 1D, 1)]
    [InlineData(1D, 1.5D, 0)]
    [InlineData(3D, 3D, 0)]
    public void FindIntersection_PointAndDiagonal_IsSymmetric(double x, double y, int expected)
    {
        Vertex point = new(x, y);
        Segment singleton = new(point, point);
        Segment diagonal = new(new(0, 0), new(2, 2));
        foreach ((Segment first, Segment second) in new[] { (singleton, diagonal), (diagonal, singleton) })
        {
            Assert.Equal(expected, PolygonUtilities.FindIntersection(first, second, out Vertex intersection, out _));
            if (expected == 1)
            {
                Assert.Equal(point, intersection);
            }
        }
    }

    /// <summary>
    /// Checks axis-aligned crossings whose coordinates are supplied by the two source segments.
    /// </summary>
    /// <param name="scale">The coordinate scale.</param>
    [Theory]
    [InlineData(1D)]
    [InlineData(1E-90)]
    [InlineData(1E90)]
    public void FindIntersection_PerpendicularAxes_ReturnsExactSourceCoordinates(double scale)
    {
        Segment horizontal = new(new(0, scale * 0.75D), new(scale, scale * 0.75D));
        Segment vertical = new(new(scale * 0.25D, 0), new(scale * 0.25D, scale));
        foreach (Segment first in new[] { horizontal, horizontal.Reverse() })
        {
            foreach (Segment second in new[] { vertical, vertical.Reverse() })
            {
                foreach ((Segment a, Segment b) in new[] { (first, second), (second, first) })
                {
                    Assert.Equal(1, PolygonUtilities.FindIntersection(a, b, out Vertex point, out _));
                    Assert.Equal(new Vertex(scale * 0.25D, scale * 0.75D), point);
                }
            }
        }
    }

    /// <summary>
    /// Checks overlap when rounded directions suggest a crossing or an endpoint contact.
    /// </summary>
    /// <param name="largeCoordinates">Whether a short overlap lies near a long segment's endpoint.</param>
    /// <param name="reverseFirst">Whether to reverse the first segment.</param>
    /// <param name="reverseSecond">Whether to reverse the second segment.</param>
    [Theory]
    [InlineData(false, false, false)]
    [InlineData(false, true, false)]
    [InlineData(false, false, true)]
    [InlineData(false, true, true)]
    [InlineData(true, false, false)]
    [InlineData(true, true, false)]
    [InlineData(true, false, true)]
    [InlineData(true, true, true)]
    public void FindIntersection_CollinearRoundedDirections_ReturnsBothOverlapEndpoints(
        bool largeCoordinates, bool reverseFirst, bool reverseSecond)
    {
        const double u = 1.0000000000000002;
        Segment overlap = largeCoordinates
            ? new(new Vertex(9999999999999998D, 9999999999999998D), new Vertex(1E16, 1E16))
            : new(new Vertex(-u, -1), new Vertex(2 * u, 2));
        Segment enclosing = largeCoordinates
            ? new(new Vertex(-3E16, -3E16), new Vertex(1E16, 1E16))
            : new(new Vertex(-2 * u, -2), new Vertex(2 * u, 2));
        Segment first = reverseFirst ? overlap.Reverse() : overlap;
        Segment second = reverseSecond ? enclosing.Reverse() : enclosing;

        // Both cases have a non-empty interval in common. Direction and operand
        // order cannot turn that interval into a single-point intersection.
        foreach ((Segment a, Segment b) in new[] { (first, second), (second, first) })
        {
            int count = PolygonUtilities.FindIntersection(a, b, out Vertex start, out Vertex end);

            Assert.Equal(2, count);
            Assert.NotEqual(start, end);
            Assert.True(start == overlap.Source || start == overlap.Target);
            Assert.True(end == overlap.Source || end == overlap.Target);

            int vertexCount = PolygonUtilities.FindIntersection(a.Source, a.Target, b.Source, b.Target, out Vertex vertexStart, out Vertex vertexEnd);
            Assert.Equal(2, vertexCount);
            Assert.Equal(start, vertexStart);
            Assert.Equal(end, vertexEnd);
        }
    }

    /// <summary>
    /// Checks source incidence when rounded coordinate differences obscure the orientation.
    /// </summary>
    [Fact]
    public void SignedArea_NearbySourceCorner_RetainsPositiveSign()
    {
        Vertex first = new(6.559375, 5.3171875);
        Vertex second = new(-0.565625, 11.0828125);
        Vertex corner = new(3.846028904999335, 7.51285572161238);

        // Exact binary-rational arithmetic gives a positive determinant of about 3.0769E-15.
        Assert.True(PolygonUtilities.SignedArea(first, second, corner) > 0);
        Assert.True(PolygonUtilities.SignedArea(second, first, corner) < 0);
    }

    /// <summary>
    /// Checks an interior crossing whose ordinary quotient rounds to an endpoint.
    /// </summary>
    [Fact]
    public void FindIntersection_RoundedEndpointParameter_RetainsInteriorCrossing()
    {
        Segment first = new(new Vertex(3.846028904999335, 7.51285572161238), new Vertex(13.846028904999335, 7.51285572161238));
        Segment second = new(new Vertex(6.559375, 5.3171875), new Vertex(-0.565625, 11.0828125));

        int count = PolygonUtilities.FindIntersection(first, second, out Vertex intersection, out _);

        // The exact intersection rounds one X step above the source corner.
        Assert.Equal(1, count);
        Assert.Equal(new Vertex(3.8460289049993355, 7.51285572161238), intersection);
    }

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

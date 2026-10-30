// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

namespace SixLabors.PolygonClipper.Tests;

/// <summary>
/// Checks open clipping against exact segment and rectangle constructions.
/// </summary>
public class OpenPathClippingTests
{
    /// <summary>
    /// Checks a crossing point and the line pieces selected by each Boolean operation.
    /// </summary>
    /// <param name="operation">The requested operation.</param>
    [Theory]
    [InlineData(BooleanOperation.Intersection)]
    [InlineData(BooleanOperation.Union)]
    [InlineData(BooleanOperation.Difference)]
    [InlineData(BooleanOperation.Xor)]
    public void CrossingLines_ReturnPointOrCutContours(BooleanOperation operation)
    {
        Vertex left = new(-2, 0);
        Vertex right = new(2, 0);
        Vertex bottom = new(0, -2);
        Vertex top = new(0, 2);
        Vertex point = new(0, 0);
        Polygon subject = [Open(left, right)];
        Polygon clip = [Open(bottom, top)];

        Polygon result = Execute(subject, clip, operation);
        if (operation == BooleanOperation.Intersection)
        {
            Contour intersection = Assert.Single(result);
            Assert.False(intersection.IsClosed);
            Assert.Equal(point, Assert.Single(intersection));
        }
        else if (operation == BooleanOperation.Difference)
        {
            AssertLines(result, (left, point), (point, right));
        }
        else
        {
            AssertLines(result, (left, point), (point, right), (bottom, point), (point, top));
        }

        Assert.Equal<Vertex>([left, right], subject[0]);
        Assert.Equal<Vertex>([bottom, top], clip[0]);
        Assert.False(subject[0].IsClosed);
        Assert.False(clip[0].IsClosed);
    }

    /// <summary>
    /// Checks overlapping line membership without using signed area or input direction.
    /// </summary>
    /// <param name="operation">The requested operation.</param>
    /// <param name="reverse">Whether the second input runs in reverse.</param>
    [Theory]
    [InlineData(BooleanOperation.Intersection, false)]
    [InlineData(BooleanOperation.Intersection, true)]
    [InlineData(BooleanOperation.Union, false)]
    [InlineData(BooleanOperation.Union, true)]
    [InlineData(BooleanOperation.Difference, false)]
    [InlineData(BooleanOperation.Difference, true)]
    [InlineData(BooleanOperation.Xor, false)]
    [InlineData(BooleanOperation.Xor, true)]
    public void CollinearOverlap_UsesLineMembership(BooleanOperation operation, bool reverse)
    {
        Vertex a = new(0, 0);
        Vertex b = new(5, 0);
        Vertex c = new(10, 0);
        Vertex d = new(15, 0);
        Contour second = reverse ? Open(d, b) : Open(b, d);

        Polygon result = Execute([Open(a, c)], [second], operation);
        switch (operation)
        {
            case BooleanOperation.Intersection:
                AssertLines(result, (b, c));
                break;
            case BooleanOperation.Difference:
                AssertLines(result, (a, b));
                break;
            case BooleanOperation.Xor:
                AssertLines(result, (a, b), (c, d));
                break;
            default:
                AssertLines(result, (a, b), (b, c), (c, d));
                break;
        }
    }

    /// <summary>
    /// Checks that open membership does not depend on an area fill rule.
    /// </summary>
    /// <param name="rule">The selected area fill rule.</param>
    [Theory]
    [InlineData(FillRule.EvenOdd)]
    [InlineData(FillRule.NonZero)]
    [InlineData(FillRule.Positive)]
    [InlineData(FillRule.Negative)]
    public void OpenInputs_DoNotAcquireAreaWinding(FillRule rule)
    {
        Polygon result = PolygonClipper.Intersection(
            [Open(new(-1, 0), new(1, 0))], [Open(new(0, -1), new(0, 1))], rule);

        Assert.Equal(new Vertex(0, 0), Assert.Single(Assert.Single(result)));
    }

    /// <summary>
    /// Checks rectangle entry, exit, and hole boundaries against analytic line intervals.
    /// </summary>
    /// <param name="rule">The area fill rule.</param>
    [Theory]
    [InlineData(FillRule.EvenOdd)]
    [InlineData(FillRule.NonZero)]
    public void LineAgainstDonut_RetainsOnlyFilledIntervals(FillRule rule)
    {
        Contour hole = Rectangle(3, 3, 7, 7);
        hole.Reverse();
        Polygon result = PolygonClipper.Intersection(
            [Open(new(-1, 5), new(11, 5))], [Rectangle(0, 0, 10, 10), hole], FillRule.NonZero, rule);

        AssertLines(result, (new(0, 5), new(3, 5)), (new(7, 5), new(10, 5)));
    }

    /// <summary>
    /// Checks coincident line and area boundaries with different terminal endpoints.
    /// </summary>
    /// <param name="reverse">Whether the line runs backwards.</param>
    /// <param name="swap">Whether the area is the subject.</param>
    [Theory]
    [InlineData(false, false)]
    [InlineData(true, false)]
    [InlineData(false, true)]
    [InlineData(true, true)]
    public void LineOnAreaBoundary_RetainsTheSharedSpan(bool reverse, bool swap)
    {
        Contour line = reverse ? Open(new(15, 0), new(-5, 0)) : Open(new(-5, 0), new(15, 0));
        Polygon first = [line];
        Polygon second = [Rectangle(0, 0, 10, 10)];

        Polygon result = PolygonClipper.Intersection(swap ? second : first, swap ? first : second, FillRule.NonZero);

        AssertLines(result, (new(0, 0), new(10, 0)));
    }

    /// <summary>
    /// Checks that a line touching a filled corner produces one point without a manufactured edge.
    /// </summary>
    [Fact]
    public void TangentLine_ReturnsOneCornerPoint()
    {
        Polygon result = PolygonClipper.Intersection(
            [Open(new(-2, 2), new(2, -2))], [Rectangle(0, 0, 10, 10)], FillRule.NonZero);

        Contour point = Assert.Single(result);
        Assert.False(point.IsClosed);
        Assert.Equal(new Vertex(0, 0), Assert.Single(point));
    }

    /// <summary>
    /// Checks that a point result can participate in subsequent clipping and splitting.
    /// </summary>
    [Fact]
    public void PointResults_CanBeReusedAsOperands()
    {
        Polygon horizontal = [Open(new(-2, 0), new(2, 0))];
        Polygon point = PolygonClipper.Intersection(horizontal, [Open(new(0, -2), new(0, 2))], FillRule.NonZero);

        Assert.Equal(new Vertex(0, 0), Assert.Single(Assert.Single(PolygonClipper.Intersection(point, horizontal, FillRule.NonZero))));
        AssertLines(
            PolygonClipper.Difference(horizontal, point, FillRule.NonZero),
            (new(-2, 0), new(0, 0)),
            (new(0, 0), new(2, 0)));
        Assert.Empty(PolygonClipper.Intersection(point, [Open(new(3, 0), new(4, 0))], FillRule.NonZero));
    }

    /// <summary>
    /// Checks that line output joins ordinary source vertices while remaining open.
    /// </summary>
    [Fact]
    public void PolylineInsideArea_PreservesItsBend()
    {
        Polygon result = PolygonClipper.Intersection(
            [Open(new(2, 2), new(4, 6), new(8, 2))], [Rectangle(0, 0, 10, 10)], FillRule.NonZero);

        Contour line = Assert.Single(result);
        Assert.False(line.IsClosed);
        Assert.Equal(3, line.Count);
        Assert.Contains(new Vertex(2, 2), line);
        Assert.Contains(new Vertex(4, 6), line);
        Assert.Contains(new Vertex(8, 2), line);
    }

    /// <summary>
    /// Checks that retained areas suppress covered linework without discarding its outside pieces.
    /// </summary>
    [Fact]
    public void MixedUnion_PreservesAreaAndOutsideLines()
    {
        Polygon result = PolygonClipper.Union(
            [Rectangle(0, 0, 10, 10), Open(new(-5, 5), new(15, 5))], [], FillRule.NonZero);

        Contour area = Assert.Single(result, contour => contour.IsClosed);
        Assert.Equal(100D, PolygonUtilities.Area(area.AsVertexSpan()));
        Assert.Equal(3, result.Count);
        Polygon open = [];
        foreach (Contour contour in result.Where(contour => !contour.IsClosed))
        {
            Assert.Null(contour.ParentIndex);
            Assert.Equal(0, contour.HoleCount);
            open.Add(contour);
        }

        AssertLines(open, (new(-5, 5), new(0, 5)), (new(10, 5), new(15, 5)));
    }

    /// <summary>
    /// Checks that an explicit line cycle remains line geometry rather than acquiring a filled interior.
    /// </summary>
    [Fact]
    public void OpenCycle_DoesNotBecomeAFilledBoundary()
    {
        Contour source = Open(new(0, 0), new(2, 0), new(2, 2), new(0, 2), new(0, 0));
        Contour line = Assert.Single(PolygonClipper.Normalize([source], FillRule.NonZero));

        Assert.False(line.IsClosed);
        Assert.Equal(5, line.Count);
        Assert.Equal(line[0], line[^1]);
    }

    /// <summary>
    /// Checks tiny and large crossings without a tolerance or integer coordinate conversion.
    /// </summary>
    /// <param name="scale">The segment length.</param>
    [Theory]
    [InlineData(1E-90)]
    [InlineData(1E90)]
    public void ScaledLines_PreserveCrossingPoint(double scale)
    {
        Polygon result = PolygonClipper.Intersection(
            [Open(new(0, 0), new(scale, scale))], [Open(new(0, scale), new(scale, 0))], FillRule.NonZero);

        Assert.Equal(new Vertex(scale * 0.5D, scale * 0.5D), Assert.Single(Assert.Single(result)));
    }

    /// <summary>
    /// Checks the terminal vertex that is not visited as an open segment's start.
    /// </summary>
    /// <param name="coordinate">The invalid terminal coordinate.</param>
    [Theory]
    [InlineData(double.NaN)]
    [InlineData(double.PositiveInfinity)]
    [InlineData(double.NegativeInfinity)]
    public void NonFiniteTerminalVertex_IsRejected(double coordinate)
    {
        Assert.Throws<ArgumentException>(() => PolygonClipper.Normalize([Open(new(0, 0), new(coordinate, 1))], FillRule.NonZero));
        AssertLines(PolygonClipper.Normalize([Open(new(0, 0), new(1, 0))], FillRule.NonZero), (new(0, 0), new(1, 0)));
    }

    /// <summary>
    /// Checks every crossing in a line network and all pieces between those crossings.
    /// </summary>
    /// <param name="operation">The requested operation.</param>
    [Theory]
    [InlineData(BooleanOperation.Intersection)]
    [InlineData(BooleanOperation.Union)]
    [InlineData(BooleanOperation.Difference)]
    [InlineData(BooleanOperation.Xor)]
    public void GridLines_AllCrossingsAreRetained(BooleanOperation operation)
    {
        const int size = 10;
        Polygon horizontal = [];
        Polygon vertical = [];
        for (int i = 0; i < size; i++)
        {
            horizontal.Add(Open(new(-1, i), new(size, i)));
            vertical.Add(Open(new(i, -1), new(i, size)));
        }

        Polygon result = Execute(horizontal, vertical, operation);
        if (operation == BooleanOperation.Intersection)
        {
            Assert.Equal(size * size, result.Count);
            for (int y = 0; y < size; y++)
            {
                for (int x = 0; x < size; x++)
                {
                    Vertex expected = new(x, y);
                    Assert.Contains(result, contour => !contour.IsClosed && contour.Count == 1 && contour[0] == expected);
                }
            }
        }
        else
        {
            int expected = size * (size + 1) * (operation == BooleanOperation.Difference ? 1 : 2);
            Assert.Equal(expected, result.Count);
            foreach (Contour contour in result)
            {
                Assert.False(contour.IsClosed);
                Assert.Equal(2, contour.Count);
                Vertex difference = contour[1] - contour[0];
                Assert.Equal(1D, Math.Abs(difference.X) + Math.Abs(difference.Y));
            }
        }
    }

    /// <summary>
    /// Checks cancelled area winding on a line boundary without cancelling the open source.
    /// </summary>
    [Fact]
    public void OpposingAreaContours_DoNotCancelCoincidentLinework()
    {
        Contour first = Rectangle(0, 0, 10, 10);
        Contour second = first.DeepClone();
        second.Reverse();

        Polygon result = PolygonClipper.Normalize([first, second, Open(new(-5, 0), new(15, 0))], FillRule.NonZero);

        Assert.All(result, contour => Assert.False(contour.IsClosed));
        AssertLines(result, (new(-5, 0), new(0, 0)), (new(0, 0), new(10, 0)), (new(10, 0), new(15, 0)));
    }

    /// <summary>
    /// Checks that an open clipping call does not leak line state into later closed operations.
    /// </summary>
    [Fact]
    public void PooledReuse_SeparatesOpenAndClosedState()
    {
        for (int i = 0; i < 3; i++)
        {
            Polygon crossing = PolygonClipper.Intersection(
                [Open(new(-2, 0), new(2, 0))], [Open(new(0, -2), new(0, 2))], FillRule.Negative);
            Assert.Equal(new Vertex(0, 0), Assert.Single(Assert.Single(crossing)));

            Polygon area = PolygonClipper.Intersection([Rectangle(0, 0, 10, 10)], [Rectangle(5, 5, 15, 15)], FillRule.NonZero);
            Contour contour = Assert.Single(area);
            Assert.True(contour.IsClosed);
            Assert.Equal(25D, PolygonUtilities.Area(contour.AsVertexSpan()));
        }
    }

    /// <summary>
    /// Checks constructed integer crossings with known coordinates and four exact ray endpoints.
    /// </summary>
    /// <param name="operation">The requested operation.</param>
    [Theory]
    [InlineData(BooleanOperation.Intersection)]
    [InlineData(BooleanOperation.Union)]
    [InlineData(BooleanOperation.Difference)]
    [InlineData(BooleanOperation.Xor)]
    public void ConstructedCrossings_MatchKnownIntegerGeometry(BooleanOperation operation)
    {
        for (int i = 1; i <= 50; i++)
        {
            Vertex point = new(((i * 73) % 101) - 50, ((i * 31) % 103) - 51);
            Vertex first = new((i % 7) + 1, (i % 5) + 1);
            Vertex second = new(-((i % 11) + 1), (i % 3) + 1);
            Vertex a = point - first;
            Vertex b = point + first;
            Vertex c = point - second;
            Vertex d = point + second;

            Polygon result = Execute([Open(a, b)], [Open(c, d)], operation);
            if (operation == BooleanOperation.Intersection)
            {
                Assert.Equal(point, Assert.Single(Assert.Single(result)));
            }
            else if (operation == BooleanOperation.Difference)
            {
                AssertLines(result, (a, point), (point, b));
            }
            else
            {
                AssertLines(result, (a, point), (point, b), (c, point), (point, d));
            }
        }
    }

    /// <summary>
    /// Checks a self-crossing open path without filling either of its enclosed lobes.
    /// </summary>
    [Fact]
    public void SelfCrossingPolyline_RetainsNodedLinework()
    {
        Polygon result = PolygonClipper.Normalize(
            [Open(new(-2, -2), new(2, 2), new(-2, 2), new(2, -2))], FillRule.NonZero);

        Assert.Equal(3, result.Count);
        Assert.All(result, contour => Assert.False(contour.IsClosed));
        Contour cycle = Assert.Single(result, contour => contour.Count == 4);
        Assert.Equal(new Vertex(0, 0), cycle[0]);
        Assert.Equal(cycle[0], cycle[^1]);
        Assert.Contains(new Vertex(2, 2), cycle);
        Assert.Contains(new Vertex(-2, 2), cycle);
    }

    /// <summary>
    /// Builds an explicitly open input without adding a closing vertex.
    /// </summary>
    /// <param name="vertices">The source vertex sequence.</param>
    /// <returns>The open contour.</returns>
    private static Contour Open(params Vertex[] vertices)
    {
        Contour contour = new(vertices.Length) { IsClosed = false };
        foreach (Vertex vertex in vertices)
        {
            contour.Add(vertex);
        }

        return contour;
    }

    /// <summary>
    /// Builds a counterclockwise closed rectangle with implicit closure.
    /// </summary>
    /// <param name="left">The left coordinate.</param>
    /// <param name="bottom">The bottom coordinate.</param>
    /// <param name="right">The right coordinate.</param>
    /// <param name="top">The top coordinate.</param>
    /// <returns>The closed rectangle.</returns>
    private static Contour Rectangle(double left, double bottom, double right, double top)
        => [new(left, bottom), new(right, bottom), new(right, top), new(left, top)];

    /// <summary>
    /// Executes the requested existing public operation.
    /// </summary>
    /// <param name="subject">The subject contours.</param>
    /// <param name="clip">The clip contours.</param>
    /// <param name="operation">The requested operation.</param>
    /// <returns>The clipped contours.</returns>
    private static Polygon Execute(Polygon subject, Polygon clip, BooleanOperation operation)
        => operation switch
        {
            BooleanOperation.Intersection => PolygonClipper.Intersection(subject, clip, FillRule.NonZero),
            BooleanOperation.Union => PolygonClipper.Union(subject, clip, FillRule.NonZero),
            BooleanOperation.Difference => PolygonClipper.Difference(subject, clip, FillRule.NonZero),
            _ => PolygonClipper.Xor(subject, clip, FillRule.NonZero)
        };

    /// <summary>
    /// Checks exact undirected line pieces without depending on contour ordering or traversal direction.
    /// </summary>
    /// <param name="result">The returned contours.</param>
    /// <param name="expected">The analytically expected line pieces.</param>
    private static void AssertLines(Polygon result, params (Vertex First, Vertex Last)[] expected)
    {
        Assert.Equal(expected.Length, result.Count);
        foreach ((Vertex first, Vertex last) in expected)
        {
            Assert.Contains(result, contour => !contour.IsClosed && contour.Count == 2 &&
                ((contour[0] == first && contour[1] == last) || (contour[0] == last && contour[1] == first)));
        }
    }
}

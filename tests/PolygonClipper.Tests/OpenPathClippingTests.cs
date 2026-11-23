// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

namespace SixLabors.PolygonClipper.Tests;

/// <summary>
/// Checks open clipping against exact points, line pieces and area boundaries.
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

        Polygon result = Execute(subject, clip, operation, nameof(this.CrossingLines_ReturnPointOrCutContours), $"Operation-{operation}", null);
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

        string details = $"Operation-{operation}_Reverse-{reverse}";
        Polygon result = Execute([Open(a, c)], [second], operation, nameof(this.CollinearOverlap_UsesLineMembership), details, null);
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

        Polygon result = Execute(horizontal, vertical, operation, nameof(this.GridLines_AllCrossingsAreRetained), $"Operation-{operation}", null);
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

            // Keep the current generated sample. A failure leaves its geometry in this same file.
            Polygon result = Execute(
                [Open(a, b)],
                [Open(c, d)],
                operation,
                nameof(this.ConstructedCrossings_MatchKnownIntegerGeometry),
                $"Operation-{operation}",
                null);
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
    /// Checks four crossings between sampled waves and the bends between their cut endpoints.
    /// </summary>
    /// <param name="operation">The requested operation.</param>
    /// <param name="reverse">Whether both input paths run backwards.</param>
    [Theory]
    [InlineData(BooleanOperation.Intersection, false)]
    [InlineData(BooleanOperation.Intersection, true)]
    [InlineData(BooleanOperation.Union, false)]
    [InlineData(BooleanOperation.Union, true)]
    [InlineData(BooleanOperation.Difference, false)]
    [InlineData(BooleanOperation.Difference, true)]
    [InlineData(BooleanOperation.Xor, false)]
    [InlineData(BooleanOperation.Xor, true)]
    public void SampledWaves_PreserveAllCrossingsAndBends(BooleanOperation operation, bool reverse)
    {
        Polygon subject =
        [Open([
            new(-12, 4), new(-10, 5), new(-8, 4), new(-6, -4), new(-4, -5), new(-2, -4),
            new(0, 4), new(2, 5), new(4, 4), new(6, -4), new(8, -5), new(10, -4), new(12, 4)
        ])];
        Polygon clip =
        [Open([
            new(-12, -4), new(-10, -5), new(-8, -4), new(-6, 4), new(-4, 5), new(-2, 4),
            new(0, -4), new(2, -5), new(4, -4), new(6, 4), new(8, 5), new(10, 4), new(12, -4)
        ])];

        // The paths are reflections in y = 0. Their crossing segments meet at their midpoints.
        // The other segments occupy separate X intervals or remain on opposite sides of y = 0.
        Polygon expected = operation == BooleanOperation.Intersection
            ? [Open(new Vertex(-7, 0)), Open(new Vertex(-1, 0)), Open(new Vertex(5, 0)), Open(new Vertex(11, 0))]
            :
            [
                Open(new(-12, 4), new(-10, 5), new(-8, 4), new(-7, 0)),
                Open(new(-7, 0), new(-6, -4), new(-4, -5), new(-2, -4), new(-1, 0)),
                Open(new(-1, 0), new(0, 4), new(2, 5), new(4, 4), new(5, 0)),
                Open(new(5, 0), new(6, -4), new(8, -5), new(10, -4), new(11, 0)),
                Open(new(11, 0), new(12, 4))
            ];

        if (operation is BooleanOperation.Union or BooleanOperation.Xor)
        {
            // Reflect the known subject pieces to obtain the clip pieces.
            int subjectCount = expected.Count;
            for (int i = 0; i < subjectCount; i++)
            {
                Contour reflected = new() { IsClosed = false };
                foreach (Vertex vertex in expected[i])
                {
                    reflected.Add(new(vertex.X, -vertex.Y));
                }

                expected.Add(reflected);
            }
        }

        if (reverse)
        {
            subject[0].Reverse();
            clip[0].Reverse();
        }

        string details = $"Operation-{operation}_Reverse-{reverse}";
        Polygon actual = Execute(subject, clip, operation, nameof(this.SampledWaves_PreserveAllCrossingsAndBends), details, expected);
        AssertOpenPaths(actual, expected);
    }

    /// <summary>
    /// Checks densely sampled curves whose four crossings are known polynomial roots.
    /// </summary>
    /// <param name="operation">The requested operation.</param>
    /// <param name="reverse">Whether both input paths run backwards.</param>
    [Theory]
    [InlineData(BooleanOperation.Intersection, false)]
    [InlineData(BooleanOperation.Intersection, true)]
    [InlineData(BooleanOperation.Union, false)]
    [InlineData(BooleanOperation.Union, true)]
    [InlineData(BooleanOperation.Difference, false)]
    [InlineData(BooleanOperation.Difference, true)]
    [InlineData(BooleanOperation.Xor, false)]
    [InlineData(BooleanOperation.Xor, true)]
    public void SampledQuartic_PreservesFourVertexCrossings(BooleanOperation operation, bool reverse)
    {
        Contour first = new() { IsClosed = false };
        Contour second = new() { IsClosed = false };
        for (int sample = -16; sample <= 16; sample++)
        {
            Vertex point = SampleQuartic(sample);
            first.Add(point);
            second.Add(new(point.X, -point.Y));
        }

        Polygon subject = [first];
        Polygon clip = [second];
        Polygon expected;
        if (operation == BooleanOperation.Intersection)
        {
            expected = [Open(new Vertex(-3, 0)), Open(new Vertex(-1, 0)), Open(new Vertex(1, 0)), Open(new Vertex(3, 0))];
        }
        else
        {
            expected = [];

            // For y = (x^2 - 9)(x^2 - 1) / 16 and its reflection, the only crossings
            // are x = -3, -1, 1 and 3. These fixed intervals define the expected pieces.
            int[] endpoints = [-16, -12, -4, 4, 12, 16];
            for (int i = 0; i < endpoints.Length - 1; i++)
            {
                Contour piece = new() { IsClosed = false };
                for (int sample = endpoints[i]; sample <= endpoints[i + 1]; sample++)
                {
                    Vertex point = SampleQuartic(sample);
                    piece.Add(point);
                }

                expected.Add(piece);
                if (operation is BooleanOperation.Union or BooleanOperation.Xor)
                {
                    Contour reflection = new() { IsClosed = false };
                    foreach (Vertex point in piece)
                    {
                        reflection.Add(new(point.X, -point.Y));
                    }

                    expected.Add(reflection);
                }
            }
        }

        if (reverse)
        {
            first.Reverse();
            second.Reverse();
        }

        string details = $"Operation-{operation}_Reverse-{reverse}";
        Polygon actual = Execute(subject, clip, operation, nameof(this.SampledQuartic_PreservesFourVertexCrossings), details, expected);
        AssertOpenPaths(actual, expected);
    }

    /// <summary>
    /// Checks repeated area and hole crossings, retained bends and isolated corner contacts.
    /// </summary>
    /// <param name="operation">The requested operation.</param>
    /// <param name="reverse">Whether the source path runs backwards.</param>
    [Theory]
    [InlineData(BooleanOperation.Intersection, false)]
    [InlineData(BooleanOperation.Intersection, true)]
    [InlineData(BooleanOperation.Union, false)]
    [InlineData(BooleanOperation.Union, true)]
    [InlineData(BooleanOperation.Difference, false)]
    [InlineData(BooleanOperation.Difference, true)]
    [InlineData(BooleanOperation.Xor, false)]
    [InlineData(BooleanOperation.Xor, true)]
    public void CurvedPath_ClipsAgainstTwoHolesAndCornerContacts(BooleanOperation operation, bool reverse)
    {
        Polygon subject =
        [Open([
            new(-2, 2), new(1, 2), new(2, 3), new(4, 3), new(6, 2), new(8, 2), new(10, 3),
            new(13, 3), new(14, 4), new(13, 6), new(10, 6), new(8, 7), new(6, 7), new(4, 6),
            new(2, 6), new(-2, 8), new(0, 10), new(4, 11), new(8, 11), new(12, 10), new(14, 8)
        ])];
        Contour firstHole = Rectangle(3, 1, 5, 9);
        Contour secondHole = Rectangle(7, 1, 9, 9);
        firstHole.Reverse();
        secondHole.Reverse();
        Polygon clip = [Rectangle(0, 0, 12, 10), firstHole, secondHole];
        clip[0].AddHoleIndex(1);
        clip[0].AddHoleIndex(2);
        firstHole.ParentIndex = secondHole.ParentIndex = 0;
        firstHole.Depth = secondHole.Depth = 1;

        // Vertical boundaries cut these segments at integer or half-integer coordinates.
        // The upper arc touches (0, 10) and (12, 10), but remains outside between those points.
        Polygon expected = operation == BooleanOperation.Intersection
            ?
            [
                Open(new(0, 2), new(1, 2), new(2, 3), new(3, 3)),
                Open(new(5, 2.5), new(6, 2), new(7, 2)),
                Open(new(9, 2.5), new(10, 3), new(12, 3)),
                Open(new(12, 6), new(10, 6), new(9, 6.5)),
                Open(new(7, 7), new(6, 7), new(5, 6.5)),
                Open(new(3, 6), new(2, 6), new(0, 7)),
                Open(new Vertex(0, 10)),
                Open(new Vertex(12, 10))
            ]
            :
            [
                Open(new(-2, 2), new(0, 2)),
                Open(new(3, 3), new(4, 3), new(5, 2.5)),
                Open(new(7, 2), new(8, 2), new(9, 2.5)),
                Open(new(12, 3), new(13, 3), new(14, 4), new(13, 6), new(12, 6)),
                Open(new(9, 6.5), new(8, 7), new(7, 7)),
                Open(new(5, 6.5), new(4, 6), new(3, 6)),
                Open(new(0, 7), new(-2, 8), new(0, 10)),
                Open(new(0, 10), new(4, 11), new(8, 11), new(12, 10)),
                Open(new(12, 10), new(14, 8))
            ];

        Polygon expectedLines = expected;
        bool retainsArea = operation is BooleanOperation.Union or BooleanOperation.Xor;
        if (retainsArea)
        {
            expected = clip.DeepClone();
            foreach (Contour line in expectedLines)
            {
                expected.Add(line);
            }
        }

        if (reverse)
        {
            subject[0].Reverse();
        }

        string details = $"Operation-{operation}_Reverse-{reverse}";
        Polygon actual = Execute(subject, clip, operation, nameof(this.CurvedPath_ClipsAgainstTwoHolesAndCornerContacts), details, expected);
        Polygon actualLines = [];
        List<Contour> actualArea = [];
        foreach (Contour contour in actual)
        {
            if (contour.IsClosed)
            {
                actualArea.Add(contour);
            }
            else
            {
                actualLines.Add(contour);
            }
        }

        AssertOpenPaths(actualLines, expectedLines);
        Assert.Equal(retainsArea ? 3 : 0, actualArea.Count);
        if (retainsArea)
        {
            // Line clipping must retain the rectangle and its two holes.
            foreach (Contour reference in clip)
            {
                Assert.Contains(actualArea, contour => contour.GetBoundingBox() == reference.GetBoundingBox() &&
                    PolygonUtilities.Area(contour.AsVertexSpan()) == PolygonUtilities.Area(reference.AsVertexSpan()));
            }
        }
    }

    /// <summary>
    /// Checks a shared curved span and a separate crossing in the same pair of paths.
    /// </summary>
    /// <param name="operation">The requested operation.</param>
    /// <param name="reverse">Whether both paths run backwards.</param>
    [Theory]
    [InlineData(BooleanOperation.Intersection, false)]
    [InlineData(BooleanOperation.Intersection, true)]
    [InlineData(BooleanOperation.Union, false)]
    [InlineData(BooleanOperation.Union, true)]
    [InlineData(BooleanOperation.Difference, false)]
    [InlineData(BooleanOperation.Difference, true)]
    [InlineData(BooleanOperation.Xor, false)]
    [InlineData(BooleanOperation.Xor, true)]
    public void SharedCurvedSpan_RetainsOverlapAndSeparateCrossing(BooleanOperation operation, bool reverse)
    {
        Polygon subject =
        [Open(new(-6, 0), new(-4, 3), new(-2, 4), new(0, 3), new(2, 0), new(4, -3), new(6, -4), new(8, -3), new(10, 0))];
        Polygon clip =
        [Open(new(-6, -2), new(-4, 3), new(-2, 4), new(0, 3), new(2, 0), new(4, 2), new(6, 0), new(8, -3), new(10, -1))];

        Polygon expected = operation == BooleanOperation.Intersection
            ?
            [
                Open(new(-4, 3), new(-2, 4)),
                Open(new(-2, 4), new(0, 3)),
                Open(new(0, 3), new(2, 0)),
                Open(new Vertex(8, -3))
            ]
            :
            [
                Open(new(-6, 0), new(-4, 3)),
                Open(new(2, 0), new(4, -3), new(6, -4), new(8, -3)),
                Open(new(8, -3), new(10, 0))
            ];

        if (operation is BooleanOperation.Union or BooleanOperation.Xor)
        {
            expected.Add(Open(new(-6, -2), new(-4, 3)));
            expected.Add(Open(new(2, 0), new(4, 2), new(6, 0), new(8, -3)));
            expected.Add(Open(new(8, -3), new(10, -1)));
        }

        if (operation == BooleanOperation.Union)
        {
            // Both operands meet at each shared vertex, so those vertices remain cut endpoints.
            expected.Add(Open(new(-4, 3), new(-2, 4)));
            expected.Add(Open(new(-2, 4), new(0, 3)));
            expected.Add(Open(new(0, 3), new(2, 0)));
        }

        if (reverse)
        {
            subject[0].Reverse();
            clip[0].Reverse();
        }

        string details = $"Operation-{operation}_Reverse-{reverse}";
        Polygon actual = Execute(subject, clip, operation, nameof(this.SharedCurvedSpan_RetainsOverlapAndSeparateCrossing), details, expected);
        AssertOpenPaths(actual, expected);
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
    /// Samples y = (x^2 - 9)(x^2 - 1) / 16 at quarter-unit X positions.
    /// </summary>
    /// <param name="sample">Four times the X coordinate.</param>
    /// <returns>The exact double coordinates of this polynomial sample.</returns>
    private static Vertex SampleQuartic(int sample)
    {
        double x = sample / 4D;
        double squared = x * x;

        // Quarter-unit inputs and the vertical scale of 1/16 use powers of two.
        // Every source coordinate is exact, including the four zero crossings.
        return new(x, (squared - 9) * (squared - 1) / 16D);
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
    /// Executes the public operation and saves the actual and available expected geometry.
    /// </summary>
    /// <param name="subject">The subject contours.</param>
    /// <param name="clip">The clip contours.</param>
    /// <param name="operation">The requested operation.</param>
    /// <param name="testName">The test method name.</param>
    /// <param name="details">The invariant parameter details for the file name.</param>
    /// <param name="expected">The known expected geometry, or null when unavailable.</param>
    /// <returns>The clipped contours.</returns>
    private static Polygon Execute(
        Polygon subject, Polygon clip, BooleanOperation operation, string testName, string details, Polygon expected)
    {
        Polygon result = operation switch
        {
            BooleanOperation.Intersection => PolygonClipper.Intersection(subject, clip, FillRule.NonZero),
            BooleanOperation.Union => PolygonClipper.Union(subject, clip, FillRule.NonZero),
            BooleanOperation.Difference => PolygonClipper.Difference(subject, clip, FillRule.NonZero),
            _ => PolygonClipper.Xor(subject, clip, FillRule.NonZero)
        };

        if (expected is null)
        {
            SvgTestOutput.SaveSvg(nameof(OpenPathClippingTests), testName, details, subject, clip, result);
        }
        else
        {
            SvgTestOutput.SaveSvg(nameof(OpenPathClippingTests), testName, details, subject, clip, result, expected);
        }

        return result;
    }

    /// <summary>
    /// Checks every open contour's vertices in either direction, including points and bends.
    /// </summary>
    /// <param name="actual">The clipped open geometry.</param>
    /// <param name="expected">The independently constructed open geometry.</param>
    private static void AssertOpenPaths(Polygon actual, Polygon expected)
    {
        Assert.Equal(expected.Count, actual.Count);
        Assert.All(actual, contour =>
        {
            Assert.False(contour.IsClosed);
            Assert.Null(contour.ParentIndex);
            Assert.Equal(0, contour.Depth);
            Assert.Equal(0, contour.HoleCount);
        });

        foreach (Contour reference in expected)
        {
            Assert.Contains(actual, contour =>
            {
                if (contour.Count != reference.Count)
                {
                    return false;
                }

                bool forward = true;
                bool backward = true;
                for (int i = 0; i < reference.Count; i++)
                {
                    forward &= contour[i] == reference[i];
                    backward &= contour[i] == reference[reference.Count - 1 - i];
                }

                return forward || backward;
            });
        }
    }

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

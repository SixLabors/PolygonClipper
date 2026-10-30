// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

namespace SixLabors.PolygonClipper.Tests;

/// <summary>
/// Checks polygon offsets against known geometry and collapse distances.
/// </summary>
public class PolygonOffsetTests
{
    /// <summary>
    /// Checks that erosion past the center removes a square instead of leaving a false island.
    /// </summary>
    [Fact]
    public void Offset_DeflationBeyondSquareCenter_ReturnsEmpty()
    {
        Polygon subject = [[new(0, 0), new(10, 0), new(10, 10), new(0, 10)]];
        OffsetOptions options = new() { LineJoin = LineJoin.Miter };
        Polygon actual = PolygonOffsetter.Offset(subject, -6, FillRule.NonZero, options);
        SvgTestOutput.SaveSvg(
            nameof(PolygonOffsetTests), nameof(this.Offset_DeflationBeyondSquareCenter_ReturnsEmpty), string.Empty, subject, [], actual, []);

        Assert.Empty(actual);
    }

    /// <summary>
    /// Checks exact rectangle boundaries before, at and beyond collapse.
    /// </summary>
    /// <param name="delta">The signed boundary distance.</param>
    [Theory]
    [InlineData(-12D)]
    [InlineData(-5D)]
    [InlineData(-4D)]
    [InlineData(-1D)]
    [InlineData(0D)]
    [InlineData(1D)]
    [InlineData(6D)]
    public void Offset_Rectangle_MatchesKnownBounds(double delta)
    {
        Polygon subject = [Rectangle(0, 0, 10, 10)];
        Polygon expected = delta <= -5 ? [] : [Rectangle(-delta, -delta, 10 + delta, 10 + delta)];
        Polygon actual = PolygonOffsetter.Offset(subject, delta, FillRule.NonZero, new() { LineJoin = LineJoin.Miter });

        AssertGeometry(nameof(this.Offset_Rectangle_MatchesKnownBounds), $"Delta-{delta}", subject, actual, expected);
    }

    /// <summary>
    /// Checks shrinking holes and the disappearance of an eroded ring.
    /// </summary>
    /// <param name="delta">The signed boundary distance.</param>
    [Theory]
    [InlineData(-4D)]
    [InlineData(-3D)]
    [InlineData(-2D)]
    [InlineData(-1D)]
    [InlineData(1D)]
    [InlineData(4D)]
    [InlineData(6D)]
    public void Offset_Donut_HandlesBothBoundaryCollapses(double delta)
    {
        Contour hole = Rectangle(6, 6, 14, 14);
        hole.Reverse();
        Polygon subject = [Rectangle(0, 0, 20, 20), hole];
        Polygon expected = [];
        if (delta > -3)
        {
            expected.Add(Rectangle(-delta, -delta, 20 + delta, 20 + delta));
            if (delta < 4)
            {
                Contour expectedHole = Rectangle(6 + delta, 6 + delta, 14 - delta, 14 - delta);
                expectedHole.Reverse();
                expected.Add(expectedHole);
            }
        }

        Polygon actual = PolygonOffsetter.Offset(subject, delta, FillRule.NonZero, new() { LineJoin = LineJoin.Miter });
        AssertGeometry(nameof(this.Offset_Donut_HandlesBothBoundaryCollapses), $"Delta-{delta}", subject, actual, expected);
        if (expected.Count == 2)
        {
            Assert.Null(actual[0].ParentIndex);
            Assert.Equal(1, actual[0].HoleCount);
            Assert.Equal(0, actual[1].ParentIndex);
            Assert.Equal(1, actual[1].Depth);
        }
    }

    /// <summary>
    /// Checks inner corners and complete collapse of two narrow arms.
    /// </summary>
    /// <param name="delta">The signed boundary distance.</param>
    [Theory]
    [InlineData(-3D)]
    [InlineData(-2D)]
    [InlineData(-1D)]
    [InlineData(1D)]
    public void Offset_ConcaveBoundary_MatchesKnownCorners(double delta)
    {
        Polygon subject = [[new(0, 0), new(10, 0), new(10, 4), new(4, 4), new(4, 10), new(0, 10)]];
        Polygon expected = delta <= -2
            ? []
            : [[new(-delta, -delta), new(10 + delta, -delta), new(10 + delta, 4 + delta),
                new(4 + delta, 4 + delta), new(4 + delta, 10 + delta), new(-delta, 10 + delta)]];
        Polygon actual = PolygonOffsetter.Offset(subject, delta, FillRule.NonZero, new() { LineJoin = LineJoin.Miter });

        AssertGeometry(nameof(this.Offset_ConcaveBoundary_MatchesKnownCorners), $"Delta-{delta}", subject, actual, expected);
    }

    /// <summary>
    /// Checks that eroding a neck splits the area into two surviving components.
    /// </summary>
    [Fact]
    public void Offset_NarrowNeck_SplitsComponents()
    {
        Polygon subject =
        [[new(0, 0), new(6, 0), new(6, 2), new(10, 2), new(10, 0), new(16, 0),
            new(16, 6), new(10, 6), new(10, 4), new(6, 4), new(6, 6), new(0, 6)]];
        Polygon expected = [Rectangle(1, 1, 5, 5), Rectangle(11, 1, 15, 5)];
        Polygon actual = PolygonOffsetter.Offset(subject, -1, FillRule.NonZero, new() { LineJoin = LineJoin.Miter });

        AssertGeometry(nameof(this.Offset_NarrowNeck_SplitsComponents), string.Empty, subject, actual, expected);
    }

    /// <summary>
    /// Checks the union of separately expanded components when their boundaries meet.
    /// </summary>
    [Fact]
    public void Offset_ExpandingComponents_Merge()
    {
        Polygon subject = [Rectangle(0, 0, 4, 4), Rectangle(6, 0, 10, 4)];
        Polygon expected = [Rectangle(-1, -1, 11, 5)];
        Polygon actual = PolygonOffsetter.Offset(subject, 1, FillRule.NonZero, new() { LineJoin = LineJoin.Miter });

        AssertGeometry(nameof(this.Offset_ExpandingComponents_Merge), string.Empty, subject, actual, expected);
    }

    /// <summary>
    /// Checks the full containment hierarchy for an island inside a hole.
    /// </summary>
    [Fact]
    public void Offset_NestedIsland_PreservesHierarchy()
    {
        Contour hole = Rectangle(4, 4, 26, 26);
        hole.Reverse();
        Polygon subject = [Rectangle(0, 0, 30, 30), hole, Rectangle(10, 10, 20, 20)];
        Contour expectedHole = Rectangle(5, 5, 25, 25);
        expectedHole.Reverse();
        Polygon expected = [Rectangle(-1, -1, 31, 31), expectedHole, Rectangle(9, 9, 21, 21)];
        Polygon actual = PolygonOffsetter.Offset(subject, 1, FillRule.NonZero, new() { LineJoin = LineJoin.Miter });

        AssertGeometry(nameof(this.Offset_NestedIsland_PreservesHierarchy), string.Empty, subject, actual, expected);
        Assert.Null(actual[0].ParentIndex);
        Assert.Equal(0, actual[1].ParentIndex);
        Assert.Equal(1, actual[2].ParentIndex);
        Assert.Equal(2, actual[2].Depth);
    }

    /// <summary>
    /// Checks that raw overlap and direction are interpreted before offsetting the filled boundary.
    /// </summary>
    /// <param name="rule">The source fill rule.</param>
    /// <param name="reverse">Whether both source contours run clockwise.</param>
    [Theory]
    [InlineData(FillRule.EvenOdd, false)]
    [InlineData(FillRule.EvenOdd, true)]
    [InlineData(FillRule.NonZero, false)]
    [InlineData(FillRule.NonZero, true)]
    [InlineData(FillRule.Positive, false)]
    [InlineData(FillRule.Positive, true)]
    [InlineData(FillRule.Negative, false)]
    [InlineData(FillRule.Negative, true)]
    public void Offset_OverlappingContours_UsesSourceFillRule(FillRule rule, bool reverse)
    {
        Polygon subject = [Rectangle(0, 0, 10, 10), Rectangle(5, 0, 15, 10)];
        if (reverse)
        {
            foreach (Contour contour in subject)
            {
                contour.Reverse();
            }
        }

        Polygon expected = rule == FillRule.EvenOdd
            ? [Rectangle(-0.5, -0.5, 5.5, 10.5), Rectangle(9.5, -0.5, 15.5, 10.5)]
            : rule == FillRule.NonZero || (rule == FillRule.Positive && !reverse) || (rule == FillRule.Negative && reverse)
                ? [Rectangle(-0.5, -0.5, 15.5, 10.5)]
                : [];
        Polygon actual = PolygonOffsetter.Offset(subject, 0.5, rule, new() { LineJoin = LineJoin.Miter });

        AssertGeometry(nameof(this.Offset_OverlappingContours_UsesSourceFillRule), $"Rule-{rule}_Reverse-{reverse}", subject, actual, expected);
    }

    /// <summary>
    /// Checks the eight known vertices of a beveled rectangle offset.
    /// </summary>
    /// <param name="join">The bevel join or miter fallback to check.</param>
    [Theory]
    [InlineData(LineJoin.Bevel)]
    [InlineData(LineJoin.MiterRevert)]
    public void Offset_Bevel_ClipsOuterCorners(LineJoin join)
    {
        Polygon subject = [Rectangle(0, 0, 10, 10)];
        Polygon expected = [[new(-2, 0), new(0, -2), new(10, -2), new(12, 0), new(12, 10), new(10, 12), new(0, 12), new(-2, 10)]];
        Polygon actual = PolygonOffsetter.Offset(subject, 2, FillRule.NonZero, new() { LineJoin = join, MiterLimit = 1 });

        AssertGeometry(nameof(this.Offset_Bevel_ClipsOuterCorners), $"Join-{join}", subject, actual, expected);
    }

    /// <summary>
    /// Checks the radius and area of tessellated round corners without assuming a vertex count.
    /// </summary>
    /// <param name="join">The round join or miter fallback to check.</param>
    [Theory]
    [InlineData(LineJoin.Round)]
    [InlineData(LineJoin.MiterRound)]
    public void Offset_RoundCorners_StayAtRequestedDistance(LineJoin join)
    {
        Polygon subject = [Rectangle(0, 0, 10, 10)];
        Polygon actual = PolygonOffsetter.Offset(subject, 2, FillRule.NonZero, new() { LineJoin = join, MiterLimit = 1, ArcDetailScale = 32 });
        SvgTestOutput.SaveSvg(nameof(PolygonOffsetTests), nameof(this.Offset_RoundCorners_StayAtRequestedDistance), $"Join-{join}", subject, [], actual);
        Contour outline = Assert.Single(actual);
        foreach (Vertex vertex in outline)
        {
            double dx = vertex.X - Math.Clamp(vertex.X, 0, 10);
            double dy = vertex.Y - Math.Clamp(vertex.Y, 0, 10);
            Assert.Equal(4D, (dx * dx) + (dy * dy), 10);
        }

        // The exact area is the square, four edge strips and four quarter circles.
        // Straight chords approximate the arcs from inside, so their area is slightly smaller.
        double exactArea = 180 + (4 * Math.PI);
        Assert.InRange(PolygonUtilities.Area(outline.AsVertexSpan()), exactArea - 0.1, exactArea);
    }

    /// <summary>
    /// Checks that round deflation expands a hole while contracting the exterior boundary.
    /// </summary>
    [Fact]
    public void Offset_RoundDeflation_ExpandsHoleCorners()
    {
        Contour hole = Rectangle(6, 6, 14, 14);
        hole.Reverse();
        Polygon subject = [Rectangle(0, 0, 20, 20), hole];
        Polygon actual = PolygonOffsetter.Offset(subject, -2, FillRule.NonZero, new() { LineJoin = LineJoin.Round, ArcDetailScale = 32 });
        SvgTestOutput.SaveSvg(nameof(PolygonOffsetTests), nameof(this.Offset_RoundDeflation_ExpandsHoleCorners), string.Empty, subject, [], actual);

        Assert.Equal(2, actual.Count);
        ApproximateDoubleComparer comparer = new(1E-10D);
        Assert.Equal(new Vertex(2, 2), actual[0].GetBoundingBox().Min, comparer);
        Assert.Equal(new Vertex(18, 18), actual[0].GetBoundingBox().Max, comparer);
        Assert.Equal(0, actual[1].ParentIndex);
        foreach (Vertex vertex in actual[1])
        {
            double dx = vertex.X - Math.Clamp(vertex.X, 6, 14);
            double dy = vertex.Y - Math.Clamp(vertex.Y, 6, 14);
            Assert.Equal(4D, (dx * dx) + (dy * dy), comparer);
        }

        // The exterior area is 16 squared. The hole adds four width-two strips and a radius-two circle.
        // Chords slightly reduce the hole area, so the retained area slightly exceeds the exact value.
        double exactArea = 256 - (128 + (4 * Math.PI));
        double area = actual.Sum(contour => PolygonUtilities.Area(contour.AsVertexSpan()));
        Assert.InRange(area, exactArea, exactArea + 0.1);
    }

    /// <summary>
    /// Checks that zero distance returns independent resolved contours without changing the source.
    /// </summary>
    [Fact]
    public void Offset_Zero_ReturnsDetachedResolvedArea()
    {
        Polygon subject = [Rectangle(0, 0, 10, 10), Rectangle(5, 0, 15, 10)];
        string before = subject.ToDebugString();
        Polygon actual = PolygonOffsetter.Offset(subject, 0, FillRule.NonZero, new());
        Polygon expected = [Rectangle(0, 0, 15, 10)];

        AssertGeometry(nameof(this.Offset_Zero_ReturnsDetachedResolvedArea), string.Empty, subject, actual, expected);
        actual[0].Reverse();
        Assert.Equal(before, subject.ToDebugString());
    }

    /// <summary>
    /// Checks that pooled offset execution cannot change later ordinary strokes or mutate the options.
    /// </summary>
    [Fact]
    public void Offset_PooledReuse_PreservesStrokeBehavior()
    {
        Polygon subject = [Rectangle(0, 0, 10, 10)];
        StrokeOptions options = new() { LineJoin = LineJoin.Miter, NormalizeOutput = true, LineCap = LineCap.Round };
        StrokeOptions rawOptions = new() { LineJoin = LineJoin.Miter };
        OffsetOptions offsetOptions = new() { LineJoin = LineJoin.Miter };
        string before = subject.ToDebugString();
        Polygon stroke = PolygonStroker.Stroke(subject, 2, options);
        Polygon rawStroke = PolygonStroker.Stroke(subject, 2, rawOptions);
        for (int i = 0; i < 4; i++)
        {
            Polygon actual = PolygonOffsetter.Offset(subject, (i & 1) == 0 ? -6 : 1, FillRule.NonZero, offsetOptions);
            Polygon expected = (i & 1) == 0 ? [] : [Rectangle(-1, -1, 11, 11)];
            AssertGeometry(nameof(this.Offset_PooledReuse_PreservesStrokeBehavior), $"Iteration-{i}", subject, actual, expected);
            Assert.Equal(stroke.ToDebugString(), PolygonStroker.Stroke(subject, 2, options).ToDebugString());
            Assert.Equal(rawStroke.ToDebugString(), PolygonStroker.Stroke(subject, 2, rawOptions).ToDebugString());
        }

        Assert.Equal(before, subject.ToDebugString());
        Assert.True(options.NormalizeOutput);
        Assert.Equal(LineCap.Round, options.LineCap);
        Assert.Equal(LineJoin.Miter, offsetOptions.LineJoin);
        Assert.Equal(4D, offsetOptions.MiterLimit);
        Assert.Equal(1D, offsetOptions.ArcDetailScale);
    }

    /// <summary>
    /// Checks public distance and closure validation before offset generation.
    /// </summary>
    [Fact]
    public void Offset_InvalidArguments_AreRejected()
    {
        Polygon subject = [Rectangle(0, 0, 10, 10)];
        OffsetOptions options = new();
        Assert.Throws<ArgumentNullException>(() => PolygonOffsetter.Offset(null, 1, FillRule.NonZero, options));
        Assert.Throws<ArgumentNullException>(() => PolygonOffsetter.Offset(subject, 1, FillRule.NonZero, null));
        Assert.Throws<ArgumentOutOfRangeException>(() => PolygonOffsetter.Offset(subject, double.NaN, FillRule.NonZero, options));
        Assert.Throws<ArgumentOutOfRangeException>(() => PolygonOffsetter.Offset(subject, double.PositiveInfinity, FillRule.NonZero, options));
        Assert.Throws<ArgumentOutOfRangeException>(() => PolygonOffsetter.Offset(subject, double.NegativeInfinity, FillRule.NonZero, options));
        Assert.Throws<ArgumentOutOfRangeException>(() => PolygonOffsetter.Offset(subject, 0, (FillRule)99, options));
        subject[0].IsClosed = false;
        Assert.Throws<ArgumentException>(() => PolygonOffsetter.Offset(subject, 1, FillRule.NonZero, options));
        Assert.False(subject[0].IsClosed);
    }

    /// <summary>
    /// Checks both lobes of an unprepared bowtie against the six independently derived miter vertices.
    /// </summary>
    [Fact]
    public void Offset_SelfCrossingBoundary_ResolvesBothLobes()
    {
        Polygon subject = [[new(0, 0), new(10, 10), new(0, 10), new(10, 0)]];
        double diagonal = Math.Sqrt(2);

        // Each diagonal moves by one perpendicular unit. Its coordinate sum or difference
        // therefore changes by sqrt(2). Intersect those lines with y = -1, 5 and 11.
        Polygon expected =
        [[new(-1 - diagonal, -1), new(11 + diagonal, -1), new(5 + diagonal, 5),
            new(11 + diagonal, 11), new(-1 - diagonal, 11), new(5 - diagonal, 5)]];
        Polygon actual = PolygonOffsetter.Offset(subject, 1, FillRule.NonZero, new() { LineJoin = LineJoin.Miter });

        AssertGeometry(nameof(this.Offset_SelfCrossingBoundary_ResolvesBothLobes), string.Empty, subject, actual, expected);
    }

    /// <summary>
    /// Checks that offset distance scales with the coordinates without an integer conversion.
    /// </summary>
    /// <param name="scale">The coordinate and distance scale.</param>
    /// <param name="delta">The signed distance before scaling.</param>
    [Theory]
    [InlineData(1E-8, 1D)]
    [InlineData(1E-8, -6D)]
    [InlineData(1E8, 1D)]
    [InlineData(1E8, -6D)]
    [InlineData(1E90, 1D)]
    [InlineData(1E90, -6D)]
    public void Offset_ScaledRectangle_PreservesDistance(double scale, double delta)
    {
        Polygon subject = [Rectangle(0, 0, 10 * scale, 10 * scale)];
        Polygon expected = delta < -5 ? [] : [Rectangle(-scale, -scale, 11 * scale, 11 * scale)];
        Polygon actual = PolygonOffsetter.Offset(subject, delta * scale, FillRule.NonZero, new() { LineJoin = LineJoin.Miter });

        AssertGeometry(
            nameof(this.Offset_ScaledRectangle_PreservesDistance),
            FormattableString.Invariant($"Scale-{scale:R}_Delta-{delta}"),
            subject,
            actual,
            expected);
    }

    /// <summary>
    /// Checks dense regular polygons against the offset of their common edge distance from the center.
    /// </summary>
    /// <param name="vertexCount">The number of regular polygon vertices.</param>
    /// <param name="radius">The source vertex distance from the center.</param>
    /// <param name="delta">The signed boundary distance.</param>
    /// <remarks>Radius 10 provides visible offsets. Radius 1000 retains the small-offset accuracy cases.</remarks>
    [Theory]
    [InlineData(16, 10D, -3D)]
    [InlineData(16, 10D, -1D)]
    [InlineData(16, 10D, 1D)]
    [InlineData(16, 10D, 3D)]
    [InlineData(16, 1000D, -1D)]
    [InlineData(16, 1000D, 1D)]
    [InlineData(256, 10D, -3D)]
    [InlineData(256, 10D, -1D)]
    [InlineData(256, 10D, 1D)]
    [InlineData(256, 10D, 3D)]
    [InlineData(256, 1000D, -1D)]
    [InlineData(256, 1000D, 1D)]
    [InlineData(1024, 10D, -3D)]
    [InlineData(1024, 10D, -1D)]
    [InlineData(1024, 10D, 1D)]
    [InlineData(1024, 10D, 3D)]
    [InlineData(1024, 1000D, -1D)]
    [InlineData(1024, 1000D, 1D)]
    public void Offset_RegularPolygon_MatchesKnownRadius(int vertexCount, double radius, double delta)
    {
        Contour source = new(vertexCount);
        Contour reference = new(vertexCount);

        // A regular polygon's edge distance from its center is radius * cos(pi / vertexCount).
        // Moving every edge by delta therefore changes the vertex radius by delta / cos(pi / vertexCount).
        double expectedRadius = radius + (delta / Math.Cos(Math.PI / vertexCount));
        for (int i = 0; i < vertexCount; i++)
        {
            double angle = i * Math.PI * 2 / vertexCount;
            Vertex direction = new(Math.Cos(angle), Math.Sin(angle));
            source.Add(direction * radius);
            reference.Add(direction * expectedRadius);
        }

        Polygon subject = [source];
        Polygon actual = PolygonOffsetter.Offset(subject, delta, FillRule.NonZero, new() { LineJoin = LineJoin.Miter });
        AssertGeometry(
            nameof(this.Offset_RegularPolygon_MatchesKnownRadius),
            FormattableString.Invariant($"VertexCount-{vertexCount}_Radius-{radius}_Delta-{delta}"),
            subject,
            actual,
            [reference]);
    }

    /// <summary>
    /// Checks short obtuse corners where the side strips overlap past the neighboring edge midpoints.
    /// </summary>
    /// <param name="distance">The inward boundary distance.</param>
    [Theory]
    [InlineData(1D)]
    [InlineData(1.4D)]
    public void Offset_ShortHexagon_HandlesOverlappingCornerStrips(double distance)
    {
        Contour source = new(6);
        Contour reference = new(6);
        double expectedRadius = 1.5 - (distance / Math.Cos(Math.PI / 6));
        for (int i = 0; i < 6; i++)
        {
            double angle = i * Math.PI / 3;
            Vertex direction = new(Math.Cos(angle), Math.Sin(angle));
            source.Add(direction * 1.5);
            reference.Add(direction * expectedRadius);
        }

        Polygon subject = [source];
        Polygon expected = expectedRadius > 0 ? [reference] : [];
        Polygon actual = PolygonOffsetter.Offset(subject, -distance, FillRule.NonZero, new() { LineJoin = LineJoin.Miter });

        AssertGeometry(nameof(this.Offset_ShortHexagon_HandlesOverlappingCornerStrips), $"Distance-{distance}", subject, actual, expected);
    }

    /// <summary>
    /// Checks irregular orthogonal areas against independently expanded source or complement cells.
    /// </summary>
    /// <param name="seed">The fixed cell selection seed.</param>
    /// <param name="delta">The signed distance.</param>
    [Theory]
    [InlineData(0, 0.5D)]
    [InlineData(0, -0.5D)]
    [InlineData(0, 3D)]
    [InlineData(0, -3D)]
    [InlineData(17, 0.5D)]
    [InlineData(17, -0.5D)]
    [InlineData(17, 3D)]
    [InlineData(17, -3D)]
    [InlineData(42, 0.5D)]
    [InlineData(42, -0.5D)]
    [InlineData(42, 3D)]
    [InlineData(42, -3D)]
    public void Offset_RectangleUnions_MatchCellConstruction(int seed, double delta)
    {
        Random random = new(seed);
        bool[,] occupied = new bool[5, 5];
        Polygon subject = [];
        for (int y = 0; y < 5; y++)
        {
            for (int x = 0; x < 5; x++)
            {
                occupied[x, y] = random.Next(3) != 0;
                if (occupied[x, y])
                {
                    subject.Add(Rectangle(x * 2, y * 2, (x + 1) * 2, (y + 1) * 2));
                }
            }
        }

        // Axis-aligned miter dilation distributes over a union of rectangles. Erosion is
        // the complement of the dilated complement. The extra two cell rows on each side
        // cover the largest tested distance of three units without truncating that construction.
        Polygon expandedCells = [];
        double distance = Math.Abs(delta);
        for (int y = -2; y < 7; y++)
        {
            for (int x = -2; x < 7; x++)
            {
                bool filled = x >= 0 && x < 5 && y >= 0 && y < 5 && occupied[x, y];
                if (filled == (delta > 0))
                {
                    expandedCells.Add(Rectangle((x * 2) - distance, (y * 2) - distance, ((x + 1) * 2) + distance, ((y + 1) * 2) + distance));
                }
            }
        }

        Polygon expected = delta > 0
            ? PolygonClipper.Normalize(expandedCells, FillRule.NonZero)
            : PolygonClipper.Difference(subject, expandedCells, FillRule.NonZero);
        Polygon actual = PolygonOffsetter.Offset(subject, delta, FillRule.NonZero, new() { LineJoin = LineJoin.Miter });

        AssertGeometry(nameof(this.Offset_RectangleUnions_MatchCellConstruction), $"Seed-{seed}_Delta-{delta}", subject, actual, expected);
    }

    /// <summary>
    /// Checks that a collapsed component cannot cut the retained bevel of another component.
    /// </summary>
    [Fact]
    public void Offset_CollapsedComponent_DoesNotCutNeighbor()
    {
        Polygon subject =
        [
            [new(-10, -10), new(10, -10), new(10, 0), new(0, 0), new(0, 10), new(-10, 10)],
            [new(0.01, 0.01), new(0.03, 0.01), new(0.01, 0.03)]
        ];
        Polygon expected = [[new(-9, -9), new(9, -9), new(9, -1), new(0, -1), new(-1, 0), new(-1, 9), new(-9, 9)]];
        Polygon actual = PolygonOffsetter.Offset(subject, -1, FillRule.NonZero, new() { LineJoin = LineJoin.Bevel });

        AssertGeometry(nameof(this.Offset_CollapsedComponent_DoesNotCutNeighbor), string.Empty, subject, actual, expected);
    }

    /// <summary>
    /// Checks complete collapse of a densely sampled convex boundary.
    /// </summary>
    [Fact]
    public void Offset_DenseConvexCollapse_ReturnsEmpty()
    {
        Contour contour = new(128);
        for (int i = 0; i < 128; i++)
        {
            double angle = i * Math.PI * 2 / 128;
            contour.Add(new(10 * Math.Cos(angle), 10 * Math.Sin(angle)));
        }

        Polygon subject = [contour];
        Polygon actual = PolygonOffsetter.Offset(subject, -12, FillRule.NonZero, new() { LineJoin = LineJoin.Miter });
        AssertGeometry(nameof(this.Offset_DenseConvexCollapse_ReturnsEmpty), string.Empty, subject, actual, []);
    }

    /// <summary>
    /// Checks that bounding-box subtraction rounding cannot classify a surviving area as collapsed.
    /// </summary>
    [Fact]
    public void Offset_BoundsRounding_DoesNotDropThinArea()
    {
        double left = -1D / 9_007_199_254_740_992D;
        Polygon subject = [Rectangle(left, 0, 2, 4)];
        Polygon expected = [Rectangle(1 + left, 1, 1, 3)];
        Polygon actual = PolygonOffsetter.Offset(subject, -1, FillRule.NonZero, new() { LineJoin = LineJoin.Miter });

        AssertGeometry(nameof(this.Offset_BoundsRounding_DoesNotDropThinArea), string.Empty, subject, actual, expected);
        Assert.True(actual[0].GetBoundingBox().Max.X > actual[0].GetBoundingBox().Min.X);
    }

    /// <summary>
    /// Saves the comparison and checks both boundaries, contour count and output closure.
    /// </summary>
    /// <param name="testName">The test method name.</param>
    /// <param name="details">The parameter details for the SVG file name.</param>
    /// <param name="subject">The source polygon.</param>
    /// <param name="actual">The offset output.</param>
    /// <param name="expected">Independently constructed expected boundaries.</param>
    private static void AssertGeometry(string testName, string details, Polygon subject, Polygon actual, Polygon expected)
    {
        SvgTestOutput.SaveSvg(nameof(PolygonOffsetTests), testName, details, subject, [], actual, expected);
        Assert.Equal(expected.Count, actual.Count);
        if (expected.Count > 0)
        {
            Box2 bounds = expected.GetBoundingBox();
            double magnitude = Math.Max(Math.Max(Math.Abs(bounds.Min.X), Math.Abs(bounds.Min.Y)), Math.Max(Math.Abs(bounds.Max.X), Math.Abs(bounds.Max.Y)));

            // Unit roundoff is 2^-53. Allow 64 rounding units for offset normals and line intersections.
            // This allowance scales with the coordinates; it does not hide a fixed-size geometric defect.
            double tolerance = magnitude * (64D / 9_007_199_254_740_992D);
            ApproximateDoubleComparer comparer = new(tolerance);
            AssertBoundaryOn(actual, expected, comparer);
            AssertBoundaryOn(expected, actual, comparer);
        }

        Assert.All(actual, contour =>
        {
            Assert.True(contour.IsClosed);
            Assert.NotEqual(contour[0], contour[^1]);
            Assert.Equal((contour.Depth & 1) == 0, contour.IsCounterClockwise());
        });
    }

    /// <summary>
    /// Checks each edge endpoint and midpoint against the other polygon's boundary segments.
    /// </summary>
    /// <param name="first">The boundary to inspect.</param>
    /// <param name="second">The comparison boundary.</param>
    /// <param name="comparer">The coordinate comparer for the xUnit assertions.</param>
    private static void AssertBoundaryOn(Polygon first, Polygon second, ApproximateDoubleComparer comparer)
    {
        foreach (Contour contour in first)
        {
            for (int i = 0; i < contour.Count; i++)
            {
                for (int sample = 0; sample < 2; sample++)
                {
                    Vertex point = sample == 0 ? contour[i] : (contour[i] + contour[(i + 1) % contour.Count]) * 0.5D;
                    double nearest = double.PositiveInfinity;
                    Vertex nearestPoint = default;
                    foreach (Contour reference in second)
                    {
                        for (int j = 0; j < reference.Count; j++)
                        {
                            Vertex closest = PolygonUtilities.ClosestPointOnSegment(point, reference[j], reference[(j + 1) % reference.Count]);
                            double distance = Vertex.DistanceSquared(point, closest);
                            if (distance < nearest)
                            {
                                nearest = distance;
                                nearestPoint = closest;
                            }
                        }
                    }

                    Assert.Equal(point, nearestPoint, comparer);
                }
            }
        }
    }

    /// <summary>
    /// Constructs a counterclockwise rectangle with implicit closure.
    /// </summary>
    /// <param name="left">The minimum X coordinate.</param>
    /// <param name="bottom">The minimum Y coordinate.</param>
    /// <param name="right">The maximum X coordinate.</param>
    /// <param name="top">The maximum Y coordinate.</param>
    /// <returns>The rectangle boundary.</returns>
    private static Contour Rectangle(double left, double bottom, double right, double top)
        => [new(left, bottom), new(right, bottom), new(right, top), new(left, top)];
}

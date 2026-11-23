// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

namespace SixLabors.PolygonClipper.Tests;

/// <summary>
/// Checks filled geometry through the public single-sweep APIs.
/// </summary>
public class FillRuleClippingTests
{
    /// <summary>
    /// Checks the result bounds when the clipping operand extends beyond the subject.
    /// </summary>
    [Fact]
    public void BoundedOperations_LongClip_PreserveCompleteResult()
    {
        Polygon subject = [Rectangle(0, 0, 10, 10)];
        Polygon clip = [Rectangle(5, -1, 1000, 11)];

        Polygon intersection = PolygonClipper.Intersection(subject, clip, FillRule.NonZero);
        Polygon difference = PolygonClipper.Difference(subject, clip, FillRule.NonZero);

        Assert.Equal(50D, Area(intersection));
        Assert.Equal(50D, Area(difference));
        Assert.Equal(new Vertex(5, 0), intersection.GetBoundingBox().Min);
        Assert.Equal(new Vertex(10, 10), intersection.GetBoundingBox().Max);
        Assert.Equal(new Vertex(0, 0), difference.GetBoundingBox().Min);
        Assert.Equal(new Vertex(5, 10), difference.GetBoundingBox().Max);
    }

    /// <summary>
    /// Checks reuse of a full nesting hierarchy in a second clipping operation.
    /// </summary>
    [Fact]
    public void NestedCleanup_ReusedInIntersection_PreservesAreaAndInputHierarchy()
    {
        Polygon source = [Rectangle(0, 0, 10, 10), Rectangle(2, 2, 8, 8), Rectangle(3, 3, 7, 7), Rectangle(4, 4, 6, 6)];
        Polygon cleaned = PolygonClipper.Normalize(source, FillRule.EvenOdd);
        Polygon original = cleaned.DeepClone();

        Polygon result = PolygonClipper.Intersection(cleaned, [Rectangle(5, -1, 11, 11)], FillRule.NonZero);

        // The mask bisects the symmetric area: (100 - 36 + 16 - 4) / 2 = 38.
        Assert.Equal(38D, Area(result));
        Assert.Equal(2, result.Count);
        foreach (Contour contour in result)
        {
            Assert.Null(contour.ParentIndex);
            Assert.Equal(0, contour.Depth);
            Assert.Equal(0, contour.HoleCount);
        }

        for (int i = 0; i < cleaned.Count; i++)
        {
            Assert.Equal<Vertex>(original[i], cleaned[i]);
            Assert.Equal(original[i].ParentIndex, cleaned[i].ParentIndex);
            Assert.Equal(original[i].Depth, cleaned[i].Depth);
            Assert.Equal(original[i].HoleCount, cleaned[i].HoleCount);
        }
    }

    /// <summary>
    /// Checks rule validation before the empty-intersection shortcut.
    /// </summary>
    /// <param name="value">An undefined fill-rule value.</param>
    [Theory]
    [InlineData(-1)]
    [InlineData(4)]
    public void InvalidRules_EmptyOperands_Throw(int value)
    {
        FillRule invalid = (FillRule)value;
        Assert.Throws<ArgumentOutOfRangeException>(() => PolygonClipper.Intersection([], [], invalid));
        Assert.Throws<ArgumentOutOfRangeException>(() => PolygonClipper.Intersection([], [], FillRule.NonZero, invalid));
        Assert.Throws<ArgumentOutOfRangeException>(() => PolygonClipper.Normalize([], invalid));
        Assert.Throws<ArgumentOutOfRangeException>(() => new PolygonClipper([], [], BooleanOperation.Union, invalid, FillRule.NonZero));
        Assert.Throws<ArgumentOutOfRangeException>(() => new PolygonClipper([], [], (BooleanOperation)value, FillRule.NonZero, FillRule.NonZero));
    }

    /// <summary>
    /// Checks coordinate validation and pooled reuse after an operation fails during ingestion.
    /// </summary>
    /// <param name="coordinate">The non-finite source coordinate.</param>
    [Theory]
    [InlineData(double.NaN)]
    [InlineData(double.PositiveInfinity)]
    [InlineData(double.NegativeInfinity)]
    public void NonFiniteCoordinates_UsedInputsThrow_AndPoolRemainsUsable(double coordinate)
    {
        Polygon invalid = [Rectangle(0, 0, 2, 2), [new Vertex(coordinate, 0), new Vertex(1, 0), new Vertex(0, 1)]];

        // Empty intersection does not inspect unused geometry. Normalize does.
        Assert.Empty(PolygonClipper.Intersection(invalid, [], FillRule.NonZero));
        Assert.Throws<ArgumentException>(() => PolygonClipper.Normalize(invalid, FillRule.NonZero));
        Assert.Equal(4D, Area(PolygonClipper.Normalize([Rectangle(0, 0, 2, 2)], FillRule.NonZero)));
    }

    /// <summary>
    /// Checks that rounded equal products do not cancel distinct source edges.
    /// </summary>
    /// <param name="rule">The source fill rule.</param>
    [Theory]
    [InlineData(FillRule.EvenOdd)]
    [InlineData(FillRule.NonZero)]
    [InlineData(FillRule.Positive)]
    public void Normalize_NearlyParallelIntegerEdges_PreservesTriangle(FillRule rule)
    {
        const double n = 134217728D;
        Contour triangle = [new Vertex(0, 0), new Vertex(n, n + 1), new Vertex(n - 1, n)];

        Polygon result = PolygonClipper.Normalize([triangle], rule);

        // The determinant is n*n - (n+1)*(n-1) = 1, so the area is exactly one half.
        Contour boundary = Assert.Single(result);
        Assert.Equal(3, boundary.Count);
        Assert.Equal(0.5D, Area(result));
        foreach (Vertex vertex in triangle)
        {
            Assert.Contains(vertex, boundary);
        }
    }

    /// <summary>
    /// Checks that input fill rules select the area while cleanup returns positive-oriented boundaries.
    /// </summary>
    /// <param name="rule">The input fill rule.</param>
    /// <param name="expectedArea">The analytically expected filled area.</param>
    [Theory]
    [InlineData(FillRule.EvenOdd, 64D)]
    [InlineData(FillRule.NonZero, 64D)]
    [InlineData(FillRule.Positive, 0D)]
    [InlineData(FillRule.Negative, 64D)]
    public void Cleanup_ClockwiseDonut_SelectsInputRuleAndCanonicalOutput(FillRule rule, double expectedArea)
    {
        Contour exterior = Rectangle(0, 0, 10, 10);
        exterior.Reverse();
        Polygon input = [exterior, Rectangle(2, 2, 8, 8)];
        Polygon original = input.DeepClone();

        Polygon cleaned = PolygonClipper.Normalize(input, rule);

        // The clockwise exterior contributes -1. Its counterclockwise inner
        // contour cancels that count in the 6-by-6 hole: 100 - 36 = 64.
        Assert.Equal(expectedArea, Area(cleaned));
        for (int i = 0; i < input.Count; i++)
        {
            Assert.Equal<Vertex>(original[i], input[i]);
            Assert.Equal(original[i].ParentIndex, input[i].ParentIndex);
        }

        if (expectedArea == 0)
        {
            Assert.Empty(cleaned);
        }
        else
        {
            Assert.Equal(2, cleaned.Count);
            Assert.True(Area([cleaned[0]]) > 0);
            Assert.True(Area([cleaned[1]]) < 0);
            Assert.Equal(0, cleaned[1].ParentIndex);
            Assert.Equal(1, cleaned[0].GetHoleIndex(0));

            for (int i = 0; i < cleaned.Count; i++)
            {
                Assert.True(cleaned[i].IsClosed);
                Assert.NotEqual(cleaned[i][0], cleaned[i][^1]);
            }

            // Resolved output describes an area with positive orientation.
            // NonZero, EvenOdd, and Positive therefore preserve that area on reuse.
            Assert.Equal(expectedArea, Area(PolygonClipper.Normalize(cleaned, FillRule.NonZero)));
            Assert.Equal(expectedArea, Area(PolygonClipper.Normalize(cleaned, FillRule.EvenOdd)));
            Assert.Equal(expectedArea, Area(PolygonClipper.Normalize(cleaned, FillRule.Positive)));
            Assert.Empty(PolygonClipper.Normalize(cleaned, FillRule.Negative));
        }
    }

    /// <summary>
    /// Checks cleanup against the bowtie's analytically defined winding regions.
    /// </summary>
    [Fact]
    public void Cleanup_Bowtie_SelectedRuleDeterminesFilledLobes()
    {
        Contour bowtie = [new Vertex(0, 0), new Vertex(100, 100), new Vertex(100, 0), new Vertex(0, 100)];

        Polygon positive = PolygonClipper.Normalize([bowtie], FillRule.Positive);
        Polygon negative = PolygonClipper.Normalize([bowtie], FillRule.Negative);
        Polygon evenOdd = PolygonClipper.Normalize([bowtie], FillRule.EvenOdd);
        Polygon nonZero = PolygonClipper.Normalize([bowtie], FillRule.NonZero);
        bowtie.Reverse();
        Polygon reversed = PolygonClipper.Normalize([bowtie], FillRule.Positive);
        Polygon reversedNonZero = PolygonClipper.Normalize([bowtie], FillRule.NonZero);

        // The two triangular lobes each have base 100 and perpendicular height 50.
        Assert.Equal(2500D, Area(positive));
        Assert.Equal(2500D, Area(negative));
        Assert.Equal(2500D, Area(reversed));
        Assert.Equal(5000D, Area(evenOdd));
        Assert.Equal(5000D, Area(nonZero));
        Assert.Equal(5000D, Area(reversedNonZero));
        Assert.Equal(new Vertex(0, 0), positive.GetBoundingBox().Min);
        Assert.Equal(new Vertex(50, 100), positive.GetBoundingBox().Max);
        Assert.Equal(new Vertex(50, 0), negative.GetBoundingBox().Min);
        Assert.Equal(new Vertex(100, 100), negative.GetBoundingBox().Max);
        Assert.Equal(new Vertex(50, 0), reversed.GetBoundingBox().Min);
        Assert.Equal(new Vertex(100, 100), reversed.GetBoundingBox().Max);

        foreach (Polygon filled in new[] { nonZero, evenOdd, reversedNonZero })
        {
            // The crossing is a shared vertex, not a bridge between the filled lobes.
            Assert.Equal(2, filled.Count);
            Assert.Contains(filled, triangle => triangle.Contains(new Vertex(0, 0)) &&
                triangle.Contains(new Vertex(50, 50)) && triangle.Contains(new Vertex(0, 100)));
            Assert.Contains(filled, triangle => triangle.Contains(new Vertex(50, 50)) &&
                triangle.Contains(new Vertex(100, 0)) && triangle.Contains(new Vertex(100, 100)));
            foreach (Contour triangle in filled)
            {
                Assert.Equal(3, triangle.Count);
                Assert.True(triangle.IsClosed);
                Assert.Null(triangle.ParentIndex);
                Assert.Equal(0, triangle.HoleCount);
                Assert.Equal(2500D, Area([triangle]));
            }
        }
    }

    /// <summary>
    /// Checks that retraced lines have no filled area in cleanup or binary clipping.
    /// </summary>
    /// <param name="rule">The input fill rule.</param>
    [Theory]
    [InlineData(FillRule.EvenOdd)]
    [InlineData(FillRule.NonZero)]
    [InlineData(FillRule.Positive)]
    [InlineData(FillRule.Negative)]
    public void Cleanup_RetracedLine_ProducesNoPolygon(FillRule rule)
    {
        Polygon line = [[new Vertex(0, 0), new Vertex(1, 0), new Vertex(0, 0)]];
        Polygon distant = [Rectangle(10, 10, 20, 20)];

        Assert.Empty(PolygonClipper.Normalize(line, rule));
        Assert.Empty(PolygonClipper.Intersection(line, distant, rule, FillRule.NonZero));
        Assert.Empty(PolygonClipper.Difference(line, distant, rule, FillRule.NonZero));
        Assert.Equal(100D, Area(PolygonClipper.Union(line, distant, rule, FillRule.NonZero)));
        Assert.Equal(100D, Area(PolygonClipper.Xor(line, distant, rule, FillRule.NonZero)));
    }

    /// <summary>
    /// Checks that cleanup retains real small features without fixed distance or area cutoffs.
    /// </summary>
    /// <param name="scale">The triangle's side length.</param>
    [Theory]
    [InlineData(1E-7)]
    [InlineData(1E-90)]
    [InlineData(1E90)]
    public void Cleanup_ScaledTriangle_PreservesFilledArea(double scale)
    {
        Polygon triangle = [[new Vertex(0, 0), new Vertex(scale, 0), new Vertex(0, scale)]];

        Polygon cleaned = PolygonClipper.Normalize(triangle, FillRule.NonZero);

        Assert.Equal(3, Assert.Single(cleaned).Count);
        Assert.Equal(0.5D, Area(cleaned) / (scale * scale), 10);
    }

    /// <summary>
    /// Checks full nesting metadata independently of the input contour directions.
    /// </summary>
    [Fact]
    public void Cleanup_FourNestedRings_PreservesContainmentAndArea()
    {
        Polygon input = [Rectangle(0, 0, 10, 10), Rectangle(2, 2, 8, 8), Rectangle(3, 3, 7, 7), Rectangle(4, 4, 6, 6)];

        Polygon cleaned = PolygonClipper.Normalize(input, FillRule.EvenOdd);

        Assert.Equal(76D, Area(cleaned));
        Assert.Equal(4, cleaned.Count);
        for (int i = 0; i < cleaned.Count; i++)
        {
            Assert.Equal(i, cleaned[i].Depth);
            Assert.Equal(i == 0 ? (int?)null : i - 1, cleaned[i].ParentIndex);
            Assert.Equal((i & 1) == 0, Area([cleaned[i]]) > 0);
            Assert.Equal(i == 3 ? 0 : 1, cleaned[i].HoleCount);
            if (i < 3)
            {
                Assert.Equal(i + 1, cleaned[i].GetHoleIndex(0));
            }
        }
    }

    /// <summary>
    /// Checks direct fill-aware clipping of overlapping glyph-like contours without preparation.
    /// </summary>
    [Fact]
    public void DirectClipping_OverlappingContours_NeedsNoCleanupPass()
    {
        Polygon subject = [Rectangle(0, 0, 10, 10), Rectangle(5, 0, 15, 10)];
        Polygon clip = [Rectangle(2, -1, 13, 11)];

        Polygon nonZero = PolygonClipper.Intersection(subject, clip, FillRule.NonZero, FillRule.EvenOdd);
        Polygon evenOdd = PolygonClipper.Intersection(subject, clip, FillRule.EvenOdd);

        // NonZero keeps the whole 11-by-10 clipped band. EvenOdd removes the
        // doubly covered 5-by-10 middle band and leaves two 3-by-10 pieces.
        Assert.Equal(110D, Area(nonZero));
        Assert.Equal(60D, Area(evenOdd));
        AssertCoverage(
            nameof(this.DirectClipping_OverlappingContours_NeedsNoCleanupPass),
            string.Empty,
            string.Empty,
            subject,
            clip,
            nonZero,
            BooleanOperation.Intersection,
            FillRule.NonZero,
            FillRule.EvenOdd);
        AssertCoverage(
            nameof(this.DirectClipping_OverlappingContours_NeedsNoCleanupPass),
            string.Empty,
            string.Empty,
            subject,
            clip,
            evenOdd,
            BooleanOperation.Intersection,
            FillRule.EvenOdd,
            FillRule.EvenOdd);
    }

    /// <summary>
    /// Checks direct NonZero clipping of both bowtie lobes without prior normalization.
    /// </summary>
    [Fact]
    public void DirectClipping_SelfIntersection_PreservesBothNonZeroLobes()
    {
        Polygon subject = [[new Vertex(0, 0), new Vertex(100, 100), new Vertex(100, 0), new Vertex(0, 100)]];
        Polygon clip = [Rectangle(25, -10, 75, 110)];

        Polygon result = PolygonClipper.Intersection(subject, clip, FillRule.NonZero, FillRule.EvenOdd);

        // Each clipped lobe is a triangle with base 50 and height 25.
        // NonZero retains both signs, giving total area 2 * 50 * 25 / 2 = 1250.
        Assert.Equal(1250D, Area(result));
        Assert.Equal(new Vertex(25, 25), result.GetBoundingBox().Min);
        Assert.Equal(new Vertex(75, 75), result.GetBoundingBox().Max);
        AssertCoverage(
            nameof(this.DirectClipping_SelfIntersection_PreservesBothNonZeroLobes),
            string.Empty,
            string.Empty,
            subject,
            clip,
            result,
            BooleanOperation.Intersection,
            FillRule.NonZero,
            FillRule.EvenOdd);
    }

    /// <summary>
    /// Checks that input ownership metadata cannot override the selected fill rule.
    /// </summary>
    /// <param name="rule">The input fill rule.</param>
    /// <param name="expectedArea">The analytically expected filled area.</param>
    [Theory]
    [InlineData(FillRule.EvenOdd, 64D)]
    [InlineData(FillRule.NonZero, 100D)]
    [InlineData(FillRule.Positive, 100D)]
    [InlineData(FillRule.Negative, 0D)]
    public void Cleanup_InputHierarchy_DoesNotOverrideWinding(FillRule rule, double expectedArea)
    {
        Polygon input = [Rectangle(0, 0, 10, 10), Rectangle(2, 2, 8, 8)];
        input[0].AddHoleIndex(1);
        input[1].ParentIndex = 0;
        input[1].Depth = 1;

        Polygon cleaned = PolygonClipper.Normalize(input, rule);

        Assert.Equal(expectedArea, Area(cleaned));
        Assert.Equal(0, input[1].ParentIndex);
        Assert.Equal(1, input[0].GetHoleIndex(0));
        Assert.Equal(4, input[0].Count);
        Assert.Equal(4, input[1].Count);
    }

    /// <summary>
    /// Checks dense self-crossings through the established star input generator.
    /// </summary>
    /// <param name="vertexCount">The number of vertices in each star.</param>
    [Theory]
    [InlineData(31)]
    [InlineData(101)]
    [InlineData(301)]
    public void DenseStars_ExplicitRules_MatchSourceCoverage(int vertexCount)
    {
        Polygon subject = GenericTestCases.BuildStarPolygon(vertexCount, 100D, 0D, 0D);
        Polygon clip = GenericTestCases.BuildStarPolygon(vertexCount, 100D, 35D, 22D);
        for (int rule = 0; rule < 4; rule++)
        {
            for (int operation = 0; operation < 4; operation++)
            {
                Polygon result = Execute(subject, clip, (BooleanOperation)operation, (FillRule)rule, (FillRule)rule);
                AssertCoverage(
                    nameof(this.DenseStars_ExplicitRules_MatchSourceCoverage),
                    string.Empty,
                    $"VertexCount-{vertexCount}",
                    subject,
                    clip,
                    result,
                    (BooleanOperation)operation,
                    (FillRule)rule,
                    (FillRule)rule);
            }
        }
    }

    /// <summary>
    /// Checks both small and large double coordinate scales without integer conversion.
    /// </summary>
    /// <param name="scale">The coordinate scale.</param>
    [Theory]
    [InlineData(1E-90)]
    [InlineData(1E90)]
    public void Normalize_ScaledCrossing_PreservesFilledArea(double scale)
    {
        Polygon input = [[new Vertex(0, 0), new Vertex(10 * scale, 10 * scale), new Vertex(10 * scale, 0), new Vertex(0, 10 * scale)]];

        Polygon result = PolygonClipper.Normalize(input, FillRule.NonZero);

        Assert.Equal(50D, Area(result) / (scale * scale), 10);
    }

    /// <summary>
    /// Checks that empty-operand shortcuts cannot return unresolved duplicate contours.
    /// </summary>
    /// <param name="rule">The subject fill rule.</param>
    /// <param name="expected">The expected filled area.</param>
    [Theory]
    [InlineData(FillRule.EvenOdd, 0D)]
    [InlineData(FillRule.NonZero, 16D)]
    [InlineData(FillRule.Positive, 16D)]
    [InlineData(FillRule.Negative, 0D)]
    public void EmptyClippingOperand_PreservesSelectedFillRule(FillRule rule, double expected)
    {
        Polygon input = [Rectangle(0, 0, 4, 4), Rectangle(0, 0, 4, 4)];

        Assert.Equal(expected, Area(PolygonClipper.Union(input, [], rule)));
        Assert.Equal(expected, Area(PolygonClipper.Difference(input, [], rule)));
        Assert.Equal(expected, Area(PolygonClipper.Xor(input, [], rule)));
        Assert.Empty(PolygonClipper.Intersection(input, [], rule));
    }

    /// <summary>
    /// Checks raw geographic contours at the scales used by the benchmark suite.
    /// </summary>
    /// <param name="file">The geographic benchmark fixture.</param>
    [Theory]
    [InlineData("hole_hole.geojson")]
    [InlineData("states_source.geojson")]
    [InlineData("asia.geojson")]
    public void GeographicFixtures_MixedRules_MatchSourceCoverage(string file)
    {
        (Polygon subject, Polygon clip) = TestPolygonUtilities.BuildPolygon(TestData.Benchmarks.GetFeatureCollection(file));
        for (int operation = 0; operation < 4; operation++)
        {
            for (int rule = 0; rule < 2; rule++)
            {
                Polygon result = Execute(subject, clip, (BooleanOperation)operation, (FillRule)rule, FillRule.NonZero);
                AssertCoverage(
                    nameof(this.GeographicFixtures_MixedRules_MatchSourceCoverage),
                    Path.GetFileNameWithoutExtension(file),
                    string.Empty,
                    subject,
                    clip,
                    result,
                    (BooleanOperation)operation,
                    (FillRule)rule,
                    FillRule.NonZero);
            }
        }
    }

    /// <summary>
    /// Checks the established intersection defect corpus under independent fill rules.
    /// </summary>
    /// <param name="testCaseFile">The existing GeoJSON defect fixture.</param>
    [Theory]
    [MemberData(nameof(GenericTestCases.GetTestCases), MemberType = typeof(GenericTestCases))]
    public void ExistingDefectFixtures_ExplicitRules_MatchSourceCoverage(string testCaseFile)
    {
        (Polygon subject, Polygon clip) = TestPolygonUtilities.BuildPolygon(TestData.Generic.GetFeatureCollection(testCaseFile));
        for (int operation = 0; operation < 4; operation++)
        {
            for (int subjectRule = 0; subjectRule < 4; subjectRule++)
            {
                for (int clipRule = 0; clipRule < 4; clipRule++)
                {
                    Polygon result = Execute(subject, clip, (BooleanOperation)operation, (FillRule)subjectRule, (FillRule)clipRule);
                    AssertCoverage(
                        nameof(this.ExistingDefectFixtures_ExplicitRules_MatchSourceCoverage),
                        Path.GetFileNameWithoutExtension(testCaseFile),
                        string.Empty,
                        subject,
                        clip,
                        result,
                        (BooleanOperation)operation,
                        (FillRule)subjectRule,
                        (FillRule)clipRule);
                }
            }
        }
    }

    /// <summary>
    /// Checks arbitrary directed crossings against the independent source winding oracle.
    /// </summary>
    [Fact]
    public void SeededSelfCrossingContours_MixedRules_MatchSourceCoverage()
    {
        Random random = new(6492);
        for (int sample = 0; sample < 1024; sample++)
        {
            Polygon subject = [RandomContour(random), RandomContour(random)];
            Polygon clip = [RandomContour(random)];
            for (int subjectRule = 0; subjectRule < 4; subjectRule++)
            {
                for (int clipRule = 0; clipRule < 4; clipRule++)
                {
                    for (int operation = 0; operation < 4; operation++)
                    {
                        Polygon result = Execute(subject, clip, (BooleanOperation)operation, (FillRule)subjectRule, (FillRule)clipRule);
                        AssertCoverage(
                            nameof(this.SeededSelfCrossingContours_MixedRules_MatchSourceCoverage),
                            string.Empty,
                            string.Empty,
                            subject,
                            clip,
                            result,
                            (BooleanOperation)operation,
                            (FillRule)subjectRule,
                            (FillRule)clipRule);
                    }
                }
            }
        }
    }

    /// <summary>
    /// Constructs a deterministic closed contour with unconstrained direction and crossings.
    /// </summary>
    /// <param name="random">The seeded generator.</param>
    /// <returns>The raw directed contour.</returns>
    private static Contour RandomContour(Random random)
    {
        int count = random.Next(3, 8);
        Contour contour = new(count);
        for (int i = 0; i < count; i++)
        {
            contour.Add(new Vertex(random.Next(-4, 5), random.Next(-4, 5)));
        }

        return contour;
    }

    /// <summary>
    /// Enumerates independent operand rules and all Boolean operations.
    /// </summary>
    /// <returns>The geometric case and operation combinations.</returns>
    public static TheoryData<int, BooleanOperation, FillRule, FillRule> Combinations()
    {
        TheoryData<int, BooleanOperation, FillRule, FillRule> data = [];
        for (int geometry = 0; geometry < 7; geometry++)
        {
            for (int operation = 0; operation < 4; operation++)
            {
                for (int subject = 0; subject < 4; subject++)
                {
                    for (int clip = 0; clip < 4; clip++)
                    {
                        data.Add(geometry, (BooleanOperation)operation, (FillRule)subject, (FillRule)clip);
                    }
                }
            }
        }

        return data;
    }

    /// <summary>
    /// Distinguishes raw signed winding from nesting-based hole preparation.
    /// </summary>
    /// <param name="rule">The input fill rule.</param>
    /// <param name="area">The analytically expected area.</param>
    [Theory]
    [InlineData(FillRule.EvenOdd, 64D)]
    [InlineData(FillRule.NonZero, 100D)]
    [InlineData(FillRule.Positive, 100D)]
    [InlineData(FillRule.Negative, 0D)]
    public void Normalize_SameDirectionNestedRings_UsesSelectedRule(FillRule rule, double area)
    {
        Polygon input = [Rectangle(0, 0, 10, 10), Rectangle(2, 2, 8, 8)];

        Polygon result = PolygonClipper.Normalize(input, rule);

        Assert.Equal(area, Area(result), 10);
        Assert.Equal(2, input.Count);
        Assert.Equal(4, input[0].Count);
        Assert.Null(input[1].ParentIndex);
    }

    /// <summary>
    /// Retains both winding signs of a zero-net-area self-crossing contour.
    /// </summary>
    /// <param name="rule">The input fill rule.</param>
    /// <param name="area">The analytically expected area.</param>
    [Theory]
    [InlineData(FillRule.EvenOdd, 50D)]
    [InlineData(FillRule.NonZero, 50D)]
    [InlineData(FillRule.Positive, 25D)]
    [InlineData(FillRule.Negative, 25D)]
    public void Normalize_Bowtie_ResolvesBothWindingSigns(FillRule rule, double area)
    {
        Polygon input = [[new Vertex(0, 0), new Vertex(10, 10), new Vertex(10, 0), new Vertex(0, 10)]];

        Polygon result = PolygonClipper.Normalize(input, rule);

        Assert.Equal(area, Area(result), 10);
    }

    /// <summary>
    /// Checks binary output against direct winding of the original, unsplit contours.
    /// </summary>
    /// <param name="geometry">The directed input configuration.</param>
    /// <param name="operation">The Boolean operation.</param>
    /// <param name="subjectRule">The subject rule.</param>
    /// <param name="clipRule">The clipping rule.</param>
    [Theory]
    [MemberData(nameof(Combinations))]
    public void BinaryOperations_MixedRules_MatchOriginalWinding(
        int geometry, BooleanOperation operation, FillRule subjectRule, FillRule clipRule)
    {
        (Polygon subject, Polygon clip) = Inputs(geometry);
        Polygon result = Execute(subject, clip, operation, subjectRule, clipRule);

        AssertCoverage(
            nameof(this.BinaryOperations_MixedRules_MatchOriginalWinding),
            string.Empty,
            $"Geometry-{geometry}",
            subject,
            clip,
            result,
            operation,
            subjectRule,
            clipRule);
    }

    /// <summary>
    /// Checks sign reversal and rule changes between manual and pooled calls.
    /// </summary>
    [Fact]
    public void ManualClipperAndPooledCalls_DoNotLeakRules()
    {
        Polygon subject = [Rectangle(0, 0, 10, 10)];
        subject[0].Reverse();
        PolygonClipper manual = new(subject, [], BooleanOperation.Union, FillRule.Negative, FillRule.EvenOdd);
        Assert.Equal(100D, Area(manual.Run()));
        Assert.Equal(100D, Area(manual.Run()));
        Assert.Equal(0D, Area(PolygonClipper.Normalize(subject, FillRule.Positive)));
        Assert.Equal(100D, Area(PolygonClipper.Normalize(subject, FillRule.Negative)));

        Polygon evenOdd = PolygonClipper.Union(subject, [], FillRule.EvenOdd);
        Assert.Equal(subject[0].Count, evenOdd[0].Count);
        Assert.True(evenOdd[0].IsCounterClockwise());
        Assert.NotSame(subject[0], evenOdd[0]);
    }

    /// <summary>
    /// Checks double-coordinate behavior without integer scaling.
    /// </summary>
    [Fact]
    public void Normalize_TranslatedUnitRings_PreservesSmallArea()
    {
        const double origin = 1E12;
        Polygon input = [Rectangle(origin, origin, origin + 1, origin + 1)];

        Polygon result = PolygonClipper.Normalize(input, FillRule.NonZero);

        Assert.Equal(1D, Area(result));
        Assert.Equal(origin, result[0].GetBoundingBox().Min.X);
    }

    /// <summary>
    /// Checks output against signed source coverage away from input boundaries.
    /// </summary>
    /// <param name="testName">The test method name.</param>
    /// <param name="sourceName">The input file name without an extension, or an empty string.</param>
    /// <param name="inputDetails">The invariant input parameter details, or an empty string.</param>
    /// <param name="subject">The original subject contours.</param>
    /// <param name="clip">The original clipping contours.</param>
    /// <param name="result">The clipped geometry.</param>
    /// <param name="operation">The Boolean operation.</param>
    /// <param name="subjectRule">The subject rule.</param>
    /// <param name="clipRule">The clipping rule.</param>
    private static void AssertCoverage(
        string testName,
        string sourceName,
        string inputDetails,
        Polygon subject,
        Polygon clip,
        Polygon result,
        BooleanOperation operation,
        FillRule subjectRule,
        FillRule clipRule)
    {
        // Generated samples replace this test's current result. A failure leaves the failing geometry available.
        string group = sourceName.Length == 0
            ? nameof(FillRuleClippingTests)
            : Path.Combine(nameof(FillRuleClippingTests), sourceName);

        string details = $"Operation-{operation}_SubjectRule-{subjectRule}_ClipRule-{clipRule}";
        if (inputDetails.Length > 0)
        {
            details = $"{inputDetails}_{details}";
        }

        string svgPath = SvgTestOutput.SaveSvg(group, testName, details, subject, clip, result);

        Box2 bounds = subject.GetBoundingBox().Add(clip.GetBoundingBox());
        for (int y = 0; y < 9; y++)
        {
            for (int x = 0; x < 9; x++)
            {
                Vertex point = new(
                    bounds.Min.X + ((bounds.Max.X - bounds.Min.X) * (x + 0.25D) / 9D),
                    bounds.Min.Y + ((bounds.Max.Y - bounds.Min.Y) * (y + 0.375D) / 9D));
                if (OnBoundary(subject, point) || OnBoundary(clip, point))
                {
                    continue;
                }

                bool first = Filled(Winding(subject, point), subjectRule);
                bool second = Filled(Winding(clip, point), clipRule);
                bool expected = operation switch
                {
                    BooleanOperation.Intersection => first && second,
                    BooleanOperation.Union => first || second,
                    BooleanOperation.Difference => first && !second,
                    _ => first ^ second
                };
                bool actual = Winding(result, point) != 0;
                if (expected != actual)
                {
                    Assert.Fail($"{operation}, {subjectRule}/{clipRule}, point {point}: expected {expected}.\nSVG: {svgPath}" +
                        $"\nSubject: {subject.ToDebugString()}\nClip: {clip.ToDebugString()}\nResult: {result.ToDebugString()}");
                }
            }
        }

        for (int i = 0; i < result.Count; i++)
        {
            Contour contour = result[i];
            Assert.NotEqual(contour[0], contour[^1]);
            Assert.Equal(contour.IsExternal, contour.IsCounterClockwise());
            for (int j = 0; j < contour.HoleCount; j++)
            {
                Assert.Equal(i, result[contour.GetHoleIndex(j)].ParentIndex);
            }
        }
    }

    /// <summary>
    /// Builds raw directed input cases with crossings, vertical runs, and duplicates.
    /// </summary>
    /// <param name="geometry">The requested case.</param>
    /// <returns>The subject and clipping contours.</returns>
    private static (Polygon Subject, Polygon Clip) Inputs(int geometry)
    {
        Polygon subject = geometry switch
        {
            0 => [Rectangle(0, 0, 10, 10), Rectangle(2, 2, 8, 8)],
            1 => [Rectangle(0, 0, 10, 10), Rectangle(0, 2, 8, 8)],
            2 => [[new Vertex(0, 0), new Vertex(10, 10), new Vertex(10, 0), new Vertex(0, 10)]],
            3 => [Rectangle(0, 0, 10, 10), Rectangle(0, 0, 10, 10)],
            4 => [Rectangle(0, 0, 10, 10), Rectangle(2, 2, 8, 8), Rectangle(3, 3, 7, 7)],
            5 => [Rectangle(0, 0, 4, 4), Rectangle(4, 4, 8, 8)],
            _ => [Rectangle(0, 0, 10, 10), Rectangle(0, 0, 10, 10)]
        };
        if (geometry is 4 or 6)
        {
            subject[1].Reverse();
        }

        Polygon clip = [Rectangle(0, 3, 6, 7), Rectangle(2, 1, 9, 9)];
        clip[1].Reverse();
        return (subject, clip);
    }

    /// <summary>
    /// Executes the selected public binary operation.
    /// </summary>
    /// <param name="subject">The subject.</param>
    /// <param name="clip">The clipping contours.</param>
    /// <param name="operation">The Boolean operation.</param>
    /// <param name="subjectRule">The subject rule.</param>
    /// <param name="clipRule">The clipping rule.</param>
    /// <returns>The resulting geometry.</returns>
    private static Polygon Execute(Polygon subject, Polygon clip, BooleanOperation operation, FillRule subjectRule, FillRule clipRule)
    {
        try
        {
            return operation switch
            {
                BooleanOperation.Intersection => PolygonClipper.Intersection(subject, clip, subjectRule, clipRule),
                BooleanOperation.Union => PolygonClipper.Union(subject, clip, subjectRule, clipRule),
                BooleanOperation.Difference => PolygonClipper.Difference(subject, clip, subjectRule, clipRule),
                _ => PolygonClipper.Xor(subject, clip, subjectRule, clipRule)
            };
        }
        catch (Exception exception)
        {
            Assert.Fail($"{operation}, {subjectRule}/{clipRule}: {exception}\nSubject: {subject.ToDebugString()}\nClip: {clip.ToDebugString()}");
            throw;
        }
    }

    /// <summary>
    /// Constructs a positively directed rectangle.
    /// </summary>
    /// <param name="left">The left coordinate.</param>
    /// <param name="top">The top coordinate.</param>
    /// <param name="right">The right coordinate.</param>
    /// <param name="bottom">The bottom coordinate.</param>
    /// <returns>The four directed vertices.</returns>
    private static Contour Rectangle(double left, double top, double right, double bottom)
        => [new Vertex(left, top), new Vertex(right, top), new Vertex(right, bottom), new Vertex(left, bottom)];

    /// <summary>
    /// Evaluates the documented winding predicates independently of the clipper.
    /// </summary>
    /// <param name="winding">The winding count.</param>
    /// <param name="rule">The selected rule.</param>
    /// <returns>The filled membership.</returns>
    private static bool Filled(int winding, FillRule rule)
        => rule switch
        {
            FillRule.EvenOdd => (winding % 2) != 0,
            FillRule.NonZero => winding != 0,
            FillRule.Positive => winding > 0,
            _ => winding < 0
        };

    /// <summary>
    /// Counts signed horizontal-ray crossings on the original contours.
    /// </summary>
    /// <param name="polygon">The directed contours.</param>
    /// <param name="point">A point away from input boundaries.</param>
    /// <returns>The signed winding count.</returns>
    private static int Winding(Polygon polygon, Vertex point)
    {
        int winding = 0;
        foreach (Contour contour in polygon)
        {
            for (int i = 0; i < contour.Count; i++)
            {
                Vertex first = contour[i];
                Vertex second = contour[(i + 1) % contour.Count];
                double side = Vertex.Cross(second - first, point - first);
                if (first.Y <= point.Y && second.Y > point.Y && side > 0)
                {
                    winding++;
                }
                else if (first.Y > point.Y && second.Y <= point.Y && side < 0)
                {
                    winding--;
                }
            }
        }

        return winding;
    }

    /// <summary>
    /// Excludes points whose membership is boundary-convention dependent.
    /// </summary>
    /// <param name="polygon">The input contours.</param>
    /// <param name="point">The sampled point.</param>
    /// <returns>Whether the point lies on a source segment.</returns>
    private static bool OnBoundary(Polygon polygon, Vertex point)
    {
        foreach (Contour contour in polygon)
        {
            for (int i = 0; i < contour.Count; i++)
            {
                Vertex first = contour[i];
                Vertex second = contour[(i + 1) % contour.Count];
                Vertex direction = second - first;
                Vertex offset = point - first;

                // Grid interpolation and intersection construction can place a
                // boundary sample a few rounding units on either side. Coverage
                // is asserted only where that boundary convention is immaterial.
                double scale = Math.Max(
                    Math.Max(Math.Abs(first.X), Math.Abs(first.Y)),
                    Math.Max(Math.Max(Math.Abs(second.X), Math.Abs(second.Y)), Math.Max(Math.Abs(point.X), Math.Abs(point.Y))));
                double tolerance = 16 * 2.2204460492503131E-16 * scale;
                double crossTolerance = tolerance * (Math.Abs(direction.X) + Math.Abs(direction.Y));
                if (Math.Abs(Vertex.Cross(direction, offset)) <= crossTolerance &&
                    point.X >= Math.Min(first.X, second.X) - tolerance && point.X <= Math.Max(first.X, second.X) + tolerance &&
                    point.Y >= Math.Min(first.Y, second.Y) - tolerance && point.Y <= Math.Max(first.Y, second.Y) + tolerance)
                {
                    return true;
                }
            }
        }

        return false;
    }

    /// <summary>
    /// Measures signed filled area using a local origin for each result ring.
    /// </summary>
    /// <param name="polygon">The result geometry.</param>
    /// <returns>The total signed area.</returns>
    private static double Area(Polygon polygon)
    {
        double twiceArea = 0;
        foreach (Contour contour in polygon)
        {
            Vertex origin = contour[0];
            for (int i = 0; i < contour.Count; i++)
            {
                twiceArea += Vertex.Cross(contour[i] - origin, contour[(i + 1) % contour.Count] - origin);
            }
        }

        return twiceArea * 0.5D;
    }
}

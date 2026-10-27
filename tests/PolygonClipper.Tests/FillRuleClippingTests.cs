// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

namespace SixLabors.PolygonClipper.Tests;

/// <summary>Checks filled geometry through the public single-sweep APIs.</summary>
public class FillRuleClippingTests
{
    /// <summary>Checks dense self-crossings through the established star input generator.</summary>
    /// <param name="vertexCount">The number of vertices in each star.</param>
    [Theory]
    [InlineData(31)]
    [InlineData(101)]
    public void DenseStars_ExplicitRules_MatchSourceCoverage(int vertexCount)
    {
        Polygon subject = GenericTestCases.BuildStarPolygon(vertexCount, 100D, 0D, 0D);
        Polygon clip = GenericTestCases.BuildStarPolygon(vertexCount, 100D, 35D, 22D);
        for (int rule = 0; rule < 4; rule++)
        {
            for (int operation = 0; operation < 4; operation++)
            {
                Polygon result = Execute(subject, clip, (BooleanOperation)operation, (FillRule)rule, (FillRule)rule);
                AssertCoverage(subject, clip, result, (BooleanOperation)operation, (FillRule)rule, (FillRule)rule);
            }
        }
    }

    /// <summary>Checks both small and large double coordinate scales without integer conversion.</summary>
    /// <param name="scale">The coordinate scale.</param>
    [Theory]
    [InlineData(1E-90)]
    [InlineData(1E90)]
    public void UnaryUnion_ScaledCrossing_PreservesFilledArea(double scale)
    {
        Polygon input = [[new Vertex(0, 0), new Vertex(10 * scale, 10 * scale), new Vertex(10 * scale, 0), new Vertex(0, 10 * scale)]];

        Polygon result = PolygonClipper.Union(input, FillRule.NonZero);

        Assert.Equal(50D, Area(result) / (scale * scale), 10);
    }

    /// <summary>Checks that empty-operand shortcuts cannot return unresolved duplicate contours.</summary>
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

    /// <summary>Checks raw geographic contours at the scales used by the benchmark suite.</summary>
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
                AssertCoverage(subject, clip, result, (BooleanOperation)operation, (FillRule)rule, FillRule.NonZero);
            }
        }
    }

    /// <summary>Checks the established intersection defect corpus under independent fill rules.</summary>
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
                    AssertCoverage(subject, clip, result, (BooleanOperation)operation, (FillRule)subjectRule, (FillRule)clipRule);
                }
            }
        }
    }

    /// <summary>Checks arbitrary directed crossings against the independent source winding oracle.</summary>
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
                        AssertCoverage(subject, clip, result, (BooleanOperation)operation, (FillRule)subjectRule, (FillRule)clipRule);
                    }
                }
            }
        }
    }

    /// <summary>Constructs a deterministic closed contour with unconstrained direction and crossings.</summary>
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

    /// <summary>Enumerates independent operand rules and all Boolean operations.</summary>
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

    /// <summary>Distinguishes raw signed winding from nesting-based hole preparation.</summary>
    /// <param name="rule">The input fill rule.</param>
    /// <param name="area">The analytically expected area.</param>
    [Theory]
    [InlineData(FillRule.EvenOdd, 64D)]
    [InlineData(FillRule.NonZero, 100D)]
    [InlineData(FillRule.Positive, 100D)]
    [InlineData(FillRule.Negative, 0D)]
    public void UnaryUnion_SameDirectionNestedRings_UsesSelectedRule(FillRule rule, double area)
    {
        Polygon input = [Rectangle(0, 0, 10, 10), Rectangle(2, 2, 8, 8)];

        Polygon result = PolygonClipper.Union(input, rule);

        Assert.Equal(area, Area(result), 10);
        Assert.Equal(2, input.Count);
        Assert.Equal(4, input[0].Count);
        Assert.Null(input[1].ParentIndex);
    }

    /// <summary>Retains both winding signs of a zero-net-area self-crossing contour.</summary>
    /// <param name="rule">The input fill rule.</param>
    /// <param name="area">The analytically expected area.</param>
    [Theory]
    [InlineData(FillRule.EvenOdd, 50D)]
    [InlineData(FillRule.NonZero, 50D)]
    [InlineData(FillRule.Positive, 25D)]
    [InlineData(FillRule.Negative, 25D)]
    public void UnaryUnion_Bowtie_ResolvesBothWindingSigns(FillRule rule, double area)
    {
        Polygon input = [[new Vertex(0, 0), new Vertex(10, 10), new Vertex(10, 0), new Vertex(0, 10)]];

        Polygon result = PolygonClipper.Union(input, rule);

        Assert.Equal(area, Area(result), 10);
    }

    /// <summary>Checks binary output against direct winding of the original, unsplit contours.</summary>
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

        AssertCoverage(subject, clip, result, operation, subjectRule, clipRule);
    }

    /// <summary>Checks sign reversal, manual reuse, and the preserved legacy overload path.</summary>
    [Fact]
    public void ManualClipperAndPooledLegacyCalls_DoNotLeakRules()
    {
        Polygon subject = [Rectangle(0, 0, 10, 10)];
        subject[0].Reverse();
        PolygonClipper manual = new(subject, [], BooleanOperation.Union, FillRule.Negative, FillRule.EvenOdd);
        Assert.Equal(100D, Area(manual.Run()));
        Assert.Equal(100D, Area(manual.Run()));
        Assert.Equal(0D, Area(PolygonClipper.Union(subject, FillRule.Positive)));
        Assert.Equal(100D, Area(PolygonClipper.Union(subject, FillRule.Negative)));

        Polygon legacy = PolygonClipper.Union(subject, []);
        Assert.Equal(subject[0].Count, legacy[0].Count);
        Assert.False(legacy[0].IsCounterClockwise());
        Assert.NotSame(subject[0], legacy[0]);
    }

    /// <summary>Checks double-coordinate behavior without integer scaling.</summary>
    [Fact]
    public void UnaryUnion_TranslatedUnitRings_PreservesSmallArea()
    {
        const double origin = 1E12;
        Polygon input = [Rectangle(origin, origin, origin + 1, origin + 1)];

        Polygon result = PolygonClipper.Union(input, FillRule.NonZero);

        Assert.Equal(1D, Area(result));
        Assert.Equal(origin, result[0].GetBoundingBox().Min.X);
    }

    /// <summary>Checks output against signed source coverage away from input boundaries.</summary>
    /// <param name="subject">The original subject contours.</param>
    /// <param name="clip">The original clipping contours.</param>
    /// <param name="result">The clipped geometry.</param>
    /// <param name="operation">The Boolean operation.</param>
    /// <param name="subjectRule">The subject rule.</param>
    /// <param name="clipRule">The clipping rule.</param>
    private static void AssertCoverage(Polygon subject, Polygon clip, Polygon result, BooleanOperation operation, FillRule subjectRule, FillRule clipRule)
    {
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
                    Assert.Fail($"{operation}, {subjectRule}/{clipRule}, point {point}: expected {expected}." +
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

    /// <summary>Builds raw directed input cases with crossings, vertical runs, and duplicates.</summary>
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

    /// <summary>Executes the selected public binary operation.</summary>
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

    /// <summary>Constructs a positively directed rectangle.</summary>
    /// <param name="left">The left coordinate.</param>
    /// <param name="top">The top coordinate.</param>
    /// <param name="right">The right coordinate.</param>
    /// <param name="bottom">The bottom coordinate.</param>
    /// <returns>The four directed vertices.</returns>
    private static Contour Rectangle(double left, double top, double right, double bottom)
        => [new Vertex(left, top), new Vertex(right, top), new Vertex(right, bottom), new Vertex(left, bottom)];

    /// <summary>Evaluates the documented winding predicates independently of the clipper.</summary>
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

    /// <summary>Counts signed horizontal-ray crossings on the original contours.</summary>
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

    /// <summary>Excludes points whose membership is boundary-convention dependent.</summary>
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

    /// <summary>Measures signed filled area using a local origin for each result ring.</summary>
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

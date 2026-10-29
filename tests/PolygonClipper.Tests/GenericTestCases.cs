// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using GeoJson.Feature;
using GeoJson.Geometry;

namespace SixLabors.PolygonClipper.Tests;

public class GenericTestCases
{
    private readonly ITestOutputHelper testOutputHelper;

    public GenericTestCases(ITestOutputHelper testOutputHelper) => this.testOutputHelper = testOutputHelper;

    public static TheoryData<string> GetTestCases()
        => [.. TestData.Generic.GetFileNames()];

    [Theory]
    [MemberData(nameof(GetTestCases))]
    public void GenericTestCase(string testCaseFile)
    {
        // Arrange
        FeatureCollection data = TestData.Generic.GetFeatureCollection(testCaseFile);

        Assert.True(data.Features.Count >= 2, "Test case file must contain at least two features.");

        IGeometryObject subjectGeometry = data.Features[0].Geometry;
        IGeometryObject clippingGeometry = data.Features[1].Geometry;

        (Polygon subject, Polygon clipping) = TestPolygonUtilities.BuildPolygon(data);

#pragma warning disable RCS1124 // Inline local variable
        List<ExpectedResult> expectedResults = ExtractExpectedResults([.. data.Features.Skip(2)]);
#pragma warning restore RCS1124 // Inline local variable

        foreach (ExpectedResult result in expectedResults)
        {
            Polygon expected = result.Coordinates;
            Polygon actual = result.Operation(subject, clipping);

            Assert.Equal(expected.Count, actual.Count);
            for (int i = 0; i < expected.Count; i++)
            {
                this.testOutputHelper.WriteLine($"Current Contour {i}");

                Contour reference = expected[i];
                Assert.Equal(reference[0], reference[^1]);

                // GeoJSON stores a closing vertex. V2 returns each ring without it.
                int count = reference.Count - 1;
                Assert.Equal(count, actual[i].Count);
                Assert.NotEqual(actual[i][0], actual[i][^1]);
                Assert.Equal(reference.IsExternal, actual[i].IsExternal);

                // Derive reference direction from its coordinates and GeoJSON ring
                // role, without using the production orientation calculation.
                double area = 0;
                Vertex origin = reference[0];
                for (int j = 1; j < count - 1; j++)
                {
                    Vertex first = reference[j];
                    Vertex second = reference[j + 1];
                    area += ((first.X - origin.X) * (second.Y - origin.Y)) -
                            ((first.Y - origin.Y) * (second.X - origin.X));
                }

                bool reverse = (area >= 0) != reference.IsExternal;
                for (int j = 0; j < count; j++)
                {
                    Vertex expectedVertex = reference[reverse ? count - 1 - j : j];
                    Vertex actualVertex = actual[i][j];
                    Assert.Equal(expectedVertex.X, actualVertex.X, 3);
                    Assert.Equal(expectedVertex.Y, actualVertex.Y, 3);
                }
            }
        }
    }

    /// <summary>
    /// Reads the reference operations and their GeoJSON geometry.
    /// </summary>
    /// <param name="features">The reference operations and their GeoJSON output.</param>
    /// <returns>The operations and their expected contours.</returns>
    private static List<ExpectedResult> ExtractExpectedResults(List<Feature> features)
        => features.ConvertAll(feature =>
        {
            string mode = feature.Properties["operation"]?.ToString();
            Func<Polygon, Polygon, Polygon> operation = mode switch
            {
                "union" => (a, b) => PolygonClipper.Union(a, b, FillRule.EvenOdd),
                "intersection" => (a, b) => PolygonClipper.Intersection(a, b, FillRule.EvenOdd),
                "xor" => (a, b) => PolygonClipper.Xor(a, b, FillRule.EvenOdd),
                "diff" => (a, b) => PolygonClipper.Difference(a, b, FillRule.EvenOdd),
                "diff_ba" => (a, b) => PolygonClipper.Difference(b, a, FillRule.EvenOdd),
                _ => throw new InvalidOperationException($"Invalid mode: {mode}")
            };

            return new ExpectedResult
            {
                Operation = operation,
                Coordinates = TestPolygonUtilities.ConvertToPolygon(feature.Geometry)
            };
        });

    private sealed class ExpectedResult
    {
        public Func<Polygon, Polygon, Polygon> Operation { get; set; }

        public Polygon Coordinates { get; set; }
    }

    private enum TestType
    {
        Polygon = 0,

        MultiPolygon = 1
    }

    [Fact(Skip = "Used for performance tracing only.")]
    public void StarBenchmark()
    {
        Polygon subject = BuildStarPolygon(1001, 100D, 0D, 0D);
        Polygon clipping = BuildStarPolygon(1001, 100D, 35D, 22D);
        Polygon result = PolygonClipper.Union(subject, clipping, FillRule.EvenOdd);
    }

    /// <summary>
    /// Constructs the self-crossing star used by the clipping stress cases.
    /// </summary>
    /// <param name="vertexCount">The odd vertex count.</param>
    /// <param name="radius">The distance from the centre to each vertex.</param>
    /// <param name="centerX">The horizontal centre.</param>
    /// <param name="centerY">The vertical centre.</param>
    /// <returns>The directed star contour.</returns>
    internal static Polygon BuildStarPolygon(int vertexCount, double radius, double centerX, double centerY)
    {
        if (vertexCount < 5 || (vertexCount & 1) == 0)
        {
            throw new ArgumentOutOfRangeException(nameof(vertexCount), "Vertex count must be an odd number >= 5.");
        }

        int step = (vertexCount - 1) / 2;
        Contour contour = new(vertexCount + 1);
        for (int i = 0; i < vertexCount; i++)
        {
            int index = (i * step) % vertexCount;
            double angle = (index * Math.PI * 2D) / vertexCount;
            double x = centerX + (Math.Cos(angle) * radius);
            double y = centerY + (Math.Sin(angle) * radius);
            contour.Add(new Vertex(x, y));
        }

        contour.Add(contour[0]);
        Polygon polygon = [contour];
        return polygon;
    }
}

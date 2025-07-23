// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using GeoJson;
using GeoJson.Feature;
using GeoJson.Geometry;
using Xunit.Abstractions;
using GeoPolygon = GeoJson.Geometry.Polygon;

namespace SixLabors.PolygonClipper.Tests;

public class GenericTestCases
{
    private readonly ITestOutputHelper testOutputHelper;

    public GenericTestCases(ITestOutputHelper testOutputHelper) => this.testOutputHelper = testOutputHelper;

    public static TheoryData<string> GetTestCases()
        => new(TestData.Generic.GetFileNames());

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
        List<ExpectedResult> expectedResults = ExtractExpectedResults([.. data.Features.Skip(2)], data.Type);
#pragma warning restore RCS1124 // Inline local variable

        foreach (ExpectedResult result in expectedResults)
        {
            Polygon expected = result.Coordinates;
            Polygon actual = result.Operation(subject, clipping);

            Assert.Equal(expected.Count, actual.Count);
            for (int i = 0; i < expected.Count; i++)
            {
                // We don't test for holes here as the reference tests do not do so.
                this.testOutputHelper.WriteLine($"Current Contour {i}");

                Assert.Equal(expected[i].Count, actual[i].Count);
                for (int j = 0; j < expected[i].Count; j++)
                {
                    Vertex expectedVertex = expected[i][j];
                    Vertex actualVertex = actual[i][j];
                    Assert.Equal(expectedVertex.X, actualVertex.X, 3);
                    Assert.Equal(expectedVertex.Y, actualVertex.Y, 3);
                }
            }
        }
    }

    private static List<ExpectedResult> ExtractExpectedResults(List<Feature> features, GeoJSONObjectType type)
        => features.ConvertAll(feature =>
        {
            string mode = feature.Properties["operation"]?.ToString();
            Func<Polygon, Polygon, Polygon> operation = mode switch
            {
                "union" => PolygonClipper.Union,
                "intersection" => PolygonClipper.Intersection,
                "xor" => PolygonClipper.Xor,
                "diff" => PolygonClipper.Difference,
                "diff_ba" => (a, b) => PolygonClipper.Difference(b, a),
                _ => throw new InvalidOperationException($"Invalid mode: {mode}")
            };

            if (type == GeoJSONObjectType.Polygon)
            {
                return new ExpectedResult
                {
                    Operation = operation,
                    Coordinates = TestPolygonUtilities.ConvertToPolygon(feature.Geometry as GeoPolygon)
                };
            }

            return new ExpectedResult
            {
                Operation = operation,
                Coordinates = TestPolygonUtilities.ConvertToPolygon(feature.Geometry as MultiPolygon)
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
}

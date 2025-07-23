// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System;
using System.Collections.Generic;
using System.Linq;
using GeoJSON.Text;
using GeoJSON.Text.Feature;
using GeoJSON.Text.Geometry;
using PolygonClipper.Tests.TestCases;
using Xunit;
using Xunit.Abstractions;
using GeoPolygon = GeoJSON.Text.Geometry.Polygon;

namespace PolygonClipper.Tests;

public class GenericTestCases
{
    private readonly ITestOutputHelper testOutputHelper;

    public GenericTestCases(ITestOutputHelper testOutputHelper) => this.testOutputHelper = testOutputHelper;

    public static IEnumerable<object[]> GetTestCases()
        => TestData.Generic.GetFileNames().Select(x => new object[] { x });

    [Fact]
    public Polygon Profile()
    {
        // Use this test for profiling purposes. (Rider)
        FeatureCollection data = TestData.Generic.GetFeatureCollection("issue71.geojson");

        IGeometryObject subjectGeometry = data.Features[0].Geometry;
        IGeometryObject clippingGeometry = data.Features[1].Geometry;

        (Polygon subject, Polygon clipping) = TestPolygonUtilities.BuildPolygon(data);
        return PolygonClipper.Union(subject, clipping);
    }

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

        // ExpectedResult result = expectedResults[1];
        // Polygon actual = result.Operation(subject, clipping);
        // Assert.Equal(result.Coordinates.ContourCount, actual.ContourCount);

        foreach (ExpectedResult result in expectedResults)
        {
            Polygon expected = result.Coordinates;
            Polygon actual = result.Operation(subject, clipping);

            Assert.Equal(expected.Count, actual.Count);
            for (int i = 0; i < expected.Count; i++)
            {
                // We don't test for holes here as the reference tests do not do so.
                this.testOutputHelper.WriteLine($"Current Countour {i}");

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

    private class ExpectedResult
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

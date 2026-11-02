// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

namespace SixLabors.PolygonClipper.Tests;

/// <summary>
/// Checks finite stroke widths and the empty result for zero width.
/// </summary>
public class PolygonStrokerWidthTests
{
    /// <summary>
    /// Checks non-finite widths at the static entry points, including empty input.
    /// </summary>
    /// <param name="width">The non-finite stroke width.</param>
    [Theory]
    [InlineData(double.NaN)]
    [InlineData(double.PositiveInfinity)]
    [InlineData(double.NegativeInfinity)]
    public void Stroke_NonFiniteWidth_IsRejected(double width)
    {
        Polygon polygon = [[new(0, 0), new(10, 0), new(10, 10), new(0, 10), new(0, 0)]];
        StrokeOptions options = new() { LineJoin = LineJoin.Round, LineCap = LineCap.Round, NormalizeOutput = true };

        Assert.Throws<ArgumentOutOfRangeException>("width", () => PolygonStroker.Stroke(polygon, width));
        Assert.Throws<ArgumentOutOfRangeException>("width", () => PolygonStroker.Stroke(polygon, width, options));
        Assert.Throws<ArgumentOutOfRangeException>("width", () => PolygonStroker.Stroke([], width));
        Assert.Throws<ArgumentOutOfRangeException>("width", () => PolygonStroker.Stroke([], width, options));
    }

    /// <summary>
    /// Checks that a rejected width assignment preserves the previous width and geometry.
    /// </summary>
    /// <param name="width">The non-finite stroke width.</param>
    [Theory]
    [InlineData(double.NaN)]
    [InlineData(double.PositiveInfinity)]
    [InlineData(double.NegativeInfinity)]
    public void Width_NonFiniteValue_PreservesPreviousState(double width)
    {
        Polygon polygon = [[new(0, 0), new(10, 0), new(10, 10), new(0, 10), new(0, 0)]];
        PolygonStroker stroker = new(new StrokeOptions()) { Width = 2 };
        string before = stroker.Stroke(polygon).ToDebugString();

        Assert.Throws<ArgumentOutOfRangeException>("value", () => stroker.Width = width);
        Assert.Equal(2D, stroker.Width);
        Assert.Equal(before, stroker.Stroke(polygon).ToDebugString());
    }

    /// <summary>
    /// Checks zero width before outline generation, point caps and optional cleanup.
    /// </summary>
    /// <param name="join">The join style.</param>
    /// <param name="cap">The cap style.</param>
    /// <param name="normalizeOutput">Whether to resolve the generated outlines.</param>
    [Theory]
    [InlineData(LineJoin.Miter, LineCap.Butt, false)]
    [InlineData(LineJoin.Round, LineCap.Round, false)]
    [InlineData(LineJoin.Miter, LineCap.Butt, true)]
    [InlineData(LineJoin.Round, LineCap.Round, true)]
    public void Stroke_ZeroWidth_ReturnsEmpty(LineJoin join, LineCap cap, bool normalizeOutput)
    {
        Polygon[] inputs =
        [
            [],
            [[new(0, 0), new(10, 0), new(10, 10), new(0, 10), new(0, 0)]],
            [[new(0, 0), new(10, 0)]],
            [[new(5, 5), new(5, 5)]]
        ];
        StrokeOptions options = new() { LineJoin = join, LineCap = cap, NormalizeOutput = normalizeOutput };
        PolygonStroker stroker = new(options) { Width = 0 };

        foreach (Polygon input in inputs)
        {
            Assert.Empty(PolygonStroker.Stroke(input, 0D, options));
            Assert.Empty(PolygonStroker.Stroke(input, -0D, options));
            Assert.Empty(stroker.Stroke(input));
        }
    }

    /// <summary>
    /// Checks that zero width does not bypass the null-polygon contract.
    /// </summary>
    [Fact]
    public void Stroke_ZeroWidth_StillRejectsNullPolygon()
    {
        PolygonStroker stroker = new(new StrokeOptions()) { Width = 0 };

        Assert.Throws<ArgumentNullException>("polygon", () => PolygonStroker.Stroke(null, 0));
        Assert.Throws<ArgumentNullException>("polygon", () => stroker.Stroke(null));
    }
}

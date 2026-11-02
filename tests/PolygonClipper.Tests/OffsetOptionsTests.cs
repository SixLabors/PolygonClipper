// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

namespace SixLabors.PolygonClipper.Tests;

/// <summary>
/// Checks the defaults and value comparison of offset settings.
/// </summary>
public class OffsetOptionsTests
{
    /// <summary>
    /// Checks that offset settings retain the established join and arc defaults.
    /// </summary>
    [Fact]
    public void Defaults_AreAsDocumented()
    {
        OffsetOptions options = new();

        Assert.Equal(4D, options.MiterLimit);
        Assert.Equal(1D, options.ArcDetailScale);
        Assert.Equal(LineJoin.Bevel, options.LineJoin);
    }

    /// <summary>
    /// Checks that each setting retains its assigned value.
    /// </summary>
    [Fact]
    public void Properties_RoundTrip()
    {
        OffsetOptions options = new() { MiterLimit = 2.5, ArcDetailScale = 0.5, LineJoin = LineJoin.Round };

        Assert.Equal(2.5, options.MiterLimit);
        Assert.Equal(0.5, options.ArcDetailScale);
        Assert.Equal(LineJoin.Round, options.LineJoin);
    }

    /// <summary>
    /// Checks that value comparison includes every offset setting.
    /// </summary>
    /// <param name="miterLimit">The comparison miter limit.</param>
    /// <param name="arcDetailScale">The comparison arc detail scale.</param>
    /// <param name="lineJoin">The comparison join style.</param>
    /// <param name="equal">Whether all settings match the defaults.</param>
    [Theory]
    [InlineData(4D, 1D, LineJoin.Bevel, true)]
    [InlineData(2D, 1D, LineJoin.Bevel, false)]
    [InlineData(4D, 2D, LineJoin.Bevel, false)]
    [InlineData(4D, 1D, LineJoin.Round, false)]
    public void Equality_UsesAllSettings(double miterLimit, double arcDetailScale, LineJoin lineJoin, bool equal)
    {
        OffsetOptions first = new();
        OffsetOptions second = new() { MiterLimit = miterLimit, ArcDetailScale = arcDetailScale, LineJoin = lineJoin };

        Assert.Equal(equal, first.Equals(second));
        Assert.Equal(equal, first.Equals((object)second));
        if (equal)
        {
            Assert.Equal(first.GetHashCode(), second.GetHashCode());
        }
    }

    /// <summary>
    /// Checks value comparison against null and an unrelated object.
    /// </summary>
    [Fact]
    public void Equality_RejectsNullAndOtherTypes()
    {
        OffsetOptions options = new();

        Assert.False(options.Equals(null));
        Assert.False(options.Equals((object)null));
        Assert.False(options.Equals("not options"));
    }
}

// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

namespace SixLabors.PolygonClipper.Tests;

public class StrokeOptionsTests
{
    [Fact]
    public void Defaults_AreAsDocumented()
    {
        StrokeOptions options = new();

        Assert.False(options.NormalizeOutput);
        Assert.Equal(4D, options.MiterLimit);
        Assert.Equal(1D, options.ArcDetailScale);
        Assert.Equal(LineJoin.Bevel, options.LineJoin);
        Assert.Equal(LineCap.Butt, options.LineCap);
    }

    [Fact]
    public void Properties_RoundTrip()
    {
        StrokeOptions options = new()
        {
            NormalizeOutput = true,
            MiterLimit = 2.5,
            ArcDetailScale = 0.5,
            LineJoin = LineJoin.Round,
            LineCap = LineCap.Square,
        };

        Assert.True(options.NormalizeOutput);
        Assert.Equal(2.5, options.MiterLimit);
        Assert.Equal(0.5, options.ArcDetailScale);
        Assert.Equal(LineJoin.Round, options.LineJoin);
        Assert.Equal(LineCap.Square, options.LineCap);
    }

    [Fact]
    public void Equals_TrueWhenAllFieldsMatch()
    {
        StrokeOptions a = new()
        {
            NormalizeOutput = true,
            MiterLimit = 3,
            ArcDetailScale = 2,
            LineJoin = LineJoin.MiterRound,
            LineCap = LineCap.Round,
        };
        StrokeOptions b = new()
        {
            NormalizeOutput = true,
            MiterLimit = 3,
            ArcDetailScale = 2,
            LineJoin = LineJoin.MiterRound,
            LineCap = LineCap.Round,
        };

        Assert.True(a.Equals(b));
        Assert.True(a.Equals((object)b));
        Assert.Equal(a.GetHashCode(), b.GetHashCode());
    }

    [Fact]
    public void Equals_TrueForTwoDefaultInstances()
    {
        StrokeOptions a = new();
        StrokeOptions b = new();

        Assert.True(a.Equals(b));
        Assert.Equal(a.GetHashCode(), b.GetHashCode());
    }

    [Fact]
    public void Equals_ReturnsFalseForNull()
    {
        StrokeOptions options = new();

        Assert.False(options.Equals(null));
        Assert.False(options.Equals((object)null));
    }

    [Fact]
    public void Equals_ReturnsFalseForOtherType()
    {
        StrokeOptions options = new();

        Assert.False(options.Equals("not options"));
    }

    public static TheoryData<Action<StrokeOptions>> FieldMutators => new()
    {
        o => o.NormalizeOutput = !o.NormalizeOutput,
        o => o.MiterLimit = 99,
        o => o.ArcDetailScale = 99,
        o => o.LineJoin = LineJoin.Round,
        o => o.LineCap = LineCap.Round,
    };

    [Theory]
    [MemberData(nameof(FieldMutators))]
    public void Equals_FalseWhenAnyFieldDiffers(Action<StrokeOptions> mutate)
    {
        StrokeOptions a = new();
        StrokeOptions b = new();
        mutate(b);

        Assert.False(a.Equals(b));
    }
}

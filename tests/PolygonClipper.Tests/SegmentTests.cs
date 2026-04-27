// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

namespace SixLabors.PolygonClipper.Tests;

public class SegmentTests
{
    [Fact]
    public void Constructor_AssignsSourceAndTarget()
    {
        Vertex a = new(1, 2);
        Vertex b = new(3, 4);

        Segment s = new(a, b);

        Assert.Equal(a, s.Source);
        Assert.Equal(b, s.Target);
    }

    [Fact]
    public void Constructor_SetsLexicographicMinAndMax()
    {
        // Source is lexicographically larger than target.
        Vertex source = new(5, 7);
        Vertex target = new(1, 2);

        Segment s = new(source, target);

        Assert.Equal(new Vertex(1, 2), s.Min);
        Assert.Equal(new Vertex(5, 7), s.Max);
    }

    [Fact]
    public void Constructor_MixesPerComponentMinMax()
    {
        // Vertex.Min/Max operate per-component, not as a true lex compare.
        Vertex source = new(1, 7);
        Vertex target = new(5, 2);

        Segment s = new(source, target);

        Assert.Equal(new Vertex(1, 2), s.Min);
        Assert.Equal(new Vertex(5, 7), s.Max);
    }

    [Fact]
    public void IsDegenerate_TrueWhenSourceEqualsTarget()
    {
        Vertex p = new(3, 4);

        Assert.True(new Segment(p, p).IsDegenerate());
        Assert.False(new Segment(p, new Vertex(3, 5)).IsDegenerate());
    }

    [Fact]
    public void IsVertical_TrueWhenXMatches()
    {
        Assert.True(new Segment(new Vertex(2, 0), new Vertex(2, 5)).IsVertical());
        Assert.True(new Segment(new Vertex(2, 0), new Vertex(2, 0)).IsVertical());
        Assert.False(new Segment(new Vertex(0, 0), new Vertex(1, 0)).IsVertical());
    }

    [Fact]
    public void Reverse_SwapsSourceAndTarget()
    {
        Vertex a = new(1, 2);
        Vertex b = new(3, 4);
        Segment original = new(a, b);

        Segment reversed = original.Reverse();

        Assert.Equal(b, reversed.Source);
        Assert.Equal(a, reversed.Target);

        // Min/Max are independent of orientation.
        Assert.Equal(original.Min, reversed.Min);
        Assert.Equal(original.Max, reversed.Max);
    }

    [Fact]
    public void Equality_RequiresSameSourceAndTarget()
    {
        Segment a = new(new Vertex(0, 0), new Vertex(1, 1));
        Segment b = new(new Vertex(0, 0), new Vertex(1, 1));
        Segment reversed = a.Reverse();

        Assert.True(a == b);
        Assert.False(a != b);
        Assert.True(a.Equals(b));
        Assert.True(a.Equals((object)b));
        Assert.False(a.Equals(reversed));
        Assert.False(a.Equals("not a segment"));
        Assert.True(a != reversed);
    }

    [Fact]
    public void GetHashCode_MatchesForEqualSegments()
    {
        Segment a = new(new Vertex(1, 2), new Vertex(3, 4));
        Segment b = new(new Vertex(1, 2), new Vertex(3, 4));

        Assert.Equal(a.GetHashCode(), b.GetHashCode());
    }
}

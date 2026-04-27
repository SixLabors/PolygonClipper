// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

namespace SixLabors.PolygonClipper.Tests;

public class Box2Tests
{
    [Fact]
    public void SinglePointConstructor_SetsMinAndMaxToSamePoint()
    {
        Vertex p = new(3, 4);
        Box2 box = new(p);

        Assert.Equal(p, box.Min);
        Assert.Equal(p, box.Max);
    }

    [Fact]
    public void MinMaxConstructor_AssignsFields()
    {
        Vertex min = new(-1, -2);
        Vertex max = new(5, 7);
        Box2 box = new(min, max);

        Assert.Equal(min, box.Min);
        Assert.Equal(max, box.Max);
    }

    [Fact]
    public void DefaultBox_IsEmpty()
    {
        Box2 box = default;

        Assert.True(box.IsEmpty());
        Assert.Equal(default, box.Min);
        Assert.Equal(default, box.Max);
    }

    [Fact]
    public void Invalid_IsEmptyAndHasExtremeBounds()
    {
        Box2 invalid = Box2.Invalid;

        Assert.True(invalid.IsEmpty());
        Assert.Equal(new Vertex(double.MaxValue, double.MaxValue), invalid.Min);
        Assert.Equal(new Vertex(-double.MaxValue, -double.MaxValue), invalid.Max);
    }

    public static TheoryData<double, double, double, double, bool> IsEmptyData => new()
    {
        { 0, 0, 1, 1, false },
        { 0, 0, 0, 1, true },
        { 0, 0, 1, 0, true },
        { 5, 5, 1, 1, true },
    };

    [Theory]
    [MemberData(nameof(IsEmptyData))]
    public void IsEmpty_ReturnsExpected(double minX, double minY, double maxX, double maxY, bool expected)
    {
        Box2 box = new(new Vertex(minX, minY), new Vertex(maxX, maxY));

        Assert.Equal(expected, box.IsEmpty());
    }

    [Fact]
    public void ContainsPoint_UsesStrictInequality()
    {
        Box2 box = new(new Vertex(0, 0), new Vertex(10, 10));

        // Strictly inside.
        Assert.True(box.Contains(new Vertex(5, 5)));

        // On the edge - excluded.
        Assert.False(box.Contains(new Vertex(0, 5)));
        Assert.False(box.Contains(new Vertex(10, 5)));
        Assert.False(box.Contains(new Vertex(5, 0)));
        Assert.False(box.Contains(new Vertex(5, 10)));

        // At a corner - excluded.
        Assert.False(box.Contains(new Vertex(0, 0)));
        Assert.False(box.Contains(new Vertex(10, 10)));

        // Outside.
        Assert.False(box.Contains(new Vertex(-1, 5)));
        Assert.False(box.Contains(new Vertex(11, 5)));
    }

    [Fact]
    public void ContainsBox_AllowsEdgeContact()
    {
        Box2 outer = new(new Vertex(0, 0), new Vertex(10, 10));
        Box2 inner = new(new Vertex(2, 2), new Vertex(8, 8));
        Box2 edge = new(new Vertex(0, 0), new Vertex(10, 10));
        Box2 spilling = new(new Vertex(-1, 0), new Vertex(10, 10));
        Box2 disjoint = new(new Vertex(20, 20), new Vertex(30, 30));

        Assert.True(outer.Contains(inner));
        Assert.True(outer.Contains(edge));
        Assert.False(outer.Contains(spilling));
        Assert.False(outer.Contains(disjoint));
    }

    [Fact]
    public void Intersects_ReturnsExpected()
    {
        Box2 a = new(new Vertex(0, 0), new Vertex(10, 10));

        // Overlap.
        Assert.True(a.Intersects(new Box2(new Vertex(5, 5), new Vertex(15, 15))));

        // Touching edge counts as intersecting (uses <=).
        Assert.True(a.Intersects(new Box2(new Vertex(10, 0), new Vertex(20, 10))));

        // Fully contained.
        Assert.True(a.Intersects(new Box2(new Vertex(2, 2), new Vertex(8, 8))));

        // Disjoint along x.
        Assert.False(a.Intersects(new Box2(new Vertex(11, 0), new Vertex(20, 10))));

        // Disjoint along y.
        Assert.False(a.Intersects(new Box2(new Vertex(0, 11), new Vertex(10, 20))));

        // Intersection is symmetric.
        Box2 b = new(new Vertex(5, 5), new Vertex(15, 15));
        Assert.Equal(a.Intersects(b), b.Intersects(a));
    }

    [Fact]
    public void MidPoint_ReturnsAverageOfMinAndMax()
    {
        Box2 box = new(new Vertex(-2, 4), new Vertex(8, 10));

        Assert.Equal(new Vertex(3, 7), box.MidPoint());
    }

    [Fact]
    public void Add_ProducesUnionOfBothBoxes()
    {
        Box2 a = new(new Vertex(0, 0), new Vertex(5, 5));
        Box2 b = new(new Vertex(-1, 2), new Vertex(8, 3));

        Box2 union = a.Add(b);

        Assert.Equal(new Vertex(-1, 0), union.Min);
        Assert.Equal(new Vertex(8, 5), union.Max);
    }

    [Fact]
    public void Add_WithInvalidActsAsIdentity()
    {
        Box2 a = new(new Vertex(0, 0), new Vertex(5, 5));

        Box2 result = Box2.Invalid.Add(a);

        Assert.Equal(a, result);
    }

    [Fact]
    public void Equality_ComparesMinAndMax()
    {
        Box2 a = new(new Vertex(0, 0), new Vertex(5, 5));
        Box2 b = new(new Vertex(0, 0), new Vertex(5, 5));
        Box2 c = new(new Vertex(0, 0), new Vertex(5, 6));

        Assert.True(a == b);
        Assert.False(a == c);
        Assert.True(a != c);
        Assert.False(a != b);
        Assert.True(a.Equals(b));
        Assert.True(a.Equals((object)b));
        Assert.False(a.Equals(c));
        Assert.False(a.Equals("not a box"));
        Assert.False(a.Equals(null));
    }

    [Fact]
    public void GetHashCode_MatchesForEqualBoxes()
    {
        Box2 a = new(new Vertex(1, 2), new Vertex(3, 4));
        Box2 b = new(new Vertex(1, 2), new Vertex(3, 4));

        Assert.Equal(a.GetHashCode(), b.GetHashCode());
    }
}

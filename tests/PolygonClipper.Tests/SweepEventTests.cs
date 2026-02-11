// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

namespace SixLabors.PolygonClipper.Tests;

public class SweepEventTests
{
    [Fact]
    public void IsBelow()
    {
        // Arrange
        SweepEvent s1 = new(new Vertex64(0, 0), true, new SweepEvent(new Vertex64(1, 1), false));
        SweepEvent s2 = new(new Vertex64(0, 1), false, new SweepEvent(new Vertex64(0, 0), false));

        // Act & Assert
        Assert.True(s1.IsBelow(new Vertex64(0, 1)));
        Assert.True(s1.IsBelow(new Vertex64(1, 2)));
        Assert.False(s1.IsBelow(new Vertex64(0, 0)));
        Assert.False(s1.IsBelow(new Vertex64(5, -1)));

        Assert.False(s2.IsBelow(new Vertex64(0, 1)));
        Assert.False(s2.IsBelow(new Vertex64(1, 2)));
        Assert.False(s2.IsBelow(new Vertex64(0, 0)));
        Assert.False(s2.IsBelow(new Vertex64(5, -1)));
    }

    [Fact]
    public void IsAbove()
    {
        // Arrange
        SweepEvent s1 = new(new Vertex64(0, 0), true, new SweepEvent(new Vertex64(1, 1), false));
        SweepEvent s2 = new(new Vertex64(0, 1), false, new SweepEvent(new Vertex64(0, 0), false));

        // Act & Assert
        Assert.False(s1.IsAbove(new Vertex64(0, 1)));
        Assert.False(s1.IsAbove(new Vertex64(1, 2)));
        Assert.True(s1.IsAbove(new Vertex64(0, 0)));
        Assert.True(s1.IsAbove(new Vertex64(5, -1)));

        Assert.True(s2.IsAbove(new Vertex64(0, 1)));
        Assert.True(s2.IsAbove(new Vertex64(1, 2)));
        Assert.True(s2.IsAbove(new Vertex64(0, 0)));
        Assert.True(s2.IsAbove(new Vertex64(5, -1)));
    }

    [Fact]
    public void IsVertical()
    {
        // Act & Assert
        Assert.True(new SweepEvent(new Vertex64(0, 0), true, new SweepEvent(new Vertex64(0, 1), false)).IsVertical());
        Assert.False(new SweepEvent(new Vertex64(0, 0), true, new SweepEvent(new Vertex64(1, 1), false)).IsVertical());
    }
}

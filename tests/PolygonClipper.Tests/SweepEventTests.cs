// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using Xunit;

namespace PolygonClipper.Tests;

public class SweepEventTests
{
    [Fact]
    public void IsBelow()
    {
        // Arrange
        SweepEvent s1 = new(new Vertex(0, 0), true, new SweepEvent(new Vertex(1, 1), false));
        SweepEvent s2 = new(new Vertex(0, 1), false, new SweepEvent(new Vertex(0, 0), false));

        // Act & Assert
        Assert.True(s1.Below(new Vertex(0, 1)));
        Assert.True(s1.Below(new Vertex(1, 2)));
        Assert.False(s1.Below(new Vertex(0, 0)));
        Assert.False(s1.Below(new Vertex(5, -1)));

        Assert.False(s2.Below(new Vertex(0, 1)));
        Assert.False(s2.Below(new Vertex(1, 2)));
        Assert.False(s2.Below(new Vertex(0, 0)));
        Assert.False(s2.Below(new Vertex(5, -1)));
    }

    [Fact]
    public void IsAbove()
    {
        // Arrange
        SweepEvent s1 = new(new Vertex(0, 0), true, new SweepEvent(new Vertex(1, 1), false));
        SweepEvent s2 = new(new Vertex(0, 1), false, new SweepEvent(new Vertex(0, 0), false));

        // Act & Assert
        Assert.False(s1.Above(new Vertex(0, 1)));
        Assert.False(s1.Above(new Vertex(1, 2)));
        Assert.True(s1.Above(new Vertex(0, 0)));
        Assert.True(s1.Above(new Vertex(5, -1)));

        Assert.True(s2.Above(new Vertex(0, 1)));
        Assert.True(s2.Above(new Vertex(1, 2)));
        Assert.True(s2.Above(new Vertex(0, 0)));
        Assert.True(s2.Above(new Vertex(5, -1)));
    }

    [Fact]
    public void IsVertical()
    {
        // Act & Assert
        Assert.True(new SweepEvent(new Vertex(0, 0), true, new SweepEvent(new Vertex(0, 1), false)).Vertical());
        Assert.False(new SweepEvent(new Vertex(0, 0), true, new SweepEvent(new Vertex(0.0001F, 1), false)).Vertical());
    }
}

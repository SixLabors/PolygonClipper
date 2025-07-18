// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using SixLabors.PolygonClipper;

namespace SixLabors.PolygonClipper.Tests;

public class SweepEventTests
{
    [Fact]
    public void IsBelow()
    {
        // Arrange
        SweepEvent s1 = new(new Vertex(0, 0), true, new SweepEvent(new Vertex(1, 1), false));
        SweepEvent s2 = new(new Vertex(0, 1), false, new SweepEvent(new Vertex(0, 0), false));

        // Act & Assert
        Assert.True((bool)s1.Below(new Vertex(0, 1)));
        Assert.True((bool)s1.Below(new Vertex(1, 2)));
        Assert.False((bool)s1.Below(new Vertex(0, 0)));
        Assert.False((bool)s1.Below(new Vertex(5, -1)));

        Assert.False((bool)s2.Below(new Vertex(0, 1)));
        Assert.False((bool)s2.Below(new Vertex(1, 2)));
        Assert.False((bool)s2.Below(new Vertex(0, 0)));
        Assert.False((bool)s2.Below(new Vertex(5, -1)));
    }

    [Fact]
    public void IsAbove()
    {
        // Arrange
        SweepEvent s1 = new(new Vertex(0, 0), true, new SweepEvent(new Vertex(1, 1), false));
        SweepEvent s2 = new(new Vertex(0, 1), false, new SweepEvent(new Vertex(0, 0), false));

        // Act & Assert
        Assert.False((bool)s1.Above(new Vertex(0, 1)));
        Assert.False((bool)s1.Above(new Vertex(1, 2)));
        Assert.True((bool)s1.Above(new Vertex(0, 0)));
        Assert.True((bool)s1.Above(new Vertex(5, -1)));

        Assert.True((bool)s2.Above(new Vertex(0, 1)));
        Assert.True((bool)s2.Above(new Vertex(1, 2)));
        Assert.True((bool)s2.Above(new Vertex(0, 0)));
        Assert.True((bool)s2.Above(new Vertex(5, -1)));
    }

    [Fact]
    public void IsVertical()
    {
        // Act & Assert
        Assert.True((bool)new SweepEvent(new Vertex(0, 0), true, new SweepEvent(new Vertex(0, 1), false)).Vertical());
        Assert.False((bool)new SweepEvent(new Vertex(0, 0), true, new SweepEvent(new Vertex(0.0001F, 1), false)).Vertical());
    }
}

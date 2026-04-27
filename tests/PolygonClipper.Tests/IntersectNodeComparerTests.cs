// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using IntersectNodeComparer = SixLabors.PolygonClipper.SelfIntersectionSweepLine.IntersectNodeComparer;

namespace SixLabors.PolygonClipper.Tests;

public class IntersectNodeComparerTests
{
    private static IntersectNode Node(double x, double y)
    {
        ActiveEdge dummy = new();
        return new IntersectNode(new Vertex(x, y), dummy, dummy);
    }

    [Fact]
    public void HigherY_SortsBeforeLowerY()
    {
        IntersectNodeComparer comparer = default;

        // Higher Y must come first, so the higher-Y argument is "less" than the lower-Y one.
        Assert.True(comparer.Compare(Node(0, 10), Node(0, 1)) < 0);
        Assert.True(comparer.Compare(Node(0, 1), Node(0, 10)) > 0);
    }

    [Fact]
    public void EqualY_OrdersByAscendingX()
    {
        IntersectNodeComparer comparer = default;

        Assert.True(comparer.Compare(Node(1, 5), Node(7, 5)) < 0);
        Assert.True(comparer.Compare(Node(7, 5), Node(1, 5)) > 0);
    }

    [Fact]
    public void EqualPoints_AreEqual()
    {
        IntersectNodeComparer comparer = default;

        Assert.Equal(0, comparer.Compare(Node(3, 4), Node(3, 4)));
    }

    [Fact]
    public void Sort_ProducesTopToBottomLeftToRight()
    {
        IntersectNode a = Node(5, 1);
        IntersectNode b = Node(2, 10);
        IntersectNode c = Node(7, 10);
        IntersectNode d = Node(0, 5);

        List<IntersectNode> list = [a, b, c, d];
        list.Sort(default(IntersectNodeComparer));

        // Y descending, then X ascending within the same Y.
        Assert.Equal(new Vertex(2, 10), list[0].Point);
        Assert.Equal(new Vertex(7, 10), list[1].Point);
        Assert.Equal(new Vertex(0, 5), list[2].Point);
        Assert.Equal(new Vertex(5, 1), list[3].Point);
    }
}

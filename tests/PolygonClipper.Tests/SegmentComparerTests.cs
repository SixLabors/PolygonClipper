// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System.Collections.Generic;
using Xunit;

namespace PolygonClipper.Tests;
public class SegmentComparerTests
{
    private readonly SegmentComparer segmentComparer = new();

    [Fact]
    public void NotCollinear_SharedLeftPoint_RightPointFirst()
    {
        SortedSet<SweepEvent> tree = new(this.segmentComparer);
        Vertex pt = default;
        SweepEvent se1 = new(pt, true, new SweepEvent(new Vertex(1, 1), false));
        SweepEvent se2 = new(pt, true, new SweepEvent(new Vertex(2, 3), false));

        tree.Add(se1);
        tree.Add(se2);

        Assert.Equal(new Vertex(2, 3), tree.Max.OtherEvent.Point);
        Assert.Equal(new Vertex(1, 1), tree.Min.OtherEvent.Point);
    }

    [Fact]
    public void NotCollinear_DifferentLeftPoint_RightPointYCoordToSort()
    {
        SortedSet<SweepEvent> tree = new(this.segmentComparer);
        SweepEvent se1 = new(new Vertex(0, 1), true, new SweepEvent(new Vertex(1, 1), false));
        SweepEvent se2 = new(new Vertex(0, 2), true, new SweepEvent(new Vertex(2, 3), false));

        tree.Add(se1);
        tree.Add(se2);

        Assert.Equal(new Vertex(1, 1), tree.Min.OtherEvent.Point);
        Assert.Equal(new Vertex(2, 3), tree.Max.OtherEvent.Point);
    }

    [Fact]
    public void NotCollinear_EventsOrderInSweepLine()
    {
        SweepEvent se1 = new(new Vertex(0, 1), true, new SweepEvent(new Vertex(2, 1), false));
        SweepEvent se2 = new(new Vertex(-1, 0), true, new SweepEvent(new Vertex(2, 3), false));

        SweepEvent se3 = new(new Vertex(0, 1), true, new SweepEvent(new Vertex(3, 4), false));
        SweepEvent se4 = new(new Vertex(-1, 0), true, new SweepEvent(new Vertex(3, 1), false));

        SweepEventComparer eventComparer = new();

        Assert.Equal(1, eventComparer.Compare(se1, se2));
        Assert.False(se2.Below(se1.Point));
        Assert.True(se2.Above(se1.Point));

        Assert.Equal(1, this.segmentComparer.Compare(se1, se2));
        Assert.Equal(1, this.segmentComparer.Compare(se2, se1));

        Assert.Equal(1, eventComparer.Compare(se3, se4));
        Assert.False(se4.Above(se3.Point));
    }

    [Fact]
    public void FirstPointIsBelow()
    {
        SweepEvent se2 = new(new Vertex(0, 1), true, new SweepEvent(new Vertex(2, 1), false));
        SweepEvent se1 = new(new Vertex(-1, 0), true, new SweepEvent(new Vertex(2, 3), false));

        Assert.False(se1.Below(se2.Point));

        Assert.Equal(1, this.segmentComparer.Compare(se1, se2));
    }

    [Fact]
    public void CollinearSegments()
    {
        SweepEvent se1 = new(new Vertex(1, 1), true, new SweepEvent(new Vertex(5, 1), false), PolygonType.Subject);
        SweepEvent se2 = new(new Vertex(2, 1), true, new SweepEvent(new Vertex(3, 1), false), PolygonType.Clipping);

        // Assert that the segments belong to different polygons
        Assert.NotEqual(se1.PolygonType, se2.PolygonType);
        Assert.Equal(-1, this.segmentComparer.Compare(se1, se2));
    }

    [Fact]
    public void CollinearSharedLeftPoint()
    {
        // Arrange
        Vertex pt = new(0, 1);

        SweepEvent se1 = new(pt, true, new SweepEvent(new Vertex(5, 1), false), PolygonType.Clipping);
        SweepEvent se2 = new(pt, true, new SweepEvent(new Vertex(3, 1), false), PolygonType.Clipping);

        se1.ContourId = 1;
        se2.ContourId = 2;

        // Assert that the segments belong to the same polygon type
        Assert.Equal(se1.PolygonType, se2.PolygonType);

        // Assert that the segments share the same starting point
        Assert.Equal(se1.Point, se2.Point);
        Assert.Equal(-1, this.segmentComparer.Compare(se1, se2));

        se1.ContourId = 2;
        se2.ContourId = 1;

        Assert.Equal(1, this.segmentComparer.Compare(se1, se2));
    }

    [Fact]
    public void CollinearSamePolygonDifferentLeftPoints()
    {
        SweepEvent se1 = new(new Vertex(1, 1), true, new SweepEvent(new Vertex(5, 1), false), PolygonType.Subject);
        SweepEvent se2 = new(new Vertex(2, 1), true, new SweepEvent(new Vertex(3, 1), false), PolygonType.Clipping);

        Assert.NotEqual(se1.PolygonType, se2.PolygonType);
        Assert.NotEqual(se1.Point, se2.Point);

        Assert.Equal(-1, this.segmentComparer.Compare(se1, se2));
    }
}

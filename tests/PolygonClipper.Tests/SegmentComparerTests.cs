// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.
namespace SixLabors.PolygonClipper.Tests;

public class SegmentComparerTests
{
    private readonly SegmentComparer segmentComparer = new();

    [Fact]
    public void NotCollinear_SharedLeftPoint_RightPointFirst()
    {
        StatusLine tree = new();
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
        StatusLine tree = new();
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
        Assert.False(se2.IsBelow(se1.Point));
        Assert.True(se2.IsAbove(se1.Point));

        this.AssertOrder(se1, se2, true);

        Assert.Equal(1, eventComparer.Compare(se3, se4));
        Assert.False(se4.IsAbove(se3.Point));
    }

    [Fact]
    public void FirstPointIsBelow()
    {
        SweepEvent se2 = new(new Vertex(0, 1), true, new SweepEvent(new Vertex(2, 1), false));
        SweepEvent se1 = new(new Vertex(-1, 0), true, new SweepEvent(new Vertex(2, 3), false));

        Assert.False(se1.IsBelow(se2.Point));

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

    [Fact]
    public void TShapedCases()
    {
        // shape:  /
        //        /\
        (SweepEvent se1, SweepEvent _) = MakeSimple(0, 0.0, 0.0, 1.0, 1.0, true);
        (SweepEvent se2, SweepEvent _) = MakeSimple(0, 0.5, 0.5, 1.0, 0.0, true);
        this.AssertOrder(se1, se2, false);

        // shape: \/
        //         \
        (se1, _) = MakeSimple(0, 0.0, 1.0, 1.0, 0.0, true);
        (se2, _) = MakeSimple(0, 0.5, 0.5, 1.0, 1.0, true);
        this.AssertOrder(se1, se2, true);

        // shape: T
        (se1, _) = MakeSimple(0, 0.0, 1.0, 1.0, 1.0, true);
        (se2, _) = MakeSimple(0, 0.5, 0.0, 0.5, 1.0, true);
        this.AssertOrder(se1, se2, false);

        // shape: T upside down
        (se1, _) = MakeSimple(0, 0.0, 0.0, 1.0, 0.0, true);
        (se2, _) = MakeSimple(0, 0.5, 0.0, 0.5, 1.0, true);
        this.AssertOrder(se1, se2, true);
    }

    [Fact]
    public void VerticalSegment()
    {
        // Vertical Reference segment at x = 0, from y = -1 to +1
        (SweepEvent se1, SweepEvent _) = MakeSimple(0, 0.0, -1.0, 0.0, 1.0, true);

        // "above" Cases
        this.AssertOrder(se1, MakeSimple(0, -1.0, 1.0, 0.0, 1.0, true).Se1, true);
        this.AssertOrder(se1, MakeSimple(0, 0.0, 1.0, 1.0, 1.0, true).Se1, true);
        this.AssertOrder(se1, MakeSimple(0, -1.0, 2.0, 0.0, 2.0, true).Se1, true);
        this.AssertOrder(se1, MakeSimple(0, 0.0, 2.0, 1.0, 2.0, true).Se1, true);
        this.AssertOrder(se1, MakeSimple(0, 0.0, 1.0, 0.0, 2.0, true).Se1, true);

        // "below" Cases
        this.AssertOrder(se1, MakeSimple(0, -1.0, -1.0, 0.0, -1.0, true).Se1, false);
        this.AssertOrder(se1, MakeSimple(0, 0.0, -1.0, 1.0, -1.0, true).Se1, false);
        this.AssertOrder(se1, MakeSimple(0, -1.0, -2.0, 0.0, -2.0, true).Se1, false);
        this.AssertOrder(se1, MakeSimple(0, 0.0, -2.0, 1.0, -2.0, true).Se1, false);
        this.AssertOrder(se1, MakeSimple(0, 0.0, -2.0, 0.0, -1.0, true).Se1, false);

        // Overlapping Cases
        this.AssertOrder(se1, MakeSimple(0, 0.0, -0.5, 0.0, 0.5, true).Se1, true);
    }

    private static (SweepEvent Se1, SweepEvent Se2) MakeSimple(
        int contourId,
        double x1,
        double y1,
        double x2,
        double y2,
        bool left)
    {
        Vertex v1 = new(x1, y1);
        Vertex v2 = new(x2, y2);
        SweepEvent se2 = new(v2, !left);
        SweepEvent se1 = new(v1, left, se2) { ContourId = contourId };
        se2.OtherEvent = se1;
        se2.ContourId = contourId;

        return (se1, se2);
    }

    private void AssertOrder(SweepEvent se1, SweepEvent se2, bool less)
    {
        int order = less ? -1 : 1;
        int inverseOrder = less ? 1 : -1;
        Assert.Equal(order, this.segmentComparer.Compare(se1, se2));
        Assert.Equal(inverseOrder, this.segmentComparer.Compare(se2, se1));
    }
}

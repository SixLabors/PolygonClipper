// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

namespace SixLabors.PolygonClipper.Tests;

public class HorizontalJoinTests
{
    private static OutputPoint CreatePoint(double x, double y)
        => new(new Vertex(x, y), new OutputRecord());

    [Fact]
    public void Constructor_AssignsBothEndpoints()
    {
        OutputPoint left = CreatePoint(0, 0);
        OutputPoint right = CreatePoint(10, 0);

        HorizontalJoin join = new(left, right);

        Assert.Same(left, join.LeftToRight);
        Assert.Same(right, join.RightToLeft);
    }

    [Fact]
    public void Endpoints_AreReassignable()
    {
        HorizontalJoin join = new(CreatePoint(0, 0), CreatePoint(10, 0));

        OutputPoint newLeft = CreatePoint(1, 1);
        OutputPoint newRight = CreatePoint(9, 1);
        join.LeftToRight = newLeft;
        join.RightToLeft = newRight;

        Assert.Same(newLeft, join.LeftToRight);
        Assert.Same(newRight, join.RightToLeft);
    }

    [Fact]
    public void Endpoints_AcceptNull()
    {
        HorizontalJoin join = new(CreatePoint(0, 0), CreatePoint(10, 0));

        join.LeftToRight = null;
        join.RightToLeft = null;

        Assert.Null(join.LeftToRight);
        Assert.Null(join.RightToLeft);
    }
}

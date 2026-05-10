// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

namespace SixLabors.PolygonClipper.Tests;

public class HorizontalJoinPoolListTests
{
    private static OutputPoint CreatePoint(double x, double y)
        => new(new Vertex(x, y), new OutputRecord());

    [Fact]
    public void Add_GrowsCountAndReturnsEntry()
    {
        HorizontalJoinPoolList pool = new();
        OutputPoint l = CreatePoint(0, 0);
        OutputPoint r = CreatePoint(10, 0);

        HorizontalJoin join = pool.Add(l, r);

        Assert.Equal(1, pool.Count);
        Assert.Same(join, pool[0]);
        Assert.Same(l, join.LeftToRight);
        Assert.Same(r, join.RightToLeft);
    }

    [Fact]
    public void Add_PastCapacity_GrowsBackingArray()
    {
        HorizontalJoinPoolList pool = new();

        for (int i = 0; i < 9; i++)
        {
            pool.Add(CreatePoint(i, 0), CreatePoint(i + 1, 0));
        }

        Assert.Equal(9, pool.Count);
        Assert.True(pool.Capacity >= 9);
        for (int i = 0; i < 9; i++)
        {
            Assert.Equal(new Vertex(i, 0), pool[i].LeftToRight!.Point);
            Assert.Equal(new Vertex(i + 1, 0), pool[i].RightToLeft!.Point);
        }
    }

    [Fact]
    public void Clear_ResetsCountWithoutShrinkingCapacity()
    {
        HorizontalJoinPoolList pool = new();
        pool.Add(CreatePoint(0, 0), CreatePoint(1, 0));
        pool.Add(CreatePoint(2, 0), CreatePoint(3, 0));
        int capacityBefore = pool.Capacity;

        pool.Clear();

        Assert.Equal(0, pool.Count);
        Assert.Equal(capacityBefore, pool.Capacity);
    }

    [Fact]
    public void Add_AfterClear_ReusesExistingInstance()
    {
        HorizontalJoinPoolList pool = new();
        HorizontalJoin first = pool.Add(CreatePoint(0, 0), CreatePoint(1, 0));

        pool.Clear();

        OutputPoint newL = CreatePoint(5, 5);
        OutputPoint newR = CreatePoint(6, 5);
        HorizontalJoin reused = pool.Add(newL, newR);

        Assert.Same(first, reused);
        Assert.Same(newL, reused.LeftToRight);
        Assert.Same(newR, reused.RightToLeft);
        Assert.Equal(1, pool.Count);
    }

    [Fact]
    public void EnsureCapacity_GrowsBackingArrayWithoutChangingCount()
    {
        HorizontalJoinPoolList pool = new();

        pool.EnsureCapacity(32);

        Assert.True(pool.Capacity >= 32);
        Assert.Equal(0, pool.Count);
    }

    [Fact]
    public void Enumeration_YieldsItemsInInsertionOrder()
    {
        HorizontalJoinPoolList pool = new();
        HorizontalJoin a = pool.Add(CreatePoint(0, 0), CreatePoint(1, 0));
        HorizontalJoin b = pool.Add(CreatePoint(2, 0), CreatePoint(3, 0));
        HorizontalJoin c = pool.Add(CreatePoint(4, 0), CreatePoint(5, 0));

        List<HorizontalJoin> seen = [];
        foreach (HorizontalJoin entry in pool)
        {
            seen.Add(entry);
        }

        Assert.Equal([a, b, c], seen);
    }
}

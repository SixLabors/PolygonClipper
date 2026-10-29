// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

namespace SixLabors.PolygonClipper.Tests;

/// <summary>
/// Checks event reuse when retained storage grows between clipping runs.
/// </summary>
public class SweepEventPoolListTests
{
    /// <summary>
    /// Checks that growth preserves cached events outside the active range.
    /// </summary>
    /// <param name="activeCount">The number of events reused before growth.</param>
    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    public void GrowthAfterClear_PreservesCachedEvents(int activeCount)
    {
        SweepEventPoolList pool = [];
        SweepEvent[] original = new SweepEvent[4];
        for (int i = 0; i < original.Length; i++)
        {
            original[i] = pool.Add(new Vertex(i, 0), new Vertex(i + 1, 0), new PolygonClipper.WindingCount(1, 0));
        }

        int capacity = pool.Capacity;
        pool.Clear();
        for (int i = 0; i < activeCount; i++)
        {
            Assert.Same(original[i], pool.Add(new Vertex(i, 1), new Vertex(i + 1, 1), new PolygonClipper.WindingCount(0, 1)));
        }

        pool.EnsureCapacity(capacity + 1);

        Assert.Equal(activeCount, pool.Count);
        Assert.True(pool.Capacity > capacity);
        for (int i = activeCount; i < original.Length; i++)
        {
            SweepEvent reused = pool.Add(new Vertex(i, 1), new Vertex(i + 1, 1), new PolygonClipper.WindingCount(0, 1));
            Assert.Same(original[i], reused);
            Assert.Equal(new Vertex(i, 1), reused.Point);
            Assert.Equal(new Vertex(i + 1, 1), reused.End);
        }

        pool.Add(new Vertex(4, 1), new Vertex(5, 1), new PolygonClipper.WindingCount(0, 1));
        Assert.Equal(5, pool.Count);
    }
}

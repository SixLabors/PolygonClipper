// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System.Collections.Generic;

namespace SixLabors.PolygonClipper.Tests;

public class OrderedSetTests
{
    [Fact]
    public void Add_EnumeratesInOrder()
    {
        OrderedSet<int> set = new();
        set.Add(5);
        set.Add(1);
        set.Add(3);

        List<int> values = new();
        foreach (int value in set)
        {
            values.Add(value);
        }

        Assert.Equal(3, values.Count);
        Assert.Equal(1, values[0]);
        Assert.Equal(3, values[1]);
        Assert.Equal(5, values[2]);
        Assert.Equal(1, set.Min);
        Assert.Equal(5, set.Max);
    }

    [Fact]
    public void Add_DuplicatesPreserveInsertionOrder()
    {
        OrderedSet<KeyedItem> set = new(new KeyedItemComparer());
        set.Add(new KeyedItem(1, 1));
        set.Add(new KeyedItem(1, 2));
        set.Add(new KeyedItem(1, 3));

        List<int> ids = new();
        foreach (KeyedItem item in set)
        {
            ids.Add(item.Id);
        }

        Assert.Equal(3, ids.Count);
        Assert.Equal(1, ids[0]);
        Assert.Equal(2, ids[1]);
        Assert.Equal(3, ids[2]);
    }

    [Fact]
    public void LowerBoundUpperBound_ReturnExpectedNodes()
    {
        OrderedSet<int> set = new();
        set.Add(1);
        set.Add(3);
        set.Add(5);
        set.Add(7);

        Assert.True(set.TryGetLowerBound(0, out int lower));
        Assert.Equal(1, lower);
        Assert.True(set.TryGetLowerBound(4, out lower));
        Assert.Equal(5, lower);
        Assert.True(set.TryGetLowerBound(7, out lower));
        Assert.Equal(7, lower);
        Assert.False(set.TryGetLowerBound(8, out _));

        Assert.True(set.TryGetUpperBound(1, out int upper));
        Assert.Equal(3, upper);
        Assert.True(set.TryGetUpperBound(5, out upper));
        Assert.Equal(7, upper);
        Assert.False(set.TryGetUpperBound(7, out _));
    }

    [Fact]
    public void RemoveByValue_UpdatesOrder()
    {
        OrderedSet<int> set = new();
        set.Add(1);
        set.Add(2);
        set.Add(3);

        Assert.True(set.Remove(2));

        Assert.Equal(2, set.Count);
        Assert.Equal(1, set.Min);
        Assert.Equal(3, set.Max);

        List<int> values = new();
        foreach (int value in set)
        {
            values.Add(value);
        }

        Assert.Equal(2, values.Count);
        Assert.Equal(1, values[0]);
        Assert.Equal(3, values[1]);
    }

    [Fact]
    public void RemoveByValue_RemovesFirstMatch()
    {
        OrderedSet<KeyedItem> set = new(new KeyedItemComparer());
        set.Add(new KeyedItem(1, 1));
        set.Add(new KeyedItem(1, 2));
        set.Add(new KeyedItem(1, 3));

        Assert.True(set.Remove(new KeyedItem(1, 0)));

        List<int> ids = new();
        foreach (KeyedItem item in set)
        {
            ids.Add(item.Id);
        }

        Assert.Equal(2, ids.Count);
        Assert.Equal(2, ids[0]);
        Assert.Equal(3, ids[1]);
    }

    [Fact]
    public void MinMax_ThrowsOnEmpty()
    {
        OrderedSet<int> set = new();
        Assert.Throws<InvalidOperationException>(() => _ = set.Min);
        Assert.Throws<InvalidOperationException>(() => _ = set.Max);
    }

    [Fact]
    public void Clear_ResetsSet()
    {
        OrderedSet<int> set = new();
        set.Add(2);
        set.Add(1);
        set.Clear();

        Assert.Equal(0, set.Count);
        Assert.False(set.TryGetMin(out _));
        Assert.False(set.TryGetMax(out _));
        Assert.False(set.TryGetLowerBound(1, out _));
    }

    private readonly struct KeyedItem
    {
        public KeyedItem(int key, int id)
        {
            this.Key = key;
            this.Id = id;
        }

        public int Key { get; }

        public int Id { get; }
    }

    private sealed class KeyedItemComparer : IComparer<KeyedItem>
    {
        public int Compare(KeyedItem x, KeyedItem y) => x.Key.CompareTo(y.Key);
    }
}

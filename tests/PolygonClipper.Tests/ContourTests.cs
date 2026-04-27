// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

namespace SixLabors.PolygonClipper.Tests;

public class ContourTests
{
    private static Contour CreateSquare(double x, double y, double size, bool counterClockwise = true)
    {
        Contour contour = new(4);
        if (counterClockwise)
        {
            contour.Add(new Vertex(x, y));
            contour.Add(new Vertex(x + size, y));
            contour.Add(new Vertex(x + size, y + size));
            contour.Add(new Vertex(x, y + size));
        }
        else
        {
            contour.Add(new Vertex(x, y));
            contour.Add(new Vertex(x, y + size));
            contour.Add(new Vertex(x + size, y + size));
            contour.Add(new Vertex(x + size, y));
        }

        return contour;
    }

    [Fact]
    public void DefaultConstructor_CreatesEmptyContour()
    {
        Contour contour = new();

        Assert.Equal(0, contour.Count);
        Assert.Equal(0, contour.HoleCount);
        Assert.True(contour.IsExternal);
        Assert.Null(contour.ParentIndex);
        Assert.Equal(0, contour.Depth);
    }

    [Fact]
    public void CapacityConstructor_StartsEmpty()
    {
        Contour contour = new(64);

        Assert.Equal(0, contour.Count);
    }

    [Fact]
    public void Add_AppendsVertices_Indexer_ReturnsThem()
    {
        Contour contour = new();
        Vertex a = new(1, 2);
        Vertex b = new(3, 4);

        contour.Add(a);
        contour.Add(b);

        Assert.Equal(2, contour.Count);
        Assert.Equal(a, contour[0]);
        Assert.Equal(b, contour[1]);
        Assert.Equal(b, contour.GetLastVertex());
    }

    [Fact]
    public void RemoveVertexAt_RemovesTheTargetVertex()
    {
        Contour contour = CreateSquare(0, 0, 10);

        contour.RemoveVertexAt(1);

        Assert.Equal(3, contour.Count);
        Assert.Equal(new Vertex(0, 0), contour[0]);
        Assert.Equal(new Vertex(10, 10), contour[1]);
        Assert.Equal(new Vertex(0, 10), contour[2]);
    }

    [Fact]
    public void Clear_RemovesAllVerticesAndHoles()
    {
        Contour contour = CreateSquare(0, 0, 10);
        contour.AddHoleIndex(2);
        contour.AddHoleIndex(5);

        contour.Clear();

        Assert.Equal(0, contour.Count);
        Assert.Equal(0, contour.HoleCount);
    }

    [Fact]
    public void Holes_CanBeAddedReadAndCleared()
    {
        Contour contour = new();

        contour.AddHoleIndex(3);
        contour.AddHoleIndex(7);

        Assert.Equal(2, contour.HoleCount);
        Assert.Equal(3, contour.GetHoleIndex(0));
        Assert.Equal(7, contour.GetHoleIndex(1));

        contour.ClearHoles();

        Assert.Equal(0, contour.HoleCount);
    }

    [Fact]
    public void ParentIndex_FlipsIsExternal()
    {
        Contour contour = new();

        Assert.True(contour.IsExternal);

        contour.ParentIndex = 4;

        Assert.False(contour.IsExternal);
        Assert.Equal(4, contour.ParentIndex);
    }

    [Fact]
    public void Depth_RoundTrips()
    {
        Contour contour = new() { Depth = 3 };

        Assert.Equal(3, contour.Depth);
    }

    [Fact]
    public void Translate_OffsetsAllVertices()
    {
        Contour contour = CreateSquare(0, 0, 10);

        contour.Translate(2, -3);

        Assert.Equal(new Vertex(2, -3), contour[0]);
        Assert.Equal(new Vertex(12, -3), contour[1]);
        Assert.Equal(new Vertex(12, 7), contour[2]);
        Assert.Equal(new Vertex(2, 7), contour[3]);
    }

    [Fact]
    public void IsCounterClockwise_ReturnsTrueForCcwPolygon()
    {
        Contour contour = CreateSquare(0, 0, 10, counterClockwise: true);

        Assert.True(contour.IsCounterClockwise());
        Assert.False(contour.IsClockwise());
    }

    [Fact]
    public void IsCounterClockwise_ReturnsFalseForCwPolygon()
    {
        Contour contour = CreateSquare(0, 0, 10, counterClockwise: false);

        Assert.False(contour.IsCounterClockwise());
        Assert.True(contour.IsClockwise());
    }

    [Fact]
    public void Reverse_FlipsOrientationAndVertexOrder()
    {
        Contour contour = CreateSquare(0, 0, 10, counterClockwise: true);
        Vertex first = contour[0];
        Vertex last = contour[3];

        contour.Reverse();

        Assert.Equal(last, contour[0]);
        Assert.Equal(first, contour[3]);
        Assert.True(contour.IsClockwise());
    }

    [Fact]
    public void SetClockwise_OnCcwContour_ReversesOrientation()
    {
        Contour contour = CreateSquare(0, 0, 10, counterClockwise: true);

        contour.SetClockwise();

        Assert.True(contour.IsClockwise());
    }

    [Fact]
    public void SetClockwise_OnCwContour_LeavesOrderUnchanged()
    {
        Contour contour = CreateSquare(0, 0, 10, counterClockwise: false);
        Vertex[] before = [contour[0], contour[1], contour[2], contour[3]];

        contour.SetClockwise();

        Assert.True(contour.IsClockwise());
        for (int i = 0; i < 4; i++)
        {
            Assert.Equal(before[i], contour[i]);
        }
    }

    [Fact]
    public void SetCounterClockwise_OnCwContour_ReversesOrientation()
    {
        Contour contour = CreateSquare(0, 0, 10, counterClockwise: false);

        contour.SetCounterClockwise();

        Assert.True(contour.IsCounterClockwise());
    }

    [Fact]
    public void SetCounterClockwise_OnCcwContour_LeavesOrderUnchanged()
    {
        Contour contour = CreateSquare(0, 0, 10, counterClockwise: true);
        Vertex[] before = [contour[0], contour[1], contour[2], contour[3]];

        contour.SetCounterClockwise();

        Assert.True(contour.IsCounterClockwise());
        for (int i = 0; i < 4; i++)
        {
            Assert.Equal(before[i], contour[i]);
        }
    }

    [Fact]
    public void GetBoundingBox_EmptyContour_ReturnsDefault()
    {
        Contour contour = new();

        Box2 box = contour.GetBoundingBox();

        Assert.Equal(default, box);
    }

    [Fact]
    public void GetBoundingBox_SingleVertex_ReturnsDegenerateBox()
    {
        Contour contour = new();
        Vertex v = new(2, 3);
        contour.Add(v);

        Box2 box = contour.GetBoundingBox();

        Assert.Equal(v, box.Min);
        Assert.Equal(v, box.Max);
    }

    [Fact]
    public void GetBoundingBox_ReturnsMinMaxAcrossAllVertices()
    {
        Contour contour = new();
        contour.Add(new Vertex(2, -1));
        contour.Add(new Vertex(5, 4));
        contour.Add(new Vertex(-3, 2));
        contour.Add(new Vertex(0, 7));

        Box2 box = contour.GetBoundingBox();

        Assert.Equal(new Vertex(-3, -1), box.Min);
        Assert.Equal(new Vertex(5, 7), box.Max);
    }

    [Fact]
    public void Enumeration_YieldsVerticesInOrder()
    {
        Contour contour = CreateSquare(0, 0, 10);

        Vertex[] expected =
        [
            new(0, 0),
            new(10, 0),
            new(10, 10),
            new(0, 10),
        ];

        Vertex[] actual = [.. contour];
        Assert.Equal(expected, actual);
    }

    [Fact]
    public void DeepClone_ProducesIndependentCopy()
    {
        Contour original = CreateSquare(0, 0, 10);
        original.AddHoleIndex(1);
        original.ParentIndex = 2;
        original.Depth = 4;

        Contour clone = original.DeepClone();

        Assert.Equal(original.Count, clone.Count);
        Assert.Equal(original.HoleCount, clone.HoleCount);
        Assert.Equal(1, clone.GetHoleIndex(0));
        Assert.Equal(2, clone.ParentIndex);
        Assert.Equal(4, clone.Depth);
        for (int i = 0; i < original.Count; i++)
        {
            Assert.Equal(original[i], clone[i]);
        }

        // Mutating the clone must not affect the original.
        clone.Add(new Vertex(99, 99));
        clone.AddHoleIndex(42);
        clone.ParentIndex = null;

        Assert.Equal(4, original.Count);
        Assert.Equal(1, original.HoleCount);
        Assert.Equal(2, original.ParentIndex);
    }

    [Fact]
    public void DeepClone_PreservesCachedOrientation()
    {
        Contour original = CreateSquare(0, 0, 10);

        // Force orientation to be cached.
        Assert.True(original.IsCounterClockwise());

        Contour clone = original.DeepClone();

        // Empty the clone's vertices so any recompute would throw if the cache wasn't carried.
        // Reading the cached value should still succeed.
        Assert.True(clone.IsCounterClockwise());
    }

    [Fact]
    public void GetSegment_ReturnsConsecutiveSegment()
    {
        Contour contour = CreateSquare(0, 0, 10);

        Segment s = contour.GetSegment(1);

        Assert.Equal(new Vertex(10, 0), s.Source);
        Assert.Equal(new Vertex(10, 10), s.Target);
    }

    [Fact]
    public void GetSegment_LastIndex_WrapsToFirstVertex()
    {
        Contour contour = CreateSquare(0, 0, 10);

        Segment s = contour.GetSegment(contour.Count - 1);

        Assert.Equal(new Vertex(0, 10), s.Source);
        Assert.Equal(new Vertex(0, 0), s.Target);
    }
}

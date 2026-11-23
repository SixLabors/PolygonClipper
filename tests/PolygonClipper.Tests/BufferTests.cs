// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

namespace SixLabors.PolygonClipper.Tests;

/// <summary>
/// Checks typed scratch storage and pooled buffer allocations.
/// </summary>
public class BufferTests
{
    /// <summary>
    /// Checks the requested span length and writable vertex storage.
    /// </summary>
    /// <param name="length">The requested number of vertices.</param>
    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(17)]
    public void GetSpan_ReturnsRequestedElements(int length)
    {
        using Buffer<Vertex> buffer = new(length);
        Span<Vertex> span = buffer.GetSpan();
        Assert.Equal(length, span.Length);
        for (int i = 0; i < span.Length; i++)
        {
            span[i] = new Vertex(i, -i);
        }

        for (int i = 0; i < span.Length; i++)
        {
            Assert.Equal(new Vertex(i, -i), buffer.GetSpan()[i]);
        }
    }

    /// <summary>
    /// Checks that buffer construction and return allocate no objects after pool warmup.
    /// </summary>
    [Fact]
    public void RentAndReturn_WarmedPool_DoesNotAllocate()
    {
        // Warm both element types and the thread-local pool before measuring.
        for (int i = 0; i < 256; i++)
        {
            using Buffer<int> integers = new(17);
            using Buffer<Vertex> vertices = new(17);
            integers.GetSpan()[0] = i;
            vertices.GetSpan()[0] = new Vertex(i);
        }

        int sum = 0;
        long before = GC.GetAllocatedBytesForCurrentThread();
        for (int i = 0; i < 1000; i++)
        {
            using Buffer<int> integers = new(17);
            using Buffer<Vertex> vertices = new(17);
            integers.GetSpan()[0] = i;
            vertices.GetSpan()[0] = new Vertex(i);
            sum += integers.GetSpan()[0] + (int)vertices.GetSpan()[0].X;
        }

        long allocated = GC.GetAllocatedBytesForCurrentThread() - before;
        Assert.Equal(999000, sum);
        Assert.Equal(0, allocated);
    }

    /// <summary>
    /// Checks that disposing twice does not return the same array twice.
    /// </summary>
    [Fact]
    public void Dispose_CalledTwice_ReturnsArrayOnce()
    {
        Buffer<int> buffer = new(17);
        buffer.Dispose();
        buffer.Dispose();

        using Buffer<int> first = new(17);
        using Buffer<int> second = new(17);
        first.GetSpan()[0] = 1;
        second.GetSpan()[0] = 2;
        Assert.Equal(1, first.GetSpan()[0]);
        Assert.Equal(2, second.GetSpan()[0]);
    }
}

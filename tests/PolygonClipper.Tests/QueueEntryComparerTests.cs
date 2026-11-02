// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

namespace SixLabors.PolygonClipper.Tests;

/// <summary>
/// Checks endpoint ordering through the value-type heap comparer.
/// </summary>
public class QueueEntryComparerTests
{
    /// <summary>
    /// Checks point, interval, version and endpoint order with both heap entry paths.
    /// </summary>
    /// <param name="heapify">Whether to build the heap from its shared backing list.</param>
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Queue_OrdersAllEndpointKeys(bool heapify)
    {
        QueueEntry[] expected =
        [
            CreateEntry(new Vertex(-1, 5), 4, 3, true),
            CreateEntry(new Vertex(0, -1), 4, 3, true),
            CreateEntry(new Vertex(0, 0), 0, 3, true),
            CreateEntry(new Vertex(0, 0), 1, 0, true),
            CreateEntry(new Vertex(0, 0), 1, 1, false),
            CreateEntry(new Vertex(0, 0), 1, 1, true),
            CreateEntry(new Vertex(1, -5), 0, 0, false)
        ];

        List<QueueEntry> storage = [];
        StablePriorityQueue<QueueEntry, QueueEntryComparer> queue = new(default(QueueEntryComparer), storage);
        int[] insertionOrder = [6, 3, 0, 4, 2, 1, 5];
        foreach (int index in insertionOrder)
        {
            if (heapify)
            {
                storage.Add(expected[index]);
            }
            else
            {
                queue.Enqueue(expected[index]);
            }
        }

        if (heapify)
        {
            queue.Heapify();
        }

        foreach (QueueEntry entry in expected)
        {
            QueueEntry actual = queue.Dequeue();
            Assert.Equal(entry.Point, actual.Point);
            Assert.Equal(entry.EdgeIndex, actual.EdgeIndex);
            Assert.Equal(entry.Version, actual.Version);
            Assert.Equal(entry.IsStart, actual.IsStart);
        }

        Assert.Equal(0, queue.Count);
        Assert.Equal(0, queue.Comparer.Compare(expected[0], expected[0]));
    }

    /// <summary>
    /// Creates a queued endpoint from an initialized interval.
    /// </summary>
    /// <param name="point">The queued point.</param>
    /// <param name="index">The source interval index.</param>
    /// <param name="version">The interval version.</param>
    /// <param name="isStart">Whether this endpoint starts the interval.</param>
    /// <returns>The queued endpoint.</returns>
    private static QueueEntry CreateEntry(Vertex point, int index, int version, bool isStart)
    {
        Vertex end = new(point.X + 1, point.Y);
        SweepEvent edge = new(point, true, new SweepEvent(end, false));
        edge.InitializeInterval(point, end, index, default);
        edge.Version = version;
        return new QueueEntry(point, edge, isStart);
    }
}

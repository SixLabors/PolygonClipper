// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

namespace SixLabors.PolygonClipper;

/// <summary>
/// Stores a versioned endpoint in the existing priority queue.
/// </summary>
internal readonly struct QueueEntry
{
    /// <summary>
    /// The queued geometric point.
    /// </summary>
    public readonly Vertex Point;

    /// <summary>
    /// The source pool index, complemented for a start record.
    /// </summary>
    private readonly int sourceIndex;

    /// <summary>
    /// The interval version when queued.
    /// </summary>
    public readonly int Version;

    /// <summary>
    /// Initializes a new instance of the <see cref="QueueEntry"/> struct.
    /// </summary>
    /// <param name="point">The endpoint.</param>
    /// <param name="edge">The interval.</param>
    /// <param name="isStart">Whether the record starts the interval.</param>
    public QueueEntry(Vertex point, SweepEvent edge, bool isStart)
    {
        this.Point = point;

        // Pool indices remain stable until the operation ends. Encoding
        // start/end in their sign keeps every version bit and removes the
        // managed reference, so heap movement needs no GC write barriers.
        this.sourceIndex = isStart ? ~edge.ContourId : edge.ContourId;
        this.Version = edge.Version;
    }

    /// <summary>
    /// Gets a value indicating whether this record starts a source interval.
    /// </summary>
    public bool IsStart => this.sourceIndex < 0;

    /// <summary>
    /// Gets the stable index in the source interval pool.
    /// </summary>
    public int EdgeIndex => this.sourceIndex < 0 ? ~this.sourceIndex : this.sourceIndex;

    /// <summary>
    /// Checks whether the source interval still owns this queued record.
    /// </summary>
    /// <param name="edge">The interval at <see cref="EdgeIndex"/>.</param>
    /// <returns>Whether the interval version and activity match.</returns>
    public bool IsValid(SweepEvent edge) => this.Version == edge.Version && (this.IsStart || edge.Active);
}

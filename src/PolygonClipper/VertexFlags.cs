// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

namespace SixLabors.PolygonClipper;

/// <summary>
/// Classifies sweep vertices by local-extrema role during bound construction.
/// </summary>
/// <remarks>
/// The self-intersection sweep decomposes each contour into monotonic bounds that
/// start at local minima and terminate at local maxima. These flags annotate
/// each <see cref="SweepVertex"/> with that role.
/// </remarks>
[Flags]
internal enum VertexFlags
{
    /// <summary>
    /// No extrema role is assigned.
    /// </summary>
    None = 0,

    /// <summary>
    /// Marks a local maximum vertex (bound endpoint).
    /// </summary>
    LocalMax = 1 << 0,

    /// <summary>
    /// Marks a local minimum vertex (bound start).
    /// </summary>
    LocalMin = 1 << 1
}

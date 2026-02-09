// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

namespace SixLabors.PolygonClipper;

/// <summary>
/// Indicates whether an active edge should be joined with a neighbor after a split.
/// </summary>
internal enum JoinWith
{
    /// <summary>
    /// No pending join.
    /// </summary>
    None,

    /// <summary>
    /// Join with the left neighbor in the AEL.
    /// </summary>
    Left,

    /// <summary>
    /// Join with the right neighbor in the AEL.
    /// </summary>
    Right
}

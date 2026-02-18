// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

namespace SixLabors.PolygonClipper;

/// <summary>
/// Specifies how corners are rendered when stroking.
/// </summary>
public enum LineJoin
{
    /// <summary>
    /// Joins lines by extending their outer edges until they meet.
    /// </summary>
    Miter = 0,

    /// <summary>
    /// Joins lines by extending their outer edges to form a miter, but clamps past miter limit.
    /// </summary>
    MiterRevert = 1,

    /// <summary>
    /// Joins lines using a circular arc.
    /// </summary>
    Round = 2,

    /// <summary>
    /// Joins lines with a straight bevel edge.
    /// </summary>
    Bevel = 3,

    /// <summary>
    /// Joins lines by miter, and falls back to round when miter limit is exceeded.
    /// </summary>
    MiterRound = 4
}

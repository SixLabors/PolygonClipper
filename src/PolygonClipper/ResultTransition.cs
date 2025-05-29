// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

namespace PolygonClipper;

/// <summary>
/// Represents the result transition for a sweep event.
/// </summary>
public enum ResultTransition
{
    /// <summary>
    /// The event does not contribute to the result.
    /// </summary>
    NonContributing = -1,

    /// <summary>
    /// The event transitions within the result.
    /// </summary>
    Neutral = 0,

    /// <summary>
    /// The event contributes to the result.
    /// </summary>
    Contributing = 1
}

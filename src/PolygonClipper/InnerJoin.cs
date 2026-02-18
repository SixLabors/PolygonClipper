// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

namespace SixLabors.PolygonClipper;

/// <summary>
/// Specifies join styles for sharp interior angles.
/// </summary>
public enum InnerJoin
{
    /// <summary>
    /// Use an interior miter join.
    /// </summary>
    Miter = 0,

    /// <summary>
    /// Use an interior jag join.
    /// </summary>
    Jag = 1,

    /// <summary>
    /// Use an interior round join.
    /// </summary>
    Round = 2,

    /// <summary>
    /// Use an interior bevel join.
    /// </summary>
    Bevel = 3
}

// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

namespace SixLabors.PolygonClipper;

/// <summary>
/// Specifies the winding rule used to decide which regions are filled.
/// </summary>
public enum FillRule
{
    /// <summary>
    /// Fills regions covered by an odd number of contours.
    /// </summary>
    EvenOdd,

    /// <summary>
    /// Fills regions whose winding count is non-zero.
    /// </summary>
    NonZero,

    /// <summary>
    /// Counts winding in the positive direction as filled.
    /// </summary>
    Positive,

    /// <summary>
    /// Counts winding in the negative direction as filled.
    /// </summary>
    Negative
}

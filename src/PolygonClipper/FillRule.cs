// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

namespace SixLabors.PolygonClipper;

/// <summary>
/// Specifies the winding rule used to decide which regions are filled.
/// </summary>
internal enum FillRule
{
    /// <summary>
    /// Counts winding in the positive direction as filled.
    /// </summary>
    Positive,

    /// <summary>
    /// Counts winding in the negative direction as filled.
    /// </summary>
    Negative
}

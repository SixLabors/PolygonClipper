// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

namespace SixLabors.PolygonClipper;

/// <summary>
/// Specifies the type of a polygon in a boolean operation.
/// </summary>
internal enum PolygonType
{
    /// <summary>
    /// Represents the subject polygon in a boolean operation.
    /// </summary>
    Subject = 0,

    /// <summary>
    /// Represents the clipping polygon in a boolean operation.
    /// </summary>
    Clipping = 1
}

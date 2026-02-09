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

/// <summary>
/// Describes the relationship between a point and a polygon.
/// </summary>
internal enum PointInPolygonResult
{
    /// <summary>
    /// The point lies on the polygon boundary.
    /// </summary>
    IsOn = 0,

    /// <summary>
    /// The point lies strictly inside the polygon.
    /// </summary>
    IsInside = 1,

    /// <summary>
    /// The point lies outside the polygon.
    /// </summary>
    IsOutside = 2
}

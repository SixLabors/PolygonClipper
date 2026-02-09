// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

namespace SixLabors.PolygonClipper;

/// <summary>
/// Describes the relationship between a point and a polygon.
/// </summary>
internal enum PointInPolygonResult
{
    /// <summary>
    /// The point lies on the polygon boundary.
    /// </summary>
    On = 0,

    /// <summary>
    /// The point lies strictly inside the polygon.
    /// </summary>
    Inside = 1,

    /// <summary>
    /// The point lies outside the polygon.
    /// </summary>
    Outside = 2
}

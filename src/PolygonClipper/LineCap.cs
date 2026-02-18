// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

namespace SixLabors.PolygonClipper;

/// <summary>
/// Specifies cap styles for open paths.
/// </summary>
public enum LineCap
{
    /// <summary>
    /// The stroke ends exactly at the path endpoint.
    /// </summary>
    Butt,

    /// <summary>
    /// The stroke extends half-width beyond the endpoint with a square edge.
    /// </summary>
    Square,

    /// <summary>
    /// The stroke ends with a semicircular cap.
    /// </summary>
    Round
}

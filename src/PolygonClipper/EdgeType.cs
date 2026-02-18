// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

namespace SixLabors.PolygonClipper;

/// <summary>
/// Specifies the type of an edge in a boolean operation on polygons.
/// </summary>
internal enum EdgeType
{
    /// <summary>
    /// A normal edge that contributes to the resulting polygon.
    /// </summary>
    Normal = 0,

    /// <summary>
    /// An edge that does not contribute to the resulting polygon.
    /// This typically occurs when the edge lies entirely inside another polygon.
    /// </summary>
    NonContributing = 1,

    /// <summary>
    /// An edge that represents a transition within the same polygon,
    /// meaning it does not cross into another polygon.
    /// </summary>
    SameTransition = 2,

    /// <summary>
    /// An edge that represents a transition between different polygons,
    /// meaning it crosses from one polygon to another.
    /// </summary>
    DifferentTransition = 3
}

// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

namespace SixLabors.PolygonClipper;

/// <summary>
/// Specifies how the connection between two consecutive line segments (a join)
/// is rendered when stroking paths or polygons.
/// </summary>
public enum LineJoin
{
    /// <summary>
    /// Joins lines by extending their outer edges until they meet at a sharp corner.
    /// If the miter limit is exceeded, the join is truncated at the limit distance.
    /// </summary>
    Miter = 0,

    /// <summary>
    /// Joins lines by extending their outer edges to form a miter.
    /// If the miter limit is exceeded, the join falls back to a bevel.
    /// </summary>
    MiterRevert = 1,

    /// <summary>
    /// Joins lines by connecting them with a circular arc centered at the join point,
    /// producing a smooth, rounded corner.
    /// </summary>
    Round = 2,

    /// <summary>
    /// Joins lines by connecting the outer corners directly with a straight line,
    /// forming a flat edge at the join point.
    /// </summary>
    Bevel = 3,

    /// <summary>
    /// Joins lines by forming a miter, but if the miter limit is exceeded,
    /// the join falls back to a round join instead of a bevel.
    /// </summary>
    MiterRound = 4
}

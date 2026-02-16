// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

namespace SixLabors.PolygonClipper;

/// <summary>
/// Encodes path commands used by the stroker path-emission pipeline.
/// </summary>
[Flags]
internal enum PathCommand : byte
{
    /// <summary>
    /// Marks the end of a command stream.
    /// </summary>
    Stop = 0,

    /// <summary>
    /// Starts a new contour at the supplied vertex.
    /// </summary>
    MoveTo = 1,

    /// <summary>
    /// Emits a line segment to the supplied vertex.
    /// </summary>
    LineTo = 2,

    /// <summary>
    /// Terminates the current contour and applies path flags.
    /// </summary>
    EndPoly = 0x0F,

    /// <summary>
    /// Bit mask for extracting the command portion of a value.
    /// </summary>
    Mask = 0x0F
}

// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

namespace SixLabors.PolygonClipper;

/// <summary>
/// Flags that annotate <see cref="PathCommand.EndPoly"/> path-termination commands.
/// </summary>
[Flags]
internal enum PathFlags : byte
{
    /// <summary>
    /// No path flags are set.
    /// </summary>
    None = 0,

    /// <summary>
    /// Marks a counter-clockwise contour orientation.
    /// </summary>
    Ccw = 0x10,

    /// <summary>
    /// Marks a clockwise contour orientation.
    /// </summary>
    Cw = 0x20,

    /// <summary>
    /// Marks the contour as closed.
    /// </summary>
    Close = 0x40,

    /// <summary>
    /// Bit mask for extracting the flag portion of a command value.
    /// </summary>
    Mask = 0xF0
}

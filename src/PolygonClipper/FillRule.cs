// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

namespace SixLabors.PolygonClipper;

/// <summary>
/// Specifies how directed contours define a filled area.
/// </summary>
/// <remarks>
/// Counterclockwise contours contribute positive winding; clockwise contours contribute negative winding.
/// The directed vertices determine winding independently of contour parent and hole metadata.
/// </remarks>
public enum FillRule
{
    /// <summary>The area is filled where the winding count is odd.</summary>
    EvenOdd = 0,

    /// <summary>The area is filled where the winding count is non-zero.</summary>
    NonZero = 1,

    /// <summary>The area is filled where the winding count is positive.</summary>
    Positive = 2,

    /// <summary>The area is filled where the winding count is negative.</summary>
    Negative = 3
}

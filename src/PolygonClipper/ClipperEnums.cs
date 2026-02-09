// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

namespace SixLabors.PolygonClipper;

internal enum ClipperFillRule
{
    EvenOdd,
    NonZero,
    Positive,
    Negative
}

internal enum ClipperPathType
{
    Subject,
    Clip
}

internal enum ClipperPointInPolygonResult
{
    IsOn = 0,
    IsInside = 1,
    IsOutside = 2
}

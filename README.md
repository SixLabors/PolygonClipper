<h1 align="center">

<img src="https://github.com/SixLabors/Branding/raw/main/icons/polygonclipper/sixlabors.polygonclipper.svg?sanitize=true" alt="SixLabors.PolygonClipper" width="256"/>
<br/>
SixLabors.PolygonClipper
</h1>

<div align="center">

[![Build Status](https://img.shields.io/github/actions/workflow/status/SixLabors/PolygonClipper/build-and-test.yml?branch=main)](https://github.com/SixLabors/PolygonClipper/actions)
[![codecov](https://codecov.io/github/SixLabors/PolygonClipper/graph/badge.svg?token=ZEK38fv18V)](https://codecov.io/github/SixLabors/PolygonClipper)
[![License: Six Labors Split](https://img.shields.io/badge/license-Six%20Labors%20Split-%23e30183)](https://github.com/SixLabors/PolygonClipper/blob/main/LICENSE)

</div>

SixLabors.PolygonClipper performs polygon clipping and stroking in C#.
It uses the Martinez-Rueda algorithm with double coordinates. It supports holes,
self-intersections, overlapping edges, and multiple contours.

## Clipping

The Boolean operations are `Intersection`, `Union`, `Difference`, and `Xor`.
Each accepts `EvenOdd`, `NonZero`, `Positive`, or `Negative` filling.

```csharp
using SixLabors.PolygonClipper;

Polygon subject =
[
    [new Vertex(0, 0), new Vertex(10, 0), new Vertex(10, 10), new Vertex(0, 10)]
];
Polygon clip =
[
    [new Vertex(5, 5), new Vertex(15, 5), new Vertex(15, 15), new Vertex(5, 15)]
];

// Intersect both polygons using NonZero filling.
Polygon result = PolygonClipper.Intersection(subject, clip, FillRule.NonZero);
```

The subject and clip can use different fill rules:

```csharp
// Use EvenOdd for the subject and NonZero for the clip.
Polygon result = PolygonClipper.Intersection(subject, clip, FillRule.EvenOdd, FillRule.NonZero);
```

To resolve self-intersections and overlaps in one polygon, use unary union:

```csharp
// Keep every region whose winding count is non-zero.
Polygon result = PolygonClipper.Union(subject, FillRule.NonZero);
```

These operations apply the fill rules during the clipping sweep. They do not require
an input normalization pass. Coordinates must be finite doubles.

Fill rules interpret the input contours. Every result uses positive orientation:
counterclockwise exterior contours and islands, with clockwise holes, in Cartesian coordinates.
Use `NonZero`, `EvenOdd`, or `Positive` filling to consume the resolved area.
`Negative` selects negatively wound input; it does not request negatively oriented output.

## Normalization

`Normalize(polygon, fillRule)` is unary union. It resolves self-intersections and overlaps,
removes zero-area retraces, and returns the full contour hierarchy.
It reads the original contour directions without changing the input or applying a preparation pass.
Small features are not discarded using fixed distance or area thresholds.

```csharp
// Clean the NonZero-filled area and retain parent and hole information.
Polygon result = PolygonClipper.Normalize(subject, FillRule.NonZero);
```

Clipping accepts raw contours and their fill rules directly. It does not require `Normalize` first.
The positive output orientation follows [Clipper2's output convention](https://www.angusj.com/clipper2/Docs/Overview.htm).

## Stroking

`PolygonStroker` creates stroke polygons with configurable joins, caps, and width.
Each source contour has an `IsClosed` property, which defaults to `true`.
Set it to `false` for an open source path. Endpoint coordinates and stroke width do not select closure.
One polygon can contain both open and closed stroke inputs.

Boolean operations use the filled area of each contour and implicitly close open inputs.
Both Boolean and stroke output contain closed contours without a repeated closing vertex.
When `NormalizeOutput` is enabled, stroke cleanup uses unary union with `NonZero` filling.
This preserves the raw outlines' filled area for either sign of stroke width.

```csharp
StrokeOptions options = new()
{
    LineJoin = LineJoin.Round,
    LineCap = LineCap.Round,
    MiterLimit = 4,
    ArcDetailScale = 1
};

// Build a stroke with width 12 and the specified joins and caps.
Polygon result = PolygonStroker.Stroke(subject, width: 12, options);
```

## Complexity

Martinez has a published complexity of `O((n + k) log n)` for `n` input edges and `k` intersections.

This implementation stores active edges in a list, so inserting or removing an edge can take linear time.
It does not guarantee that bound.

## Algorithm References

This project draws on the following algorithm and implementation references:

> F. Martínez et al., "A simple algorithm for Boolean operations on polygons", *Advances in Engineering Software*, 64 (2013), pp. 11-19.  
> https://doi.org/10.1016/j.advengsoft.2013.04.004

> 21re, *rust-geo-booleanop* (Rust Martínez-Rueda implementation reference).  
> https://github.com/21re/rust-geo-booleanop

> Angus Johnson, *Clipper2* polygon clipping library.
> https://github.com/AngusJohnson/Clipper2

## License

Six Labors Split License. See [`LICENSE`](https://github.com/SixLabors/PolygonClipper/blob/main/LICENSE) for details.

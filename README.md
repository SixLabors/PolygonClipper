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

SixLabors.PolygonClipper performs polygon clipping, stroking and offsets in C#.
It uses the Martinez-Rueda algorithm with double coordinates. It supports holes,
self-intersections, overlapping edges, and multiple contours.

## Clipping

The Boolean operations are `Intersection`, `Union`, `Difference`, and `Xor`.
Each combines a subject polygon with a clip polygon. Both operands can contain open and closed contours.
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

These operations apply the fill rules during the clipping sweep. They do not require
an input normalization pass. Coordinates must be finite doubles.

Fill rules interpret closed input contours. Closed results use positive orientation:
counterclockwise exterior contours and islands, with clockwise holes, in Cartesian coordinates.
Use `NonZero`, `EvenOdd`, or `Positive` filling to consume the resolved area.
`Negative` selects negatively wound input; it does not request negatively oriented output.

## Normalization

`Normalize(polygon, fillRule)` cleans one polygon. It resolves self-intersections and overlaps in filled boundaries
and retains their full contour hierarchy. It also resolves overlapping open paths without filling them.
It reads the original contour directions without changing the input or applying a preparation pass.

```csharp
// Clean the NonZero-filled area and retain parent and hole information.
Polygon result = PolygonClipper.Normalize(subject, FillRule.NonZero);
```

Clipping accepts raw contours and their fill rules directly. It does not require `Normalize` first.
The positive output orientation follows [Clipper2's output convention](https://www.angusj.com/clipper2/Docs/Overview.htm).

## Open paths

`Contour.IsClosed` controls the closing edge for both clipping and stroking.
Closed contours enclose filled areas. Open contours contain consecutive line segments with no filled interior.
Fill rules do not change open path membership.

All clipping operations return `Polygon`. Each result contour identifies its geometry:

- A closed contour is a filled boundary. Its first vertex is not repeated at the end.
- An open contour with two or more vertices is a line piece.
- An open contour with one vertex is an isolated point.

Clipping splits linework at intersections and retains the cut endpoints in each resulting piece.
Ordinary source bends stay connected. An explicit line cycle remains open and retains its repeated terminal vertex.
Open contours have no parent or holes. Covered linework and points are omitted when a retained area already represents them.

```csharp
Contour horizontal = [new Vertex(-2, 0), new Vertex(2, 0)];
horizontal.IsClosed = false;
Contour vertical = [new Vertex(0, -2), new Vertex(0, 2)];
vertical.IsClosed = false;

// Return one open contour containing the crossing point (0, 0).
Polygon result = PolygonClipper.Intersection([horizontal], [vertical], FillRule.NonZero);
```

## Stroking

`PolygonStroker` creates stroke polygons with configurable joins, caps, and width.
Each source contour has an `IsClosed` property, which defaults to `true`.
Set it to `false` for an open source path. Endpoint coordinates and stroke width do not select closure.
One polygon can contain both open and closed stroke inputs.

Stroke output contains closed contours without a repeated closing vertex.
When `NormalizeOutput` is enabled, stroke cleanup calls `Normalize` with `NonZero` filling.
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

## Offsets

`PolygonOffsetter.Offset(polygon, delta, fillRule, options)` expands or contracts a filled polygon.
Positive distance expands exterior boundaries and shrinks holes. Negative distance contracts exterior boundaries and expands holes.
Collapsed holes and components disappear. Zero distance returns an independent, resolved copy of the source filled area.

All source contours must be closed. The fill rule interprets the original source contours before their boundaries are offset.
`OffsetOptions` selects the join style, miter limit and arc detail. Offset results always contain resolved boundaries.

```csharp
OffsetOptions offsetOptions = new() { LineJoin = LineJoin.Round };

// Expand the NonZero-filled area by two coordinate units with the selected joins.
Polygon expanded = PolygonOffsetter.Offset(subject, 2, FillRule.NonZero, offsetOptions);

// Contract the same source area by two coordinate units.
Polygon contracted = PolygonOffsetter.Offset(subject, -2, FillRule.NonZero, offsetOptions);
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

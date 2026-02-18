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

SixLabors.PolygonClipper provides high-performance polygon clipping and stroking in C#.
Boolean operations (union, intersection, difference, xor) are implemented with a Martínez-Rueda sweep-line pipeline for complex polygons with holes and multiple contours.
Contour normalization is handled by a dedicated Vatti/Clipper2-inspired pipeline (`PolygonClipper.Normalize`) that resolves self-intersections/overlaps into positive-winding output.
`PolygonStroker` can optionally run that normalization pass on emitted stroke geometry.

## Features

- Works with non-convex polygons, including holes and multiple disjoint regions
- Handles edge cases like overlapping edges and vertical segments
- Preserves topology: output polygons include hole/contour hierarchy
- Deterministic and robust sweep line algorithm with O((n + k) log n) complexity
- Includes `PolygonClipper.Normalize` (Clipper2-inspired) for positive-winding contour normalization
- Includes `PolygonStroker` for configurable geometric stroking (joins, caps, miter limits)
- Uses double precision geometry without coordinate quantization

## Usage

The API centers around `Polygon` and `Contour` types. Construct input polygons using contours, then apply Boolean operations via `PolygonClipper`:

```csharp
Polygon result = PolygonClipper.Union(subject, clipping);
```

Boolean operations can process self-intersecting inputs directly.
Use normalization when you want canonical positive-winding contours (for example, before export or rendering pipelines that rely on winding semantics):

```csharp
Polygon clean = PolygonClipper.Normalize(input);
```

`Normalize` uses a fixed positive-winding normalization path.

### Stroking

Use `PolygonStroker` to generate filled stroke polygons from input contours:

```csharp
Polygon stroked = PolygonStroker.Stroke(input, width: 12);
```

Configure join/cap behavior through `StrokeOptions`:

```csharp
StrokeOptions options = new()
{
    LineJoin = LineJoin.Round,
    LineCap = LineCap.Round,
    MiterLimit = 4,
    InnerMiterLimit = 1.01,
    ArcDetailScale = 1
};

Polygon stroked = PolygonStroker.Stroke(input, width: 12, options);
```

## Algorithm References

This project draws on the following algorithm and implementation references:

> F. Martínez et al., "A simple algorithm for Boolean operations on polygons", *Advances in Engineering Software*, 64 (2013), pp. 11-19.  
> https://doi.org/10.1016/j.advengsoft.2013.04.004

> B. R. Vatti, "A generic solution to polygon clipping", *Communications of the ACM*, 35(7), 1992, pp. 56-63.  
> https://dl.acm.org/doi/pdf/10.1145/129902.129906

> 21re, *rust-geo-booleanop* (Rust Martínez-Rueda implementation reference).  
> https://github.com/21re/rust-geo-booleanop

> Angus Johnson, *Clipper2* polygon clipping library (reference implementation for the self-intersection pipeline).  
> https://github.com/AngusJohnson/Clipper2

## License

Six Labors Split License. See [`LICENSE`](https://github.com/SixLabors/PolygonClipper/blob/main/LICENSE) for details.

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

SixLabors.PolygonClipper provides boolean operations on polygons in C# using a Martínez-Rueda sweep-line implementation. It supports union, intersection, difference, and xor on complex polygons with holes and multiple contours. Self-intersection normalization is provided via a separate Clipper2-inspired pipeline.

## Features

- Works with non-convex polygons, including holes and multiple disjoint regions
- Handles edge cases like overlapping edges and vertical segments
- Preserves topology: output polygons include hole/contour hierarchy
- Deterministic and robust sweep line algorithm with O((n + k) log n) complexity
- Includes self-intersection removal (Clipper2-inspired) to normalize input contours for boolean operations
- Uses double precision geometry without coordinate quantization

## Usage

The API centers around `Polygon` and `Contour` types. Construct input polygons using contours, then apply Boolean operations via the `PolygonClipper` class:

```csharp
Polygon result = PolygonClipper.Intersection(subject, clipping);
```

You can also normalize input before boolean operations if your contours contain self-intersections:

```csharp
Polygon clean = PolygonClipper.RemoveSelfIntersections(input);
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

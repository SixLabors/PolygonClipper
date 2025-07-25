# PolygonClipper

[![License: Six Labors Split](https://img.shields.io/badge/license-Six%20Labors%20Split-%23e30183)](https://github.com/SixLabors/PolygonClipper/blob/master/LICENSE)

A C# implementation of the Martínez–Rueda algorithm for performing Boolean operations on polygons. This library supports union, intersection, difference, and xor operations on complex polygons with holes, multiple contours, and self-intersections.

## Features

- Works with non-convex polygons, including holes and multiple disjoint regions
- Handles edge cases like overlapping edges and vertical segments
- Preserves topology: output polygons include hole/contour hierarchy
- Deterministic and robust sweep line algorithm with O((n + k) log n) complexity

## Usage

The API centers around `Polygon` and `Contour` types. Construct input polygons using contours, then apply Boolean operations via the `PolygonClipper` class:

```csharp
Polygon result = PolygonClipper.Intersect(subject, clipping);
```

## Based On

This implementation is based on the algorithm described in:

> F. Martínez et al., "A simple algorithm for Boolean operations on polygons", *Advances in Engineering Software*, 64 (2013), pp. 11–19.  
> https://sci-hub.se/10.1016/j.advengsoft.2013.04.004

## License

Six Labors Split License. See [`LICENSE`](https://github.com/SixLabors/PolygonClipper/blob/main/LICENSE) for details.

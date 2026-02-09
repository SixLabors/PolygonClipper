// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

namespace SixLabors.PolygonClipper;

// ClipVertex: a pre-clipping data structure. It is used to separate polygons
// into ascending and descending 'bounds' (or sides) that start at local
// minima and ascend to a local maxima, before descending again.
[Flags]
internal enum VertexFlags
{
    None = 0,
    LocalMax = 4,
    LocalMin = 8
}

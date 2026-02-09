// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

namespace SixLabors.PolygonClipper;

internal class ReusableClipperData
{
    internal ReusableClipperData()
    {
    }

    internal List<LocalMinima> MinimaList { get; } = [];

    internal VertexPoolList VertexList { get; } = [];

    internal void Clear()
    {
        this.MinimaList.Clear();
        this.VertexList.Clear();
    }

    internal void AddPaths(List<Contour> paths, ClipperPathType pt, bool isOpen) => ClipperInputBuilder.AddPathsToVertexList(paths, pt, isOpen, this.MinimaList, this.VertexList);
}

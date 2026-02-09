// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

namespace SixLabors.PolygonClipper;

internal static class ClipperInputBuilder
{
    internal static void AddLocMin(ClipVertex vert, ClipperPathType polytype, bool isOpen, List<LocalMinima> minimaList)
    {
        // make sure the ClipVertex is added only once ...
        if ((vert.Flags & VertexFlags.LocalMin) != VertexFlags.None)
        {
            return;
        }

        vert.Flags |= VertexFlags.LocalMin;

        LocalMinima lm = new(vert, polytype, isOpen);
        minimaList.Add(lm);
    }

    internal static void AddPathsToVertexList(List<Contour> paths, ClipperPathType polytype, bool isOpen, List<LocalMinima> minimaList, VertexPoolList vertexList)
    {
        int totalVertCnt = 0;
        foreach (Contour path in paths)
        {
            totalVertCnt += path.Count;
        }

        vertexList.EnsureCapacity(vertexList.Count + totalVertCnt);

        foreach (Contour path in paths)
        {
            ClipVertex? v0 = null;
            ClipVertex? prev_v = null;
            ClipVertex? curr_v;
            foreach (Vertex pt in path)
            {
                if (v0 == null)
                {
                    v0 = vertexList.Add(pt, VertexFlags.None, null);
                    prev_v = v0;
                    continue;
                }

                if (!ClipGeometry.PointEquals(prev_v!.Point, pt))
                {
                    curr_v = vertexList.Add(pt, VertexFlags.None, prev_v);
                    prev_v.Next = curr_v;
                    prev_v = curr_v;
                }
            }

            if (v0 == null || prev_v?.Prev == null)
            {
                continue;
            }

            if (!isOpen && ClipGeometry.PointEquals(prev_v.Point, v0.Point))
            {
                prev_v = prev_v.Prev;
            }

            prev_v.Next = v0;
            v0.Prev = prev_v;
            if (!isOpen && prev_v.Next == prev_v)
            {
                continue;
            }

            // OK, we have a valid path
            bool going_up;
            if (isOpen)
            {
                curr_v = v0.Next;
                while (curr_v != v0 && ClipGeometry.IsAlmostZero(curr_v!.Point.Y - v0.Point.Y))
                {
                    curr_v = curr_v.Next;
                }

                going_up = curr_v!.Point.Y <= v0.Point.Y;
                if (going_up)
                {
                    v0.Flags = VertexFlags.OpenStart;
                    AddLocMin(v0, polytype, true, minimaList);
                }
                else
                {
                    v0.Flags = VertexFlags.OpenStart | VertexFlags.LocalMax;
                }
            }

            // Closed path.
            else
            {
                prev_v = v0.Prev;
                while (prev_v != v0 && ClipGeometry.IsAlmostZero(prev_v!.Point.Y - v0.Point.Y))
                {
                    prev_v = prev_v.Prev;
                }

                if (prev_v == v0)
                {
                    // Only open paths can be completely flat.
                    continue;
                }

                going_up = prev_v.Point.Y > v0.Point.Y;
            }

            bool going_up0 = going_up;
            prev_v = v0;
            curr_v = v0.Next;
            while (curr_v != v0)
            {
                if (curr_v!.Point.Y > prev_v.Point.Y && going_up)
                {
                    prev_v.Flags |= VertexFlags.LocalMax;
                    going_up = false;
                }
                else if (curr_v.Point.Y < prev_v.Point.Y && !going_up)
                {
                    going_up = true;
                    AddLocMin(prev_v, polytype, isOpen, minimaList);
                }

                prev_v = curr_v;
                curr_v = curr_v.Next;
            }

            if (isOpen)
            {
                prev_v.Flags |= VertexFlags.OpenEnd;
                if (going_up)
                {
                    prev_v.Flags |= VertexFlags.LocalMax;
                }
                else
                {
                    AddLocMin(prev_v, polytype, isOpen, minimaList);
                }
            }
            else if (going_up != going_up0)
            {
                if (going_up0)
                {
                    AddLocMin(prev_v, polytype, false, minimaList);
                }
                else
                {
                    prev_v.Flags |= VertexFlags.LocalMax;
                }
            }
        }
    }
}

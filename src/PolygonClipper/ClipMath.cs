// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System.Runtime.CompilerServices;

namespace SixLabors.PolygonClipper;

internal static class ClipMath
{
    public static readonly ClipBounds InvalidBounds = new(false);

    public static double Area(Contour path)
    {
        int count = path.Count;
        if (count < 3)
        {
            return 0D;
        }

        double area = 0D;
        Vertex prev = path[count - 1];
        for (int i = 0; i < count; i++)
        {
            Vertex current = path[i];
            area += (prev.Y + current.Y) * (prev.X - current.X);
            prev = current;
        }

        return area * 0.5D;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static double PerpendicularDistanceSquared(Vertex point, Vertex line1, Vertex line2)
    {
        double a = point.X - line1.X;
        double b = point.Y - line1.Y;
        double c = line2.X - line1.X;
        double d = line2.Y - line1.Y;
        if (c == 0D && d == 0D)
        {
            return 0D;
        }

        return ((a * d) - (c * b)) * ((a * d) - (c * b)) / ((c * c) + (d * d));
    }
}

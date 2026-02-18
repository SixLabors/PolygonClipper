// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

namespace SixLabors.PolygonClipper.Tests;

public class PolygonClipperContourClosureTests
{
    [Fact]
    public void BooleanOperations_OpenAndClosedContours_ProduceSameOutput()
    {
        Polygon openSubject = [CreateRectangleContour(0, 0, 10, 10, false)];
        Polygon openClip = [CreateRectangleContour(5, 5, 15, 15, false)];
        Polygon closedSubject = [CreateRectangleContour(0, 0, 10, 10, true)];
        Polygon closedClip = [CreateRectangleContour(5, 5, 15, 15, true)];

        AssertEquivalent(
            PolygonClipper.Intersection(openSubject, openClip),
            PolygonClipper.Intersection(closedSubject, closedClip));
        AssertEquivalent(
            PolygonClipper.Union(openSubject, openClip),
            PolygonClipper.Union(closedSubject, closedClip));
        AssertEquivalent(
            PolygonClipper.Difference(openSubject, openClip),
            PolygonClipper.Difference(closedSubject, closedClip));
        AssertEquivalent(
            PolygonClipper.Xor(openSubject, openClip),
            PolygonClipper.Xor(closedSubject, closedClip));
    }

    private static Contour CreateRectangleContour(
        double minX,
        double minY,
        double maxX,
        double maxY,
        bool closed)
    {
        Contour contour = [];
        contour.Add(new Vertex(minX, minY));
        contour.Add(new Vertex(maxX, minY));
        contour.Add(new Vertex(maxX, maxY));
        contour.Add(new Vertex(minX, maxY));
        if (closed)
        {
            contour.Add(contour[0]);
        }

        return contour;
    }

    private static void AssertEquivalent(Polygon left, Polygon right)
    {
        List<List<Vertex>> leftContours = BuildNormalizedContours(left);
        List<List<Vertex>> rightContours = BuildNormalizedContours(right);

        Assert.Equal(leftContours.Count, rightContours.Count);
        for (int i = 0; i < leftContours.Count; i++)
        {
            List<Vertex> leftContour = leftContours[i];
            List<Vertex> rightContour = rightContours[i];
            Assert.Equal(leftContour.Count, rightContour.Count);
            for (int j = 0; j < leftContour.Count; j++)
            {
                Assert.Equal(leftContour[j], rightContour[j]);
            }
        }
    }

    private static List<List<Vertex>> BuildNormalizedContours(Polygon polygon)
    {
        List<List<Vertex>> contours = new(polygon.Count);
        for (int i = 0; i < polygon.Count; i++)
        {
            contours.Add(NormalizeContour(polygon[i]));
        }

        contours.Sort(CompareContourSequences);
        return contours;
    }

    private static List<Vertex> NormalizeContour(Contour contour)
    {
        int count = contour.Count;
        if (count > 1 && contour[0] == contour[^1])
        {
            count--;
        }

        List<Vertex> vertices = new(count);
        for (int i = 0; i < count; i++)
        {
            vertices.Add(contour[i]);
        }

        if (vertices.Count <= 1)
        {
            return vertices;
        }

        List<Vertex> forward = NormalizeDirection(vertices);
        vertices.Reverse();
        List<Vertex> reverse = NormalizeDirection(vertices);

        return CompareContourSequences(forward, reverse) <= 0 ? forward : reverse;
    }

    private static List<Vertex> NormalizeDirection(List<Vertex> vertices)
    {
        int count = vertices.Count;
        int bestStart = 0;
        for (int i = 1; i < count; i++)
        {
            if (CompareRotation(vertices, i, bestStart) < 0)
            {
                bestStart = i;
            }
        }

        List<Vertex> normalized = new(count);
        for (int i = 0; i < count; i++)
        {
            normalized.Add(vertices[(bestStart + i) % count]);
        }

        return normalized;
    }

    private static int CompareRotation(List<Vertex> vertices, int leftStart, int rightStart)
    {
        int count = vertices.Count;
        for (int i = 0; i < count; i++)
        {
            Vertex left = vertices[(leftStart + i) % count];
            Vertex right = vertices[(rightStart + i) % count];
            int compareX = left.X.CompareTo(right.X);
            if (compareX != 0)
            {
                return compareX;
            }

            int compareY = left.Y.CompareTo(right.Y);
            if (compareY != 0)
            {
                return compareY;
            }
        }

        return 0;
    }

    private static int CompareContourSequences(List<Vertex> left, List<Vertex> right)
    {
        int countCompare = left.Count.CompareTo(right.Count);
        if (countCompare != 0)
        {
            return countCompare;
        }

        for (int i = 0; i < left.Count; i++)
        {
            int compareX = left[i].X.CompareTo(right[i].X);
            if (compareX != 0)
            {
                return compareX;
            }

            int compareY = left[i].Y.CompareTo(right[i].Y);
            if (compareY != 0)
            {
                return compareY;
            }
        }

        return 0;
    }
}

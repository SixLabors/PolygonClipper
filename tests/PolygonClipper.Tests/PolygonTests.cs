// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

namespace SixLabors.PolygonClipper.Tests;

public class PolygonTests
{
    private static Contour CreateSquare(double x, double y, double size)
    {
        Contour contour = new(4);
        contour.Add(new Vertex(x, y));
        contour.Add(new Vertex(x + size, y));
        contour.Add(new Vertex(x + size, y + size));
        contour.Add(new Vertex(x, y + size));
        return contour;
    }

    [Fact]
    public void DefaultConstructor_CreatesEmptyPolygon()
    {
        Polygon polygon = new();

        Assert.Equal(0, polygon.Count);
        Assert.Equal(0, polygon.VertexCount);
    }

    [Fact]
    public void CapacityConstructor_StartsEmpty()
    {
        Polygon polygon = new(8);

        Assert.Equal(0, polygon.Count);
    }

    [Fact]
    public void Add_AppendsContoursAndIndexerReturnsThem()
    {
        Polygon polygon = new();
        Contour a = CreateSquare(0, 0, 10);
        Contour b = CreateSquare(20, 20, 5);

        polygon.Add(a);
        polygon.Add(b);

        Assert.Equal(2, polygon.Count);
        Assert.Same(a, polygon[0]);
        Assert.Same(b, polygon[1]);
        Assert.Same(b, polygon.GetLastContour());
    }

    [Fact]
    public void VertexCount_SumsAcrossAllContours()
    {
        Polygon polygon = new();
        polygon.Add(CreateSquare(0, 0, 10));
        polygon.Add(CreateSquare(20, 20, 5));

        Contour triangle = new();
        triangle.Add(new Vertex(0, 0));
        triangle.Add(new Vertex(1, 0));
        triangle.Add(new Vertex(0, 1));
        polygon.Add(triangle);

        Assert.Equal(4 + 4 + 3, polygon.VertexCount);
    }

    [Fact]
    public void GetBoundingBox_EmptyPolygon_ReturnsDefault()
    {
        Polygon polygon = new();

        Assert.Equal(default, polygon.GetBoundingBox());
    }

    [Fact]
    public void GetBoundingBox_SingleContour_MatchesContourBox()
    {
        Polygon polygon = new();
        Contour contour = CreateSquare(2, 3, 10);
        polygon.Add(contour);

        Assert.Equal(contour.GetBoundingBox(), polygon.GetBoundingBox());
    }

    [Fact]
    public void GetBoundingBox_MultipleContours_ReturnsUnion()
    {
        Polygon polygon = new();
        polygon.Add(CreateSquare(0, 0, 10));
        polygon.Add(CreateSquare(20, -5, 4));

        Box2 box = polygon.GetBoundingBox();

        Assert.Equal(new Vertex(0, -5), box.Min);
        Assert.Equal(new Vertex(24, 10), box.Max);
    }

    [Fact]
    public void Translate_OffsetsAllContours()
    {
        Polygon polygon = new();
        polygon.Add(CreateSquare(0, 0, 10));
        polygon.Add(CreateSquare(20, 20, 5));

        polygon.Translate(1, 2);

        Assert.Equal(new Vertex(1, 2), polygon[0][0]);
        Assert.Equal(new Vertex(11, 2), polygon[0][1]);
        Assert.Equal(new Vertex(21, 22), polygon[1][0]);
    }

    [Fact]
    public void Clear_RemovesAllContours()
    {
        Polygon polygon = new();
        polygon.Add(CreateSquare(0, 0, 10));
        polygon.Add(CreateSquare(20, 20, 5));

        polygon.Clear();

        Assert.Equal(0, polygon.Count);
        Assert.Equal(0, polygon.VertexCount);
    }

    [Fact]
    public void Enumeration_YieldsContoursInInsertionOrder()
    {
        Polygon polygon = new();
        Contour a = CreateSquare(0, 0, 10);
        Contour b = CreateSquare(20, 20, 5);
        polygon.Add(a);
        polygon.Add(b);

        Contour[] enumerated = [.. polygon];

        Assert.Equal(2, enumerated.Length);
        Assert.Same(a, enumerated[0]);
        Assert.Same(b, enumerated[1]);
    }

    [Fact]
    public void DeepClone_ProducesIndependentCopy()
    {
        Polygon original = new();
        original.Add(CreateSquare(0, 0, 10));
        original.Add(CreateSquare(20, 20, 5));

        Polygon clone = original.DeepClone();

        // Same shape.
        Assert.Equal(original.Count, clone.Count);
        Assert.Equal(original.VertexCount, clone.VertexCount);
        Assert.Equal(original.GetBoundingBox(), clone.GetBoundingBox());

        // Different contour instances.
        for (int i = 0; i < original.Count; i++)
        {
            Assert.NotSame(original[i], clone[i]);
        }

        // Mutating the clone must not affect the original.
        clone.Translate(100, 100);
        Assert.Equal(new Vertex(0, 0), original[0][0]);
    }

    [Fact]
    public void Join_AppendsAllContours()
    {
        Polygon a = new();
        a.Add(CreateSquare(0, 0, 10));
        a.Add(CreateSquare(20, 0, 5));

        Polygon b = new();
        b.Add(CreateSquare(40, 0, 5));
        b.Add(CreateSquare(60, 0, 5));

        a.Join(b);

        Assert.Equal(4, a.Count);
        Assert.Equal(16, a.VertexCount);
        Assert.Equal(new Vertex(40, 0), a[2][0]);
        Assert.Equal(new Vertex(60, 0), a[3][0]);
    }

    [Fact]
    public void Join_OnEmptyTarget_CopiesContoursFromSource()
    {
        Polygon target = new();
        Polygon source = new();
        source.Add(CreateSquare(0, 0, 10));

        target.Join(source);

        Assert.Equal(1, target.Count);
        Assert.Equal(4, target.VertexCount);
    }

    [Fact]
    public void ToDebugString_ContainsVertexCoordinates()
    {
        Polygon polygon = new();
        polygon.Add(CreateSquare(0, 0, 10));

        string debug = polygon.ToDebugString();

        Assert.Contains("Vertex(0, 0)", debug);
        Assert.Contains("Vertex(10, 0)", debug);
        Assert.Contains("Vertex(10, 10)", debug);
        Assert.Contains("Vertex(0, 10)", debug);
    }
}

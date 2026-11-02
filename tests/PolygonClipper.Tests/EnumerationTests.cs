// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System.Collections;
using System.Runtime.CompilerServices;

namespace SixLabors.PolygonClipper.Tests;

/// <summary>
/// Checks concrete and interface enumeration of polygons and contour vertices.
/// </summary>
public class EnumerationTests
{
    /// <summary>
    /// Checks that nested concrete foreach loops allocate no enumerator objects.
    /// </summary>
    [Fact]
    public void ConcreteForeach_DoesNotAllocate()
    {
        Polygon polygon = [[new(1, 0), new(2, 0)], [new(3, 0), new(4, 0)]];
        double sum = 0;
        for (int i = 0; i < 256; i++)
        {
            sum += Enumerate(polygon);
        }

        Assert.Equal(2560D, sum);
        sum = 0;
        long before = GC.GetAllocatedBytesForCurrentThread();
        for (int i = 0; i < 1000; i++)
        {
            sum += Enumerate(polygon);
        }

        long allocated = GC.GetAllocatedBytesForCurrentThread() - before;
        Assert.Equal(10000D, sum);
        Assert.Equal(0, allocated);
    }

    /// <summary>
    /// Checks stored order through generic and non-generic interfaces, including empty collections.
    /// </summary>
    /// <param name="empty">Whether the collections have no stored items.</param>
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void InterfaceEnumeration_PreservesStoredOrder(bool empty)
    {
        Contour contour = empty ? [] : [new(3, 4), new(1, 2)];
        Polygon polygon = empty ? [] : [contour, new Contour()];
        IEnumerable<Contour> genericPolygon = polygon;
        IEnumerable<Vertex> genericContour = contour;
        int count = 0;
        foreach (Contour item in genericPolygon)
        {
            Assert.Same(polygon[count++], item);
        }

        Assert.Equal(polygon.Count, count);
        count = 0;
        foreach (object item in (IEnumerable)polygon)
        {
            Assert.Same(polygon[count++], item);
        }

        Assert.Equal(polygon.Count, count);
        count = 0;
        foreach (Vertex vertex in genericContour)
        {
            Assert.Equal(contour[count++], vertex);
        }

        Assert.Equal(contour.Count, count);
        count = 0;
        foreach (object item in (IEnumerable)contour)
        {
            Assert.Equal(contour[count++], Assert.IsType<Vertex>(item));
        }

        Assert.Equal(contour.Count, count);
    }

    /// <summary>
    /// Checks that changing the underlying collection invalidates a concrete enumerator.
    /// </summary>
    [Fact]
    public void ConcreteEnumeration_CollectionChange_InvalidatesEnumerator()
    {
        Contour contour = [new(1, 2), new(3, 4)];
        Polygon polygon = [contour];
        List<Contour>.Enumerator contours = polygon.GetEnumerator();
        List<Vertex>.Enumerator vertices = contour.GetEnumerator();
        Assert.True(contours.MoveNext());
        Assert.True(vertices.MoveNext());

        polygon.Add(new Contour());
        contour.Add(new Vertex(5, 6));
        Assert.Throws<InvalidOperationException>(() => contours.MoveNext());
        Assert.Throws<InvalidOperationException>(() => vertices.MoveNext());
    }

    /// <summary>
    /// Enumerates both concrete collections through their public foreach pattern.
    /// </summary>
    /// <param name="polygon">The stored contours and vertices.</param>
    /// <returns>The sum of the vertex X coordinates.</returns>
    /// <remarks>
    /// The warmup and measured loops must call the same compiled method.
    /// </remarks>
    [MethodImpl(MethodImplOptions.NoInlining)]
    private static double Enumerate(Polygon polygon)
    {
        double sum = 0;
        foreach (Contour contour in polygon)
        {
            foreach (Vertex vertex in contour)
            {
                sum += vertex.X;
            }
        }

        return sum;
    }
}

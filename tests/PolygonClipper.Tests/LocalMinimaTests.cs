// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

namespace SixLabors.PolygonClipper.Tests;

public class LocalMinimaTests
{
    private static SweepVertex Vert(double x, double y)
        => new(new Vertex(x, y), VertexFlags.None, prev: null);

    [Fact]
    public void Constructor_StoresVertex()
    {
        SweepVertex v = Vert(1, 2);

        LocalMinima lm = new(v);

        Assert.Same(v, lm.Vertex);
    }

    [Fact]
    public void Equality_UsesReferenceIdentityOfVertex()
    {
        SweepVertex shared = Vert(1, 2);
        SweepVertex twin = Vert(1, 2);

        LocalMinima a = new(shared);
        LocalMinima b = new(shared);
        LocalMinima c = new(twin);

        Assert.True(a == b);
        Assert.False(a != b);
        Assert.True(a.Equals(b));
        Assert.True(a.Equals((object)b));
        Assert.False(a == c);
        Assert.True(a != c);
        Assert.False(a.Equals(c));
        Assert.False(a.Equals("not a local minima"));
    }

    [Fact]
    public void GetHashCode_MatchesUnderlyingVertexHash()
    {
        SweepVertex v = Vert(7, 9);
        LocalMinima lm = new(v);

        Assert.Equal(v.GetHashCode(), lm.GetHashCode());
    }

    [Fact]
    public void Comparer_OrdersHigherYFirst()
    {
        // The sweep wants higher Y processed first, so the comparer
        // returns a negative value when the second's Y is larger.
        LocalMinima high = new(Vert(0, 10));
        LocalMinima mid = new(Vert(0, 5));
        LocalMinima low = new(Vert(0, 1));

        LocalMinimaComparer comparer = new();

        Assert.True(comparer.Compare(high, low) < 0);
        Assert.True(comparer.Compare(low, high) > 0);
        Assert.Equal(0, comparer.Compare(mid, mid));
    }

    [Fact]
    public void Comparer_SortsCollectionDescendingByY()
    {
        LocalMinima a = new(Vert(0, 1));
        LocalMinima b = new(Vert(0, 10));
        LocalMinima c = new(Vert(0, 5));

        List<LocalMinima> list = [a, b, c];
        list.Sort(new LocalMinimaComparer());

        Assert.Equal(10, list[0].Vertex.Point.Y);
        Assert.Equal(5, list[1].Vertex.Point.Y);
        Assert.Equal(1, list[2].Vertex.Point.Y);
    }
}

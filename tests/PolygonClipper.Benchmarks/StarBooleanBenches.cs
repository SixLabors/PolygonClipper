// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using BenchmarkDotNet.Attributes;
using Clipper2Lib;

namespace SixLabors.PolygonClipper.Benchmarks;

/// <summary>
/// Benchmarks PolygonClipper boolean operations using two overlapping star polygons.
/// </summary>
[MemoryDiagnoser]
[OperationsPerSecond]
public class StarBooleanBenches
{
    private const int ClipperPrecision = 6;
    private Polygon subject;
    private Polygon clipping;
    private PathsD clipperSubject;
    private PathsD clipperClipping;

    [Params(101, 301, 1001)]
    public int VertexCount { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        this.subject = BuildStarPolygon(this.VertexCount, 100D, 0D, 0D);
        this.clipping = BuildStarPolygon(this.VertexCount, 100D, 35D, 22D);
        this.clipperSubject = BuildClipperPaths(this.subject);
        this.clipperClipping = BuildClipperPaths(this.clipping);
    }

    [Benchmark]
    public Polygon PolygonClipperUnion()
        => PolygonClipper.Union(this.subject, this.clipping);

    [Benchmark]
    public PolyTreeD Clipper2Union()
    {
        ClipperD clipper = CreateClipper();
        clipper.AddSubject(this.clipperSubject);
        clipper.AddClip(this.clipperClipping);
        PolyTreeD solution = [];
        clipper.Execute(ClipType.Union, Clipper2Lib.FillRule.EvenOdd, solution);
        return solution;
    }

    [Benchmark]
    public Polygon PolygonClipperIntersection()
        => PolygonClipper.Intersection(this.subject, this.clipping);

    [Benchmark]
    public PolyTreeD Clipper2Intersection()
    {
        ClipperD clipper = CreateClipper();
        clipper.AddSubject(this.clipperSubject);
        clipper.AddClip(this.clipperClipping);
        PolyTreeD solution = [];
        clipper.Execute(ClipType.Intersection, Clipper2Lib.FillRule.EvenOdd, solution);
        return solution;
    }

    [Benchmark]
    public Polygon PolygonClipperDifference()
        => PolygonClipper.Difference(this.subject, this.clipping);

    [Benchmark]
    public PolyTreeD Clipper2Difference()
    {
        ClipperD clipper = CreateClipper();
        clipper.AddSubject(this.clipperSubject);
        clipper.AddClip(this.clipperClipping);
        PolyTreeD solution = [];
        clipper.Execute(ClipType.Difference, Clipper2Lib.FillRule.EvenOdd, solution);
        return solution;
    }

    [Benchmark]
    public Polygon PolygonClipperXor()
        => PolygonClipper.Xor(this.subject, this.clipping);

    [Benchmark]
    public PolyTreeD Clipper2Xor()
    {
        ClipperD clipper = CreateClipper();
        clipper.AddSubject(this.clipperSubject);
        clipper.AddClip(this.clipperClipping);
        PolyTreeD solution = [];
        clipper.Execute(ClipType.Xor, Clipper2Lib.FillRule.EvenOdd, solution);
        return solution;
    }

    private static ClipperD CreateClipper()
        => new(ClipperPrecision)
        {
            PreserveCollinear = false,
            ReverseSolution = false
        };

    private static Polygon BuildStarPolygon(int vertexCount, double radius, double centerX, double centerY)
    {
        if (vertexCount < 5 || (vertexCount & 1) == 0)
        {
            throw new ArgumentOutOfRangeException(nameof(vertexCount), "Vertex count must be an odd number >= 5.");
        }

        int step = (vertexCount - 1) / 2;
        Contour contour = new(vertexCount + 1);
        for (int i = 0; i < vertexCount; i++)
        {
            int index = (i * step) % vertexCount;
            double angle = (index * Math.PI * 2D) / vertexCount;
            double x = centerX + (Math.Cos(angle) * radius);
            double y = centerY + (Math.Sin(angle) * radius);
            contour.Add(new Vertex(x, y));
        }

        contour.Add(contour[0]);
        Polygon polygon = [contour];
        return polygon;
    }

    private static PathsD BuildClipperPaths(Polygon polygon)
    {
        PathsD paths = new(polygon.Count);
        for (int i = 0; i < polygon.Count; i++)
        {
            Contour contour = polygon[i];
            PathD path = new(contour.Count);
            for (int j = 0; j < contour.Count; j++)
            {
                Vertex vertex = contour[j];
                path.Add(new PointD(vertex.X, vertex.Y));
            }

            paths.Add(path);
        }

        return paths;
    }
}

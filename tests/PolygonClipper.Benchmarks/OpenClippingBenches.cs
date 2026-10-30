// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using BenchmarkDotNet.Attributes;
using Clipper2Lib;

namespace SixLabors.PolygonClipper.Benchmarks;

/// <summary>
/// Compares open line clipping against the same closed rectangle and hole.
/// </summary>
[MemoryDiagnoser]
[OperationsPerSecond]
public class OpenClippingBenches
{
    private Polygon subject = null!;
    private Polygon clip = null!;
    private PathsD openSubject = null!;
    private PathsD closedClip = null!;

    /// <summary>
    /// Gets or sets the number of independent source paths.
    /// </summary>
    [Params(10, 1000)]
    public int PathCount { get; set; }

    /// <summary>
    /// Gets or sets the number of vertices in each source path.
    /// </summary>
    [Params(2, 13)]
    public int VerticesPerPath { get; set; }

    /// <summary>
    /// Gets or sets whether both operands exchange their x and y coordinates.
    /// </summary>
    [Params(false, true)]
    public bool Transpose { get; set; }

    /// <summary>
    /// Builds equal input geometry and checks exact output edges before measurement.
    /// </summary>
    [GlobalSetup]
    public void Setup()
    {
        this.subject = new(this.PathCount);
        this.openSubject = new(this.PathCount);
        for (int i = 0; i < this.PathCount; i++)
        {
            Contour contour = new(this.VerticesPerPath) { IsClosed = false };
            PathD path = new(this.VerticesPerPath);
            for (int j = 0; j < this.VerticesPerPath; j++)
            {
                double x = -100D + (1200D * j / (this.VerticesPerPath - 1));
                double y = (i * 100D) + 25D + (this.VerticesPerPath == 2 ? 0D : (j & 1) * 25D);
                if (this.Transpose)
                {
                    (x, y) = (y, x);
                }

                contour.Add(new Vertex(x, y));
                path.Add(new PointD(x, y));
            }

            this.subject.Add(contour);
            this.openSubject.Add(path);
        }

        double height = this.PathCount * 100D;
        Contour exterior = [new(0, 0), new(1000, 0), new(1000, height), new(0, height)];
        Contour hole = [new(300, height * 0.25D), new(300, height * 0.75D), new(700, height * 0.75D), new(700, height * 0.25D)];
        this.clip = new(2);
        this.closedClip = new(2);
        foreach (Contour contour in new[] { exterior, hole })
        {
            PathD path = new(contour.Count);
            Contour transformed = new(contour.Count);
            foreach (Vertex vertex in contour)
            {
                Vertex point = this.Transpose ? new(vertex.Y, vertex.X) : vertex;
                path.Add(new PointD(point.X, point.Y));
                transformed.Add(point);
            }

            this.closedClip.Add(path);
            this.clip.Add(transformed);
        }

        Polygon result = (Polygon)this.PolygonClipperIntersection();
        PathsD expected = (PathsD)this.Clipper2Intersection();
        List<(double X1, double Y1, double X2, double Y2)> actualEdges = [];
        foreach (Contour contour in result)
        {
            if (contour.IsClosed)
            {
                throw new InvalidOperationException("Open-subject intersection returned an area contour.");
            }

            for (int i = 1; i < contour.Count; i++)
            {
                AddEdge(actualEdges, contour[i - 1].X, contour[i - 1].Y, contour[i].X, contour[i].Y);
            }
        }

        List<(double X1, double Y1, double X2, double Y2)> expectedEdges = [];
        foreach (PathD path in expected)
        {
            for (int i = 1; i < path.Count; i++)
            {
                AddEdge(expectedEdges, path[i - 1].x, path[i - 1].y, path[i].x, path[i].y);
            }
        }

        actualEdges.Sort();
        expectedEdges.Sort();
        if (!actualEdges.SequenceEqual(expectedEdges))
        {
            throw new InvalidOperationException("The benchmark inputs did not produce equivalent open output edges.");
        }
    }

    /// <summary>
    /// Clips the original open paths with the closed mask using Martinez.
    /// </summary>
    /// <returns>The open result contours.</returns>
    [Benchmark]
    public object PolygonClipperIntersection() => PolygonClipper.Intersection(this.subject, this.clip, FillRule.NonZero);

    /// <summary>
    /// Clips the same original paths with the same closed mask using Clipper2.
    /// </summary>
    /// <returns>The open result paths.</returns>
    [Benchmark(Baseline = true)]
    public object Clipper2Intersection()
    {
        ClipperD engine = new(6);
        engine.AddOpenSubject(this.openSubject);
        engine.AddClip(this.closedClip);
        PathsD closed = [];
        PathsD open = [];
        engine.Execute(ClipType.Intersection, Clipper2Lib.FillRule.NonZero, closed, open);
        return open;
    }

    /// <summary>
    /// Adds a direction-independent edge without rounding coordinates or changing subdivisions.
    /// </summary>
    /// <param name="edges">The collected output edges.</param>
    /// <param name="x1">The first endpoint's x coordinate.</param>
    /// <param name="y1">The first endpoint's y coordinate.</param>
    /// <param name="x2">The second endpoint's x coordinate.</param>
    /// <param name="y2">The second endpoint's y coordinate.</param>
    private static void AddEdge(List<(double X1, double Y1, double X2, double Y2)> edges, double x1, double y1, double x2, double y2)
    {
        if (x1 < x2 || (x1 == x2 && y1 <= y2))
        {
            edges.Add((x1, y1, x2, y2));
        }
        else
        {
            edges.Add((x2, y2, x1, y1));
        }
    }
}

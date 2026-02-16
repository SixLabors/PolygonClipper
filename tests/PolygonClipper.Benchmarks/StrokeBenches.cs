// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using BenchmarkDotNet.Attributes;
using Clipper2Lib;

namespace SixLabors.PolygonClipper.Benchmarks;

/// <summary>
/// Benchmarks stroke generation and clipping against Clipper2 path inflation.
/// </summary>
[MemoryDiagnoser]
[OperationsPerSecond]
public class StrokeBenches
{
    private const int ClipperPrecision = 6;
    private const double StrokeWidth = 12D;
    private Polygon polygon;
    private PathsD clipperPaths;
    private StrokeOptions strokeOptions;

    [Params(101, 301, 1001)]
    public int VertexCount { get; set; }

    [GlobalSetup]
    public void Setup()
    {
        this.polygon = BuildCompoundGearPolygon(this.VertexCount);
        this.clipperPaths = BuildClipperPaths(this.polygon);
        this.strokeOptions = new StrokeOptions()
        {
            LineJoin = LineJoin.Round,
            LineCap = LineCap.Round,
        };
    }

    [Benchmark]
    public Polygon PolygonStrokerProcessPolygonAndClip()
        => PolygonStroker.Stroke(this.polygon, StrokeWidth, this.strokeOptions);

    [Benchmark(Baseline = true)]
    public PathsD Clipper2InflatePaths()
        => Clipper.InflatePaths(
            this.clipperPaths,
            StrokeWidth * 0.5D,
            JoinType.Round,
            EndType.Round,
            4D,
            ClipperPrecision);

    private static Polygon BuildCompoundGearPolygon(int toothCount)
    {
        if (toothCount < 3)
        {
            throw new ArgumentOutOfRangeException(nameof(toothCount), "Tooth count must be >= 3.");
        }

        Contour outer = BuildGearContour(toothCount, 120D, 104D, 0D, 0D, clockwise: false);
        Contour inner = BuildGearContour(toothCount, 62D, 50D, 0D, 0D, clockwise: true);
        return [outer, inner];
    }

    private static Contour BuildGearContour(
        int toothCount,
        double outerRadius,
        double innerRadius,
        double centerX,
        double centerY,
        bool clockwise)
    {
        int vertexCount = toothCount * 2;
        double angleStep = Math.PI / toothCount;
        Contour contour = new(vertexCount + 1);

        for (int i = 0; i < vertexCount; i++)
        {
            double radius = (i & 1) == 0 ? outerRadius : innerRadius;
            double angle = i * angleStep;
            contour.Add(
                new Vertex(
                    centerX + (Math.Cos(angle) * radius),
                    centerY + (Math.Sin(angle) * radius)));
        }

        contour.Add(contour[0]);
        bool shouldBeCounterClockwise = !clockwise;
        if (contour.IsCounterClockwise() != shouldBeCounterClockwise)
        {
            contour.Reverse();
        }

        return contour;
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

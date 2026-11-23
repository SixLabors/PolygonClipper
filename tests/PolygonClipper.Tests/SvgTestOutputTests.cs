// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System.Globalization;
using System.Xml.Linq;

namespace SixLabors.PolygonClipper.Tests;

/// <summary>
/// Checks SVG output through real clipping results.
/// </summary>
public class SvgTestOutputTests
{
    private static readonly XNamespace Svg = "http://www.w3.org/2000/svg";

    /// <summary>
    /// Checks explicit closure even when an open contour repeats its first vertex.
    /// </summary>
    /// <param name="closed">Whether the input includes its closing edge.</param>
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SaveSvg_PreservesExplicitClosure(bool closed)
    {
        Contour contour = new() { IsClosed = closed };
        contour.Add(new(0, 0));
        contour.Add(new(6, 0));
        contour.Add(new(0, 6));
        contour.Add(new(0, 0));
        Polygon subject = [contour];
        Polygon clip = [];
        Polygon actual = PolygonClipper.Union(subject, clip, FillRule.NonZero);
        string before = subject.ToDebugString();

        string path = SvgTestOutput.SaveSvg(
            nameof(SvgTestOutputTests), nameof(this.SaveSvg_PreservesExplicitClosure), $"Closed-{closed}", subject, clip, actual);
        string outputDirectory = Path.Combine(TestEnvironment.SolutionDirectoryFullPath, "tests", "Images", "ActualOutput");
        Assert.StartsWith(outputDirectory + Path.DirectorySeparatorChar, path);
        Assert.Equal(
            Path.Combine(outputDirectory, nameof(SvgTestOutputTests), $"SaveSvg_PreservesExplicitClosure_Closed-{closed}.svg"),
            path);

        XElement root = XDocument.Load(path).Root!;
        XElement input = Assert.Single(root.Elements().Single(element => (string)element.Attribute("id") == "inputs")
            .Descendants(Svg + "path"));

        Assert.Equal(closed, ((string)input.Attribute("d")!).EndsWith(" Z", StringComparison.Ordinal));
        Assert.Equal(closed, (bool)input.Attribute("data-closed")!);
        Assert.Equal("none", (string)input.Attribute("fill")!);
        Assert.Equal(before, subject.ToDebugString());
        Assert.Equal(closed, contour.IsClosed);
        Assert.DoesNotContain(root.Elements(), element => (string)element.Attribute("id") == "expected");
    }

    /// <summary>
    /// Checks compound holes, nested islands and a shared scale across a large coordinate range.
    /// </summary>
    /// <param name="scale">The source coordinate scale.</param>
    [Theory]
    [InlineData(1E-90)]
    [InlineData(1D)]
    [InlineData(1E90)]
    public void SaveSvg_PreservesHolesAndIslands(double scale)
    {
        Polygon subject =
        [
            [new(0, 0), new(10 * scale, 0), new(10 * scale, 10 * scale), new(0, 10 * scale)],
            [new(2 * scale, 2 * scale), new(8 * scale, 2 * scale), new(8 * scale, 8 * scale), new(2 * scale, 8 * scale)],
            [new(4 * scale, 4 * scale), new(6 * scale, 4 * scale), new(6 * scale, 6 * scale), new(4 * scale, 6 * scale)]
        ];
        Polygon actual = PolygonClipper.Normalize(subject, FillRule.EvenOdd);
        Assert.Equal(3, actual.Count);
        string before = actual.ToDebugString();

        // The expected rings share a direction. Outlines must preserve their closure without adding a filled area.
        string details = FormattableString.Invariant($"Scale-{scale:R}");
        string path = SvgTestOutput.SaveSvg(
            nameof(SvgTestOutputTests), nameof(this.SaveSvg_PreservesHolesAndIslands), details, subject, [], actual, subject);

        XElement root = XDocument.Load(path).Root!;
        XElement actualPanel = root.Elements().Single(element => (string)element.Attribute("id") == "actual");
        XElement expectedPanel = root.Elements().Single(element => (string)element.Attribute("id") == "expected");
        foreach (XElement panel in new[] { actualPanel, expectedPanel })
        {
            XElement[] contours = panel.Descendants(Svg + "path").ToArray();
            Assert.Equal(3, contours.Length);
            Assert.All(contours, contour =>
            {
                Assert.Equal("contour", (string)contour.Attribute("class")!);
                Assert.Equal("none", (string)contour.Attribute("fill")!);
                string data = (string)contour.Attribute("d")!;
                Assert.Equal(1, data.Count(value => value == 'M'));
                Assert.Equal(1, data.Count(value => value == 'Z'));
                Assert.DoesNotContain("NaN", data);
                Assert.DoesNotContain("Infinity", data);
            });
        }

        Assert.All(root.Descendants(Svg + "path"), contour => Assert.Equal("none", (string)contour.Attribute("fill")!));

        XElement[] outlines = actualPanel.Descendants(Svg + "path").Where(element => (string)element.Attribute("class") == "contour").ToArray();
        for (int i = 0; i < actual.Count; i++)
        {
            Assert.Equal(actual[i].Depth, (int)outlines[i].Attribute("data-depth")!);
            Assert.Equal(actual[i].ParentIndex?.ToString(CultureInfo.InvariantCulture) ?? "none", (string)outlines[i].Attribute("data-parent")!);
            string holes = string.Join(" ", Enumerable.Range(0, actual[i].HoleCount).Select(actual[i].GetHoleIndex));
            Assert.Equal(holes, (string)outlines[i].Attribute("data-holes")!);
        }

        // Both panels map the same exterior vertex to (20, 380), regardless of source scale.
        string expectedExterior = (string)expectedPanel.Descendants(Svg + "path")
            .Single(element => (string)element.Attribute("class") == "contour" && (int)element.Attribute("data-contour")! == 0).Attribute("d")!;
        Assert.StartsWith("M 20 380", expectedExterior);
        Assert.All(root.Descendants(Svg + "svg"), element => Assert.Equal("0 0 400 400", (string)element.Attribute("viewBox")!));

        // A ten-unit exterior occupies 360 pixels. A 60-pixel scale bar must therefore
        // represent 5/3 source units at every tested coordinate scale.
        XElement scaleBar = root.Elements().Single(element => (string)element.Attribute("id") == "scale");
        XElement bar = scaleBar.Elements(Svg + "line").Single(element => element.Attribute("data-units") is not null);
        Assert.Equal(60D, (double)bar.Attribute("x2")! - (double)bar.Attribute("x1")!, 10);
        Assert.Equal(5D / 3, (double)bar.Attribute("data-units")! / scale, 12);
        Assert.EndsWith(" units", scaleBar.Element(Svg + "text")!.Value);
        Assert.Equal(before, actual.ToDebugString());
    }

    /// <summary>
    /// Checks point results and culture-independent source coordinates.
    /// </summary>
    [Fact]
    public void SaveSvg_PreservesPointResults()
    {
        Polygon subject = [Open(new(-2, 0.5), new(2, 0.5))];
        Polygon clip = [Open(new(0.25, -2), new(0.25, 2))];
        Polygon actual = PolygonClipper.Intersection(subject, clip, FillRule.NonZero);
        Assert.Equal(new Vertex(0.25, 0.5), Assert.Single(Assert.Single(actual)));
        CultureInfo previous = CultureInfo.CurrentCulture;
        try
        {
            CultureInfo.CurrentCulture = CultureInfo.GetCultureInfo("fr-FR");

            string path = SvgTestOutput.SaveSvg(
                nameof(SvgTestOutputTests), nameof(this.SaveSvg_PreservesPointResults), string.Empty, subject, clip, actual);

            XElement root = XDocument.Load(path).Root!;
            XElement panel = root.Elements().Single(element => (string)element.Attribute("id") == "actual");
            XElement point = Assert.Single(panel.Descendants(Svg + "rect"), element => (string)element.Attribute("class") == "point");
            XElement outline = Assert.Single(panel.Descendants(Svg + "path"), element => (string)element.Attribute("class") == "contour");
            double lineWidth = (double)outline.Attribute("stroke-width")!;

            Assert.Equal(lineWidth, (double)point.Attribute("width")!);
            Assert.Equal(lineWidth, (double)point.Attribute("height")!);
            Assert.Equal(222.5, (double)point.Attribute("x")! + (lineWidth / 2), 10);
            Assert.Equal(155D, (double)point.Attribute("y")! + (lineWidth / 2), 10);
            Assert.Equal((string)outline.Attribute("stroke")!, (string)point.Attribute("fill")!);
            Assert.Equal("Point (0.25, 0.5). Marker only.", point.Element(Svg + "title")!.Value);
            Assert.Empty(root.Elements(Svg + "text"));
            Assert.Equal("0 0 800 440", (string)root.Attribute("viewBox")!);
            Assert.Empty(root.Descendants(Svg + "circle"));
            Assert.Equal("M 0.25 0.5", (string)outline.Attribute("data-source-d")!);
            Assert.False((bool)outline.Attribute("data-closed")!);
            Assert.Equal("none", (string)outline.Attribute("fill")!);
        }
        finally
        {
            CultureInfo.CurrentCulture = previous;
        }
    }

    /// <summary>
    /// Checks that coincident source vertices have a visible point marker.
    /// </summary>
    /// <param name="isClosed">The source closure state.</param>
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SaveSvg_CoincidentStrokeVertices_HavePointMarker(bool isClosed)
    {
        Contour contour = [new(5, 5), new(5, 5)];
        contour.IsClosed = isClosed;
        Polygon subject = [contour];
        Polygon actual = PolygonStroker.Stroke(subject, 4, new() { LineCap = LineCap.Round });
        string before = subject.ToDebugString();
        string path = SvgTestOutput.SaveSvg(
            nameof(SvgTestOutputTests),
            nameof(this.SaveSvg_CoincidentStrokeVertices_HavePointMarker),
            $"IsClosed-{isClosed}",
            subject,
            [],
            actual);

        XElement root = XDocument.Load(path).Root!;
        XElement input = root.Elements().Single(element => (string)element.Attribute("id") == "inputs");
        XElement outline = Assert.Single(input.Descendants(Svg + "path"));
        XElement marker = Assert.Single(input.Descendants(Svg + "rect"), element => (string)element.Attribute("class") == "point");
        double lineWidth = (double)outline.Attribute("stroke-width")!;
        string[] projected = ((string)outline.Attribute("d")!).Split(' ');

        Assert.Equal(lineWidth, (double)marker.Attribute("width")!);
        Assert.Equal(lineWidth, (double)marker.Attribute("height")!);
        Assert.Equal(double.Parse(projected[1], CultureInfo.InvariantCulture), (double)marker.Attribute("x")! + (lineWidth / 2), 10);
        Assert.Equal(double.Parse(projected[2], CultureInfo.InvariantCulture), (double)marker.Attribute("y")! + (lineWidth / 2), 10);
        Assert.Equal((string)outline.Attribute("stroke")!, (string)marker.Attribute("fill")!);
        Assert.Equal(isClosed ? "M 5 5 L 5 5 Z" : "M 5 5 L 5 5", (string)outline.Attribute("data-source-d")!);
        Assert.Equal(isClosed, (bool)outline.Attribute("data-closed")!);
        Assert.Equal(before, subject.ToDebugString());
    }

    /// <summary>
    /// Checks open output intervals against available analytic expected geometry.
    /// </summary>
    [Fact]
    public void SaveSvg_PreservesOpenOutputAndExpectedGeometry()
    {
        Polygon subject = [Open(new(-1, 5), new(11, 5))];
        Polygon clip =
        [
            [new(0, 0), new(10, 0), new(10, 10), new(0, 10)],
            [new(3, 3), new(7, 3), new(7, 7), new(3, 7)]
        ];
        Polygon actual = PolygonClipper.Intersection(subject, clip, FillRule.EvenOdd);
        Polygon expected =
        [
            Open(new(0, 5), new(3, 5)),
            Open(new(7, 5), new(10, 5))
        ];

        string path = SvgTestOutput.SaveSvg(
            nameof(SvgTestOutputTests), nameof(this.SaveSvg_PreservesOpenOutputAndExpectedGeometry), string.Empty, subject, clip, actual, expected);

        XElement root = XDocument.Load(path).Root!;
        foreach (string id in new[] { "actual", "expected" })
        {
            XElement panel = root.Elements().Single(element => (string)element.Attribute("id") == id);
            XElement[] lines = panel.Descendants(Svg + "path").ToArray();
            Assert.Equal(2, lines.Length);
            Assert.All(lines, line =>
            {
                Assert.Equal("none", (string)line.Attribute("fill")!);
                Assert.False((bool)line.Attribute("data-closed")!);
                Assert.DoesNotContain("Z", (string)line.Attribute("d")!);
            });
        }
    }

    /// <summary>
    /// Checks empty output and the distinction between missing and empty expected geometry.
    /// </summary>
    /// <param name="hasExpected">Whether the test supplies an empty expected result.</param>
    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void SaveSvg_WritesEmptyResults(bool hasExpected)
    {
        Polygon empty = [];
        Polygon actual = PolygonClipper.Union(empty, empty, FillRule.EvenOdd);
        string details = $"HasExpected-{hasExpected}";
        string path = hasExpected
            ? SvgTestOutput.SaveSvg(nameof(SvgTestOutputTests), nameof(this.SaveSvg_WritesEmptyResults), details, empty, empty, actual, empty)
            : SvgTestOutput.SaveSvg(nameof(SvgTestOutputTests), nameof(this.SaveSvg_WritesEmptyResults), details, empty, empty, actual);
        XElement root = XDocument.Load(path).Root!;

        Assert.Equal(hasExpected ? "0 0 1200 440" : "0 0 800 440", (string)root.Attribute("viewBox")!);
        Assert.Equal(hasExpected, root.Elements().Any(element => (string)element.Attribute("id") == "expected"));
        Assert.Empty(root.Descendants(Svg + "path"));
        Assert.Empty(root.Descendants(Svg + "circle"));
    }

    /// <summary>
    /// Checks that another call replaces the previous output at the same path.
    /// </summary>
    [Fact]
    public void SaveSvg_ReplacesPreviousOutput()
    {
        Polygon subject = [Open(new(-2, 0), new(2, 0))];
        Polygon clip = [Open(new(0, -2), new(0, 2))];
        Polygon point = PolygonClipper.Intersection(subject, clip, FillRule.NonZero);

        string firstPath = SvgTestOutput.SaveSvg(
            nameof(SvgTestOutputTests), nameof(this.SaveSvg_ReplacesPreviousOutput), string.Empty, subject, clip, point, point);

        Polygon empty = PolygonClipper.Intersection(subject, [], FillRule.NonZero);

        string secondPath = SvgTestOutput.SaveSvg(
            nameof(SvgTestOutputTests), nameof(this.SaveSvg_ReplacesPreviousOutput), string.Empty, subject, [], empty);

        XElement root = XDocument.Load(secondPath).Root!;
        XElement actual = root.Elements().Single(element => (string)element.Attribute("id") == "actual");

        Assert.Equal(firstPath, secondPath);
        Assert.Equal("SaveSvg_ReplacesPreviousOutput.svg", Path.GetFileName(secondPath));
        Assert.Empty(actual.Descendants(Svg + "path"));
        Assert.Empty(actual.Descendants(Svg + "circle"));
        Assert.DoesNotContain(root.Descendants(), element => (string)element.Attribute("class") == "point");
        Assert.DoesNotContain(root.Descendants(), element => (string)element.Attribute("class") == "point-legend");
        Assert.DoesNotContain(root.Elements(), element => (string)element.Attribute("id") == "expected");
        Assert.Single(Directory.EnumerateFiles(Path.GetDirectoryName(secondPath)!, nameof(this.SaveSvg_ReplacesPreviousOutput) + ".svg"));
    }

    /// <summary>
    /// Builds an explicitly open contour for the source or expected line geometry.
    /// </summary>
    /// <param name="vertices">The source vertices in order.</param>
    /// <returns>The open contour.</returns>
    private static Contour Open(params Vertex[] vertices)
    {
        Contour contour = new() { IsClosed = false };
        foreach (Vertex vertex in vertices)
        {
            contour.Add(vertex);
        }

        return contour;
    }
}

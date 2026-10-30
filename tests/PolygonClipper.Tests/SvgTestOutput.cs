// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System.Globalization;
using System.Text;
using System.Xml.Linq;

namespace SixLabors.PolygonClipper.Tests;

/// <summary>
/// Saves polygon test geometry as SVG panels with a shared scale.
/// </summary>
internal static class SvgTestOutput
{
    private const int PanelSize = 400;
    private const int TitleHeight = 40;
    private const int Padding = 20;
    private const double LineWidth = 1.5;
    private static readonly XNamespace Svg = "http://www.w3.org/2000/svg";

    /// <summary>
    /// Saves the inputs and actual output on every call.
    /// </summary>
    /// <param name="testGroup">The test class and any input subfolder.</param>
    /// <param name="testName">The test method name.</param>
    /// <param name="details">The invariant parameter details, separated by underscores, or an empty string.</param>
    /// <param name="subject">The subject geometry.</param>
    /// <param name="clip">The clip geometry.</param>
    /// <param name="actual">The actual output.</param>
    /// <returns>The full path to the SVG file.</returns>
    public static string SaveSvg(string testGroup, string testName, string details, Polygon subject, Polygon clip, Polygon actual)
        => SaveSvgCore(GetTestOutputFileName(testGroup, testName, details), subject, clip, actual, null);

    /// <summary>
    /// Saves the inputs, actual output and available expected geometry on every call.
    /// </summary>
    /// <param name="testGroup">The test class and any input subfolder.</param>
    /// <param name="testName">The test method name.</param>
    /// <param name="details">The invariant parameter details, separated by underscores, or an empty string.</param>
    /// <param name="subject">The subject geometry.</param>
    /// <param name="clip">The clip geometry.</param>
    /// <param name="actual">The actual output.</param>
    /// <param name="expected">The expected geometry, including an expected empty result.</param>
    /// <returns>The full path to the SVG file.</returns>
    public static string SaveSvg(string testGroup, string testName, string details, Polygon subject, Polygon clip, Polygon actual, Polygon expected)
        => SaveSvgCore(GetTestOutputFileName(testGroup, testName, details), subject, clip, actual, expected);

    /// <summary>
    /// Builds an output path using the ImageSharp test group, method and parameter naming convention.
    /// </summary>
    /// <param name="testGroup">The test class and any input subfolder.</param>
    /// <param name="testName">The test method name.</param>
    /// <param name="details">The invariant parameter details, separated by underscores, or an empty string.</param>
    /// <returns>The full output path.</returns>
    public static string GetTestOutputFileName(string testGroup, string testName, string details)
    {
        string fileName = details.Length == 0 ? testName : $"{testName}_{details}";
        return Path.Combine(TestEnvironment.ActualOutputDirectoryFullPath, testGroup, fileName + ".svg");
    }

    /// <summary>
    /// Fits all available geometry to one shared view and writes the panels.
    /// </summary>
    /// <param name="outputPath">The full output path.</param>
    /// <param name="subject">The subject geometry.</param>
    /// <param name="clip">The clip geometry.</param>
    /// <param name="actual">The actual output.</param>
    /// <param name="expected">The expected geometry, or null when the test has none.</param>
    /// <returns>The full path to the SVG file.</returns>
    private static string SaveSvgCore(string outputPath, Polygon subject, Polygon clip, Polygon actual, Polygon expected)
    {
        Polygon[] geometry = expected is null ? [subject, clip, actual] : [subject, clip, actual, expected];
        Box2? bounds = null;
        foreach (Polygon polygon in geometry)
        {
            foreach (Contour contour in polygon)
            {
                // Empty contours have no bounds. Their default box would add an unrelated origin.
                if (contour.Count > 0)
                {
                    Box2 contourBounds = contour.GetBoundingBox();
                    bounds = bounds is null ? contourBounds : bounds.Value.Add(contourBounds);
                }
            }
        }

        Box2 box = bounds ?? default;
        double scale = Math.Max(Math.Max(Math.Abs(box.Min.X), Math.Abs(box.Max.X)), Math.Max(Math.Abs(box.Min.Y), Math.Abs(box.Max.Y)));
        if (scale == 0)
        {
            scale = 1;
        }

        // SVG viewers can lose very small or large coordinates. Scale before subtraction
        // to fit those inputs without overflow. Each contour also retains its source coordinates.
        Vertex min = box.Min / scale;
        Vertex max = box.Max / scale;
        double extent = Math.Max(max.X - min.X, max.Y - min.Y);
        if (extent == 0)
        {
            // An empty scene or one point still needs a visible viewport.
            extent = 1;
        }

        Vertex origin = new((min.X + max.X - extent) / 2, (min.Y + max.Y + extent) / 2);
        int panelCount = expected is null ? 2 : 3;
        XElement root = new(
            Svg + "svg",
            new XAttribute("viewBox", $"0 0 {PanelSize * panelCount} {PanelSize + TitleHeight}"),
            new XElement(Svg + "title", Path.GetFileNameWithoutExtension(outputPath)),
            new XElement(Svg + "desc", "All panels use outlines. Point markers match the line width. Paths retain source coordinates and hierarchy."));

        XElement inputs = AddPanel(root, 0, "inputs", "Inputs: subject blue, clip orange");
        AddGeometry(inputs, subject, "#2563eb", origin, scale, extent);
        AddGeometry(inputs, clip, "#ea580c", origin, scale, extent);
        AddGeometry(AddPanel(root, 1, "actual", "Actual"), actual, "#15803d", origin, scale, extent);
        if (expected is not null)
        {
            AddGeometry(AddPanel(root, 2, "expected", "Expected"), expected, "#7e22ce", origin, scale, extent);
        }

        // Auto-fit preserves relative size within a file but changes pixels per unit between files.
        // Show a source-unit scale below the input geometry so that difference stays visible.
        const double requestedBarWidth = 60;
        int drawingWidth = PanelSize - (2 * Padding);
        double barUnits = Math.Max(double.Epsilon, scale * (extent * (requestedBarWidth / drawingWidth)));

        // A subnormal source distance can round to zero. Use at least one representable unit,
        // then project that actual distance so the bar and its label still describe the same scale.
        double barWidth = ((barUnits / scale) / extent) * drawingWidth;
        double barEnd = Padding + barWidth;
        int barY = PanelSize + TitleHeight - (Padding / 2);
        root.Add(new XElement(
            Svg + "g",
            new XAttribute("id", "scale"),
            new XAttribute("stroke", "#475569"),
            new XElement(
                Svg + "line",
                new XAttribute("x1", Padding),
                new XAttribute("x2", barEnd),
                new XAttribute("y1", barY),
                new XAttribute("y2", barY),
                new XAttribute("data-units", barUnits)),
            new XElement(
                Svg + "line",
                new XAttribute("x1", Padding),
                new XAttribute("x2", Padding),
                new XAttribute("y1", barY - 3),
                new XAttribute("y2", barY + 3)),
            new XElement(
                Svg + "line",
                new XAttribute("x1", barEnd),
                new XAttribute("x2", barEnd),
                new XAttribute("y1", barY - 3),
                new XAttribute("y2", barY + 3)),
            new XElement(
                Svg + "text",
                new XAttribute("x", barEnd + 10),
                new XAttribute("y", barY),
                new XAttribute("font-family", "sans-serif"),
                new XAttribute("font-size", 11),
                new XAttribute("dominant-baseline", "middle"),
                new XAttribute("stroke", "none"),
                new XAttribute("fill", "#475569"),
                FormattableString.Invariant($"{barUnits:G6} units"))));

        Directory.CreateDirectory(Path.GetDirectoryName(outputPath)!);
        new XDocument(root).Save(outputPath);
        return outputPath;
    }

    /// <summary>
    /// Adds a labeled panel with the same viewport dimensions as the other panels.
    /// </summary>
    /// <param name="root">The SVG document root.</param>
    /// <param name="index">The panel position.</param>
    /// <param name="id">The panel identifier.</param>
    /// <param name="label">The visible panel label.</param>
    /// <returns>The group that receives the panel geometry.</returns>
    private static XElement AddPanel(XElement root, int index, string id, string label)
    {
        int x = index * PanelSize;
        XElement group = new(Svg + "g");
        root.Add(new XElement(
            Svg + "g",
            new XAttribute("id", id),
            new XElement(
                Svg + "rect",
                new XAttribute("x", x),
                new XAttribute("width", PanelSize),
                new XAttribute("height", PanelSize + TitleHeight),
                new XAttribute("fill", "white"),
                new XAttribute("stroke", "#cbd5e1")),
            new XElement(
                Svg + "text",
                new XAttribute("x", x + Padding),
                new XAttribute("y", 25),
                new XAttribute("font-family", "sans-serif"),
                new XAttribute("font-size", 14),
                label),
            new XElement(
                Svg + "svg",
                new XAttribute("x", x),
                new XAttribute("y", TitleHeight),
                new XAttribute("width", PanelSize),
                new XAttribute("height", PanelSize),
                new XAttribute("viewBox", $"0 0 {PanelSize} {PanelSize}"),
                group)));

        return group;
    }

    /// <summary>
    /// Draws contour outlines and square point markers that match the line width.
    /// </summary>
    /// <param name="group">The panel geometry group.</param>
    /// <param name="polygon">The source polygon.</param>
    /// <param name="color">The geometry color.</param>
    /// <param name="origin">The shared upper-left origin after coordinate scaling.</param>
    /// <param name="scale">The shared coordinate scale.</param>
    /// <param name="extent">The shared square extent after coordinate scaling.</param>
    private static void AddGeometry(XElement group, Polygon polygon, string color, Vertex origin, double scale, double extent)
    {
        for (int i = 0; i < polygon.Count; i++)
        {
            Contour contour = polygon[i];
            if (contour.Count == 0)
            {
                continue;
            }

            StringBuilder path = new();
            StringBuilder source = new();
            Vertex first = contour[0];
            bool singlePoint = true;

            foreach (Vertex vertex in contour)
            {
                // Repeated coincident vertices still describe one point. A zero-length
                // SVG path has no visible outline, so it needs the same point marker.
                singlePoint = singlePoint && vertex == first;

                Vertex point = Project(vertex, origin, scale, extent);
                string command = path.Length == 0 ? "M " : " L ";
                path.Append(command).Append(point.X.ToString("R", CultureInfo.InvariantCulture))
                    .Append(' ').Append(point.Y.ToString("R", CultureInfo.InvariantCulture));

                source.Append(command).Append(vertex.X.ToString("R", CultureInfo.InvariantCulture))
                    .Append(' ').Append(vertex.Y.ToString("R", CultureInfo.InvariantCulture));
            }

            if (contour.IsClosed)
            {
                path.Append(" Z");
                source.Append(" Z");
            }

            XElement outline = new(
                Svg + "path",
                new XAttribute("class", "contour"),
                new XAttribute("d", path.ToString()),
                new XAttribute("fill", "none"),
                new XAttribute("stroke", color),
                new XAttribute("stroke-width", LineWidth),
                new XAttribute("data-contour", i),
                new XAttribute("data-closed", contour.IsClosed),
                new XAttribute("data-depth", contour.Depth),
                new XAttribute("data-parent", contour.ParentIndex?.ToString(CultureInfo.InvariantCulture) ?? "none"),
                new XAttribute("data-holes", string.Join(" ", Enumerable.Range(0, contour.HoleCount).Select(contour.GetHoleIndex))),
                new XAttribute("data-source-d", source.ToString()),
                new XElement(Svg + "title", $"Contour {i}. {(contour.IsClosed ? "Closed" : "Open")}. Depth {contour.Depth}."));

            group.Add(outline);
            if (singlePoint)
            {
                // Center the square on the point and match the contour line thickness.
                Vertex point = Project(contour[0], origin, scale, extent);
                group.Add(new XElement(
                    Svg + "rect",
                    new XAttribute("class", "point"),
                    new XAttribute("x", point.X - (LineWidth / 2)),
                    new XAttribute("y", point.Y - (LineWidth / 2)),
                    new XAttribute("width", LineWidth),
                    new XAttribute("height", LineWidth),
                    new XAttribute("fill", color),
                    new XElement(Svg + "title", FormattableString.Invariant($"Point ({contour[0].X:R}, {contour[0].Y:R}). Marker only."))));
            }
        }
    }

    /// <summary>
    /// Maps a source vertex to the shared display scale with the Y axis upwards.
    /// </summary>
    /// <param name="point">The source vertex.</param>
    /// <param name="origin">The shared upper-left origin after coordinate scaling.</param>
    /// <param name="scale">The shared coordinate scale.</param>
    /// <param name="extent">The shared square extent after coordinate scaling.</param>
    /// <returns>The display position in SVG units.</returns>
    private static Vertex Project(Vertex point, Vertex origin, double scale, double extent)
    {
        Vertex offset = ((point / scale) - origin) / extent;
        return new(Padding + (offset.X * (PanelSize - (2 * Padding))), Padding - (offset.Y * (PanelSize - (2 * Padding))));
    }
}

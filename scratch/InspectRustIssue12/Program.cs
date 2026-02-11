using System.Text.Json;
using GeoJson.Feature;
using GeoJson.Geometry;
using Polygon = SixLabors.PolygonClipper.Polygon;
using Contour = SixLabors.PolygonClipper.Contour;
using Vertex = SixLabors.PolygonClipper.Vertex;
using SixLabors.PolygonClipper;

var filePath = args.Length > 0 ? args[0] : "tests/TestData/Generic/rust_issue12.geojson";
ClipperOptions? options = null;
if (args.Length > 1 && int.TryParse(args[1], out int precision))
{
    options = new ClipperOptions
    {
        ScaleMode = ClipperScaleMode.Auto,
        Precision = precision
    };
    Console.WriteLine($"Using precision: {precision}");
}
var json = File.ReadAllText(filePath);
var data = JsonSerializer.Deserialize<FeatureCollection>(json)!;
var subject = ConvertToPolygon(data.Features[0].Geometry);
var clipping = ConvertToPolygon(data.Features[1].Geometry);
var expectedFeature = data.Features[2];
var mode = expectedFeature.Properties?["operation"]?.ToString();
Func<Polygon, Polygon, Polygon> operation = mode switch
{
    "union" => (a, b) => PolygonClipper.Union(a, b, options),
    "intersection" => (a, b) => PolygonClipper.Intersection(a, b, options),
    "xor" => (a, b) => PolygonClipper.Xor(a, b, options),
    "diff" => (a, b) => PolygonClipper.Difference(a, b, options),
    "diff_ba" => (a, b) => PolygonClipper.Difference(b, a, options),
    _ => (a, b) => PolygonClipper.Union(a, b, options)
};

var expected = ConvertToPolygon(expectedFeature.Geometry);
var actual = operation(subject, clipping);
var swapped = operation(clipping, subject);

Console.WriteLine($"Mode: {mode}");
Console.WriteLine("Expected:");
DumpPolygon(expected);
Console.WriteLine("Actual:");
DumpPolygon(actual);
Console.WriteLine("Swapped:");
DumpPolygon(swapped);

static void Dump(Contour contour)
{
    for (int i = 0; i < contour.Count; i++)
    {
        var v = contour[i];
        Console.WriteLine($"  {i}: {v.X}, {v.Y}");
    }
}

static void DumpPolygon(Polygon polygon)
{
    for (int i = 0; i < polygon.Count; i++)
    {
        Console.WriteLine($"Contour {i} (Count: {polygon[i].Count})");
        Dump(polygon[i]);
    }
}

static Polygon ConvertToPolygon(IGeometryObject geometry)
{
    if (geometry is GeoJson.Geometry.Polygon geoJsonPolygon)
    {
        Polygon polygon = [];
        int exteriorIndex = -1;
        int ringIndex = 0;
        foreach (LineString ring in geoJsonPolygon.Coordinates)
        {
            Contour contour = new();
            foreach (IPosition xy in ring.Coordinates)
            {
                contour.Add(new Vertex(xy.Longitude, xy.Latitude));
            }

            polygon.Add(contour);
            int contourIndex = polygon.Count - 1;
            if (ringIndex == 0)
            {
                exteriorIndex = contourIndex;
            }
            else if (exteriorIndex >= 0)
            {
                contour.ParentIndex = exteriorIndex;
                polygon[exteriorIndex].AddHoleIndex(contourIndex);
            }

            if (!ring.IsClosed())
            {
                contour.Add(contour[0]);
            }

            ringIndex++;
        }

        return polygon;
    }
    else if (geometry is MultiPolygon geoJsonMultiPolygon)
    {
        Polygon polygon = [];
        foreach (GeoJson.Geometry.Polygon geoPolygon in geoJsonMultiPolygon.Coordinates)
        {
            int exteriorIndex = -1;
            int ringIndex = 0;
            foreach (LineString ring in geoPolygon.Coordinates)
            {
                Contour contour = new();
                foreach (IPosition xy in ring.Coordinates)
                {
                    contour.Add(new Vertex(xy.Longitude, xy.Latitude));
                }

                polygon.Add(contour);
                int contourIndex = polygon.Count - 1;
                if (ringIndex == 0)
                {
                    exteriorIndex = contourIndex;
                }
                else if (exteriorIndex >= 0)
                {
                    contour.ParentIndex = exteriorIndex;
                    polygon[exteriorIndex].AddHoleIndex(contourIndex);
                }

                if (!ring.IsClosed())
                {
                    contour.Add(contour[0]);
                }

                ringIndex++;
            }
        }

        return polygon;
    }

    throw new InvalidOperationException("Unsupported geometry type.");
}

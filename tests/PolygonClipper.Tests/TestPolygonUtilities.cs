// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using GeoJson.Feature;
using GeoJson.Geometry;
using GeoPolygon = GeoJson.Geometry.Polygon;

namespace SixLabors.PolygonClipper.Tests;

internal static class TestPolygonUtilities
{
    public static (Polygon Subject, Polygon Clipping) BuildPolygon(FeatureCollection data)
    {
        IGeometryObject subjectGeometry = data.Features[0].Geometry;
        IGeometryObject clippingGeometry = data.Features[1].Geometry;

        Polygon subject = ConvertToPolygon(subjectGeometry);
        Polygon clipping = ConvertToPolygon(clippingGeometry);

        return (subject, clipping);
    }

    /// <summary>
    /// Converts GeoJSON coordinates and preserves exterior and hole indices.
    /// </summary>
    /// <param name="geometry">The Polygon or MultiPolygon geometry.</param>
    /// <returns>The contours with their GeoJSON hierarchy.</returns>
    public static Polygon ConvertToPolygon(IGeometryObject geometry)
    {
        if (geometry is GeoPolygon geoJsonPolygon)
        {
            // Convert GeoJSON Polygon to our Polygon type
            Polygon polygon = [];
            foreach (LineString ring in geoJsonPolygon.Coordinates)
            {
                Contour contour = [];
                foreach (IPosition xy in ring.Coordinates)
                {
                    contour.Add(new Vertex(xy.Longitude, xy.Latitude));
                }

                if (polygon.Count > 0)
                {
                    // Later rings in this GeoJSON Polygon belong to its first ring.
                    contour.ParentIndex = 0;
                    contour.Depth = 1;
                    polygon[0].AddHoleIndex(polygon.Count);
                }

                polygon.Add(contour);

                if (!ring.IsClosed())
                {
                    contour.Add(contour[0]);
                }
            }

            return polygon;
        }
        else if (geometry is MultiPolygon geoJsonMultiPolygon)
        {
            // Convert GeoJSON MultiPolygon to our Polygon type
            Polygon polygon = [];
            foreach (GeoPolygon geoPolygon in geoJsonMultiPolygon.Coordinates)
            {
                int exteriorIndex = polygon.Count;
                foreach (LineString ring in geoPolygon.Coordinates)
                {
                    Contour contour = [];
                    foreach (IPosition xy in ring.Coordinates)
                    {
                        contour.Add(new Vertex(xy.Longitude, xy.Latitude));
                    }

                    if (polygon.Count > exteriorIndex)
                    {
                        // Each component has its own exterior; keep global contour indices.
                        contour.ParentIndex = exteriorIndex;
                        contour.Depth = 1;
                        polygon[exteriorIndex].AddHoleIndex(polygon.Count);
                    }

                    polygon.Add(contour);

                    if (!ring.IsClosed())
                    {
                        contour.Add(contour[0]);
                    }
                }
            }

            return polygon;
        }

        throw new InvalidOperationException("Unsupported geometry type.");
    }
}

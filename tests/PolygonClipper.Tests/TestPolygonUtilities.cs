// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System;
using GeoJSON.Text.Feature;
using GeoJSON.Text.Geometry;

using GeoPolygon = GeoJSON.Text.Geometry.Polygon;

namespace PolygonClipper.Tests;

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

    public static Polygon ConvertToPolygon(IGeometryObject geometry)
    {
        if (geometry is GeoPolygon geoJsonPolygon)
        {
            // Convert GeoJSON Polygon to our Polygon type
            Polygon polygon = [];
            foreach (LineString ring in geoJsonPolygon.Coordinates)
            {
                Contour contour = new();
                foreach (IPosition xy in ring.Coordinates)
                {
                    contour.AddVertex(new Vertex(xy.Longitude, xy.Latitude));
                }

                polygon.Add(contour);

                if (!ring.IsClosed())
                {
                    contour.AddVertex(contour[0]);
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
                foreach (LineString ring in geoPolygon.Coordinates)
                {
                    Contour contour = new();
                    foreach (IPosition xy in ring.Coordinates)
                    {
                        contour.AddVertex(new Vertex(xy.Longitude, xy.Latitude));
                    }

                    polygon.Add(contour);

                    if (!ring.IsClosed())
                    {
                        contour.AddVertex(contour[0]);
                    }
                }
            }

            return polygon;
        }

        throw new InvalidOperationException("Unsupported geometry type.");
    }
}

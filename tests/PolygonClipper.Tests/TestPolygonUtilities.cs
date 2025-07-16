// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System;
using Clipper2Lib;
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

    public static (PathsD Subject, PathsD Clipping) BuildClipper2Polygon(FeatureCollection data)
    {
        IGeometryObject subjectGeometry = data.Features[0].Geometry;
        IGeometryObject clippingGeometry = data.Features[1].Geometry;

        PathsD subject = ConvertToClipper2Polygon(subjectGeometry);
        PathsD clipping = ConvertToClipper2Polygon(clippingGeometry);

        return (subject, clipping);
    }

    private static Polygon ConvertToPolygon(IGeometryObject geometry)
    {
        if (geometry is GeoPolygon geoJsonPolygon)
        {
            // Convert GeoJSON Polygon to our Polygon type
            Polygon polygon = new();
            foreach (LineString ring in geoJsonPolygon.Coordinates)
            {
                Contour contour = new();
                foreach (IPosition xy in ring.Coordinates)
                {
                    contour.AddVertex(new Vertex(xy.Longitude, xy.Latitude));
                }

                polygon.Push(contour);

                if (!ring.IsClosed())
                {
                    contour.AddVertex(contour.GetVertex(0));
                }
            }

            return polygon;
        }
        else if (geometry is MultiPolygon geoJsonMultiPolygon)
        {
            // Convert GeoJSON MultiPolygon to our Polygon type
            Polygon polygon = new();
            foreach (GeoPolygon geoPolygon in geoJsonMultiPolygon.Coordinates)
            {
                foreach (LineString ring in geoPolygon.Coordinates)
                {
                    Contour contour = new();
                    foreach (IPosition xy in ring.Coordinates)
                    {
                        contour.AddVertex(new Vertex(xy.Longitude, xy.Latitude));
                    }

                    polygon.Push(contour);

                    if (!ring.IsClosed())
                    {
                        contour.AddVertex(contour.GetVertex(0));
                    }
                }
            }

            return polygon;
        }

        throw new InvalidOperationException("Unsupported geometry type.");
    }

    private static PathsD ConvertToClipper2Polygon(IGeometryObject geometry)
    {
        if (geometry is GeoPolygon geoJsonPolygon)
        {
            // Convert GeoJSON Polygon to our Polygon type
            PathsD polygon = [];
            foreach (LineString ring in geoJsonPolygon.Coordinates)
            {
                PathD contour = [];
                foreach (IPosition xy in ring.Coordinates)
                {
                    contour.Add(new PointD(xy.Longitude, xy.Latitude));
                }

                if (!ring.IsClosed())
                {
                    contour.Add(contour[0]);
                }

                polygon.Add(contour);
            }

            return polygon;
        }
        else if (geometry is MultiPolygon geoJsonMultiPolygon)
        {
            // Convert GeoJSON MultiPolygon to our Polygon type
            PathsD polygon = [];

            // Clipper2 expects holes run in the opposite direction
            // to the outer ring, so we need to reverse holes if they are clockwise.
            int i = 0;
            foreach (GeoPolygon geoPolygon in geoJsonMultiPolygon.Coordinates)
            {
                foreach (LineString ring in geoPolygon.Coordinates)
                {
                    PathD contour = [];
                    foreach (IPosition xy in ring.Coordinates)
                    {
                        contour.Add(new PointD(xy.Longitude, xy.Latitude));
                    }

                    if (!ring.IsClosed())
                    {
                        contour.Add(contour[0]);
                    }

                    // Assume all polygons following the initial one are holes.
                    // If the area of the contour is positive, we must reverse it.
                    if (i > 0 && Clipper.Area(contour) > 0)
                    {
                        contour.Reverse();
                    }

                    polygon.Add(contour);
                    i++;
                }
            }

            return polygon;
        }

        throw new InvalidOperationException("Unsupported geometry type.");
    }
}

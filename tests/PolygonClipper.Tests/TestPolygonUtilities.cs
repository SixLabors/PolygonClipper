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
                polygon.Add(contour);
            }

            return polygon;
        }
        else if (geometry is MultiPolygon geoJsonMultiPolygon)
        {
            // Convert GeoJSON MultiPolygon to our Polygon type
            PathsD polygon = [];
            foreach (GeoPolygon geoPolygon in geoJsonMultiPolygon.Coordinates)
            {
                foreach (LineString ring in geoPolygon.Coordinates)
                {
                    PathD contour = [];
                    foreach (IPosition xy in ring.Coordinates)
                    {
                        contour.Add(new PointD(xy.Longitude, xy.Latitude));
                    }
                    polygon.Add(contour);
                }
            }

            return polygon;
        }

        throw new InvalidOperationException("Unsupported geometry type.");
    }
}

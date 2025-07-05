// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System.Collections.Generic;
using System.IO;
using System.Text.Json;
using GeoJSON.Text.Feature;

namespace PolygonClipper.Tests.TestCases;

internal static class TestData
{
    public static class Fixtures
    {
        public static FeatureCollection GetFeatureCollection(string fileName)
            => JsonSerializer.Deserialize<FeatureCollection>(GetGeoJsonPath(fileName));

        private static string GetGeoJsonPath(string fileName)
            => GetFullPath(nameof(Fixtures), fileName);
    }

    public static class Generic
    {
        public static IEnumerable<string> GetFileNames()
        {
            DirectoryInfo info = new(Path.Combine(TestEnvironment.GeoJsonTestDataFullPath, nameof(Generic)));
            foreach (FileInfo file in info.EnumerateFiles("*.geojson"))
            {
                yield return file.Name;
            }
        }

        public static FeatureCollection GetFeatureCollection(string fileName)
        {
            string path = GetGeoJsonPath(fileName);
            return JsonSerializer.Deserialize<FeatureCollection>(File.ReadAllText(path));
        }

        private static string GetGeoJsonPath(string fileName)
            => GetFullPath(nameof(Generic), fileName);
    }

    private static string GetFullPath(string folder, string fileName)
        => Path.Combine(TestEnvironment.GeoJsonTestDataFullPath, folder, fileName);
}

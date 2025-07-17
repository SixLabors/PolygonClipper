// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System.Reflection;
using System.Runtime.InteropServices;

namespace PolygonClipper.Tests;
internal static class TestEnvironment
{
    private static readonly FileInfo TestAssemblyFile = new(typeof(TestEnvironment).GetTypeInfo().Assembly.Location);

    private const string SixLaborsSolutionFileName = "PolygonClipper.sln";

    private const string GeoJsonTestDataRelativePath = @"tests\TestData\";

    private static readonly Lazy<string> SolutionDirectoryFullPathLazy = new(GetSolutionDirectoryFullPathImpl);

    public static string SolutionDirectoryFullPath => SolutionDirectoryFullPathLazy.Value;

    /// <summary>
    /// Gets the correct full path to the GeoJson TestData directory.
    /// </summary>
    public static string GeoJsonTestDataFullPath => GetFullPath(GeoJsonTestDataRelativePath);

    public static bool IsLinux => RuntimeInformation.IsOSPlatform(OSPlatform.Linux);

    public static bool IsMacOS => RuntimeInformation.IsOSPlatform(OSPlatform.OSX);

    public static bool IsWindows => RuntimeInformation.IsOSPlatform(OSPlatform.Windows);

    public static bool Is64BitProcess => Environment.Is64BitProcess;

    public static Architecture OSArchitecture => RuntimeInformation.OSArchitecture;

    public static Architecture ProcessArchitecture => RuntimeInformation.ProcessArchitecture;

    /// <summary>
    /// Gets a value indicating whether test execution runs on CI.
    /// </summary>
#if ENV_CI
    public static bool RunsOnCI => true;
#else
    public static bool RunsOnCI => false;
#endif

    private static string GetSolutionDirectoryFullPathImpl()
    {
        DirectoryInfo directory = TestAssemblyFile.Directory;

        while (directory?.EnumerateFiles(SixLaborsSolutionFileName).Any() == false)
        {
            try
            {
                directory = directory.Parent;
            }
            catch (Exception ex)
            {
                throw new DirectoryNotFoundException(
                    $"Unable to find  solution directory from {TestAssemblyFile} because of {ex.GetType().Name}!",
                    ex);
            }

            if (directory == null)
            {
                throw new DirectoryNotFoundException($"Unable to find  solution directory from {TestAssemblyFile}!");
            }
        }

        return directory.FullName;
    }

    private static string GetFullPath(string relativePath) =>
        Path.Combine(SolutionDirectoryFullPath, relativePath)
        .Replace('\\', Path.DirectorySeparatorChar);
}

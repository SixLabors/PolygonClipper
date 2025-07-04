// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

namespace PolygonClipper;

/// <summary>
/// Helper class containing utility functions for comparisons.
/// </summary>
internal static class Helper
{
    /// <summary>
    /// Converts a boolean comparison result to an ordering value.
    /// Returns -1 if the condition is true, 1 if false.
    /// </summary>
    /// <param name="condition">The boolean condition to evaluate.</param>
    /// <returns>-1 if condition is true, 1 if false.</returns>
    public static int LessIf(bool condition) => condition ? -1 : 1;

    /// <summary>
    /// Converts a boolean comparison result to an inversed ordering value.
    /// Returns 1 if the condition is true, -1 if false.
    /// </summary>
    /// <param name="condition">The boolean condition to evaluate.</param>
    /// <returns>1 if condition is true, -1 if false.</returns>
    public static int LessIfInversed(bool condition) => condition ? 1 : -1;
}

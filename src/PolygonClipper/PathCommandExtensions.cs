// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

using System.Runtime.CompilerServices;

namespace SixLabors.PolygonClipper;

/// <summary>
/// Provides helper methods for querying <see cref="PathCommand"/> values.
/// </summary>
internal static class PathCommandExtensions
{
    /// <summary>
    /// Returns whether the command emits a vertex coordinate.
    /// </summary>
    /// <param name="command">The command to evaluate.</param>
    /// <returns>
    /// <see langword="true"/> when the command is a vertex-emitting command; otherwise, <see langword="false"/>.
    /// </returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool Vertex(this PathCommand command) => command is >= PathCommand.MoveTo and < PathCommand.EndPoly;

    /// <summary>
    /// Returns whether the command is <see cref="PathCommand.Stop"/>.
    /// </summary>
    /// <param name="command">The command to evaluate.</param>
    /// <returns><see langword="true"/> if the command is stop; otherwise, <see langword="false"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool Stop(this PathCommand command) => command == PathCommand.Stop;

    /// <summary>
    /// Returns whether the command is <see cref="PathCommand.MoveTo"/>.
    /// </summary>
    /// <param name="command">The command to evaluate.</param>
    /// <returns><see langword="true"/> if the command is move-to; otherwise, <see langword="false"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool MoveTo(this PathCommand command) => command == PathCommand.MoveTo;

    /// <summary>
    /// Returns whether the masked command type is <see cref="PathCommand.EndPoly"/>.
    /// </summary>
    /// <param name="command">The command to evaluate.</param>
    /// <returns><see langword="true"/> if the command type is end-poly; otherwise, <see langword="false"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool EndPoly(this PathCommand command) => (command & PathCommand.Mask) == PathCommand.EndPoly;

    /// <summary>
    /// Extracts the close-path flag from the command.
    /// </summary>
    /// <param name="command">The command to evaluate.</param>
    /// <returns>The command value masked with <see cref="PathFlags.Close"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int GetCloseFlag(this PathCommand command) => (int)command & (int)PathFlags.Close;
}

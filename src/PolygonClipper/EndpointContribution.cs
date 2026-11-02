// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

namespace SixLabors.PolygonClipper;

/// <summary>
/// Stores independent signed area winding and unsigned line membership.
/// </summary>
internal readonly struct EndpointContribution
{
    /// <summary>
    /// The signed area contribution ending at this source endpoint.
    /// </summary>
    public readonly WindingCount Area;

    /// <summary>
    /// The unsigned line contribution ending at this source endpoint.
    /// </summary>
    public readonly WindingCount Lines;

    /// <summary>
    /// Initializes a new instance of the <see cref="EndpointContribution"/> struct.
    /// </summary>
    /// <param name="area">The signed area contribution.</param>
    /// <param name="lines">The unsigned line contribution.</param>
    public EndpointContribution(WindingCount area, WindingCount lines)
    {
        this.Area = area;
        this.Lines = lines;
    }
}

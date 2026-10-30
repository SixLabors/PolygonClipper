// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

namespace SixLabors.PolygonClipper;

/// <summary>
/// Provides join and arc settings for polygon inflation and deflation.
/// </summary>
public sealed class OffsetOptions : IEquatable<OffsetOptions?>
{
    /// <summary>
    /// Gets or sets the miter limit used to clamp outer miter joins.
    /// </summary>
    /// <remarks>The default is 4.</remarks>
    public double MiterLimit { get; set; } = 4D;

    /// <summary>
    /// Gets or sets the detail scale for round joins. Higher values produce more vertices.
    /// </summary>
    /// <remarks>The default is 1.</remarks>
    public double ArcDetailScale { get; set; } = 1D;

    /// <summary>
    /// Gets or sets the join style for offset corners.
    /// </summary>
    /// <remarks>The default is <see cref="LineJoin.Bevel"/>.</remarks>
    public LineJoin LineJoin { get; set; } = LineJoin.Bevel;

    /// <inheritdoc/>
    public override bool Equals(object? obj) => this.Equals(obj as OffsetOptions);

    /// <inheritdoc/>
    public bool Equals(OffsetOptions? other)
        => other is not null &&
           this.MiterLimit == other.MiterLimit &&
           this.ArcDetailScale == other.ArcDetailScale &&
           this.LineJoin == other.LineJoin;

    /// <inheritdoc/>
    public override int GetHashCode()
        => HashCode.Combine(this.MiterLimit, this.ArcDetailScale, this.LineJoin);
}

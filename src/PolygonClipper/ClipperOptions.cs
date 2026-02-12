// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

namespace SixLabors.PolygonClipper;

/// <summary>
/// Defines how fixed-precision scaling is selected for clipping operations.
/// </summary>
public enum ClipperScaleMode
{
    /// <summary>
    /// Uses a fixed decimal precision (see <see cref="ClipperOptions.Precision"/>).
    /// </summary>
    FixedPrecision,

    /// <summary>
    /// Uses an explicit scaling factor (see <see cref="ClipperOptions.FixedScale"/>).
    /// </summary>
    FixedScale,

    /// <summary>
    /// Automatically reduces scale to keep coordinates within the valid integer range.
    /// </summary>
    Auto
}

/// <summary>
/// Configures fixed-precision scaling used by clipping and self-intersection removal.
/// </summary>
public sealed class ClipperOptions
{
    /// <summary>
    /// Gets the default options (auto scale with 6 decimal places).
    /// </summary>
    public static ClipperOptions Default { get; } = new();

    /// <summary>
    /// Gets or sets an optional fill rule override for self-intersection removal.
    /// When <see langword="null"/>, contour orientation is normalized by nesting depth
    /// and the effective positive/negative fill rule is inferred from the outer winding.
    /// </summary>
    public FillRule? FillRuleOverride { get; set; }

    /// <summary>
    /// Gets or sets the scaling mode used for fixed precision.
    /// </summary>
    public ClipperScaleMode ScaleMode { get; set; } = ClipperScaleMode.Auto;

    /// <summary>
    /// Gets or sets the number of decimal places to retain when <see cref="ScaleMode"/> is
    /// <see cref="ClipperScaleMode.FixedPrecision"/>.
    /// </summary>
    public int Precision { get; set; } = 6;

    /// <summary>
    /// Gets or sets the fixed scaling factor used when <see cref="ScaleMode"/> is
    /// <see cref="ClipperScaleMode.FixedScale"/>.
    /// </summary>
    public double FixedScale { get; set; }
}

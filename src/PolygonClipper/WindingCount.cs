// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

namespace SixLabors.PolygonClipper;

/// <summary>
/// Stores separate subject and clip contributions without cross-operand cancellation.
/// </summary>
internal readonly struct WindingCount
{
    /// <summary>
    /// The subject winding.
    /// </summary>
    public readonly int Subject;

    /// <summary>
    /// The clipping operand winding.
    /// </summary>
    public readonly int Clip;

    /// <summary>
    /// Initializes a new instance of the <see cref="WindingCount"/> struct.
    /// </summary>
    /// <param name="subject">The subject component.</param>
    /// <param name="clip">The clipping component.</param>
    public WindingCount(int subject, int clip)
    {
        this.Subject = subject;
        this.Clip = clip;
    }

    /// <summary>
    /// Gets a value indicating whether both contributions cancel.
    /// </summary>
    public bool IsZero => this.Subject == 0 && this.Clip == 0;

    /// <summary>
    /// Adds an interval's contribution.
    /// </summary>
    /// <param name="other">The contribution to add.</param>
    /// <returns>The summed winding.</returns>
    public WindingCount Add(WindingCount other) => new(this.Subject + other.Subject, this.Clip + other.Clip);

    /// <summary>
    /// Removes contributions terminating at an endpoint.
    /// </summary>
    /// <param name="other">The terminating contributions.</param>
    /// <returns>The remaining winding.</returns>
    public WindingCount Subtract(WindingCount other) => new(this.Subject - other.Subject, this.Clip - other.Clip);
}

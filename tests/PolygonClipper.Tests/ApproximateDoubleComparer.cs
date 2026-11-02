// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

namespace SixLabors.PolygonClipper.Tests;

/// <summary>
/// Compares double values and vertices within a specified error for test assertions.
/// </summary>
internal readonly struct ApproximateDoubleComparer : IEqualityComparer<double>, IEqualityComparer<Vertex>
{
    private readonly double epsilon;

    /// <summary>
    /// Initializes a new instance of the <see cref="ApproximateDoubleComparer"/> struct.
    /// </summary>
    /// <param name="epsilon">The maximum absolute difference between coordinate values.</param>
    public ApproximateDoubleComparer(double epsilon) => this.epsilon = epsilon;

    /// <inheritdoc/>
    public bool Equals(double x, double y)
    {
        // Match the ImageSharp comparer for equal infinities and matching NaNs.
        if (x == y || (double.IsNaN(x) && double.IsNaN(y)))
        {
            return true;
        }

        double difference = x - y;
        return difference >= -this.epsilon && difference <= this.epsilon;
    }

    /// <inheritdoc/>
    public int GetHashCode(double obj) => obj.GetHashCode();

    /// <inheritdoc/>
    public bool Equals(Vertex x, Vertex y) => this.Equals(x.X, y.X) && this.Equals(x.Y, y.Y);

    /// <inheritdoc/>
    public int GetHashCode(Vertex obj) => obj.GetHashCode();
}

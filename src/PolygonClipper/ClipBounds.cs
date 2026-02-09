// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

namespace SixLabors.PolygonClipper;

internal struct ClipBounds
{
    public ClipBounds(double left, double top, double right, double bottom)
    {
        this.Left = left;
        this.Top = top;
        this.Right = right;
        this.Bottom = bottom;
    }

    public ClipBounds(bool isValid)
    {
        if (isValid)
        {
            this.Left = 0;
            this.Top = 0;
            this.Right = 0;
            this.Bottom = 0;
        }
        else
        {
            this.Left = double.MaxValue;
            this.Top = double.MaxValue;
            this.Right = -double.MaxValue;
            this.Bottom = -double.MaxValue;
        }
    }

    public double Left { get; set; }

    public double Top { get; set; }

    public double Right { get; set; }

    public double Bottom { get; set; }

    public readonly bool IsEmpty() => this.Bottom <= this.Top || this.Right <= this.Left;

    public readonly bool Contains(Vertex point)
        => point.X > this.Left && point.X < this.Right && point.Y > this.Top && point.Y < this.Bottom;

    public readonly bool Contains(ClipBounds bounds)
        => bounds.Left >= this.Left && bounds.Right <= this.Right && bounds.Top >= this.Top && bounds.Bottom <= this.Bottom;

    public readonly bool Intersects(ClipBounds bounds)
        => Math.Max(this.Left, bounds.Left) <= Math.Min(this.Right, bounds.Right) &&
           Math.Max(this.Top, bounds.Top) <= Math.Min(this.Bottom, bounds.Bottom);

    public readonly Vertex MidPoint() => new((this.Left + this.Right) / 2D, (this.Top + this.Bottom) / 2D);
}

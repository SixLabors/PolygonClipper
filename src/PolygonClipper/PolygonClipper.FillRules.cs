// Copyright (c) Six Labors.
// Licensed under the Six Labors Split License.

namespace SixLabors.PolygonClipper;

/// <summary>Provides explicit fill-rule operations on polygon geometry.</summary>
public partial class PolygonClipper
{
    private static readonly Polygon EmptyFillOperand = [];

    private bool useFillRules;
    private FillRule subjectFillRule;
    private FillRule clipFillRule;
    private WindingSweep? windingSweep;

    /// <summary>
    /// Initializes a new instance of the <see cref="PolygonClipper"/> class with independent operand fill rules.
    /// </summary>
    /// <param name="subject">The left-hand operand.</param>
    /// <param name="clip">The right-hand operand.</param>
    /// <param name="operation">The Boolean operation.</param>
    /// <param name="subjectFillRule">The rule interpreting the subject's directed contours.</param>
    /// <param name="clipFillRule">The rule interpreting the clipping operand's directed contours.</param>
    public PolygonClipper(Polygon subject, Polygon clip, BooleanOperation operation, FillRule subjectFillRule, FillRule clipFillRule)
    {
        ValidateFillArguments(subject, clip, operation, subjectFillRule, clipFillRule);
        this.Configure(subject, clip, operation);
        this.useFillRules = true;
        this.subjectFillRule = subjectFillRule;
        this.clipFillRule = clipFillRule;
    }

    /// <summary>Intersects two filled areas using a common fill rule.</summary>
    /// <param name="subject">The left-hand operand.</param>
    /// <param name="clip">The right-hand operand.</param>
    /// <param name="fillRule">The rule interpreting both operands.</param>
    /// <returns>The filled area common to both operands.</returns>
    public static Polygon Intersection(Polygon subject, Polygon clip, FillRule fillRule)
        => Intersection(subject, clip, fillRule, fillRule);

    /// <summary>Intersects two filled areas using independent fill rules.</summary>
    /// <param name="subject">The left-hand operand.</param>
    /// <param name="clip">The right-hand operand.</param>
    /// <param name="subjectFillRule">The subject's fill rule.</param>
    /// <param name="clipFillRule">The clipping operand's fill rule.</param>
    /// <returns>The filled area common to both operands.</returns>
    public static Polygon Intersection(Polygon subject, Polygon clip, FillRule subjectFillRule, FillRule clipFillRule)
        => ExecuteFillOperation(subject, clip, BooleanOperation.Intersection, subjectFillRule, clipFillRule);

    /// <summary>Unites two filled areas using a common fill rule.</summary>
    /// <param name="subject">The left-hand operand.</param>
    /// <param name="clip">The right-hand operand.</param>
    /// <param name="fillRule">The rule interpreting both operands.</param>
    /// <returns>The filled area belonging to either operand.</returns>
    public static Polygon Union(Polygon subject, Polygon clip, FillRule fillRule)
        => Union(subject, clip, fillRule, fillRule);

    /// <summary>Unites two filled areas using independent fill rules.</summary>
    /// <param name="subject">The left-hand operand.</param>
    /// <param name="clip">The right-hand operand.</param>
    /// <param name="subjectFillRule">The subject's fill rule.</param>
    /// <param name="clipFillRule">The clipping operand's fill rule.</param>
    /// <returns>The filled area belonging to either operand.</returns>
    public static Polygon Union(Polygon subject, Polygon clip, FillRule subjectFillRule, FillRule clipFillRule)
        => ExecuteFillOperation(subject, clip, BooleanOperation.Union, subjectFillRule, clipFillRule);

    /// <summary>Resolves directed contours into their filled area.</summary>
    /// <param name="polygon">The raw directed contours.</param>
    /// <param name="fillRule">The rule interpreting the contours.</param>
    /// <returns>The resolved filled area, without changing the input contours.</returns>
    /// <remarks>Input hierarchy does not override contour direction or the selected fill rule.</remarks>
    public static Polygon Union(Polygon polygon, FillRule fillRule)
        => ExecuteFillOperation(polygon, EmptyFillOperand, BooleanOperation.Union, fillRule, FillRule.EvenOdd);

    /// <summary>Subtracts one filled area from another using a common fill rule.</summary>
    /// <param name="subject">The area to retain.</param>
    /// <param name="clip">The area to subtract.</param>
    /// <param name="fillRule">The rule interpreting both operands.</param>
    /// <returns>The subject area outside the clipping operand.</returns>
    public static Polygon Difference(Polygon subject, Polygon clip, FillRule fillRule)
        => Difference(subject, clip, fillRule, fillRule);

    /// <summary>Subtracts one filled area from another using independent fill rules.</summary>
    /// <param name="subject">The area to retain.</param>
    /// <param name="clip">The area to subtract.</param>
    /// <param name="subjectFillRule">The subject's fill rule.</param>
    /// <param name="clipFillRule">The clipping operand's fill rule.</param>
    /// <returns>The subject area outside the clipping operand.</returns>
    public static Polygon Difference(Polygon subject, Polygon clip, FillRule subjectFillRule, FillRule clipFillRule)
        => ExecuteFillOperation(subject, clip, BooleanOperation.Difference, subjectFillRule, clipFillRule);

    /// <summary>Computes exclusive filled coverage using a common fill rule.</summary>
    /// <param name="subject">The left-hand operand.</param>
    /// <param name="clip">The right-hand operand.</param>
    /// <param name="fillRule">The rule interpreting both operands.</param>
    /// <returns>The filled area belonging to exactly one operand.</returns>
    public static Polygon Xor(Polygon subject, Polygon clip, FillRule fillRule)
        => Xor(subject, clip, fillRule, fillRule);

    /// <summary>Computes exclusive filled coverage using independent fill rules.</summary>
    /// <param name="subject">The left-hand operand.</param>
    /// <param name="clip">The right-hand operand.</param>
    /// <param name="subjectFillRule">The subject's fill rule.</param>
    /// <param name="clipFillRule">The clipping operand's fill rule.</param>
    /// <returns>The filled area belonging to exactly one operand.</returns>
    public static Polygon Xor(Polygon subject, Polygon clip, FillRule subjectFillRule, FillRule clipFillRule)
        => ExecuteFillOperation(subject, clip, BooleanOperation.Xor, subjectFillRule, clipFillRule);

    /// <summary>Executes an explicit fill-rule operation through the existing clipper pool.</summary>
    /// <param name="subject">The left-hand operand.</param>
    /// <param name="clip">The right-hand operand.</param>
    /// <param name="operation">The Boolean operation.</param>
    /// <param name="subjectRule">The subject's fill rule.</param>
    /// <param name="clipRule">The clipping operand's fill rule.</param>
    /// <returns>The resolved filled area.</returns>
    private static Polygon ExecuteFillOperation(Polygon subject, Polygon clip, BooleanOperation operation, FillRule subjectRule, FillRule clipRule)
    {
        ValidateFillArguments(subject, clip, operation, subjectRule, clipRule);
        PolygonClipper clipper = Rent(subject, clip, operation);
        try
        {
            clipper.useFillRules = true;
            clipper.subjectFillRule = subjectRule;
            clipper.clipFillRule = clipRule;
            return clipper.Run();
        }
        finally
        {
            Return(clipper);
        }
    }

    /// <summary>Validates new public operation arguments before taking geometry shortcuts.</summary>
    /// <param name="subject">The left-hand operand.</param>
    /// <param name="clip">The right-hand operand.</param>
    /// <param name="operation">The Boolean operation.</param>
    /// <param name="subjectRule">The subject's fill rule.</param>
    /// <param name="clipRule">The clipping operand's fill rule.</param>
    private static void ValidateFillArguments(Polygon subject, Polygon clip, BooleanOperation operation, FillRule subjectRule, FillRule clipRule)
    {
        ArgumentNullException.ThrowIfNull(subject);
        ArgumentNullException.ThrowIfNull(clip);
        ArgumentOutOfRangeException.ThrowIfGreaterThan((uint)operation, (uint)BooleanOperation.Xor, nameof(operation));
        ArgumentOutOfRangeException.ThrowIfGreaterThan((uint)subjectRule, (uint)FillRule.Negative, nameof(subjectRule));
        ArgumentOutOfRangeException.ThrowIfGreaterThan((uint)clipRule, (uint)FillRule.Negative, nameof(clipRule));
    }
}

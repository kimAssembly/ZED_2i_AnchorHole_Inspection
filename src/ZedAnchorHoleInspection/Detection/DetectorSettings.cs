using ZedAnchorHoleInspection.Models;

namespace ZedAnchorHoleInspection.Detection;

public sealed record DetectorSettings
{
    public int MinimumRadiusPixels { get; init; } = 7;
    public int MaximumRadiusPixels { get; init; } = 30;
    public int ScanStepPixels { get; init; } = 3;
    public float MinimumDarkContrast { get; init; } = 12;
    public float PlaneToleranceMm { get; init; } = 5;
    public float MinimumRecessDepthMm { get; init; } = 6;
    public float MaximumRecessDepthMm { get; init; } = 250;
    public float MinimumDiameterMm { get; init; } = 5;
    public float MaximumDiameterMm { get; init; } = 80;
    public float MinimumSurfaceDistanceMm { get; init; } = 300;
    public float MinimumRimValidRatio { get; init; } = 0.65f;
    public float MinimumVoidContrast { get; init; } = 0.25f;
    public float MinimumDarkFillRatio { get; init; } = 0.52f;
    public float MinimumInvalidInteriorRatio { get; init; } = 0.32f;
    public bool AllowVoidOnlyEvidence { get; init; }
    public bool NearRgbOnlyMode { get; init; } = true;
    public bool UseRoboflowModel { get; init; } = true;
    public float RoboflowConfidence { get; init; } = 0.45f;
    public int MaximumCandidates { get; init; } = 24;
    public NormalizedRoi Roi { get; init; } = NormalizedRoi.Full;

    public DetectorSettings Validate()
    {
        if (MinimumRadiusPixels < 3) throw new ArgumentOutOfRangeException(nameof(MinimumRadiusPixels));
        if (MaximumRadiusPixels <= MinimumRadiusPixels) throw new ArgumentOutOfRangeException(nameof(MaximumRadiusPixels));
        if (ScanStepPixels is < 1 or > 12) throw new ArgumentOutOfRangeException(nameof(ScanStepPixels));
        if (MinimumDarkContrast is < 1 or > 200) throw new ArgumentOutOfRangeException(nameof(MinimumDarkContrast));
        if (PlaneToleranceMm is < 0.5f or > 50) throw new ArgumentOutOfRangeException(nameof(PlaneToleranceMm));
        if (MinimumRecessDepthMm < 0) throw new ArgumentOutOfRangeException(nameof(MinimumRecessDepthMm));
        if (MaximumRecessDepthMm <= MinimumRecessDepthMm) throw new ArgumentOutOfRangeException(nameof(MaximumRecessDepthMm));
        if (MinimumDiameterMm <= 0) throw new ArgumentOutOfRangeException(nameof(MinimumDiameterMm));
        if (MaximumDiameterMm <= MinimumDiameterMm) throw new ArgumentOutOfRangeException(nameof(MaximumDiameterMm));
        if (MinimumSurfaceDistanceMm is < 200 or > 3500) throw new ArgumentOutOfRangeException(nameof(MinimumSurfaceDistanceMm));
        if (MinimumRimValidRatio is < 0 or > 1) throw new ArgumentOutOfRangeException(nameof(MinimumRimValidRatio));
        if (MinimumVoidContrast is < 0 or > 1) throw new ArgumentOutOfRangeException(nameof(MinimumVoidContrast));
        if (MinimumDarkFillRatio is < 0 or > 1) throw new ArgumentOutOfRangeException(nameof(MinimumDarkFillRatio));
        if (MinimumInvalidInteriorRatio is < 0 or > 1) throw new ArgumentOutOfRangeException(nameof(MinimumInvalidInteriorRatio));
        if (RoboflowConfidence is < 0.05f or > 0.99f) throw new ArgumentOutOfRangeException(nameof(RoboflowConfidence));
        return this with { Roi = Roi.Clamp() };
    }
}

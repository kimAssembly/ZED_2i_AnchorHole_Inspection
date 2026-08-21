using System.Numerics;
using ZedAnchorHoleInspection.Models;

namespace ZedAnchorHoleInspection.Detection;

public enum HoleEvidence
{
    RecessPoints,
    StereoVoid,
    RecessAndVoid,
    DistantBackground,
    RgbOnly,
    RoboflowAi
}

public sealed record HoleDetection(
    int Id,
    int PixelX,
    int PixelY,
    float RadiusPixels,
    Vector3 SurfacePosition,
    float DiameterMm,
    float? RecessDepthMm,
    float InvalidInteriorRatio,
    double Confidence,
    HoleEvidence Evidence,
    float PlaneRmseMm,
    int PlanePoints);

public sealed record DetectionResult(
    IReadOnlyList<HoleDetection> Holes,
    int AppearanceCandidates,
    int GeometryCandidates,
    GeometryDiagnostics Diagnostics,
    float ValidDepthRatio,
    bool IsRgbOnly,
    TimeSpan Elapsed);

public sealed record GeometryDiagnostics(
    int EvaluatedCandidates,
    int RimCandidates,
    int PlaneCandidates,
    int SurfaceCandidates,
    int DistanceCandidates,
    int DiameterCandidates);

/// <summary>
/// Appearance-first detector for a passive stereo camera.
///
/// Unlike the Helios2 ToF implementation, it does not search the point cloud for
/// a local Z peak. It first finds circular dark/radial structures in the left RGB
/// image and then validates each candidate against an XYZ plane fitted only to
/// the surrounding rim. A hole may be accepted when its interior contains a
/// coherent deeper surface or when stereo matching is invalid inside a strong
/// circular rim. The reported XYZ is the hole mouth projected onto the fitted
/// surface, never a fabricated hole-bottom coordinate.
/// </summary>
public sealed class StereoHoleDetector
{
    const int AngularSectors = 16;

    public DetectionResult Detect(LiveFrame frame, DetectorSettings rawSettings)
    {
        var started = System.Diagnostics.Stopwatch.StartNew();
        DetectorSettings settings = rawSettings.Validate();
        if (frame.Bgra.Length < frame.Width * frame.Height * 4 || frame.Width < 32 || frame.Height < 32)
            return new([], 0, 0, new(0, 0, 0, 0, 0, 0), 0, settings.NearRgbOnlyMode, started.Elapsed);

        byte[] gray = ToGray(frame);
        long[] integral = BuildIntegral(gray, frame.Width, frame.Height);
        List<AppearanceCandidate> appearance = FindAppearanceCandidates(frame, gray, integral, settings);
        if (settings.NearRgbOnlyMode)
            return DetectNearRgb(frame, appearance, settings, started);

        var holes = new List<HoleDetection>();
        int geometryCandidates = 0;
        int evaluatedCandidates = 0;
        int rimCandidates = 0;
        int planeCandidates = 0;
        int surfaceCandidates = 0;
        int distanceCandidates = 0;
        int diameterCandidates = 0;
        foreach (AppearanceCandidate candidate in appearance)
        {
            evaluatedCandidates++;
            GeometryValidation? geometry = ValidateGeometry(
                frame,
                candidate,
                settings,
                out bool rimReady,
                out bool planeReady,
                out bool surfaceReady,
                out bool distanceReady,
                out bool diameterReady);
            if (rimReady) rimCandidates++;
            if (planeReady) planeCandidates++;
            if (surfaceReady) surfaceCandidates++;
            if (distanceReady) distanceCandidates++;
            if (diameterReady) diameterCandidates++;
            if (geometry is null) continue;
            geometryCandidates++;

            double appearanceConfidence = Math.Clamp(
                (candidate.Contrast - settings.MinimumDarkContrast) /
                Math.Max(1, settings.MinimumDarkContrast * 2.5f), 0, 1);
            appearanceConfidence = appearanceConfidence * .65 + candidate.DarkFillRatio * .35;
            double planeConfidence = Math.Clamp(1 - geometry.Plane.Rmse / Math.Max(1, settings.PlaneToleranceMm), 0, 1);
            double evidenceConfidence = Math.Max(
                geometry.InvalidInteriorRatio,
                Math.Clamp(geometry.DeepPointRatio * 1.6f, 0, 1));
            double confidence = Math.Clamp(
                .28 + .24 * appearanceConfidence + .22 * planeConfidence +
                .14 * geometry.AngularCoverage + .12 * evidenceConfidence, 0, 1);

            holes.Add(new HoleDetection(
                0,
                candidate.PixelX,
                candidate.PixelY,
                candidate.Radius,
                geometry.Surface.Center,
                geometry.DiameterMm,
                geometry.RecessDepthMm,
                geometry.InvalidInteriorRatio,
                confidence,
                geometry.Evidence,
                geometry.Plane.Rmse,
                geometry.Plane.Inliers.Count));
        }

        holes = SuppressOverlaps(holes)
            .OrderByDescending(hole => hole.Confidence)
            .Select((hole, index) => hole with { Id = index + 1 })
            .ToList();

        var diagnostics = new GeometryDiagnostics(
            evaluatedCandidates,
            rimCandidates,
            planeCandidates,
            surfaceCandidates,
            distanceCandidates,
            diameterCandidates);
        return new(
            holes,
            appearance.Count,
            geometryCandidates,
            diagnostics,
            CalculateValidDepthRatio(frame, settings.Roi),
            false,
            started.Elapsed);
    }

    static DetectionResult DetectNearRgb(
        LiveFrame frame,
        List<AppearanceCandidate> appearance,
        DetectorSettings settings,
        System.Diagnostics.Stopwatch started)
    {
        List<HoleDetection> holes = appearance
            .Where(candidate =>
                candidate.PositiveSectors >= 14 &&
                candidate.EdgeConsistency >= .72f &&
                candidate.DarkFillRatio >= Math.Max(.64f, settings.MinimumDarkFillRatio) &&
                candidate.Contrast >= settings.MinimumDarkContrast * 1.15f)
            .Select(candidate =>
            {
                double contrastConfidence = Math.Clamp(
                    (candidate.Contrast - settings.MinimumDarkContrast) /
                    Math.Max(1, settings.MinimumDarkContrast * 3f), 0, 1);
                double confidence = Math.Clamp(
                    .18 + .28 * contrastConfidence + .24 * candidate.DarkFillRatio +
                    .30 * candidate.EdgeConsistency, 0, 1);
                return new HoleDetection(
                    0,
                    candidate.PixelX,
                    candidate.PixelY,
                    candidate.Radius,
                    Vector3.Zero,
                    0,
                    null,
                    0,
                    confidence,
                    HoleEvidence.RgbOnly,
                    0,
                    0);
            })
            .ToList();

        holes = SuppressOverlaps(holes)
            .OrderByDescending(hole => hole.Confidence)
            .Select((hole, index) => hole with { Id = index + 1 })
            .ToList();

        var diagnostics = new GeometryDiagnostics(appearance.Count, 0, 0, 0, 0, 0);
        return new(
            holes,
            appearance.Count,
            holes.Count,
            diagnostics,
            CalculateValidDepthRatio(frame, settings.Roi),
            true,
            started.Elapsed);
    }

    static List<AppearanceCandidate> FindAppearanceCandidates(
        LiveFrame frame,
        byte[] gray,
        long[] integral,
        DetectorSettings settings)
    {
        NormalizedRoi roi = settings.Roi;
        int margin = (int)Math.Ceiling(settings.MaximumRadiusPixels * 1.55);
        int x0 = Math.Max(margin, (int)Math.Floor(roi.X * frame.Width));
        int y0 = Math.Max(margin, (int)Math.Floor(roi.Y * frame.Height));
        int x1 = Math.Min(frame.Width - margin, (int)Math.Ceiling((roi.X + roi.Width) * frame.Width));
        int y1 = Math.Min(frame.Height - margin, (int)Math.Ceiling((roi.Y + roi.Height) * frame.Height));
        if (x1 <= x0 || y1 <= y0) return [];

        var raw = new List<AppearanceCandidate>();
        int radiusStep = Math.Max(2, (settings.MaximumRadiusPixels - settings.MinimumRadiusPixels) / 7);
        for (int y = y0; y < y1; y += settings.ScanStepPixels)
        for (int x = x0; x < x1; x += settings.ScanStepPixels)
        for (int radius = settings.MinimumRadiusPixels; radius <= settings.MaximumRadiusPixels; radius += radiusStep)
        {
            int centerHalf = Math.Max(2, (int)Math.Round(radius * .42));
            int ringInner = Math.Max(centerHalf + 1, (int)Math.Round(radius * .72));
            int ringOuter = Math.Max(ringInner + 1, (int)Math.Round(radius * 1.35));
            double centerMean = MeanBox(integral, frame.Width, x - centerHalf, y - centerHalf, x + centerHalf + 1, y + centerHalf + 1);
            double outerSum = BoxSum(integral, frame.Width, x - ringOuter, y - ringOuter, x + ringOuter + 1, y + ringOuter + 1);
            double innerSum = BoxSum(integral, frame.Width, x - ringInner, y - ringInner, x + ringInner + 1, y + ringInner + 1);
            int outerArea = (ringOuter * 2 + 1) * (ringOuter * 2 + 1);
            int innerArea = (ringInner * 2 + 1) * (ringInner * 2 + 1);
            double ringMean = (outerSum - innerSum) / Math.Max(1, outerArea - innerArea);
            float contrast = (float)(ringMean - centerMean);
            if (contrast < settings.MinimumDarkContrast) continue;

            (float symmetry, int positiveSectors) = RadialSymmetry(gray, frame.Width, x, y, radius, settings.MinimumDarkContrast);
            if (positiveSectors < 12 || symmetry < settings.MinimumDarkContrast * .55f) continue;
            float edgeConsistency = RadialEdgeConsistency(
                gray,
                frame.Width,
                x,
                y,
                radius,
                settings.MinimumDarkContrast);
            if (edgeConsistency < .50f) continue;
            float darkFillRatio = DarkFillRatio(gray, frame.Width, x, y, radius, (float)ringMean, contrast);
            if (darkFillRatio < settings.MinimumDarkFillRatio) continue;
            raw.Add(new(
                x,
                y,
                radius,
                contrast,
                darkFillRatio,
                edgeConsistency,
                positiveSectors,
                contrast * .48f + symmetry * .24f +
                darkFillRatio * settings.MinimumDarkContrast * 1.4f +
                edgeConsistency * settings.MinimumDarkContrast * 1.2f));
        }

        var selected = new List<AppearanceCandidate>();
        foreach (AppearanceCandidate candidate in raw.OrderByDescending(item => item.Score))
        {
            bool overlaps = selected.Any(existing =>
            {
                float dx = existing.PixelX - candidate.PixelX;
                float dy = existing.PixelY - candidate.PixelY;
                float gate = Math.Max(existing.Radius, candidate.Radius) * .85f;
                return dx * dx + dy * dy < gate * gate;
            });
            if (overlaps) continue;
            selected.Add(candidate);
            if (selected.Count >= settings.MaximumCandidates * 3) break;
        }
        return selected;
    }

    static GeometryValidation? ValidateGeometry(
        LiveFrame frame,
        AppearanceCandidate candidate,
        DetectorSettings settings,
        out bool rimReady,
        out bool planeReady,
        out bool surfaceReady,
        out bool distanceReady,
        out bool diameterReady)
    {
        rimReady = false;
        planeReady = false;
        surfaceReady = false;
        distanceReady = false;
        diameterReady = false;
        List<SurfaceSample> rim = GetSamples(
            frame,
            candidate,
            1.22f,
            2.15f,
            validOnly: true,
            out int rimTotal,
            out int rimInvalid);
        float rimValidRatio = rimTotal == 0 ? 0 : (float)(rimTotal - rimInvalid) / rimTotal;
        if (rimValidRatio < settings.MinimumRimValidRatio) return null;
        if (rim.Count < 24) return null;
        rimReady = true;

        PlaneFit? plane = FitPlane(rim, settings.PlaneToleranceMm);
        if (plane is null || plane.Inliers.Count < 20 || plane.Rmse > settings.PlaneToleranceMm * 1.25f)
            return null;

        float angularCoverage = AngularCoverage(plane.Inliers, candidate.PixelX, candidate.PixelY);
        if (angularCoverage < .50f) return null;
        planeReady = true;

        SurfaceModel? surface = FitPixelSurface(plane.Inliers, candidate.PixelX, candidate.PixelY);
        if (surface is null) return null;
        surfaceReady = true;
        if (!float.IsFinite(surface.Center.Z) || surface.Center.Z < settings.MinimumSurfaceDistanceMm)
            return null;
        distanceReady = true;
        float millimetersPerPixel = (surface.Du.Length() + surface.Dv.Length()) * .5f;
        float diameterMm = 2 * candidate.Radius * millimetersPerPixel;
        if (!float.IsFinite(diameterMm) ||
            diameterMm < settings.MinimumDiameterMm ||
            diameterMm > settings.MaximumDiameterMm)
            return null;
        diameterReady = true;

        _ = GetSamples(frame, candidate, 0, .64f, validOnly: false, out int interiorTotal, out int interiorInvalid);
        if (interiorTotal < 4) return null;
        float invalidRatio = (float)interiorInvalid / interiorTotal;

        var recessResiduals = new List<float>();
        int recessPoints = 0;
        int distantPoints = 0;
        int validInterior = 0;
        EnumerateInterior(frame, candidate, (pixelX, pixelY, point, valid) =>
        {
            if (!valid) return;
            validInterior++;
            float residual = Vector3.Dot(plane.Normal, point) + plane.D;
            if (!float.IsFinite(residual) || residual < -settings.PlaneToleranceMm * 2)
                return;
            if (residual > settings.MaximumRecessDepthMm)
            {
                distantPoints++;
                return;
            }
            if (residual >= settings.MinimumRecessDepthMm)
            {
                recessPoints++;
                recessResiduals.Add(residual);
            }
        });

        float recessRatio = (float)recessPoints / interiorTotal;
        float distantRatio = (float)distantPoints / interiorTotal;
        float deepRatio = recessRatio + distantRatio;
        float? recessDepth = recessResiduals.Count == 0 ? null : Median(recessResiduals);
        bool hasRecess = recessPoints >= 4 && recessRatio >= .28f && recessDepth >= settings.MinimumRecessDepthMm;
        bool hasDistantBackground = distantPoints >= 4 && distantRatio >= .28f;
        float rimInvalidRatio = 1 - rimValidRatio;
        float voidContrast = invalidRatio - rimInvalidRatio;
        bool hasVoidPattern = invalidRatio >= settings.MinimumInvalidInteriorRatio &&
                              voidContrast >= Math.Max(.30f, settings.MinimumVoidContrast) &&
                              rimValidRatio >= .75f &&
                              candidate.DarkFillRatio >= Math.Max(.68f, settings.MinimumDarkFillRatio) &&
                              candidate.EdgeConsistency >= .72f &&
                              candidate.PositiveSectors >= 14;
        bool hasVoid = settings.AllowVoidOnlyEvidence && hasVoidPattern;
        if (!hasRecess && !hasDistantBackground && !hasVoid) return null;

        HoleEvidence evidence = hasDistantBackground
            ? HoleEvidence.DistantBackground
            : hasRecess && hasVoidPattern
            ? HoleEvidence.RecessAndVoid
            : hasRecess ? HoleEvidence.RecessPoints : HoleEvidence.StereoVoid;
        if (!hasRecess) recessDepth = null;

        return new(plane, surface, diameterMm, recessDepth, invalidRatio, deepRatio, angularCoverage, evidence);
    }

    static float CalculateValidDepthRatio(LiveFrame frame, NormalizedRoi roi)
    {
        int valid = 0;
        int total = 0;
        for (int gridY = 0; gridY < frame.PointHeight; gridY++)
        for (int gridX = 0; gridX < frame.PointWidth; gridX++)
        {
            (int pixelX, int pixelY) = frame.PointPixel(gridX, gridY);
            float nx = frame.Width <= 1 ? 0 : (float)pixelX / (frame.Width - 1);
            float ny = frame.Height <= 1 ? 0 : (float)pixelY / (frame.Height - 1);
            if (nx < roi.X || nx > roi.X + roi.Width || ny < roi.Y || ny > roi.Y + roi.Height)
                continue;
            total++;
            if (frame.TryGetGridPoint(gridX, gridY, out _)) valid++;
        }
        return total == 0 ? 0 : (float)valid / total;
    }

    static List<SurfaceSample> GetSamples(
        LiveFrame frame,
        AppearanceCandidate candidate,
        float minimumRadiusScale,
        float maximumRadiusScale,
        bool validOnly,
        out int total,
        out int invalid)
    {
        var samples = new List<SurfaceSample>();
        total = 0;
        invalid = 0;
        float minimumSquared = candidate.Radius * minimumRadiusScale;
        minimumSquared *= minimumSquared;
        float maximumSquared = candidate.Radius * maximumRadiusScale;
        maximumSquared *= maximumSquared;

        GridBounds bounds = GetGridBounds(frame, candidate, maximumRadiusScale);
        for (int gridY = bounds.MinimumY; gridY <= bounds.MaximumY; gridY++)
        for (int gridX = bounds.MinimumX; gridX <= bounds.MaximumX; gridX++)
        {
            (int pixelX, int pixelY) = frame.PointPixel(gridX, gridY);
            float dx = pixelX - candidate.PixelX;
            float dy = pixelY - candidate.PixelY;
            float distanceSquared = dx * dx + dy * dy;
            if (distanceSquared < minimumSquared || distanceSquared > maximumSquared) continue;
            total++;
            if (!frame.TryGetGridPoint(gridX, gridY, out Vector3 point))
            {
                invalid++;
                continue;
            }
            samples.Add(new(pixelX, pixelY, point));
        }

        return validOnly ? samples : samples;
    }

    static void EnumerateInterior(
        LiveFrame frame,
        AppearanceCandidate candidate,
        Action<int, int, Vector3, bool> visitor)
    {
        float radiusSquared = candidate.Radius * .64f;
        radiusSquared *= radiusSquared;
        GridBounds bounds = GetGridBounds(frame, candidate, .64f);
        for (int gridY = bounds.MinimumY; gridY <= bounds.MaximumY; gridY++)
        for (int gridX = bounds.MinimumX; gridX <= bounds.MaximumX; gridX++)
        {
            (int pixelX, int pixelY) = frame.PointPixel(gridX, gridY);
            float dx = pixelX - candidate.PixelX;
            float dy = pixelY - candidate.PixelY;
            if (dx * dx + dy * dy > radiusSquared) continue;
            bool valid = frame.TryGetGridPoint(gridX, gridY, out Vector3 point);
            visitor(pixelX, pixelY, point, valid);
        }
    }

    static GridBounds GetGridBounds(LiveFrame frame, AppearanceCandidate candidate, float radiusScale)
    {
        float radius = candidate.Radius * radiusScale + 3;
        double scaleX = frame.Width <= 1 ? 0 : (double)(frame.PointWidth - 1) / (frame.Width - 1);
        double scaleY = frame.Height <= 1 ? 0 : (double)(frame.PointHeight - 1) / (frame.Height - 1);
        int minimumX = Math.Clamp((int)Math.Floor((candidate.PixelX - radius) * scaleX) - 1, 0, frame.PointWidth - 1);
        int maximumX = Math.Clamp((int)Math.Ceiling((candidate.PixelX + radius) * scaleX) + 1, 0, frame.PointWidth - 1);
        int minimumY = Math.Clamp((int)Math.Floor((candidate.PixelY - radius) * scaleY) - 1, 0, frame.PointHeight - 1);
        int maximumY = Math.Clamp((int)Math.Ceiling((candidate.PixelY + radius) * scaleY) + 1, 0, frame.PointHeight - 1);
        return new(minimumX, maximumX, minimumY, maximumY);
    }

    static PlaneFit? FitPlane(List<SurfaceSample> samples, float tolerance)
    {
        if (samples.Count < 3) return null;
        var random = new Random(9127 + samples.Count);
        List<SurfaceSample> best = [];
        for (int iteration = 0; iteration < 120; iteration++)
        {
            Vector3 a = samples[random.Next(samples.Count)].Point;
            Vector3 b = samples[random.Next(samples.Count)].Point;
            Vector3 c = samples[random.Next(samples.Count)].Point;
            Vector3 normal = Vector3.Cross(b - a, c - a);
            if (normal.LengthSquared() < 1e-6f) continue;
            normal = Vector3.Normalize(normal);
            float d = -Vector3.Dot(normal, a);
            var inliers = samples.Where(sample => MathF.Abs(Vector3.Dot(normal, sample.Point) + d) <= tolerance).ToList();
            if (inliers.Count > best.Count) best = inliers;
        }
        if (best.Count < 3) return null;

        Vector3 centroid = new(
            best.Average(sample => sample.Point.X),
            best.Average(sample => sample.Point.Y),
            best.Average(sample => sample.Point.Z));
        double[,] covariance = new double[3, 3];
        foreach (SurfaceSample sample in best)
        {
            Vector3 q = sample.Point - centroid;
            covariance[0, 0] += q.X * q.X; covariance[0, 1] += q.X * q.Y; covariance[0, 2] += q.X * q.Z;
            covariance[1, 1] += q.Y * q.Y; covariance[1, 2] += q.Y * q.Z;
            covariance[2, 2] += q.Z * q.Z;
        }
        covariance[1, 0] = covariance[0, 1];
        covariance[2, 0] = covariance[0, 2];
        covariance[2, 1] = covariance[1, 2];

        Vector3 refinedNormal = SmallestEigenVector(covariance);
        if (refinedNormal.LengthSquared() < .5f) return null;
        if (refinedNormal.Z < 0) refinedNormal = -refinedNormal;
        float refinedD = -Vector3.Dot(refinedNormal, centroid);
        List<SurfaceSample> refinedInliers = samples
            .Where(sample => MathF.Abs(Vector3.Dot(refinedNormal, sample.Point) + refinedD) <= tolerance)
            .ToList();
        if (refinedInliers.Count < 3) return null;
        float rmse = MathF.Sqrt(refinedInliers.Average(sample =>
        {
            float residual = Vector3.Dot(refinedNormal, sample.Point) + refinedD;
            return residual * residual;
        }));
        return new(refinedNormal, refinedD, refinedInliers, rmse);
    }

    static SurfaceModel? FitPixelSurface(List<SurfaceSample> samples, int centerX, int centerY)
    {
        double[,] normal = new double[3, 3];
        double[] bx = new double[3], by = new double[3], bz = new double[3];
        foreach (SurfaceSample sample in samples)
        {
            double[] q = [1, sample.PixelX - centerX, sample.PixelY - centerY];
            for (int row = 0; row < 3; row++)
            {
                for (int column = 0; column < 3; column++) normal[row, column] += q[row] * q[column];
                bx[row] += q[row] * sample.Point.X;
                by[row] += q[row] * sample.Point.Y;
                bz[row] += q[row] * sample.Point.Z;
            }
        }

        double[]? x = Solve3x3(normal, bx);
        double[]? y = Solve3x3(normal, by);
        double[]? z = Solve3x3(normal, bz);
        if (x is null || y is null || z is null) return null;
        return new(
            new((float)x[0], (float)y[0], (float)z[0]),
            new((float)x[1], (float)y[1], (float)z[1]),
            new((float)x[2], (float)y[2], (float)z[2]));
    }

    static double[]? Solve3x3(double[,] source, double[] values)
    {
        var matrix = (double[,])source.Clone();
        var result = (double[])values.Clone();
        for (int pivot = 0; pivot < 3; pivot++)
        {
            int best = pivot;
            for (int row = pivot + 1; row < 3; row++)
                if (Math.Abs(matrix[row, pivot]) > Math.Abs(matrix[best, pivot])) best = row;
            if (Math.Abs(matrix[best, pivot]) < 1e-9) return null;
            if (best != pivot)
            {
                for (int column = 0; column < 3; column++)
                    (matrix[pivot, column], matrix[best, column]) = (matrix[best, column], matrix[pivot, column]);
                (result[pivot], result[best]) = (result[best], result[pivot]);
            }

            double divisor = matrix[pivot, pivot];
            for (int column = pivot; column < 3; column++) matrix[pivot, column] /= divisor;
            result[pivot] /= divisor;
            for (int row = 0; row < 3; row++)
            {
                if (row == pivot) continue;
                double factor = matrix[row, pivot];
                for (int column = pivot; column < 3; column++) matrix[row, column] -= factor * matrix[pivot, column];
                result[row] -= factor * result[pivot];
            }
        }
        return result;
    }

    static Vector3 SmallestEigenVector(double[,] source)
    {
        var a = (double[,])source.Clone();
        double[,] vectors = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } };
        for (int iteration = 0; iteration < 24; iteration++)
        {
            int p = 0, q = 1;
            double largest = Math.Abs(a[0, 1]);
            if (Math.Abs(a[0, 2]) > largest) { largest = Math.Abs(a[0, 2]); p = 0; q = 2; }
            if (Math.Abs(a[1, 2]) > largest) { largest = Math.Abs(a[1, 2]); p = 1; q = 2; }
            if (largest < 1e-10) break;

            double angle = .5 * Math.Atan2(2 * a[p, q], a[q, q] - a[p, p]);
            double c = Math.Cos(angle), s = Math.Sin(angle);
            for (int k = 0; k < 3; k++)
            {
                double apk = a[p, k], aqk = a[q, k];
                a[p, k] = c * apk - s * aqk;
                a[q, k] = s * apk + c * aqk;
            }
            for (int k = 0; k < 3; k++)
            {
                double akp = a[k, p], akq = a[k, q];
                a[k, p] = c * akp - s * akq;
                a[k, q] = s * akp + c * akq;
            }
            for (int k = 0; k < 3; k++)
            {
                double vkp = vectors[k, p], vkq = vectors[k, q];
                vectors[k, p] = c * vkp - s * vkq;
                vectors[k, q] = s * vkp + c * vkq;
            }
        }

        int smallest = a[1, 1] < a[0, 0] ? 1 : 0;
        if (a[2, 2] < a[smallest, smallest]) smallest = 2;
        var vector = new Vector3((float)vectors[0, smallest], (float)vectors[1, smallest], (float)vectors[2, smallest]);
        return vector.LengthSquared() < 1e-12f ? Vector3.Zero : Vector3.Normalize(vector);
    }

    static float AngularCoverage(List<SurfaceSample> samples, int centerX, int centerY)
    {
        Span<bool> occupied = stackalloc bool[AngularSectors];
        foreach (SurfaceSample sample in samples)
        {
            double angle = Math.Atan2(sample.PixelY - centerY, sample.PixelX - centerX) + Math.PI;
            int sector = Math.Min(AngularSectors - 1, (int)(angle / (Math.PI * 2) * AngularSectors));
            occupied[sector] = true;
        }
        int count = 0;
        foreach (bool value in occupied) if (value) count++;
        return (float)count / AngularSectors;
    }

    static (float Symmetry, int PositiveSectors) RadialSymmetry(
        byte[] gray,
        int width,
        int centerX,
        int centerY,
        int radius,
        float contrastThreshold)
    {
        float total = 0;
        int positive = 0;
        for (int sector = 0; sector < AngularSectors; sector++)
        {
            double angle = sector * Math.PI * 2 / AngularSectors;
            int innerX = centerX + (int)Math.Round(Math.Cos(angle) * radius * .42);
            int innerY = centerY + (int)Math.Round(Math.Sin(angle) * radius * .42);
            int outerX = centerX + (int)Math.Round(Math.Cos(angle) * radius * 1.10);
            int outerY = centerY + (int)Math.Round(Math.Sin(angle) * radius * 1.10);
            float difference = gray[outerY * width + outerX] - gray[innerY * width + innerX];
            total += Math.Max(0, difference);
            if (difference >= contrastThreshold * .35f) positive++;
        }
        return (total / AngularSectors, positive);
    }

    static float DarkFillRatio(
        byte[] gray,
        int width,
        int centerX,
        int centerY,
        int radius,
        float ringMean,
        float contrast)
    {
        float sampleRadius = radius * .72f;
        float radiusSquared = sampleRadius * sampleRadius;
        float threshold = ringMean - Math.Max(6, contrast * .38f);
        int dark = 0;
        int total = 0;
        int extent = (int)Math.Ceiling(sampleRadius);
        for (int dy = -extent; dy <= extent; dy++)
        for (int dx = -extent; dx <= extent; dx++)
        {
            if (dx * dx + dy * dy > radiusSquared) continue;
            total++;
            if (gray[(centerY + dy) * width + centerX + dx] <= threshold) dark++;
        }
        return total == 0 ? 0 : (float)dark / total;
    }

    static float RadialEdgeConsistency(
        byte[] gray,
        int width,
        int centerX,
        int centerY,
        int radius,
        float contrastThreshold)
    {
        Span<float> edgeRadii = stackalloc float[AngularSectors];
        int found = 0;
        float probe = Math.Max(1, radius * .12f);
        float step = Math.Max(1, radius * .08f);
        for (int sector = 0; sector < AngularSectors; sector++)
        {
            double angle = sector * Math.PI * 2 / AngularSectors;
            double cosine = Math.Cos(angle);
            double sine = Math.Sin(angle);
            float bestGradient = float.MinValue;
            float bestRadius = 0;
            for (float distance = radius * .56f; distance <= radius * 1.32f; distance += step)
            {
                int innerX = centerX + (int)Math.Round(cosine * (distance - probe));
                int innerY = centerY + (int)Math.Round(sine * (distance - probe));
                int outerX = centerX + (int)Math.Round(cosine * (distance + probe));
                int outerY = centerY + (int)Math.Round(sine * (distance + probe));
                float gradient = gray[outerY * width + outerX] - gray[innerY * width + innerX];
                if (gradient <= bestGradient) continue;
                bestGradient = gradient;
                bestRadius = distance;
            }
            if (bestGradient < contrastThreshold * .45f) continue;
            edgeRadii[found++] = bestRadius;
        }

        if (found < 1) return 0;
        Span<float> measured = edgeRadii[..found];
        measured.Sort();
        float median = measured[found / 2];
        Span<float> deviations = stackalloc float[AngularSectors];
        for (int index = 0; index < found; index++)
            deviations[index] = MathF.Abs(measured[index] - median);
        Span<float> usedDeviations = deviations[..found];
        usedDeviations.Sort();
        float normalizedMad = usedDeviations[found / 2] / Math.Max(1, median);
        float coverage = (float)found / AngularSectors;
        float consistency = 1 - Math.Clamp(normalizedMad / .30f, 0, 1);
        return coverage * consistency;
    }

    static List<HoleDetection> SuppressOverlaps(List<HoleDetection> holes)
    {
        var selected = new List<HoleDetection>();
        foreach (HoleDetection hole in holes.OrderByDescending(item => item.Confidence))
        {
            bool overlaps = selected.Any(existing =>
            {
                float dx = existing.PixelX - hole.PixelX;
                float dy = existing.PixelY - hole.PixelY;
                float gate = Math.Max(existing.RadiusPixels, hole.RadiusPixels);
                return dx * dx + dy * dy < gate * gate;
            });
            if (!overlaps) selected.Add(hole);
        }
        return selected;
    }

    static byte[] ToGray(LiveFrame frame)
    {
        var gray = new byte[frame.Width * frame.Height];
        for (int source = 0, target = 0; target < gray.Length; source += 4, target++)
            gray[target] = (byte)((frame.Bgra[source] * 29 + frame.Bgra[source + 1] * 150 + frame.Bgra[source + 2] * 77) >> 8);
        return gray;
    }

    static long[] BuildIntegral(byte[] gray, int width, int height)
    {
        var integral = new long[(width + 1) * (height + 1)];
        int stride = width + 1;
        for (int y = 0; y < height; y++)
        {
            long row = 0;
            for (int x = 0; x < width; x++)
            {
                row += gray[y * width + x];
                integral[(y + 1) * stride + x + 1] = integral[y * stride + x + 1] + row;
            }
        }
        return integral;
    }

    static double MeanBox(long[] integral, int width, int x0, int y0, int x1, int y1) =>
        BoxSum(integral, width, x0, y0, x1, y1) / Math.Max(1, (x1 - x0) * (y1 - y0));

    static long BoxSum(long[] integral, int width, int x0, int y0, int x1, int y1)
    {
        int stride = width + 1;
        return integral[y1 * stride + x1] - integral[y0 * stride + x1] -
               integral[y1 * stride + x0] + integral[y0 * stride + x0];
    }

    static float Median(List<float> values)
    {
        values.Sort();
        int middle = values.Count / 2;
        return values.Count % 2 == 1 ? values[middle] : (values[middle - 1] + values[middle]) * .5f;
    }

    sealed record AppearanceCandidate(
        int PixelX,
        int PixelY,
        float Radius,
        float Contrast,
        float DarkFillRatio,
        float EdgeConsistency,
        int PositiveSectors,
        float Score);

    readonly record struct GridBounds(int MinimumX, int MaximumX, int MinimumY, int MaximumY);
    sealed record SurfaceSample(int PixelX, int PixelY, Vector3 Point);
    sealed record PlaneFit(Vector3 Normal, float D, List<SurfaceSample> Inliers, float Rmse);
    sealed record SurfaceModel(Vector3 Center, Vector3 Du, Vector3 Dv);
    sealed record GeometryValidation(
        PlaneFit Plane,
        SurfaceModel Surface,
        float DiameterMm,
        float? RecessDepthMm,
        float InvalidInteriorRatio,
        float DeepPointRatio,
        float AngularCoverage,
        HoleEvidence Evidence);
}

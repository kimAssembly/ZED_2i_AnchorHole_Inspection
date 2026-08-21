using System.Numerics;
using ZedAnchorHoleInspection.Camera;
using ZedAnchorHoleInspection.Detection;
using ZedAnchorHoleInspection.Models;

if (args.Contains("--camera", StringComparer.OrdinalIgnoreCase))
    return await TestCameraAsync();

var detector = new StereoHoleDetector();
var settings = new DetectorSettings
{
    MinimumRadiusPixels = 7,
    MaximumRadiusPixels = 24,
    ScanStepPixels = 2,
    MinimumDarkContrast = 10,
    PlaneToleranceMm = 3,
    MinimumRecessDepthMm = 5,
    MinimumInvalidInteriorRatio = .30f,
    MaximumCandidates = 16,
    NearRgbOnlyMode = false,
    Roi = new(.12f, .12f, .76f, .76f)
};

int failures = 0;
failures += RunDetectionCase(
    "near RGB",
    BuildSyntheticFrame(SyntheticHole.DarkMark),
    HoleEvidence.RgbOnly,
    settings with { NearRgbOnlyMode = true });
DetectionResult strictVoid = detector.Detect(BuildSyntheticFrame(SyntheticHole.Void), settings);
if (strictVoid.Holes.Count != 0)
{
    Console.Error.WriteLine($"FAIL strict stereo void: expected 0, got {strictVoid.Holes.Count}");
    failures++;
}
else
{
    Console.WriteLine("PASS strict stereo void: void-only evidence rejected");
}

failures += RunDetectionCase(
    "permissive stereo void",
    BuildSyntheticFrame(SyntheticHole.Void),
    HoleEvidence.StereoVoid,
    settings with { AllowVoidOnlyEvidence = true });
failures += RunDetectionCase("visible back wall", BuildSyntheticFrame(SyntheticHole.DeepBackWall), HoleEvidence.RecessPoints, settings);
failures += RunDetectionCase("distant background", BuildSyntheticFrame(SyntheticHole.DistantBackground), HoleEvidence.DistantBackground, settings);

DetectionResult negative = detector.Detect(BuildSyntheticFrame(SyntheticHole.None), settings);
if (negative.Holes.Count != 0)
{
    Console.Error.WriteLine($"FAIL plain surface: expected 0, got {negative.Holes.Count}");
    failures++;
}
else
{
    Console.WriteLine("PASS plain surface: 0 holes");
}

DetectionResult darkMark = detector.Detect(BuildSyntheticFrame(SyntheticHole.DarkMark), settings);
if (darkMark.Holes.Count != 0)
{
    Console.Error.WriteLine($"FAIL dark mark: expected 0, got {darkMark.Holes.Count}");
    failures++;
}
else
{
    Console.WriteLine("PASS dark mark: RGB circle without depth change rejected");
}

DetectionResult tooClose = detector.Detect(BuildSyntheticFrame(SyntheticHole.CloseDeepBackWall), settings);
if (tooClose.Holes.Count != 0)
{
    Console.Error.WriteLine($"FAIL too close: expected 0, got {tooClose.Holes.Count}");
    failures++;
}
else
{
    Console.WriteLine("PASS too close: unreliable near-range geometry rejected");
}

var tracker = new TemporalHoleTracker();
DetectionResult trackedSource = detector.Detect(BuildSyntheticFrame(SyntheticHole.DeepBackWall), settings);
bool appearedEarly = false;
for (int index = 0; index < 4; index++)
    appearedEarly |= tracker.Update(trackedSource.Holes).Count != 0;
if (appearedEarly || tracker.Update(trackedSource.Holes).Count == 0)
{
    Console.Error.WriteLine("FAIL temporal tracker: a detection must become stable on frame 5");
    failures++;
}
else
{
    Console.WriteLine("PASS temporal tracker: stable on frame 5");
}

Console.WriteLine(failures == 0 ? "ALL SELF-TESTS PASSED" : $"SELF-TEST FAILURES: {failures}");
return failures == 0 ? 0 : 1;

int RunDetectionCase(string name, LiveFrame frame, HoleEvidence expectedEvidence, DetectorSettings caseSettings)
{
    DetectionResult result = detector.Detect(frame, caseSettings);
    HoleDetection? hole = result.Holes
        .OrderBy(item => Vector2.Distance(new(item.PixelX, item.PixelY), new(160, 120)))
        .FirstOrDefault();
    if (hole is null || Math.Abs(hole.PixelX - 160) > 10 || Math.Abs(hole.PixelY - 120) > 10)
    {
        Console.Error.WriteLine($"FAIL {name}: center hole not found; RGB={result.AppearanceCandidates}, XYZ={result.GeometryCandidates}");
        return 1;
    }
    if (expectedEvidence != HoleEvidence.RgbOnly &&
        (hole.DiameterMm is < 15 or > 70 || !float.IsFinite(hole.SurfacePosition.Z)))
    {
        Console.Error.WriteLine($"FAIL {name}: invalid geometry Ø={hole.DiameterMm:F1}, XYZ={hole.SurfacePosition}");
        return 1;
    }
    if (expectedEvidence == HoleEvidence.RgbOnly && hole.RadiusPixels <= 0)
    {
        Console.Error.WriteLine($"FAIL {name}: invalid pixel radius {hole.RadiusPixels:F1}");
        return 1;
    }

    bool evidenceMatches = expectedEvidence switch
    {
        HoleEvidence.StereoVoid => hole.Evidence is HoleEvidence.StereoVoid or HoleEvidence.RecessAndVoid,
        HoleEvidence.RecessPoints => hole.Evidence is HoleEvidence.RecessPoints or HoleEvidence.RecessAndVoid,
        _ => hole.Evidence == expectedEvidence
    };
    if (!evidenceMatches)
    {
        Console.Error.WriteLine($"FAIL {name}: evidence {hole.Evidence}");
        return 1;
    }

    Console.WriteLine(
        $"PASS {name}: pixel=({hole.PixelX},{hole.PixelY}) Ø={hole.DiameterMm:F1}mm " +
        $"depth={(hole.RecessDepthMm?.ToString("F1") ?? "N/A")} evidence={hole.Evidence} " +
        $"XYZ={hole.SurfacePosition} confidence={hole.Confidence:P0}");
    return 0;
}

static LiveFrame BuildSyntheticFrame(SyntheticHole mode)
{
    const int width = 320, height = 240, pointWidth = 160, pointHeight = 120;
    const int centerX = 160, centerY = 120, radius = 15;
    const float millimetersPerPixel = 1.15f;
    var random = new Random(443);
    var bgra = new byte[width * height * 4];

    for (int y = 0; y < height; y++)
    for (int x = 0; x < width; x++)
    {
        float distance = MathF.Sqrt((x - centerX) * (x - centerX) + (y - centerY) * (y - centerY));
        int noise = random.Next(-3, 4);
        int gray = 150 + noise;
        if (mode != SyntheticHole.None)
        {
            if (distance <= radius - 1) gray = 32 + noise;
            else if (distance <= radius + 2) gray = 182 + noise;
        }
        int index = (y * width + x) * 4;
        bgra[index] = bgra[index + 1] = bgra[index + 2] = (byte)Math.Clamp(gray, 0, 255);
        bgra[index + 3] = 255;
    }

    var points = new float[pointWidth * pointHeight * 4];
    for (int gridY = 0; gridY < pointHeight; gridY++)
    for (int gridX = 0; gridX < pointWidth; gridX++)
    {
        int pixelX = (int)Math.Round((double)gridX * (width - 1) / (pointWidth - 1));
        int pixelY = (int)Math.Round((double)gridY * (height - 1) / (pointHeight - 1));
        float distance = MathF.Sqrt((pixelX - centerX) * (pixelX - centerX) + (pixelY - centerY) * (pixelY - centerY));
        int index = (gridY * pointWidth + gridX) * 4;

        if (mode == SyntheticHole.Void && distance <= radius - 1)
        {
            points[index] = points[index + 1] = points[index + 2] = float.NaN;
            continue;
        }

        float x = (pixelX - centerX) * millimetersPerPixel;
        float y = (pixelY - centerY) * millimetersPerPixel;
        float surfaceZ = mode == SyntheticHole.CloseDeepBackWall ? 230 : 900;
        float z = surfaceZ + .055f * (pixelX - centerX) - .028f * (pixelY - centerY) + (float)(random.NextDouble() - .5) * .7f;
        if (mode is SyntheticHole.DeepBackWall or SyntheticHole.CloseDeepBackWall && distance <= radius - 1) z += 34;
        if (mode == SyntheticHole.DistantBackground && distance <= radius - 1) z += 700;
        points[index] = x;
        points[index + 1] = y;
        points[index + 2] = z;
        points[index + 3] = 0;
    }

    return new(width, height, bgra, pointWidth, pointHeight, points, 1, 15);
}

static async Task<int> TestCameraAsync()
{
    await using var camera = new ZedCamera();
    camera.StatusChanged += Console.WriteLine;
    try
    {
        _ = sl.Camera.GetDeviceList(out int sdkDeviceCount);
        Console.WriteLine($"ZED SDK device count: {sdkDeviceCount}");
        using var timeout = new CancellationTokenSource(TimeSpan.FromSeconds(35));
        LiveFrame frame = await camera.StartAsync(timeout.Token);
        bool centerValid = frame.TryGetPoint(frame.Width / 2, frame.Height / 2, out Vector3 center);
        Console.WriteLine(
            $"CAMERA PASS: {frame.Width}x{frame.Height}, XYZ {frame.PointWidth}x{frame.PointHeight}, " +
            $"center={(centerValid ? center.ToString() : "invalid (scene may be out of range)")}");
        await camera.StopAsync();
        return 0;
    }
    catch (Exception exception)
    {
        Console.Error.WriteLine("CAMERA FAIL: " + exception);
        return 2;
    }
}

enum SyntheticHole
{
    None,
    Void,
    DeepBackWall,
    DistantBackground,
    DarkMark,
    CloseDeepBackWall
}

using System.Globalization;
using System.IO;
using System.Net.Http;
using System.Numerics;
using System.Text;
using System.Text.Json;
using System.Text.Json.Serialization;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using ZedAnchorHoleInspection.Models;

namespace ZedAnchorHoleInspection.Detection;

public sealed class RoboflowHoleDetector : IDisposable
{
    public const string ModelId = "hole-detection-fwa4p/2";

    readonly HttpClient client = new() { Timeout = TimeSpan.FromSeconds(20) };

    public async Task<DetectionResult> DetectAsync(
        LiveFrame frame,
        DetectorSettings rawSettings,
        string apiKey,
        CancellationToken cancellationToken = default)
    {
        var started = System.Diagnostics.Stopwatch.StartNew();
        DetectorSettings settings = rawSettings.Validate();
        if (string.IsNullOrWhiteSpace(apiKey))
            throw new InvalidOperationException("Roboflow API 키가 없습니다.");

        Crop crop = CropRoi(frame, settings.Roi);
        byte[] jpeg = EncodeJpeg(crop);
        string encoded = Convert.ToBase64String(jpeg);
        string confidence = (settings.RoboflowConfidence * 100).ToString("F0", CultureInfo.InvariantCulture);
        string endpoint =
            $"https://serverless.roboflow.com/{ModelId}" +
            $"?api_key={Uri.EscapeDataString(apiKey)}&confidence={confidence}&overlap=30&format=json";

        using var content = new StringContent(encoded, Encoding.ASCII, "application/x-www-form-urlencoded");
        using HttpResponseMessage response = await client.PostAsync(endpoint, content, cancellationToken).ConfigureAwait(false);
        if (!response.IsSuccessStatusCode)
            throw new InvalidOperationException($"Roboflow API 오류: {(int)response.StatusCode} {response.ReasonPhrase}");

        await using Stream stream = await response.Content.ReadAsStreamAsync(cancellationToken).ConfigureAwait(false);
        RoboflowResponse? payload = await JsonSerializer.DeserializeAsync<RoboflowResponse>(
            stream,
            cancellationToken: cancellationToken).ConfigureAwait(false);
        IReadOnlyList<RoboflowPrediction> predictions = payload?.Predictions ?? [];

        List<HoleDetection> holes = predictions
            .Where(prediction =>
                prediction.Class.Equals("hole", StringComparison.OrdinalIgnoreCase) &&
                prediction.Confidence >= settings.RoboflowConfidence &&
                prediction.Width > 0 && prediction.Height > 0)
            .Select(prediction =>
            {
                float radius = (prediction.Width + prediction.Height) * .25f;
                return new HoleDetection(
                    0,
                    crop.X + (int)Math.Round(prediction.X),
                    crop.Y + (int)Math.Round(prediction.Y),
                    radius,
                    Vector3.Zero,
                    0,
                    null,
                    0,
                    prediction.Confidence,
                    HoleEvidence.RoboflowAi,
                    0,
                    0);
            })
            .Where(hole =>
                hole.RadiusPixels >= settings.MinimumRadiusPixels &&
                hole.RadiusPixels <= settings.MaximumRadiusPixels)
            .OrderByDescending(hole => hole.Confidence)
            .Select((hole, index) => hole with { Id = index + 1 })
            .ToList();

        var diagnostics = new GeometryDiagnostics(predictions.Count, 0, 0, 0, 0, 0);
        return new(
            holes,
            predictions.Count,
            holes.Count,
            diagnostics,
            0,
            true,
            started.Elapsed);
    }

    static Crop CropRoi(LiveFrame frame, NormalizedRoi roi)
    {
        int x = Math.Clamp((int)Math.Floor(roi.X * frame.Width), 0, frame.Width - 1);
        int y = Math.Clamp((int)Math.Floor(roi.Y * frame.Height), 0, frame.Height - 1);
        int right = Math.Clamp((int)Math.Ceiling((roi.X + roi.Width) * frame.Width), x + 1, frame.Width);
        int bottom = Math.Clamp((int)Math.Ceiling((roi.Y + roi.Height) * frame.Height), y + 1, frame.Height);
        int width = right - x;
        int height = bottom - y;
        var bgra = new byte[width * height * 4];
        for (int row = 0; row < height; row++)
        {
            Buffer.BlockCopy(
                frame.Bgra,
                ((y + row) * frame.Width + x) * 4,
                bgra,
                row * width * 4,
                width * 4);
        }
        return new(x, y, width, height, bgra);
    }

    static byte[] EncodeJpeg(Crop crop)
    {
        BitmapSource bitmap = BitmapSource.Create(
            crop.Width,
            crop.Height,
            96,
            96,
            PixelFormats.Bgra32,
            null,
            crop.Bgra,
            crop.Width * 4);
        var encoder = new JpegBitmapEncoder { QualityLevel = 85 };
        encoder.Frames.Add(BitmapFrame.Create(bitmap));
        using var stream = new MemoryStream();
        encoder.Save(stream);
        return stream.ToArray();
    }

    public void Dispose() => client.Dispose();

    sealed record Crop(int X, int Y, int Width, int Height, byte[] Bgra);

    sealed record RoboflowResponse(
        [property: JsonPropertyName("predictions")] IReadOnlyList<RoboflowPrediction>? RawPredictions)
    {
        [JsonIgnore]
        public IReadOnlyList<RoboflowPrediction> Predictions => RawPredictions ?? [];
    }

    sealed record RoboflowPrediction(
        [property: JsonPropertyName("x")] float X,
        [property: JsonPropertyName("y")] float Y,
        [property: JsonPropertyName("width")] float Width,
        [property: JsonPropertyName("height")] float Height,
        [property: JsonPropertyName("confidence")] float Confidence,
        [property: JsonPropertyName("class")] string Class);
}

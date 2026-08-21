using Microsoft.Win32;
using System.Collections.ObjectModel;
using System.Globalization;
using System.IO;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Shapes;
using ZedAnchorHoleInspection.Camera;
using ZedAnchorHoleInspection.Detection;
using ZedAnchorHoleInspection.Models;

namespace ZedAnchorHoleInspection;

public partial class MainWindow : Window
{
    readonly ZedCamera camera = new();
    readonly StereoHoleDetector detector = new();
    readonly RoboflowHoleDetector roboflowDetector = new();
    readonly TemporalHoleTracker tracker = new();
    readonly ObservableCollection<HoleRow> rows = [];

    DetectorSettings settings = new();
    IReadOnlyList<HoleDetection> stableHoles = [];
    WriteableBitmap? bitmap;
    bool inspecting;
    bool drawingRoi;
    int detectionBusy;
    int uiFramePending;
    long lastDetectionTimestamp;
    string activeRoboflowApiKey = string.Empty;
    Point roiDragStart;
    NormalizedRoi roi = NormalizedRoi.Full;

    public MainWindow()
    {
        InitializeComponent();
        ResultGrid.ItemsSource = rows;
        camera.FrameReady += OnFrame;
        camera.StatusChanged += message => Dispatcher.InvokeAsync(() => FooterText.Text = message);
        Closed += async (_, _) =>
        {
            await camera.StopAsync();
            roboflowDetector.Dispose();
        };
    }

    async void Live_Click(object sender, RoutedEventArgs e)
    {
        LiveButton.IsEnabled = false;
        try
        {
            if (camera.IsRunning)
            {
                inspecting = false;
                InspectButton.IsChecked = false;
                await camera.StopAsync();
                LiveButton.Content = "▶ LIVE START";
                StateText.Text = "IDLE";
                return;
            }

            FooterText.Text = "ZED 2i 연결 중...";
            LiveFrame first = await camera.StartAsync();
            PrepareVideoSurface(first);
            EmptyLabel.Visibility = Visibility.Collapsed;
            LiveButton.Content = "■ LIVE STOP";
            StateText.Text = "LIVE";
        }
        catch (Exception exception)
        {
            string message = camera.TerminalError?.Message ?? exception.Message;
            FooterText.Text = "카메라 연결 실패 · " + message;
            MessageBox.Show(message, "ZED 2i 연결 실패", MessageBoxButton.OK, MessageBoxImage.Error);
        }
        finally
        {
            LiveButton.IsEnabled = true;
        }
    }

    void Inspect_Click(object sender, RoutedEventArgs e)
    {
        DetectorSettings parsed = settings;
        if (InspectButton.IsChecked == true && !camera.IsRunning)
        {
            InspectButton.IsChecked = false;
            MessageBox.Show("먼저 LIVE START를 눌러 ZED 2i에 연결하세요.");
            return;
        }

        if (InspectButton.IsChecked == true && !TryReadSettings(out parsed, out string error))
        {
            InspectButton.IsChecked = false;
            MessageBox.Show(error, "검출 설정 오류", MessageBoxButton.OK, MessageBoxImage.Warning);
            return;
        }

        if (InspectButton.IsChecked == true && !TryActivateRoboflow(parsed, out error))
        {
            InspectButton.IsChecked = false;
            MessageBox.Show(error, "Roboflow 설정 오류", MessageBoxButton.OK, MessageBoxImage.Warning);
            return;
        }

        if (InspectButton.IsChecked == true) settings = parsed;
        inspecting = InspectButton.IsChecked == true;
        tracker.Reset();
        stableHoles = [];
        rows.Clear();
        DetectionLayer.Children.Clear();
        InspectButton.Content = inspecting ? "■ INSPECTION STOP" : "◎ INSPECTION START";
        StateText.Text = inspecting ? "VERIFYING" : camera.IsRunning ? "LIVE" : "IDLE";
    }

    void ApplySettings_Click(object sender, RoutedEventArgs e)
    {
        if (!TryReadSettings(out DetectorSettings parsed, out string error))
        {
            MessageBox.Show(error, "검출 설정 오류", MessageBoxButton.OK, MessageBoxImage.Warning);
            return;
        }
        if (!TryActivateRoboflow(parsed, out error))
        {
            MessageBox.Show(error, "Roboflow 설정 오류", MessageBoxButton.OK, MessageBoxImage.Warning);
            return;
        }
        settings = parsed;
        tracker.Reset();
        FooterText.Text = "검출 설정 적용됨";
    }

    bool TryReadSettings(out DetectorSettings parsed, out string error)
    {
        parsed = settings;
        error = string.Empty;
        if (!TryInt(MinimumRadiusBox.Text, out int minimumRadius) ||
            !TryInt(MaximumRadiusBox.Text, out int maximumRadius) ||
            !TryFloat(ContrastBox.Text, out float contrast) ||
            !TryFloat(PlaneToleranceBox.Text, out float planeTolerance) ||
            !TryFloat(MinimumDepthBox.Text, out float minimumDepth) ||
            !TryFloat(MinimumDiameterBox.Text, out float minimumDiameter) ||
            !TryFloat(MaximumDiameterBox.Text, out float maximumDiameter) ||
            !TryFloat(MinimumDistanceBox.Text, out float minimumDistance) ||
            !TryFloat(InvalidRatioBox.Text, out float invalidRatio) ||
            !TryFloat(RoboflowConfidenceBox.Text, out float roboflowConfidence))
        {
            error = "모든 설정값을 숫자로 입력하세요.";
            return false;
        }

        try
        {
            parsed = new DetectorSettings
            {
                MinimumRadiusPixels = minimumRadius,
                MaximumRadiusPixels = maximumRadius,
                MinimumDarkContrast = contrast,
                PlaneToleranceMm = planeTolerance,
                MinimumRecessDepthMm = minimumDepth,
                MinimumDiameterMm = minimumDiameter,
                MaximumDiameterMm = maximumDiameter,
                MinimumSurfaceDistanceMm = minimumDistance,
                MinimumInvalidInteriorRatio = invalidRatio,
                AllowVoidOnlyEvidence = AllowVoidOnlyBox.IsChecked == true,
                NearRgbOnlyMode = NearRgbOnlyBox.IsChecked == true,
                UseRoboflowModel = UseRoboflowBox.IsChecked == true,
                RoboflowConfidence = roboflowConfidence,
                Roi = roi
            }.Validate();
            return true;
        }
        catch (ArgumentOutOfRangeException exception)
        {
            error = "설정 범위를 확인하세요: " + exception.ParamName;
            return false;
        }
    }

    bool TryActivateRoboflow(DetectorSettings parsed, out string error)
    {
        error = string.Empty;
        if (!parsed.UseRoboflowModel)
        {
            activeRoboflowApiKey = string.Empty;
            return true;
        }

        string key = RoboflowApiKeyBox.Password.Trim();
        if (string.IsNullOrWhiteSpace(key))
            key = Environment.GetEnvironmentVariable("ROBOFLOW_API_KEY")?.Trim() ?? string.Empty;
        if (string.IsNullOrWhiteSpace(key))
        {
            error = "Roboflow API Key를 입력하거나 ROBOFLOW_API_KEY 환경변수를 설정하세요. 키는 채팅에 보내지 마세요.";
            return false;
        }
        activeRoboflowApiKey = key;
        return true;
    }

    void OnFrame(LiveFrame frame)
    {
        if (Interlocked.CompareExchange(ref uiFramePending, 1, 0) == 0)
        {
            Dispatcher.InvokeAsync(() =>
            {
                try
                {
                    PrepareVideoSurface(frame);
                    bitmap!.WritePixels(
                        new Int32Rect(0, 0, frame.Width, frame.Height),
                        frame.Bgra,
                        frame.Width * 4,
                        0);
                }
                finally { Interlocked.Exchange(ref uiFramePending, 0); }
            });
        }

        long now = Environment.TickCount64;
        int detectionInterval = settings.UseRoboflowModel ? 900 : 300;
        if (!inspecting || now - lastDetectionTimestamp < detectionInterval ||
            Interlocked.CompareExchange(ref detectionBusy, 1, 0) != 0)
            return;

        lastDetectionTimestamp = now;
        DetectorSettings current = settings;
        string apiKey = activeRoboflowApiKey;
        _ = RunDetectionAsync(frame, current, apiKey);
    }

    async Task RunDetectionAsync(LiveFrame frame, DetectorSettings current, string apiKey)
    {
        try
        {
            DetectionResult result = current.UseRoboflowModel
                ? await Task.Run(() => roboflowDetector.DetectAsync(frame, current, apiKey))
                : await Task.Run(() => detector.Detect(frame, current));
            await Dispatcher.InvokeAsync(() =>
            {
                if (inspecting) RenderDetections(result, frame);
            });
        }
        catch (Exception exception)
        {
            await Dispatcher.InvokeAsync(() =>
                FooterText.Text = "검출 오류 · " + exception.GetBaseException().Message);
        }
        finally
        {
            Interlocked.Exchange(ref detectionBusy, 0);
        }
    }

    void PrepareVideoSurface(LiveFrame frame)
    {
        if (bitmap is not null && bitmap.PixelWidth == frame.Width && bitmap.PixelHeight == frame.Height) return;
        bitmap = new WriteableBitmap(frame.Width, frame.Height, 96, 96, PixelFormats.Bgra32, null);
        CameraImage.Source = bitmap;
        VideoSurface.Width = frame.Width;
        VideoSurface.Height = frame.Height;
        LayoutRoi();
    }

    void RenderDetections(DetectionResult result, LiveFrame frame)
    {
        stableHoles = tracker.Update(result.Holes);
        DetectionLayer.Children.Clear();
        rows.Clear();

        foreach (HoleDetection hole in stableHoles)
        {
            double diameter = Math.Max(18, hole.RadiusPixels * 2.4);
            var circle = new Ellipse
            {
                Width = diameter,
                Height = diameter,
                Stroke = Brushes.Lime,
                StrokeThickness = 3,
                Fill = new SolidColorBrush(Color.FromArgb(25, 50, 214, 160))
            };
            Canvas.SetLeft(circle, hole.PixelX - diameter / 2);
            Canvas.SetTop(circle, hole.PixelY - diameter / 2);
            DetectionLayer.Children.Add(circle);

            bool rgbOnly = IsTwoDimensional(hole.Evidence);
            string size = rgbOnly ? $"{hole.RadiusPixels * 2:F0}px" : $"{hole.DiameterMm:F1}mm";
            string depth = hole.RecessDepthMm is float value ? $"{value:F1} mm" : "N/A";
            var label = new Border
            {
                Background = new SolidColorBrush(Color.FromArgb(220, 5, 10, 14)),
                Padding = new Thickness(6, 3, 6, 3),
                Child = new TextBlock
                {
                    Text = $"#{hole.Id}  Ø{size}  D:{depth}",
                    Foreground = Brushes.White,
                    FontSize = 12
                }
            };
            Canvas.SetLeft(label, hole.PixelX + diameter / 2);
            Canvas.SetTop(label, hole.PixelY - diameter / 2);
            DetectionLayer.Children.Add(label);

            rows.Add(new(
                hole.Id,
                $"{hole.Confidence:P0}",
                rgbOnly ? $"{hole.RadiusPixels * 2:F0}px" : $"{hole.DiameterMm:F1}",
                hole.RecessDepthMm is float measured ? $"{measured:F1}" : "N/A",
                rgbOnly ? "N/A" : $"{hole.SurfacePosition.X:F1}",
                rgbOnly ? "N/A" : $"{hole.SurfacePosition.Y:F1}",
                rgbOnly ? "N/A" : $"{hole.SurfacePosition.Z:F1}"));
        }

        StateText.Text = stableHoles.Count > 0 ? $"FOUND {stableHoles.Count}" : "VERIFYING";
        string evidence = stableHoles.Count == 0
            ? GeometryHint(result)
            : string.Join(" · ", stableHoles.Select(hole => $"#{hole.Id} {EvidenceLabel(hole.Evidence)}, void {hole.InvalidInteriorRatio:P0}"));
        EvidenceText.Text = evidence;
        GeometryDiagnostics diagnostics = result.Diagnostics;
        FooterText.Text = settings.UseRoboflowModel
            ? $"Roboflow {RoboflowHoleDetector.ModelId} · 응답 {result.AppearanceCandidates} · 통과 {result.GeometryCandidates} · " +
              $"stable {stableHoles.Count} · {result.Elapsed.TotalMilliseconds:F0} ms · 약 1 req/s"
            : result.IsRgbOnly
            ? $"근접 RGB · 원 후보 {result.AppearanceCandidates} · 강한 원형 {result.GeometryCandidates} · " +
              $"stable {stableHoles.Count} · XYZ 미사용 · {result.Elapsed.TotalMilliseconds:F0} ms · display {frame.FramesPerSecond:F1} FPS"
            : $"3D · RGB {result.AppearanceCandidates} · 검사 {diagnostics.EvaluatedCandidates} · 림 {diagnostics.RimCandidates} · " +
              $"평면 {diagnostics.PlaneCandidates} · 거리 {diagnostics.DistanceCandidates} · 지름 {diagnostics.DiameterCandidates} · " +
              $"홀증거 {result.GeometryCandidates} · XYZ 유효 {result.ValidDepthRatio:P0} · " +
              $"stable {stableHoles.Count} · {result.Elapsed.TotalMilliseconds:F0} ms · display {frame.FramesPerSecond:F1} FPS";
    }

    void Video_MouseDown(object sender, MouseButtonEventArgs e)
    {
        drawingRoi = true;
        roiDragStart = e.GetPosition(VideoSurface);
        VideoSurface.CaptureMouse();
    }

    void Video_MouseMove(object sender, MouseEventArgs e)
    {
        if (drawingRoi) DrawRoi(roiDragStart, e.GetPosition(VideoSurface), false);
    }

    void Video_MouseUp(object sender, MouseButtonEventArgs e)
    {
        if (!drawingRoi) return;
        drawingRoi = false;
        VideoSurface.ReleaseMouseCapture();
        DrawRoi(roiDragStart, e.GetPosition(VideoSurface), true);
    }

    void DrawRoi(Point first, Point second, bool commit)
    {
        double surfaceWidth = Math.Max(1, VideoSurface.Width);
        double surfaceHeight = Math.Max(1, VideoSurface.Height);
        double x = Math.Clamp(Math.Min(first.X, second.X), 0, surfaceWidth);
        double y = Math.Clamp(Math.Min(first.Y, second.Y), 0, surfaceHeight);
        double width = Math.Clamp(Math.Abs(first.X - second.X), 0, surfaceWidth - x);
        double height = Math.Clamp(Math.Abs(first.Y - second.Y), 0, surfaceHeight - y);
        Canvas.SetLeft(RoiRectangle, x);
        Canvas.SetTop(RoiRectangle, y);
        RoiRectangle.Width = width;
        RoiRectangle.Height = height;
        if (!commit || width < 20 || height < 20) return;

        roi = new((float)(x / surfaceWidth), (float)(y / surfaceHeight),
                  (float)(width / surfaceWidth), (float)(height / surfaceHeight));
        settings = settings with { Roi = roi };
        tracker.Reset();
    }

    void ResetRoi_Click(object sender, RoutedEventArgs e)
    {
        roi = NormalizedRoi.Full;
        settings = settings with { Roi = roi };
        tracker.Reset();
        LayoutRoi();
    }

    void LayoutRoi()
    {
        Canvas.SetLeft(RoiRectangle, roi.X * VideoSurface.Width);
        Canvas.SetTop(RoiRectangle, roi.Y * VideoSurface.Height);
        RoiRectangle.Width = roi.Width * VideoSurface.Width;
        RoiRectangle.Height = roi.Height * VideoSurface.Height;
    }

    void ExportCsv_Click(object sender, RoutedEventArgs e)
    {
        if (stableHoles.Count == 0)
        {
            MessageBox.Show("저장할 안정 검출 결과가 없습니다.");
            return;
        }

        var dialog = new SaveFileDialog
        {
            Filter = "CSV files (*.csv)|*.csv",
            FileName = $"zed_anchor_holes_{DateTime.Now:yyyyMMdd_HHmmss}.csv"
        };
        if (dialog.ShowDialog(this) != true) return;

        var csv = new StringBuilder("id,pixel_x,pixel_y,confidence,diameter_mm,recess_depth_mm,surface_x_mm,surface_y_mm,surface_z_mm,evidence,invalid_ratio\r\n");
        foreach (HoleDetection hole in stableHoles)
        {
            csv.Append(hole.Id).Append(',').Append(hole.PixelX).Append(',').Append(hole.PixelY).Append(',')
                .Append(hole.Confidence.ToString("F4", CultureInfo.InvariantCulture)).Append(',')
                .Append(IsTwoDimensional(hole.Evidence)
                    ? string.Empty
                    : hole.DiameterMm.ToString("F3", CultureInfo.InvariantCulture)).Append(',')
                .Append(hole.RecessDepthMm?.ToString("F3", CultureInfo.InvariantCulture) ?? string.Empty).Append(',')
                .Append(IsTwoDimensional(hole.Evidence) ? string.Empty : hole.SurfacePosition.X.ToString("F3", CultureInfo.InvariantCulture)).Append(',')
                .Append(IsTwoDimensional(hole.Evidence) ? string.Empty : hole.SurfacePosition.Y.ToString("F3", CultureInfo.InvariantCulture)).Append(',')
                .Append(IsTwoDimensional(hole.Evidence) ? string.Empty : hole.SurfacePosition.Z.ToString("F3", CultureInfo.InvariantCulture)).Append(',')
                .Append(hole.Evidence).Append(',')
                .Append(hole.InvalidInteriorRatio.ToString("F4", CultureInfo.InvariantCulture)).Append("\r\n");
        }
        File.WriteAllText(dialog.FileName, csv.ToString(), new UTF8Encoding(true));
        FooterText.Text = "CSV 저장 완료 · " + dialog.FileName;
    }

    static bool TryInt(string text, out int value) =>
        int.TryParse(text, NumberStyles.Integer, CultureInfo.CurrentCulture, out value) ||
        int.TryParse(text, NumberStyles.Integer, CultureInfo.InvariantCulture, out value);

    static bool TryFloat(string text, out float value) =>
        float.TryParse(text, NumberStyles.Float, CultureInfo.CurrentCulture, out value) ||
        float.TryParse(text, NumberStyles.Float, CultureInfo.InvariantCulture, out value);

    static string EvidenceLabel(HoleEvidence evidence) => evidence switch
    {
        HoleEvidence.RecessPoints => "깊은 XYZ",
        HoleEvidence.StereoVoid => "스테레오 무효 내부",
        HoleEvidence.DistantBackground => "관통홀/먼 배경",
        HoleEvidence.RgbOnly => "근접 RGB 중심",
        HoleEvidence.RoboflowAi => "Roboflow AI",
        _ => "깊은 XYZ+무효 내부"
    };

    string GeometryHint(DetectionResult result)
    {
        GeometryDiagnostics diagnostics = result.Diagnostics;
        if (settings.UseRoboflowModel)
            return result.AppearanceCandidates == 0
                ? "Roboflow AI 응답에 hole 검출이 없습니다. ROI·신뢰도·모델 도메인을 확인하세요."
                : "Roboflow 검출은 있지만 픽셀 반지름 범위를 벗어났습니다.";
        if (result.IsRgbOnly)
            return result.AppearanceCandidates == 0
                ? "근접 RGB 모드: 어두운 원형 후보가 없습니다. 픽셀 반지름과 ROI를 확인하세요."
                : "근접 RGB 모드: 후보는 있지만 강한 원형도 조건을 통과하지 못했습니다.";
        if (result.ValidDepthRatio < .10f)
            return "ROI의 XYZ가 거의 없습니다. 거리·조명·ZED Depth Viewer를 확인하세요.";
        if (result.AppearanceCandidates == 0)
            return "어두운 원형 후보가 없습니다. 홀 크기 범위와 암부 대비를 확인하세요.";
        if (diagnostics.RimCandidates == 0)
            return "원 후보 주변의 유효 XYZ가 부족합니다. ROI를 넓히거나 대상 거리를 늘리세요.";
        if (diagnostics.PlaneCandidates == 0)
            return "주변 XYZ는 있으나 원 둘레의 평면이 불안정합니다. 카메라·대상을 고정하세요.";
        if (diagnostics.DistanceCandidates == 0)
            return "평면은 찾았지만 최소 측정 거리보다 가깝습니다.";
        if (diagnostics.DiameterCandidates == 0)
            return "거리 조건은 통과했지만 계산된 실제 홀 지름 범위를 벗어났습니다.";
        return "원 후보와 주변 평면은 확인됐지만 내부 깊이 차가 없습니다. 검은 무늬라면 정상적으로 제외됩니다.";
    }

    static bool IsTwoDimensional(HoleEvidence evidence) =>
        evidence is HoleEvidence.RgbOnly or HoleEvidence.RoboflowAi;
}

public sealed record HoleRow(int Id, string Confidence, string Diameter, string Depth, string X, string Y, string Z);

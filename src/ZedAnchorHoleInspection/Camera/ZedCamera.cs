using System.Diagnostics;
using System.Runtime.InteropServices;
using ZedAnchorHoleInspection.Models;

namespace ZedAnchorHoleInspection.Camera;

public sealed class ZedCamera : IAsyncDisposable
{
    readonly object gate = new();
    CancellationTokenSource? cancellation;
    Task? worker;
    TaskCompletionSource<LiveFrame>? firstFrame;
    Exception? terminalError;

    public event Action<LiveFrame>? FrameReady;
    public event Action<string>? StatusChanged;

    public bool IsRunning => worker is { IsCompleted: false };
    public Exception? TerminalError => terminalError;

    public async Task<LiveFrame> StartAsync(CancellationToken cancellationToken = default)
    {
        lock (gate)
        {
            if (IsRunning && firstFrame is not null)
                return firstFrame.Task.GetAwaiter().GetResult();

            cancellation?.Dispose();
            terminalError = null;
            cancellation = CancellationTokenSource.CreateLinkedTokenSource(cancellationToken);
            firstFrame = new(TaskCreationOptions.RunContinuationsAsynchronously);
            worker = Task.Run(() => Acquire(cancellation.Token), CancellationToken.None);
        }

        try
        {
            return await firstFrame.Task.WaitAsync(TimeSpan.FromSeconds(30), cancellationToken);
        }
        catch
        {
            await StopAsync();
            throw;
        }
    }

    public async Task StopAsync()
    {
        CancellationTokenSource? source;
        Task? running;
        lock (gate)
        {
            source = cancellation;
            running = worker;
            cancellation = null;
            worker = null;
        }

        if (source is null) return;
        source.Cancel();
        if (running is not null)
        {
            try { await running; }
            catch (OperationCanceledException) { }
        }
        source.Dispose();
    }

    void Acquire(CancellationToken token)
    {
        sl.Camera? camera = null;
        sl.Mat? leftImage = null;
        sl.Mat? pointCloud = null;
        bool opened = false;

        try
        {
            StatusChanged?.Invoke("ZED 2i 연결 중...");
            _ = sl.Camera.GetDeviceList(out int sdkDeviceCount);
            if (sdkDeviceCount == 0)
            {
                throw new InvalidOperationException(
                    "ZED SDK device list is empty. Windows may still show the ZED 2i as a UVC camera. " +
                    "Close ZED tools, unplug/replug the camera directly on a USB 3.x port, and run ZED Diagnostic again.");
            }

            var initialization = new sl.InitParameters
            {
                resolution = sl.RESOLUTION.HD720,
                cameraFPS = 30,
                // ULTRA favors edge sharpness. That is important here because a neural
                // depth mode can smooth or fill the depth discontinuity across a dark hole.
                depthMode = sl.DEPTH_MODE.ULTRA,
                coordinateUnits = sl.UNIT.MILLIMETER,
                coordinateSystem = sl.COORDINATE_SYSTEM.IMAGE,
                // ZED 2i (2.1 mm optics) is specified for depth from about 0.3 m.
                // Nearer scenes remain available in RGB, but their XYZ is not reliable.
                depthMinimumDistance = 300,
                depthMaximumDistance = 4000
            };

            camera = new sl.Camera(0);
            sl.ERROR_CODE openResult = camera.Open(ref initialization);
            if (openResult != sl.ERROR_CODE.SUCCESS)
                throw new InvalidOperationException($"ZED SDK camera open failed: {openResult}");
            opened = true;

            int width = camera.ImageWidth;
            int height = camera.ImageHeight;
            // Keep the XYZ map at image resolution. Half-resolution XYZ discarded too
            // many samples around small holes at the far end of the usable range.
            int pointWidth = width;
            int pointHeight = height;
            var imageResolution = new sl.Resolution(width, height);
            var pointResolution = new sl.Resolution(pointWidth, pointHeight);

            leftImage = new sl.Mat();
            leftImage.Create(imageResolution, sl.MAT_TYPE.MAT_8U_C4, sl.MEM.CPU);
            pointCloud = new sl.Mat();
            pointCloud.Create(pointResolution, sl.MAT_TYPE.MAT_32F_C4, sl.MEM.CPU);

            var runtime = new sl.RuntimeParameters
            {
                confidenceThreshold = 70,
                textureConfidenceThreshold = 100
            };

            StatusChanged?.Invoke($"ZED 2i 연결됨 · S/N {camera.GetZEDSerialNumber()} · {width}×{height} @ 30 FPS");

            long frameId = 0;
            int fpsFrames = 0;
            double fps = 0;
            var fpsTimer = Stopwatch.StartNew();

            while (!token.IsCancellationRequested)
            {
                sl.ERROR_CODE grabResult = camera.Grab(ref runtime);
                if (grabResult != sl.ERROR_CODE.SUCCESS)
                {
                    if (token.IsCancellationRequested) break;
                    Thread.Sleep(2);
                    continue;
                }

                // The UI/detector is intentionally published at about 15 FPS to avoid
                // copying a full RGB image and a half-resolution XYZ cloud on every grab.
                if ((++frameId & 1) == 0) continue;

                EnsureSuccess(camera.RetrieveImage(leftImage, sl.VIEW.LEFT, sl.MEM.CPU, imageResolution), "RetrieveImage");
                EnsureSuccess(camera.RetrieveMeasure(pointCloud, sl.MEASURE.XYZRGBA, sl.MEM.CPU, pointResolution), "RetrieveMeasure(XYZRGBA)");

                var bgra = new byte[width * height * 4];
                CopyRows(leftImage.GetPtr(sl.MEM.CPU), leftImage.GetStepBytes(sl.MEM.CPU), bgra, width * 4, height);

                var xyzRgba = new float[pointWidth * pointHeight * 4];
                CopyFloatRows(pointCloud.GetPtr(sl.MEM.CPU), pointCloud.GetStepBytes(sl.MEM.CPU), xyzRgba, pointWidth * 4, pointHeight);

                fpsFrames++;
                if (fpsTimer.ElapsedMilliseconds >= 1000)
                {
                    fps = fpsFrames / fpsTimer.Elapsed.TotalSeconds;
                    fpsFrames = 0;
                    fpsTimer.Restart();
                }

                var frame = new LiveFrame(width, height, bgra, pointWidth, pointHeight, xyzRgba, frameId, fps);
                firstFrame?.TrySetResult(frame);
                FrameReady?.Invoke(frame);
            }
        }
        catch (Exception exception)
        {
            terminalError = exception;
            firstFrame?.TrySetException(exception);
            StatusChanged?.Invoke("카메라 오류 · " + exception.Message);
        }
        finally
        {
            try { pointCloud?.Free(); } catch { }
            try { leftImage?.Free(); } catch { }
            if (camera is not null && opened)
            {
                try { camera.Close(); } catch { }
            }
            StatusChanged?.Invoke("카메라 정지");
        }
    }

    static void EnsureSuccess(sl.ERROR_CODE result, string operation)
    {
        if (result != sl.ERROR_CODE.SUCCESS)
            throw new InvalidOperationException($"{operation} failed: {result}");
    }

    static void CopyRows(IntPtr source, int sourceStrideBytes, byte[] destination, int destinationStrideBytes, int rows)
    {
        for (int row = 0; row < rows; row++)
            Marshal.Copy(IntPtr.Add(source, row * sourceStrideBytes), destination, row * destinationStrideBytes, destinationStrideBytes);
    }

    static void CopyFloatRows(IntPtr source, int sourceStrideBytes, float[] destination, int floatsPerRow, int rows)
    {
        for (int row = 0; row < rows; row++)
            Marshal.Copy(IntPtr.Add(source, row * sourceStrideBytes), destination, row * floatsPerRow, floatsPerRow);
    }

    public async ValueTask DisposeAsync() => await StopAsync();
}

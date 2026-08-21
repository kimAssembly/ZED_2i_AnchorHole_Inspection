using System.Numerics;

namespace ZedAnchorHoleInspection.Models;

public sealed record LiveFrame(
    int Width,
    int Height,
    byte[] Bgra,
    int PointWidth,
    int PointHeight,
    float[] XyzRgba,
    long FrameId,
    double FramesPerSecond)
{
    public bool TryGetPoint(int pixelX, int pixelY, out Vector3 point)
    {
        if (PointWidth < 1 || PointHeight < 1 || XyzRgba.Length < PointWidth * PointHeight * 4)
        {
            point = default;
            return false;
        }

        int gridX = Width <= 1
            ? 0
            : Math.Clamp((int)Math.Round((double)pixelX * (PointWidth - 1) / (Width - 1)), 0, PointWidth - 1);
        int gridY = Height <= 1
            ? 0
            : Math.Clamp((int)Math.Round((double)pixelY * (PointHeight - 1) / (Height - 1)), 0, PointHeight - 1);
        int index = (gridY * PointWidth + gridX) * 4;
        point = new(XyzRgba[index], XyzRgba[index + 1], XyzRgba[index + 2]);
        return IsValid(point);
    }

    public (int PixelX, int PixelY) PointPixel(int gridX, int gridY)
    {
        int pixelX = PointWidth <= 1 ? 0 : (int)Math.Round((double)gridX * (Width - 1) / (PointWidth - 1));
        int pixelY = PointHeight <= 1 ? 0 : (int)Math.Round((double)gridY * (Height - 1) / (PointHeight - 1));
        return (pixelX, pixelY);
    }

    public bool TryGetGridPoint(int gridX, int gridY, out Vector3 point)
    {
        if ((uint)gridX >= PointWidth || (uint)gridY >= PointHeight)
        {
            point = default;
            return false;
        }

        int index = (gridY * PointWidth + gridX) * 4;
        point = new(XyzRgba[index], XyzRgba[index + 1], XyzRgba[index + 2]);
        return IsValid(point);
    }

    public static bool IsValid(Vector3 point) =>
        float.IsFinite(point.X) && float.IsFinite(point.Y) && float.IsFinite(point.Z) && point.Z > 0;
}


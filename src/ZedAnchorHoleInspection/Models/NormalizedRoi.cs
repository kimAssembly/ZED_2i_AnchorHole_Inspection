namespace ZedAnchorHoleInspection.Models;

public readonly record struct NormalizedRoi(float X, float Y, float Width, float Height)
{
    public static NormalizedRoi Full => new(0, 0, 1, 1);

    public NormalizedRoi Clamp()
    {
        float x = Math.Clamp(X, 0, 1);
        float y = Math.Clamp(Y, 0, 1);
        return new(
            x,
            y,
            Math.Clamp(Width, 0, 1 - x),
            Math.Clamp(Height, 0, 1 - y));
    }
}


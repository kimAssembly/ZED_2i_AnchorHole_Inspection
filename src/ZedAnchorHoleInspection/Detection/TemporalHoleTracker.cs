using System.Numerics;

namespace ZedAnchorHoleInspection.Detection;

public sealed class TemporalHoleTracker
{
    const int RequiredHits = 5;
    const int MaximumMisses = 3;
    const float PixelGate = 48;
    const float PositionGateMm = 90;
    const float Alpha = .28f;

    readonly List<Track> tracks = [];
    int nextId = 1;

    public IReadOnlyList<HoleDetection> Update(IReadOnlyList<HoleDetection> detections)
    {
        foreach (Track track in tracks) track.Matched = false;

        foreach (HoleDetection detection in detections.OrderByDescending(item => item.Confidence))
        {
            Track? match = tracks
                .Where(track => !track.Matched &&
                                PixelDistance(track, detection) <= PixelGate &&
                                PositionMatches(track, detection))
                .OrderBy(track => PixelDistance(track, detection))
                .FirstOrDefault();

            if (match is null)
            {
                tracks.Add(new(nextId++, detection));
                continue;
            }

            match.PixelX = Lerp(match.PixelX, detection.PixelX, Alpha);
            match.PixelY = Lerp(match.PixelY, detection.PixelY, Alpha);
            match.RadiusPixels = Lerp(match.RadiusPixels, detection.RadiusPixels, Alpha);
            match.SurfacePosition = Vector3.Lerp(match.SurfacePosition, detection.SurfacePosition, Alpha);
            match.DiameterMm = Lerp(match.DiameterMm, detection.DiameterMm, Alpha);
            match.RecessDepthMm = detection.RecessDepthMm is float depth
                ? match.RecessDepthMm is float prior ? Lerp(prior, depth, Alpha) : depth
                : null;
            match.InvalidInteriorRatio = Lerp(match.InvalidInteriorRatio, detection.InvalidInteriorRatio, Alpha);
            match.Confidence = match.Confidence * (1 - Alpha) + detection.Confidence * Alpha;
            match.Evidence = detection.Evidence;
            match.PlaneRmseMm = Lerp(match.PlaneRmseMm, detection.PlaneRmseMm, Alpha);
            match.PlanePoints = detection.PlanePoints;
            match.Hits++;
            match.Misses = 0;
            match.Matched = true;
        }

        foreach (Track track in tracks.Where(item => !item.Matched)) track.Misses++;
        tracks.RemoveAll(track => track.Misses > MaximumMisses);

        return tracks
            .Where(track => track.Hits >= RequiredHits && track.Misses == 0)
            .OrderBy(track => track.Id)
            .Select(track => track.ToDetection())
            .ToArray();
    }

    public void Reset()
    {
        tracks.Clear();
        nextId = 1;
    }

    static float PixelDistance(Track track, HoleDetection detection)
    {
        float dx = track.PixelX - detection.PixelX;
        float dy = track.PixelY - detection.PixelY;
        return MathF.Sqrt(dx * dx + dy * dy);
    }

    static bool PositionMatches(Track track, HoleDetection detection) =>
        IsTwoDimensional(track.Evidence) && IsTwoDimensional(detection.Evidence) ||
        Vector3.Distance(track.SurfacePosition, detection.SurfacePosition) <= PositionGateMm;

    static bool IsTwoDimensional(HoleEvidence evidence) =>
        evidence is HoleEvidence.RgbOnly or HoleEvidence.RoboflowAi;

    static float Lerp(float from, float to, float amount) => from + (to - from) * amount;

    sealed class Track
    {
        public Track(int id, HoleDetection detection)
        {
            Id = id;
            PixelX = detection.PixelX;
            PixelY = detection.PixelY;
            RadiusPixels = detection.RadiusPixels;
            SurfacePosition = detection.SurfacePosition;
            DiameterMm = detection.DiameterMm;
            RecessDepthMm = detection.RecessDepthMm;
            InvalidInteriorRatio = detection.InvalidInteriorRatio;
            Confidence = detection.Confidence;
            Evidence = detection.Evidence;
            PlaneRmseMm = detection.PlaneRmseMm;
            PlanePoints = detection.PlanePoints;
            Hits = 1;
            Matched = true;
        }

        public int Id { get; }
        public float PixelX { get; set; }
        public float PixelY { get; set; }
        public float RadiusPixels { get; set; }
        public Vector3 SurfacePosition { get; set; }
        public float DiameterMm { get; set; }
        public float? RecessDepthMm { get; set; }
        public float InvalidInteriorRatio { get; set; }
        public double Confidence { get; set; }
        public HoleEvidence Evidence { get; set; }
        public float PlaneRmseMm { get; set; }
        public int PlanePoints { get; set; }
        public int Hits { get; set; }
        public int Misses { get; set; }
        public bool Matched { get; set; }

        public HoleDetection ToDetection() => new(
            Id,
            (int)Math.Round(PixelX),
            (int)Math.Round(PixelY),
            RadiusPixels,
            SurfacePosition,
            DiameterMm,
            RecessDepthMm,
            InvalidInteriorRatio,
            Confidence,
            Evidence,
            PlaneRmseMm,
            PlanePoints);
    }
}

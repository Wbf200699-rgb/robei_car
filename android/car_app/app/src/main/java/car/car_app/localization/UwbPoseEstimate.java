package car.car_app.localization;

/** Rigid-body fit result for the four engineering-only UWB tags. */
public final class UwbPoseEstimate {
    public enum Quality { FULL, STALE, INCOMPLETE, NON_RIGID }

    private final Quality quality;
    private final Pose2d pose;
    private final double rmsResidualM;
    private final double maximumResidualM;
    private final long fixSkewMs;

    public UwbPoseEstimate(Quality quality, Pose2d pose, double rmsResidualM,
                           double maximumResidualM, long fixSkewMs) {
        this.quality = quality;
        this.pose = pose;
        this.rmsResidualM = rmsResidualM;
        this.maximumResidualM = maximumResidualM;
        this.fixSkewMs = fixSkewMs;
    }

    public Quality getQuality() { return quality; }
    public Pose2d getPose() { return pose; }
    public double getRmsResidualM() { return rmsResidualM; }
    public double getMaximumResidualM() { return maximumResidualM; }
    public long getFixSkewMs() { return fixSkewMs; }
    public boolean isUsable() { return quality == Quality.FULL; }
}

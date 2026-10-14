package car.car_app.mapping;

import car.car_app.localization.Pose2d;

/** Pose refinement result from matching a lidar scan to the existing grid. */
public final class ScanMatchResult {
    private final Pose2d pose;
    private final int score;
    private final int scoreGain;
    private final boolean accepted;

    public ScanMatchResult(Pose2d pose, int score, int scoreGain, boolean accepted) {
        this.pose = pose;
        this.score = score;
        this.scoreGain = scoreGain;
        this.accepted = accepted;
    }

    public Pose2d getPose() { return pose; }
    public int getScore() { return score; }
    public int getScoreGain() { return scoreGain; }
    public boolean isAccepted() { return accepted; }
}

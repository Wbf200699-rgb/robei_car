package car.car_app.calibration;

import car.car_app.localization.Pose2d;

/** One measured command interval used by the offline calibration solve. */
public final class MotionCalibrationSample {
    public enum Command { FORWARD, REVERSE, TURN_LEFT, TURN_RIGHT }

    private final Command command;
    private final long durationMs;
    private final Pose2d start;
    private final Pose2d end;

    public MotionCalibrationSample(Command command, long durationMs, Pose2d start, Pose2d end) {
        if (durationMs <= 0) throw new IllegalArgumentException("durationMs must be positive");
        this.command = command;
        this.durationMs = durationMs;
        this.start = start;
        this.end = end;
    }

    public Command getCommand() { return command; }
    public long getDurationMs() { return durationMs; }
    public Pose2d getStart() { return start; }
    public Pose2d getEnd() { return end; }
}

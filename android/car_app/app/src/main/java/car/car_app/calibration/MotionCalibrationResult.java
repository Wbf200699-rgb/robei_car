package car.car_app.calibration;

/** Robust aggregate of repeated UWB-observed motion experiments. */
public final class MotionCalibrationResult {
    private final double forwardMmPerMs;
    private final double reverseMmPerMs;
    private final double leftDegPerMs;
    private final double rightDegPerMs;
    private final double forwardYawDriftDegPerM;
    private final int acceptedSamples;
    private final int rejectedSamples;

    public MotionCalibrationResult(double forwardMmPerMs, double reverseMmPerMs,
                                   double leftDegPerMs, double rightDegPerMs,
                                   double forwardYawDriftDegPerM,
                                   int acceptedSamples, int rejectedSamples) {
        this.forwardMmPerMs = forwardMmPerMs;
        this.reverseMmPerMs = reverseMmPerMs;
        this.leftDegPerMs = leftDegPerMs;
        this.rightDegPerMs = rightDegPerMs;
        this.forwardYawDriftDegPerM = forwardYawDriftDegPerM;
        this.acceptedSamples = acceptedSamples;
        this.rejectedSamples = rejectedSamples;
    }

    public double getForwardMmPerMs() { return forwardMmPerMs; }
    public double getReverseMmPerMs() { return reverseMmPerMs; }
    public double getLeftDegPerMs() { return leftDegPerMs; }
    public double getRightDegPerMs() { return rightDegPerMs; }
    public double getForwardYawDriftDegPerM() { return forwardYawDriftDegPerM; }
    public int getAcceptedSamples() { return acceptedSamples; }
    public int getRejectedSamples() { return rejectedSamples; }
}

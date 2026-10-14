package car.car_app.calibration;

public final class MotionParameterEstimator {
    private static final int STEP_DURATION_MS = 125;
    private static final int MAX_STEP_DURATION_MS = 60000;

    private MotionParameterEstimator() {
    }

    public static double estimateSpeedMmPerSecond(
            double calibratedMmPerMs,
            int currentSpeed,
            int calibrationSpeed) {
        if (!isUsable(calibratedMmPerMs) || currentSpeed <= 0 || calibrationSpeed <= 0) {
            return Double.NaN;
        }
        return calibratedMmPerMs * 1000.0 * currentSpeed / calibrationSpeed;
    }

    public static double estimateStepDistanceMm(
            double calibratedMmPerMs,
            int currentSpeed,
            int calibrationSpeed,
            int stepLength) {
        double speedMmPerMs = estimateSpeedMmPerSecond(
                calibratedMmPerMs, currentSpeed, calibrationSpeed) / 1000.0;
        if (!isUsable(speedMmPerMs) || stepLength <= 0) {
            return Double.NaN;
        }
        int durationMs = Math.min(stepLength * STEP_DURATION_MS, MAX_STEP_DURATION_MS);
        return speedMmPerMs * durationMs;
    }

    private static boolean isUsable(double value) {
        return Double.isFinite(value) && value > 0.0;
    }
}

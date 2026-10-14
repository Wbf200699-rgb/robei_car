package car.car_app.sensors;

import car.car_app.bluetooth.MotionSensorFrame;

public final class ImuAttitudeEstimator {
    private static final double COMPLEMENTARY_ALPHA = 0.96;
    private static final double SAMPLE_RATE_HZ = 200.0;

    private boolean initialized;
    private int lastSequence;
    private long lastTimestampMs;
    private double referenceRollDeg;
    private double referencePitchDeg;
    private double rollDeg;
    private double pitchDeg;
    private double yawDeg;

    public void reset() {
        initialized = false;
        lastSequence = 0;
        lastTimestampMs = 0L;
        referenceRollDeg = 0.0;
        referencePitchDeg = 0.0;
        rollDeg = 0.0;
        pitchDeg = 0.0;
        yawDeg = 0.0;
    }

    public void update(MotionSensorFrame.ImuSample sample, int sequence, long timestampMs) {
        double absoluteRoll = Math.toDegrees(Math.atan2(
                sample.getAccelYG(), sample.getAccelZG()));
        double absolutePitch = Math.toDegrees(Math.atan2(
                -sample.getAccelXG(),
                Math.hypot(sample.getAccelYG(), sample.getAccelZG())));

        if (!initialized) {
            referenceRollDeg = absoluteRoll;
            referencePitchDeg = absolutePitch;
            lastSequence = sequence;
            lastTimestampMs = timestampMs;
            initialized = true;
            return;
        }

        double deltaSeconds = calculateDeltaSeconds(sequence, timestampMs);
        double accelRoll = normalizeAngle(absoluteRoll - referenceRollDeg);
        double accelPitch = normalizeAngle(absolutePitch - referencePitchDeg);
        rollDeg = COMPLEMENTARY_ALPHA
                * (rollDeg + sample.getGyroXDps() * deltaSeconds)
                + (1.0 - COMPLEMENTARY_ALPHA) * accelRoll;
        pitchDeg = COMPLEMENTARY_ALPHA
                * (pitchDeg + sample.getGyroYDps() * deltaSeconds)
                + (1.0 - COMPLEMENTARY_ALPHA) * accelPitch;
        yawDeg = normalizeAngle(yawDeg + sample.getGyroZDps() * deltaSeconds);
        lastSequence = sequence;
        lastTimestampMs = timestampMs;
    }

    private double calculateDeltaSeconds(int sequence, long timestampMs) {
        int sampleDelta = (sequence - lastSequence) & 0xFFFF;
        if (sampleDelta > 0 && sampleDelta <= 200) {
            return sampleDelta / SAMPLE_RATE_HZ;
        }
        long elapsedMs = timestampMs - lastTimestampMs;
        if (elapsedMs <= 0L) return 0.1;
        return Math.max(0.001, Math.min(0.5, elapsedMs / 1000.0));
    }

    private static double normalizeAngle(double angle) {
        while (angle > 180.0) angle -= 360.0;
        while (angle <= -180.0) angle += 360.0;
        return angle;
    }

    public boolean isInitialized() { return initialized; }
    public double getRollDeg() { return rollDeg; }
    public double getPitchDeg() { return pitchDeg; }
    public double getYawDeg() { return yawDeg; }
}

package car.car_app.localization;

import java.util.ArrayDeque;

/**
 * Registers the FPGA-local occupancy map against the server UWB world frame.
 *
 * <p>UWB arrives through ESP32, server and WebSocket, while the matching FPGA
 * pose arrives through BLE. Pairing both values at receipt time bends a moving
 * map by the full network delay. This class keeps recent FPGA poses and pairs
 * each UWB fix with the pose nearest the server source timestamp.</p>
 */
public final class MapWorldRegistration {
    private static final int HISTORY_LIMIT = 256;
    private static final long MAX_PAIR_ERROR_MS = 1500L;
    private static final double MAX_TRANSLATION_JUMP_M = 0.25;
    private static final double MAX_ROTATION_JUMP_RAD = Math.toRadians(20.0);

    private final ArrayDeque<LocalPose> history = new ArrayDeque<>();
    private LocalPose latestLocal;
    private boolean registered;
    private double rotationRad;
    private double translationXM;
    private double translationYM;
    private long pairErrorMs = Long.MAX_VALUE;

    public void addLocalPose(long timestampMs, int xMm, int yMm, float headingDeg) {
        LocalPose pose = new LocalPose(timestampMs, xMm / 1000.0, yMm / 1000.0,
                Math.toRadians(headingDeg));
        latestLocal = pose;
        history.addLast(pose);
        while (history.size() > HISTORY_LIMIT) history.removeFirst();
        while (!history.isEmpty() && timestampMs - history.peekFirst().timestampMs > 6000L) {
            history.removeFirst();
        }
    }

    public boolean addWorldFix(long sourceTimestampMs, double xM, double yM,
                               float headingDeg) {
        LocalPose paired = nearestLocalPose(sourceTimestampMs);
        if (paired == null) return false;
        long error = Math.abs(paired.timestampMs - sourceTimestampMs);
        if (error > MAX_PAIR_ERROR_MS) return false;

        double candidateRotation = Pose2d.normalizeRadians(
                Math.toRadians(headingDeg) - paired.headingRad);
        double cos = Math.cos(candidateRotation);
        double sin = Math.sin(candidateRotation);
        double candidateTranslationX = xM - (cos * paired.xM - sin * paired.yM);
        double candidateTranslationY = yM - (sin * paired.xM + cos * paired.yM);

        if (!registered) {
            rotationRad = candidateRotation;
            translationXM = candidateTranslationX;
            translationYM = candidateTranslationY;
            registered = true;
        } else {
            double headingDelta = Pose2d.normalizeRadians(candidateRotation - rotationRad);
            double translationJump = Math.hypot(candidateTranslationX - translationXM,
                    candidateTranslationY - translationYM);
            // This transform is fixed for one mapping session. A large change
            // is a bad UWB fix, not actual vehicle motion.
            if (Math.abs(headingDelta) > MAX_ROTATION_JUMP_RAD
                    || translationJump > MAX_TRANSLATION_JUMP_M) return false;
            // The map/world transform is a session constant. Re-fitting it on
            // every UWB frame makes a frozen local odometer absorb real vehicle
            // motion into the transform, so the map marker also appears frozen.
            // UWB now supplies translation directly through worldToLocal(); the
            // board IMU heading is transformed with this fixed initial offset.
        }
        pairErrorMs = error;
        return true;
    }

    public WorldPose currentWorldPose() {
        if (!registered || latestLocal == null) return null;
        double cos = Math.cos(rotationRad);
        double sin = Math.sin(rotationRad);
        return new WorldPose(
                translationXM + cos * latestLocal.xM - sin * latestLocal.yM,
                translationYM + sin * latestLocal.xM + cos * latestLocal.yM,
                Pose2d.normalizeRadians(rotationRad + latestLocal.headingRad));
    }

    public double[] worldToLocal(double worldXM, double worldYM) {
        if (!registered) return null;
        double dx = worldXM - translationXM;
        double dy = worldYM - translationYM;
        double cos = Math.cos(rotationRad);
        double sin = Math.sin(rotationRad);
        return new double[]{cos * dx + sin * dy, -sin * dx + cos * dy};
    }

    public boolean isRegistered() { return registered; }

    public void reset() {
        history.clear();
        latestLocal = null;
        registered = false;
        rotationRad = 0.0;
        translationXM = 0.0;
        translationYM = 0.0;
        pairErrorMs = Long.MAX_VALUE;
    }
    public double getRotationRad() { return rotationRad; }
    public double getTranslationXM() { return translationXM; }
    public double getTranslationYM() { return translationYM; }
    public long getPairErrorMs() { return pairErrorMs; }

    private LocalPose nearestLocalPose(long timestampMs) {
        LocalPose best = null;
        long bestError = Long.MAX_VALUE;
        for (LocalPose pose : history) {
            long error = Math.abs(pose.timestampMs - timestampMs);
            if (error < bestError) {
                best = pose;
                bestError = error;
            }
        }
        return best;
    }

    private static final class LocalPose {
        final long timestampMs;
        final double xM;
        final double yM;
        final double headingRad;

        LocalPose(long timestampMs, double xM, double yM, double headingRad) {
            this.timestampMs = timestampMs;
            this.xM = xM;
            this.yM = yM;
            this.headingRad = headingRad;
        }
    }

    public static final class WorldPose {
        public final double xM;
        public final double yM;
        public final double headingRad;

        WorldPose(double xM, double yM, double headingRad) {
            this.xM = xM;
            this.yM = yM;
            this.headingRad = headingRad;
        }
    }
}

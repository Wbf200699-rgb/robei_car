package car.car_app.localization;

/** Immutable planar pose in the shared world frame. */
public final class Pose2d {
    private final double xM;
    private final double yM;
    private final double headingRad;

    public Pose2d(double xM, double yM, double headingRad) {
        this.xM = xM;
        this.yM = yM;
        this.headingRad = normalizeRadians(headingRad);
    }

    public double getXM() { return xM; }
    public double getYM() { return yM; }
    public double getHeadingRad() { return headingRad; }
    public double getHeadingDeg() { return Math.toDegrees(headingRad); }

    public double distanceTo(Pose2d other) {
        return Math.hypot(other.xM - xM, other.yM - yM);
    }

    public double headingDeltaTo(Pose2d other) {
        return normalizeRadians(other.headingRad - headingRad);
    }

    public static double normalizeRadians(double angle) {
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        while (angle <= -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }
}

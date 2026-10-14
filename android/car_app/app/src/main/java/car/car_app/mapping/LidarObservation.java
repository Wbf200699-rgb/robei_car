package car.car_app.mapping;

/** One polar lidar observation relative to the vehicle forward axis. */
public final class LidarObservation {
    private final double angleRad;
    private final double distanceM;
    private final int quality;

    public LidarObservation(double angleRad, double distanceM, int quality) {
        this.angleRad = angleRad;
        this.distanceM = distanceM;
        this.quality = quality;
    }

    public double getAngleRad() { return angleRad; }
    public double getDistanceM() { return distanceM; }
    public int getQuality() { return quality; }
}

package car.car_app.bluetooth;

public final class LidarMapPoint {
    public static final int INVALID_DISTANCE_MM = -1;

    private final int binIndex;
    private final float angleDegrees;
    private final int distanceMm;
    private final int qualityClass;

    public LidarMapPoint(int binIndex, float angleDegrees, int distanceMm, int qualityClass) {
        this.binIndex = binIndex;
        this.angleDegrees = angleDegrees;
        this.distanceMm = distanceMm;
        this.qualityClass = qualityClass;
    }

    public int getBinIndex() {
        return binIndex;
    }

    public float getAngleDegrees() {
        return angleDegrees;
    }

    public int getDistanceMm() {
        return distanceMm;
    }

    public int getQualityClass() {
        return qualityClass;
    }

    public boolean isValid() {
        return distanceMm >= 0;
    }
}

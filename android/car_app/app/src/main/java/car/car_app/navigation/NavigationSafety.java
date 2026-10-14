package car.car_app.navigation;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import car.car_app.bluetooth.LidarMapPoint;
import car.car_app.bluetooth.LidarMapScan;

/** Safety helpers shared by phone-side path following. */
public final class NavigationSafety {
    private static final float FRONT_HALF_ANGLE_DEG = 22.5f;

    private NavigationSafety() {}

    /**
     * Returns a noise-resistant clearance for the vehicle's forward cone.
     * RPLIDAR angles increase clockwise from the nose. The second-nearest
     * high-quality point rejects one isolated short return without hiding a
     * real wall, which spans several adjacent five-degree bins.
     */
    public static int detailedFrontClearanceMm(LidarMapScan scan) {
        if (scan == null) return -1;
        List<Integer> distances = new ArrayList<>();
        for (LidarMapPoint point : scan.getPoints()) {
            float angle = point.getAngleDegrees() % 360f;
            if (angle < 0f) angle += 360f;
            float signedAngle = angle > 180f ? angle - 360f : angle;
            if (Math.abs(signedAngle) > FRONT_HALF_ANGLE_DEG
                    || !point.isValid() || point.getQualityClass() < 2
                    || point.getDistanceMm() <= 0) continue;
            distances.add(point.getDistanceMm());
        }
        if (distances.size() < 3) return -1;
        Collections.sort(distances);
        return distances.get(1);
    }
}

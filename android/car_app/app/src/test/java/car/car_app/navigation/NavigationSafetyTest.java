package car.car_app.navigation;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

import car.car_app.bluetooth.LidarMapPoint;
import car.car_app.bluetooth.LidarMapScan;

public class NavigationSafetyTest {
    @Test
    public void frontClearanceUsesSecondNearestHighQualityReturn() {
        List<LidarMapPoint> points = scanAt(1200, 3);
        points.set(0, new LidarMapPoint(0, 2.5f, 120, 3));
        points.set(1, new LidarMapPoint(1, 7.5f, 900, 3));
        points.set(2, new LidarMapPoint(2, 12.5f, 1000, 3));
        assertEquals(900, NavigationSafety.detailedFrontClearanceMm(
                new LidarMapScan(1, 1, 1L, points)));
    }

    @Test
    public void frontClearanceRejectsSparseOrLowQualityData() {
        List<LidarMapPoint> points = scanAt(1200, 0);
        points.set(0, new LidarMapPoint(0, 2.5f, 800, 3));
        points.set(1, new LidarMapPoint(1, 7.5f, 900, 3));
        assertEquals(-1, NavigationSafety.detailedFrontClearanceMm(
                new LidarMapScan(1, 1, 1L, points)));
    }

    private static List<LidarMapPoint> scanAt(int distanceMm, int quality) {
        List<LidarMapPoint> points = new ArrayList<>();
        for (int index = 0; index < LidarMapScan.POINT_COUNT; index++) {
            points.add(new LidarMapPoint(index, index * 5f + 2.5f,
                    distanceMm, quality));
        }
        return points;
    }
}

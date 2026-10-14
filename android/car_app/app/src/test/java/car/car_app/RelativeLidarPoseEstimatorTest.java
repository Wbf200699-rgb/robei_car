package car.car_app;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import java.util.ArrayList;
import java.util.List;

import car.car_app.bluetooth.LidarMapPoint;
import car.car_app.bluetooth.LidarMapScan;
import car.car_app.calibration.RelativeLidarPoseEstimator;

public class RelativeLidarPoseEstimatorTest {
    @Test
    public void recoversShortTranslationAndRotationInMetricRoom() {
        LidarMapScan reference = roomScan(1, 0.0, 0.0, 0.0);
        LidarMapScan moved = roomScan(2, 120.0, -36.0, 7.0);

        RelativeLidarPoseEstimator.Result result = new RelativeLidarPoseEstimator().estimate(
                reference, moved, 100.0, 0.0, 5.0);

        assertTrue(result.getReason(), result.isAccepted());
        assertEquals(120.0, result.getXMm(), 18.0);
        assertEquals(-36.0, result.getYMm(), 18.0);
        assertEquals(7.0, result.getHeadingDeg(), 2.0);
        assertTrue(result.getInlierRatio() > 0.55);
    }

    @Test
    public void rejectsScanWithoutEnoughHighQualityPoints() {
        List<LidarMapPoint> points = new ArrayList<>();
        for (int index = 0; index < LidarMapScan.POINT_COUNT; index++) {
            points.add(new LidarMapPoint(index, index * 5.0f + 2.5f, 1000, 0));
        }
        LidarMapScan lowQuality = new LidarMapScan(1, 0, 1L, points);
        RelativeLidarPoseEstimator.Result result = new RelativeLidarPoseEstimator().estimate(
                lowQuality, lowQuality, 0.0, 0.0, 0.0);
        assertFalse(result.isAccepted());
    }

    private static LidarMapScan roomScan(int sequence, double poseXmm,
                                         double poseYmm, double headingDeg) {
        List<LidarMapPoint> points = new ArrayList<>();
        for (int index = 0; index < LidarMapScan.POINT_COUNT; index++) {
            double localAngleDeg = index * 5.0 + 2.5;
            // RPLIDAR reports clockwise-positive local angles; the simulated
            // room and pose use a counter-clockwise-positive world frame.
            double angle = Math.toRadians(headingDeg - localAngleDeg);
            double cos = Math.cos(angle);
            double sin = Math.sin(angle);
            double distance = Double.POSITIVE_INFINITY;
            if (cos > 1e-9) distance = Math.min(distance, (2100.0 - poseXmm) / cos);
            if (cos < -1e-9) distance = Math.min(distance, (-1900.0 - poseXmm) / cos);
            if (sin > 1e-9) distance = Math.min(distance, (1450.0 - poseYmm) / sin);
            if (sin < -1e-9) distance = Math.min(distance, (-1550.0 - poseYmm) / sin);
            points.add(new LidarMapPoint(
                    index,
                    (float) localAngleDeg,
                    (int) Math.round(distance),
                    3
            ));
        }
        return new LidarMapScan(sequence, sequence * 10, sequence * 100L, points);
    }
}

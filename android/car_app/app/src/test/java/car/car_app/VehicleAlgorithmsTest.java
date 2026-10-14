package car.car_app;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.junit.Test;

import car.car_app.calibration.MotionCalibrationEngine;
import car.car_app.calibration.MotionCalibrationResult;
import car.car_app.calibration.MotionCalibrationSample;
import car.car_app.geometry.VehicleGeometry;
import car.car_app.localization.Pose2d;
import car.car_app.localization.UwbPoseEstimate;
import car.car_app.localization.UwbRigidBodyEstimator;
import car.car_app.localization.UwbTagFix;
import car.car_app.localization.UwbWindowedPoseEstimator;
import car.car_app.mapping.LidarObservation;
import car.car_app.mapping.OccupancyGridMapper;
import car.car_app.mapping.ScanMatchResult;

public class VehicleAlgorithmsTest {
    @Test
    public void sharedGeometryMatchesPhysicalLayout() {
        assertEquals(1, VehicleGeometry.FRONT_LEFT_TAG);
        assertEquals(2, VehicleGeometry.FRONT_RIGHT_TAG);
        assertEquals(0, VehicleGeometry.REAR_LEFT_TAG);
        assertEquals(3, VehicleGeometry.REAR_RIGHT_TAG);
        assertEquals(0.20, VehicleGeometry.TAG_RECTANGLE_WIDTH_M, 1e-9);
        assertEquals(0.22, VehicleGeometry.TAG_RECTANGLE_LENGTH_M, 1e-9);
        assertEquals(1.5, VehicleGeometry.ANCHOR_RECTANGLE_WIDTH_M, 1e-9);
        assertEquals(1.67, VehicleGeometry.ANCHOR_RECTANGLE_HEIGHT_M, 1e-9);
    }

    @Test
    public void fourTagRigidFitRecoversVehiclePose() {
        double centerX = 0.62;
        double centerY = 0.91;
        double heading = Math.toRadians(37.0);
        long timestamp = 1_000L;
        List<UwbTagFix> fixes = transformedFixes(centerX, centerY, heading, timestamp);

        UwbPoseEstimate result = new UwbRigidBodyEstimator().estimate(fixes, timestamp + 10L);

        assertTrue(result.isUsable());
        assertEquals(centerX, result.getPose().getXM(), 1e-9);
        assertEquals(centerY, result.getPose().getYM(), 1e-9);
        assertEquals(37.0, result.getPose().getHeadingDeg(), 1e-9);
        assertEquals(0.0, result.getRmsResidualM(), 1e-9);
    }

    @Test
    public void fourTagRigidFitRejectsOneMultipathOutlier() {
        List<UwbTagFix> fixes = transformedFixes(0.75, 0.8, 0.0, 1_000L);
        fixes.set(3, new UwbTagFix(3, fixes.get(3).getXM() + 0.18,
                fixes.get(3).getYM(), 1_000L, true));

        UwbPoseEstimate result = new UwbRigidBodyEstimator().estimate(fixes, 1_010L);

        assertEquals(UwbPoseEstimate.Quality.NON_RIGID, result.getQuality());
        assertFalse(result.isUsable());
        assertTrue(result.getMaximumResidualM() > VehicleGeometry.DIMENSION_TOLERANCE_M);
    }

    @Test
    public void staticMedianWindowRejectsIntermittentUwbMultipath() {
        UwbWindowedPoseEstimator estimator = new UwbWindowedPoseEstimator();
        double[] noise = {-0.012, 0.006, -0.004, 0.010, 0.0, -0.008, 0.004, 0.009, -0.003};
        for (int sample = 0; sample < noise.length; sample++) {
            List<UwbTagFix> frame = transformedFixes(0.70, 0.82,
                    Math.toRadians(20.0), 1_000L + sample * 50L);
            for (UwbTagFix fix : frame) {
                double outlier = (sample == 2 && fix.getTagId() == 0) ? 0.25 : 0.0;
                estimator.add(new UwbTagFix(fix.getTagId(),
                        fix.getXM() + noise[sample] + outlier,
                        fix.getYM() - noise[sample], fix.getTimestampMs(), true));
            }
        }

        UwbPoseEstimate result = estimator.estimate(1_450L);

        assertTrue(result.isUsable());
        assertEquals(0.70, result.getPose().getXM(), 0.015);
        assertEquals(0.82, result.getPose().getYM(), 0.015);
        assertEquals(20.0, result.getPose().getHeadingDeg(), 1.0);
    }

    @Test
    public void calibrationUsesMedianAndRejectsGrossOutlier() {
        List<MotionCalibrationSample> samples = new ArrayList<>();
        samples.add(forwardSample(0.300, 1.0, 0.0));
        samples.add(forwardSample(0.306, 1.1, 0.0));
        samples.add(forwardSample(0.294, 0.9, 0.0));
        samples.add(forwardSample(1.500, 0.0, 0.0));
        samples.add(turnSample(true, 90.0, 500));
        samples.add(turnSample(true, 88.0, 500));
        samples.add(turnSample(false, 92.0, 500));

        MotionCalibrationResult result = new MotionCalibrationEngine().solve(samples);

        assertEquals(0.300, result.getForwardMmPerMs(), 0.01);
        assertEquals(0.178, result.getLeftDegPerMs(), 0.005);
        assertEquals(0.184, result.getRightDegPerMs(), 0.005);
        assertTrue(result.getRejectedSamples() >= 1);
    }

    @Test
    public void mappingBuildsWallAndRefinesBiasedPose() {
        OccupancyGridMapper mapper = new OccupancyGridMapper();
        List<LidarObservation> wallScan = verticalWallScan(1.0);
        Pose2d truth = new Pose2d(0.0, 0.0, 0.0);
        for (int index = 0; index < 5; index++) mapper.integrate(truth, wallScan);

        assertTrue(mapper.getOccupiedCellCount() >= 12);
        assertTrue(mapper.getFreeCellCount() >= 100);
        ScanMatchResult match = mapper.refine(new Pose2d(-0.1, 0.0, 0.0), wallScan);
        assertTrue(match.isAccepted());
        assertEquals(0.0, match.getPose().getXM(), 0.03);
    }

    private static List<UwbTagFix> transformedFixes(double centerX, double centerY,
                                                     double heading, long timestamp) {
        List<UwbTagFix> result = new ArrayList<>();
        double cosine = Math.cos(heading);
        double sine = Math.sin(heading);
        for (int tagId = 0; tagId < 4; tagId++) {
            double bodyX = VehicleGeometry.TAG_POSITIONS_M[tagId][0];
            double bodyY = VehicleGeometry.TAG_POSITIONS_M[tagId][1];
            result.add(new UwbTagFix(tagId,
                    centerX + cosine * bodyX - sine * bodyY,
                    centerY + sine * bodyX + cosine * bodyY,
                    timestamp, true));
        }
        return result;
    }

    private static MotionCalibrationSample forwardSample(double distanceM,
                                                          double headingDeltaDeg,
                                                          double lateralM) {
        return new MotionCalibrationSample(MotionCalibrationSample.Command.FORWARD, 1_000,
                new Pose2d(0.0, 0.0, 0.0),
                new Pose2d(distanceM, lateralM, Math.toRadians(headingDeltaDeg)));
    }

    private static MotionCalibrationSample turnSample(boolean left, double degrees, int durationMs) {
        return new MotionCalibrationSample(
                left ? MotionCalibrationSample.Command.TURN_LEFT
                        : MotionCalibrationSample.Command.TURN_RIGHT,
                durationMs, new Pose2d(0.0, 0.0, 0.0),
                new Pose2d(0.0, 0.0, Math.toRadians(left ? degrees : -degrees)));
    }

    private static List<LidarObservation> verticalWallScan(double wallXM) {
        List<LidarObservation> scan = new ArrayList<>();
        for (int yStep = -10; yStep <= 10; yStep++) {
            double y = yStep * 0.05;
            double distance = Math.hypot(wallXM, y);
            scan.add(new LidarObservation(Math.atan2(y, wallXM), distance, 3));
        }
        return scan;
    }
}

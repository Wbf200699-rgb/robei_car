package car.car_app.diagnostics;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import car.car_app.calibration.MotionCalibrationEngine;
import car.car_app.calibration.MotionCalibrationResult;
import car.car_app.calibration.MotionCalibrationSample;
import car.car_app.geometry.VehicleGeometry;
import car.car_app.localization.Pose2d;
import car.car_app.localization.UwbPoseEstimate;
import car.car_app.localization.UwbRigidBodyEstimator;
import car.car_app.localization.UwbTagFix;
import car.car_app.mapping.LidarObservation;
import car.car_app.mapping.OccupancyGridMapper;
import car.car_app.mapping.ScanMatchResult;

/** Runs a deterministic replay through the same algorithms used for engineering data. */
public final class VehicleAlgorithmSelfCheck {
    private VehicleAlgorithmSelfCheck() {}

    public static String run() {
        long timestampMs = 10_000L;
        Pose2d expectedPose = new Pose2d(0.75, 0.835, Math.toRadians(23.0));
        UwbPoseEstimate pose = new UwbRigidBodyEstimator().estimate(
                transformedFixes(expectedPose, timestampMs), timestampMs + 10L);
        require(pose.isUsable(), "four-tag pose rejected: " + pose.getQuality());
        require(pose.getPose().distanceTo(expectedPose) < 0.001,
                "four-tag center mismatch");
        require(Math.abs(Math.toDegrees(expectedPose.headingDeltaTo(pose.getPose()))) < 0.1,
                "four-tag heading mismatch");

        List<MotionCalibrationSample> calibrationSamples = new ArrayList<>();
        calibrationSamples.add(forwardSample(0.300, 1.0));
        calibrationSamples.add(forwardSample(0.306, 1.1));
        calibrationSamples.add(forwardSample(0.294, 0.9));
        calibrationSamples.add(forwardSample(1.500, 0.0));
        calibrationSamples.add(turnSample(true, 90.0, 500L));
        calibrationSamples.add(turnSample(true, 88.0, 500L));
        calibrationSamples.add(turnSample(false, 92.0, 500L));
        MotionCalibrationResult calibration = new MotionCalibrationEngine().solve(calibrationSamples);
        require(Math.abs(calibration.getForwardMmPerMs() - 0.300) < 0.01,
                "forward calibration mismatch");
        require(Math.abs(calibration.getLeftDegPerMs() - 0.178) < 0.005,
                "left calibration mismatch");
        require(Math.abs(calibration.getRightDegPerMs() - 0.184) < 0.005,
                "right calibration mismatch");
        require(calibration.getRejectedSamples() >= 1, "calibration outlier was accepted");

        OccupancyGridMapper mapper = new OccupancyGridMapper();
        List<LidarObservation> wallScan = verticalWallScan(1.0);
        Pose2d mapOrigin = new Pose2d(0.0, 0.0, 0.0);
        for (int index = 0; index < 5; index++) mapper.integrate(mapOrigin, wallScan);
        ScanMatchResult match = mapper.refine(new Pose2d(-0.1, 0.0, 0.0), wallScan);
        require(mapper.getOccupiedCellCount() >= 12, "wall was not mapped");
        require(mapper.getFreeCellCount() >= 100, "free space was not mapped");
        require(match.isAccepted(), "scan match was rejected");
        require(Math.abs(match.getPose().getXM()) <= 0.03, "scan match pose mismatch");

        return String.format(Locale.US,
                "PASS pose=%.3f,%.3f,%.1fdeg fwd=%.3fmm/ms turn=%.3f/%.3fdeg/ms map=%d/%d gain=%d",
                pose.getPose().getXM(), pose.getPose().getYM(), pose.getPose().getHeadingDeg(),
                calibration.getForwardMmPerMs(), calibration.getLeftDegPerMs(),
                calibration.getRightDegPerMs(), mapper.getOccupiedCellCount(),
                mapper.getFreeCellCount(), match.getScoreGain());
    }

    private static List<UwbTagFix> transformedFixes(Pose2d pose, long timestampMs) {
        List<UwbTagFix> fixes = new ArrayList<>();
        double cosine = Math.cos(pose.getHeadingRad());
        double sine = Math.sin(pose.getHeadingRad());
        for (int tagId = 0; tagId < 4; tagId++) {
            double bodyX = VehicleGeometry.TAG_POSITIONS_M[tagId][0];
            double bodyY = VehicleGeometry.TAG_POSITIONS_M[tagId][1];
            fixes.add(new UwbTagFix(tagId,
                    pose.getXM() + cosine * bodyX - sine * bodyY,
                    pose.getYM() + sine * bodyX + cosine * bodyY,
                    timestampMs, true));
        }
        return fixes;
    }

    private static MotionCalibrationSample forwardSample(double distanceM,
                                                          double headingDeltaDeg) {
        return new MotionCalibrationSample(MotionCalibrationSample.Command.FORWARD, 1_000L,
                new Pose2d(0.0, 0.0, 0.0),
                new Pose2d(distanceM, 0.0, Math.toRadians(headingDeltaDeg)));
    }

    private static MotionCalibrationSample turnSample(boolean left, double angleDeg,
                                                       long durationMs) {
        return new MotionCalibrationSample(left
                ? MotionCalibrationSample.Command.TURN_LEFT
                : MotionCalibrationSample.Command.TURN_RIGHT,
                durationMs, new Pose2d(0.0, 0.0, 0.0),
                new Pose2d(0.0, 0.0, Math.toRadians(left ? angleDeg : -angleDeg)));
    }

    private static List<LidarObservation> verticalWallScan(double wallXM) {
        List<LidarObservation> scan = new ArrayList<>();
        for (int yStep = -10; yStep <= 10; yStep++) {
            double yM = yStep * 0.05;
            scan.add(new LidarObservation(Math.atan2(yM, wallXM),
                    Math.hypot(wallXM, yM), 3));
        }
        return scan;
    }

    private static void require(boolean condition, String message) {
        if (!condition) throw new IllegalStateException(message);
    }
}

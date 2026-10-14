package car.car_app.calibration;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import car.car_app.localization.Pose2d;

/** Computes repeatable motion constants from externally observed start/end poses. */
public final class MotionCalibrationEngine {
    private static final double MIN_TRANSLATION_M = 0.03;
    private static final double MIN_TURN_RAD = Math.toRadians(5.0);

    public MotionCalibrationResult solve(List<MotionCalibrationSample> samples) {
        List<Double> forwardRates = new ArrayList<>();
        List<Double> reverseRates = new ArrayList<>();
        List<Double> leftRates = new ArrayList<>();
        List<Double> rightRates = new ArrayList<>();
        List<Double> forwardYawDrift = new ArrayList<>();
        int rejected = 0;

        for (MotionCalibrationSample sample : samples) {
            Pose2d start = sample.getStart();
            Pose2d end = sample.getEnd();
            double dx = end.getXM() - start.getXM();
            double dy = end.getYM() - start.getYM();
            double forwardProjection = dx * Math.cos(start.getHeadingRad())
                    + dy * Math.sin(start.getHeadingRad());
            double lateralProjection = -dx * Math.sin(start.getHeadingRad())
                    + dy * Math.cos(start.getHeadingRad());
            double headingDelta = start.headingDeltaTo(end);
            double durationMs = sample.getDurationMs();

            switch (sample.getCommand()) {
                case FORWARD:
                    if (forwardProjection < MIN_TRANSLATION_M
                            || Math.abs(lateralProjection) > Math.max(0.08, forwardProjection * 0.5)) {
                        rejected++;
                        break;
                    }
                    forwardRates.add(forwardProjection * 1000.0 / durationMs);
                    forwardYawDrift.add(Math.toDegrees(headingDelta) / forwardProjection);
                    break;
                case REVERSE:
                    if (forwardProjection > -MIN_TRANSLATION_M
                            || Math.abs(lateralProjection) > Math.max(0.08, -forwardProjection * 0.5)) {
                        rejected++;
                        break;
                    }
                    reverseRates.add(-forwardProjection * 1000.0 / durationMs);
                    break;
                case TURN_LEFT:
                    if (headingDelta < MIN_TURN_RAD || Math.hypot(dx, dy) > 0.15) {
                        rejected++;
                        break;
                    }
                    leftRates.add(Math.toDegrees(headingDelta) / durationMs);
                    break;
                case TURN_RIGHT:
                    if (headingDelta > -MIN_TURN_RAD || Math.hypot(dx, dy) > 0.15) {
                        rejected++;
                        break;
                    }
                    rightRates.add(-Math.toDegrees(headingDelta) / durationMs);
                    break;
                default:
                    rejected++;
            }
        }

        List<Double> cleanForward = rejectOutliers(forwardRates);
        List<Double> cleanReverse = rejectOutliers(reverseRates);
        List<Double> cleanLeft = rejectOutliers(leftRates);
        List<Double> cleanRight = rejectOutliers(rightRates);
        List<Double> cleanYaw = rejectOutliers(forwardYawDrift);
        int accepted = cleanForward.size() + cleanReverse.size() + cleanLeft.size() + cleanRight.size();
        rejected += forwardRates.size() - cleanForward.size();
        rejected += reverseRates.size() - cleanReverse.size();
        rejected += leftRates.size() - cleanLeft.size();
        rejected += rightRates.size() - cleanRight.size();

        return new MotionCalibrationResult(
                medianOrNaN(cleanForward), medianOrNaN(cleanReverse),
                medianOrNaN(cleanLeft), medianOrNaN(cleanRight),
                medianOrNaN(cleanYaw), accepted, rejected);
    }

    private static List<Double> rejectOutliers(List<Double> values) {
        if (values.size() < 4) return new ArrayList<>(values);
        double median = medianOrNaN(values);
        List<Double> deviations = new ArrayList<>();
        for (double value : values) deviations.add(Math.abs(value - median));
        double mad = medianOrNaN(deviations);
        double limit = Math.max(Math.abs(median) * 0.12, mad * 3.5);
        List<Double> result = new ArrayList<>();
        for (double value : values) {
            if (Math.abs(value - median) <= limit) result.add(value);
        }
        return result;
    }

    private static double medianOrNaN(List<Double> values) {
        if (values.isEmpty()) return Double.NaN;
        List<Double> sorted = new ArrayList<>(values);
        Collections.sort(sorted);
        int middle = sorted.size() / 2;
        if ((sorted.size() & 1) != 0) return sorted.get(middle);
        return (sorted.get(middle - 1) + sorted.get(middle)) * 0.5;
    }
}

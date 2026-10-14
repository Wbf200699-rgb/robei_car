package car.car_app.calibration;

import java.util.ArrayList;
import java.util.List;

import car.car_app.bluetooth.LidarMapPoint;
import car.car_app.bluetooth.LidarMapScan;

/**
 * Estimates one short vehicle motion by matching two stopped RPLIDAR scans.
 *
 * <p>The returned transform expresses the current vehicle frame in the
 * reference vehicle frame. Positive X is forward, positive Y is left and a
 * positive heading is a counter-clockwise (left) turn. The search is bounded
 * around a command-derived prediction so repeated room geometry cannot cause a
 * large, physically impossible jump.</p>
 */
public final class RelativeLidarPoseEstimator {
    private static final int MIN_QUALITY = 2;
    private static final int MIN_RANGE_MM = 80;
    private static final int MAX_RANGE_MM = 5000;
    private static final double INLIER_DISTANCE_MM = 180.0;
    private static final double TRUNCATED_DISTANCE_MM = 320.0;

    public Result estimate(
            LidarMapScan reference,
            LidarMapScan current,
            double predictedXmm,
            double predictedYmm,
            double predictedHeadingDeg
    ) {
        PointCloud referenceCloud = PointCloud.from(reference);
        PointCloud currentCloud = PointCloud.from(current);
        if (referenceCloud.size() < 20 || currentCloud.size() < 20) {
            return Result.rejected("有效高质量点不足");
        }

        Candidate predicted = evaluate(referenceCloud, currentCloud,
                predictedXmm, predictedYmm, predictedHeadingDeg);
        Candidate best = predicted;

        // Coarse search: enough room for a 100-150 mm test move and modest
        // wheel slip, while still excluding an unrelated room symmetry.
        for (double x = predictedXmm - 180.0; x <= predictedXmm + 180.0; x += 30.0) {
            for (double y = predictedYmm - 150.0; y <= predictedYmm + 150.0; y += 30.0) {
                for (double heading = predictedHeadingDeg - 16.0;
                     heading <= predictedHeadingDeg + 16.0; heading += 4.0) {
                    Candidate candidate = evaluate(referenceCloud, currentCloud, x, y, heading);
                    if (candidate.cost < best.cost) best = candidate;
                }
            }
        }

        // Fine search around the coarse minimum. Six millimetres and one
        // degree are below the map cell/sector resolution and avoid false
        // precision in the saved calibration constants.
        Candidate coarseBest = best;
        for (double x = coarseBest.xMm - 36.0; x <= coarseBest.xMm + 36.0; x += 6.0) {
            for (double y = coarseBest.yMm - 36.0; y <= coarseBest.yMm + 36.0; y += 6.0) {
                for (double heading = coarseBest.headingDeg - 4.0;
                     heading <= coarseBest.headingDeg + 4.0; heading += 1.0) {
                    Candidate candidate = evaluate(referenceCloud, currentCloud, x, y, heading);
                    if (candidate.cost < best.cost) best = candidate;
                }
            }
        }

        int available = referenceCloud.size() + currentCloud.size();
        double inlierRatio = available == 0 ? 0.0 : best.inliers / (double) available;
        double rmsMm = best.inliers == 0
                ? Double.POSITIVE_INFINITY
                : Math.sqrt(best.inlierSquaredError / best.inliers);
        double predictionDistance = Math.hypot(best.xMm - predictedXmm, best.yMm - predictedYmm);
        double predictionAngle = Math.abs(normalizeDegrees(best.headingDeg - predictedHeadingDeg));
        boolean accepted = best.inliers >= 24
                && inlierRatio >= 0.38
                && rmsMm <= 145.0
                && predictionDistance <= 230.0
                && predictionAngle <= 19.0;
        String reason = accepted ? "通过"
                : String.format(java.util.Locale.CHINA,
                "匹配质量不足(inliers=%d, ratio=%.2f, rms=%.0fmm)",
                best.inliers, inlierRatio, rmsMm);
        return new Result(
                best.xMm,
                best.yMm,
                normalizeDegrees(best.headingDeg),
                best.inliers,
                inlierRatio,
                rmsMm,
                predicted.cost - best.cost,
                accepted,
                reason
        );
    }

    private static Candidate evaluate(PointCloud reference, PointCloud current,
                                      double xMm, double yMm, double headingDeg) {
        double radians = Math.toRadians(headingDeg);
        double cos = Math.cos(radians);
        double sin = Math.sin(radians);
        double[] transformedX = new double[current.size()];
        double[] transformedY = new double[current.size()];
        for (int index = 0; index < current.size(); index++) {
            transformedX[index] = xMm + cos * current.x[index] - sin * current.y[index];
            transformedY[index] = yMm + sin * current.x[index] + cos * current.y[index];
        }

        Score score = new Score();
        for (int index = 0; index < current.size(); index++) {
            score.addNearest(transformedX[index], transformedY[index],
                    reference.x, reference.y, reference.bin);
        }
        // Bidirectional scoring prevents a long wall from accepting a pose that
        // aligns only a small subset of the new scan.
        for (int index = 0; index < reference.size(); index++) {
            score.addNearest(reference.x[index], reference.y[index],
                    transformedX, transformedY, current.bin);
        }
        return new Candidate(xMm, yMm, headingDeg, score.cost,
                score.inliers, score.inlierSquaredError);
    }

    private static double normalizeDegrees(double angle) {
        while (angle > 180.0) angle -= 360.0;
        while (angle <= -180.0) angle += 360.0;
        return angle;
    }

    private static final class PointCloud {
        private final double[] x;
        private final double[] y;
        private final int[] bin;

        private PointCloud(double[] x, double[] y, int[] bin) {
            this.x = x;
            this.y = y;
            this.bin = bin;
        }

        static PointCloud from(LidarMapScan scan) {
            List<Double> xs = new ArrayList<>();
            List<Double> ys = new ArrayList<>();
            List<Integer> bins = new ArrayList<>();
            if (scan != null) {
                for (LidarMapPoint point : scan.getPoints()) {
                    int distance = point.getDistanceMm();
                    if (!point.isValid() || point.getQualityClass() < MIN_QUALITY
                            || distance < MIN_RANGE_MM || distance > MAX_RANGE_MM) continue;
                    // Sensor angle is clockwise; vehicle/world +Y is left and
                    // therefore uses counter-clockwise-positive angles.
                    double angle = Math.toRadians(-point.getAngleDegrees());
                    xs.add(Math.cos(angle) * distance);
                    ys.add(Math.sin(angle) * distance);
                    bins.add(point.getBinIndex());
                }
            }
            double[] x = new double[xs.size()];
            double[] y = new double[ys.size()];
            int[] bin = new int[bins.size()];
            for (int index = 0; index < xs.size(); index++) {
                x[index] = xs.get(index);
                y[index] = ys.get(index);
                bin[index] = bins.get(index);
            }
            return new PointCloud(x, y, bin);
        }

        int size() { return x.length; }
    }

    private static final class Score {
        private double cost;
        private int inliers;
        private double inlierSquaredError;

        void addNearest(double x, double y, double[] candidatesX, double[] candidatesY,
                        int[] candidateBins) {
            double nearestSquared = TRUNCATED_DISTANCE_MM * TRUNCATED_DISTANCE_MM;
            for (int index = 0; index < candidatesX.length; index++) {
                double dx = x - candidatesX[index];
                double dy = y - candidatesY[index];
                double squared = dx * dx + dy * dy;
                if (squared < nearestSquared) nearestSquared = squared;
                int next = index + 1;
                if (next < candidatesX.length
                        && candidateBins[next] == candidateBins[index] + 1) {
                    double segmentX = candidatesX[next] - candidatesX[index];
                    double segmentY = candidatesY[next] - candidatesY[index];
                    double segmentSquared = segmentX * segmentX + segmentY * segmentY;
                    // Do not bridge depth discontinuities such as two separate
                    // walls or an invalid range gap.
                    if (segmentSquared > 1.0 && segmentSquared <= 700.0 * 700.0) {
                        double projection = ((x - candidatesX[index]) * segmentX
                                + (y - candidatesY[index]) * segmentY) / segmentSquared;
                        projection = Math.max(0.0, Math.min(1.0, projection));
                        double nearestX = candidatesX[index] + projection * segmentX;
                        double nearestY = candidatesY[index] + projection * segmentY;
                        double lineDx = x - nearestX;
                        double lineDy = y - nearestY;
                        nearestSquared = Math.min(nearestSquared,
                                lineDx * lineDx + lineDy * lineDy);
                    }
                }
            }
            cost += nearestSquared;
            if (nearestSquared <= INLIER_DISTANCE_MM * INLIER_DISTANCE_MM) {
                inliers++;
                inlierSquaredError += nearestSquared;
            }
        }
    }

    private static final class Candidate {
        private final double xMm;
        private final double yMm;
        private final double headingDeg;
        private final double cost;
        private final int inliers;
        private final double inlierSquaredError;

        Candidate(double xMm, double yMm, double headingDeg, double cost,
                  int inliers, double inlierSquaredError) {
            this.xMm = xMm;
            this.yMm = yMm;
            this.headingDeg = headingDeg;
            this.cost = cost;
            this.inliers = inliers;
            this.inlierSquaredError = inlierSquaredError;
        }
    }

    public static final class Result {
        private final double xMm;
        private final double yMm;
        private final double headingDeg;
        private final int inliers;
        private final double inlierRatio;
        private final double rmsMm;
        private final double costImprovement;
        private final boolean accepted;
        private final String reason;

        private Result(double xMm, double yMm, double headingDeg, int inliers,
                       double inlierRatio, double rmsMm, double costImprovement,
                       boolean accepted, String reason) {
            this.xMm = xMm;
            this.yMm = yMm;
            this.headingDeg = headingDeg;
            this.inliers = inliers;
            this.inlierRatio = inlierRatio;
            this.rmsMm = rmsMm;
            this.costImprovement = costImprovement;
            this.accepted = accepted;
            this.reason = reason;
        }

        static Result rejected(String reason) {
            return new Result(0.0, 0.0, 0.0, 0, 0.0,
                    Double.POSITIVE_INFINITY, 0.0, false, reason);
        }

        public double getXMm() { return xMm; }
        public double getYMm() { return yMm; }
        public double getHeadingDeg() { return headingDeg; }
        public int getInliers() { return inliers; }
        public double getInlierRatio() { return inlierRatio; }
        public double getRmsMm() { return rmsMm; }
        public double getCostImprovement() { return costImprovement; }
        public boolean isAccepted() { return accepted; }
        public String getReason() { return reason; }
    }
}

package car.car_app.localization;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import car.car_app.geometry.VehicleGeometry;

/**
 * Fits the known four-tag rectangle to measured UWB positions.
 *
 * <p>The fit is a 2D orthogonal Procrustes solve. Translation is the vehicle
 * center, rotation is vehicle heading, and residuals expose multipath or an
 * accidentally swapped tag before those samples reach calibration.</p>
 */
public final class UwbRigidBodyEstimator {
    private final double maximumResidualM;
    private final long maximumSkewMs;
    private final long maximumAgeMs;

    public UwbRigidBodyEstimator() {
        this(VehicleGeometry.DIMENSION_TOLERANCE_M,
                Math.round(VehicleGeometry.MAXIMUM_FIX_SKEW_S * 1000.0),
                Math.round(VehicleGeometry.MAXIMUM_FIX_AGE_S * 1000.0));
    }

    public UwbRigidBodyEstimator(double maximumResidualM, long maximumSkewMs,
                                 long maximumAgeMs) {
        this.maximumResidualM = maximumResidualM;
        this.maximumSkewMs = maximumSkewMs;
        this.maximumAgeMs = maximumAgeMs;
    }

    public UwbPoseEstimate estimate(List<UwbTagFix> fixes, long nowMs) {
        Map<Integer, UwbTagFix> byId = new HashMap<>();
        for (UwbTagFix fix : fixes) {
            if (!fix.isValid() || fix.getTagId() < 0 || fix.getTagId() > 3) continue;
            UwbTagFix previous = byId.get(fix.getTagId());
            if (previous == null || fix.getTimestampMs() > previous.getTimestampMs()) {
                byId.put(fix.getTagId(), fix);
            }
        }
        if (byId.size() != 4) {
            return new UwbPoseEstimate(UwbPoseEstimate.Quality.INCOMPLETE,
                    null, Double.NaN, Double.NaN, Long.MAX_VALUE);
        }

        long minimumTimestamp = Long.MAX_VALUE;
        long maximumTimestamp = Long.MIN_VALUE;
        double measuredCenterX = 0.0;
        double measuredCenterY = 0.0;
        for (int tagId = 0; tagId < 4; tagId++) {
            UwbTagFix fix = byId.get(tagId);
            minimumTimestamp = Math.min(minimumTimestamp, fix.getTimestampMs());
            maximumTimestamp = Math.max(maximumTimestamp, fix.getTimestampMs());
            measuredCenterX += fix.getXM();
            measuredCenterY += fix.getYM();
        }
        measuredCenterX /= 4.0;
        measuredCenterY /= 4.0;
        long skewMs = maximumTimestamp - minimumTimestamp;

        // For centered body points, atan2(cross, dot) directly gives the
        // least-squares rotation from the vehicle frame into the world frame.
        double dot = 0.0;
        double cross = 0.0;
        for (int tagId = 0; tagId < 4; tagId++) {
            double bodyX = VehicleGeometry.TAG_POSITIONS_M[tagId][0];
            double bodyY = VehicleGeometry.TAG_POSITIONS_M[tagId][1];
            UwbTagFix fix = byId.get(tagId);
            double worldX = fix.getXM() - measuredCenterX;
            double worldY = fix.getYM() - measuredCenterY;
            dot += bodyX * worldX + bodyY * worldY;
            cross += bodyX * worldY - bodyY * worldX;
        }
        double heading = Math.atan2(cross, dot);
        double cosine = Math.cos(heading);
        double sine = Math.sin(heading);
        double squaredResidualSum = 0.0;
        double maximumResidual = 0.0;
        for (int tagId = 0; tagId < 4; tagId++) {
            double bodyX = VehicleGeometry.TAG_POSITIONS_M[tagId][0];
            double bodyY = VehicleGeometry.TAG_POSITIONS_M[tagId][1];
            double predictedX = measuredCenterX + cosine * bodyX - sine * bodyY;
            double predictedY = measuredCenterY + sine * bodyX + cosine * bodyY;
            UwbTagFix fix = byId.get(tagId);
            double residual = Math.hypot(fix.getXM() - predictedX, fix.getYM() - predictedY);
            squaredResidualSum += residual * residual;
            maximumResidual = Math.max(maximumResidual, residual);
        }
        double rmsResidual = Math.sqrt(squaredResidualSum / 4.0);
        Pose2d pose = new Pose2d(measuredCenterX, measuredCenterY, heading);

        UwbPoseEstimate.Quality quality;
        if (nowMs - minimumTimestamp > maximumAgeMs || skewMs > maximumSkewMs) {
            quality = UwbPoseEstimate.Quality.STALE;
        } else if (maximumResidual > maximumResidualM) {
            quality = UwbPoseEstimate.Quality.NON_RIGID;
        } else {
            quality = UwbPoseEstimate.Quality.FULL;
        }
        return new UwbPoseEstimate(quality, pose, rmsResidual, maximumResidual, skewMs);
    }
}

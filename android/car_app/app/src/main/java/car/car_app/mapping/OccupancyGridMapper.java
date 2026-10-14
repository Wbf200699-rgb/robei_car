package car.car_app.mapping;

import java.util.List;

import car.car_app.localization.Pose2d;

/**
 * CPU-side occupancy mapping reference used before an FPGA implementation.
 *
 * <p>The default grid is 241 x 241 at 25 mm/cell (about 6 x 6 m). Misses and
 * hits use bounded log odds. Scan matching searches a small window around the
 * motion prediction, so it is deterministic and can later be reproduced with
 * fixed-point arithmetic on FPGA.</p>
 */
public final class OccupancyGridMapper {
    public static final int DEFAULT_SIZE = 241;
    public static final double DEFAULT_RESOLUTION_M = 0.025;
    private static final int MIN_LOG_ODDS = -12;
    private static final int MAX_LOG_ODDS = 12;
    private static final int MIN_QUALITY = 2;
    private static final double MAX_RANGE_M = 5.0;

    private final int size;
    private final double resolutionM;
    private final byte[] cells;
    private int integratedScans;

    public OccupancyGridMapper() {
        this(DEFAULT_SIZE, DEFAULT_RESOLUTION_M);
    }

    public OccupancyGridMapper(int size, double resolutionM) {
        if (size < 21 || (size & 1) == 0) throw new IllegalArgumentException("size must be odd");
        if (resolutionM <= 0.0) throw new IllegalArgumentException("resolution must be positive");
        this.size = size;
        this.resolutionM = resolutionM;
        this.cells = new byte[size * size];
    }

    public void integrate(Pose2d pose, List<LidarObservation> scan) {
        int startX = worldToCell(pose.getXM());
        int startY = worldToCell(pose.getYM());
        for (LidarObservation observation : scan) {
            if (observation.getQuality() < MIN_QUALITY || observation.getDistanceM() <= 0.0) continue;
            double clippedDistance = Math.min(observation.getDistanceM(), MAX_RANGE_M);
            double worldAngle = pose.getHeadingRad() + observation.getAngleRad();
            int endX = worldToCell(pose.getXM() + Math.cos(worldAngle) * clippedDistance);
            int endY = worldToCell(pose.getYM() + Math.sin(worldAngle) * clippedDistance);
            integrateRay(startX, startY, endX, endY,
                    observation.getDistanceM() <= MAX_RANGE_M);
        }
        integratedScans++;
    }

    public ScanMatchResult refine(Pose2d predicted, List<LidarObservation> scan) {
        int baseline = score(predicted, scan);
        int bestScore = baseline;
        Pose2d bestPose = predicted;
        for (int dx = -4; dx <= 4; dx++) {
            for (int dy = -4; dy <= 4; dy++) {
                for (int angleDeg = -10; angleDeg <= 10; angleDeg += 2) {
                    if (dx == 0 && dy == 0 && angleDeg == 0) continue;
                    Pose2d candidate = new Pose2d(
                            predicted.getXM() + dx * resolutionM,
                            predicted.getYM() + dy * resolutionM,
                            predicted.getHeadingRad() + Math.toRadians(angleDeg));
                    int candidateScore = score(candidate, scan);
                    if (candidateScore > bestScore) {
                        bestScore = candidateScore;
                        bestPose = candidate;
                    }
                }
            }
        }
        int gain = bestScore - baseline;
        boolean accepted = validObservationCount(scan) >= 12 && gain >= 12;
        return new ScanMatchResult(accepted ? bestPose : predicted, bestScore, gain, accepted);
    }

    public int getCellByWorld(double xM, double yM) {
        int x = worldToCell(xM);
        int y = worldToCell(yM);
        return inBounds(x, y) ? cells[index(x, y)] : 0;
    }

    public int getOccupiedCellCount() {
        int count = 0;
        for (byte cell : cells) if (cell >= 4) count++;
        return count;
    }

    public int getFreeCellCount() {
        int count = 0;
        for (byte cell : cells) if (cell <= -3) count++;
        return count;
    }

    public int getIntegratedScans() { return integratedScans; }
    public int getSize() { return size; }
    public double getResolutionM() { return resolutionM; }

    private void integrateRay(int x0, int y0, int x1, int y1, boolean hit) {
        int dx = Math.abs(x1 - x0);
        int sx = x0 < x1 ? 1 : -1;
        int dy = -Math.abs(y1 - y0);
        int sy = y0 < y1 ? 1 : -1;
        int error = dx + dy;
        int x = x0;
        int y = y0;
        while (true) {
            if (x == x1 && y == y1) break;
            update(x, y, -1);
            int doubled = 2 * error;
            if (doubled >= dy) { error += dy; x += sx; }
            if (doubled <= dx) { error += dx; y += sy; }
        }
        if (hit) {
            update(x1, y1, 3);
            update(x1 - 1, y1, 1);
            update(x1 + 1, y1, 1);
            update(x1, y1 - 1, 1);
            update(x1, y1 + 1, 1);
        }
    }

    private int score(Pose2d pose, List<LidarObservation> scan) {
        int score = 0;
        int valid = 0;
        for (LidarObservation observation : scan) {
            if (observation.getQuality() < MIN_QUALITY
                    || observation.getDistanceM() <= 0.0
                    || observation.getDistanceM() > MAX_RANGE_M) continue;
            double angle = pose.getHeadingRad() + observation.getAngleRad();
            int endpointX = worldToCell(pose.getXM() + Math.cos(angle) * observation.getDistanceM());
            int endpointY = worldToCell(pose.getYM() + Math.sin(angle) * observation.getDistanceM());
            int localBest = 0;
            for (int ox = -1; ox <= 1; ox++) {
                for (int oy = -1; oy <= 1; oy++) {
                    int x = endpointX + ox;
                    int y = endpointY + oy;
                    if (inBounds(x, y)) localBest = Math.max(localBest, cells[index(x, y)]);
                }
            }
            score += localBest;
            valid++;
        }
        return valid >= 8 ? score : Integer.MIN_VALUE / 4;
    }

    private int validObservationCount(List<LidarObservation> scan) {
        int valid = 0;
        for (LidarObservation observation : scan) {
            if (observation.getQuality() >= MIN_QUALITY
                    && observation.getDistanceM() > 0.0
                    && observation.getDistanceM() <= MAX_RANGE_M) valid++;
        }
        return valid;
    }

    private void update(int x, int y, int delta) {
        if (!inBounds(x, y)) return;
        int offset = index(x, y);
        cells[offset] = (byte) Math.max(MIN_LOG_ODDS,
                Math.min(MAX_LOG_ODDS, cells[offset] + delta));
    }

    private int worldToCell(double coordinateM) {
        return (int) Math.round(coordinateM / resolutionM) + size / 2;
    }

    private boolean inBounds(int x, int y) {
        return x >= 0 && y >= 0 && x < size && y < size;
    }

    private int index(int x, int y) { return y * size + x; }
}

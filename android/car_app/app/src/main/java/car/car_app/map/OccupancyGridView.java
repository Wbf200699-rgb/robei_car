package car.car_app.map;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Path;
import android.util.AttributeSet;
import android.util.Base64;
import android.view.MotionEvent;
import android.view.ScaleGestureDetector;
import android.view.View;

import androidx.annotation.Nullable;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

import car.car_app.bluetooth.LidarMapPoint;
import car.car_app.bluetooth.LidarMapScan;
import car.car_app.bluetooth.OnboardMapChunk;
import car.car_app.bluetooth.OnboardMapStatus;
import car.car_app.bluetooth.RadarFrame;
import car.car_app.bluetooth.StatusFrame;
import car.car_app.geometry.VehicleGeometry;
import car.car_app.navigation.GridPathPlanner;
import car.car_app.navigation.NavigationSteering;

public class OccupancyGridView extends View {
    // Keep the live Android map aligned with OccupancyGridMapper: an odd-sized
    // 241 x 241 grid centered on the vehicle, at 25 mm per cell (~6 m square).
    private static final int GRID_SIZE = 241;
    private static final int GRID_CENTER = GRID_SIZE / 2;
    private static final int RESOLUTION_MM = 25;
    private static final int ONBOARD_GRID_SIZE = 128;
    private static final int ONBOARD_RESOLUTION_MM = 50;
    private static final int INVALID_DISTANCE = 0xFFFF;
    private static final int MAX_RAY_MM = 5000;
    private static final int LIDAR_TEMPORAL_SPREAD_MM = 300;
    private static final int SNAPSHOT_MAGIC = 0x504d4150; // "PMAP"
    private static final int SNAPSHOT_HEADER_BYTES = 44;
    // RPLIDAR angles increase clockwise (front, right-front, right, ...),
    // while the world frame uses mathematical counter-clockwise angles.
    private static final float[] SECTOR_ANGLES = {0f, -45f, -90f, -135f, 180f, 135f, 90f, 45f};

    private final byte[][] occupancy = new byte[GRID_SIZE][GRID_SIZE];
    private final byte[][] onboardOccupancy = new byte[ONBOARD_GRID_SIZE][ONBOARD_GRID_SIZE];
    private final boolean[] onboardChunkSeen = new boolean[ONBOARD_GRID_SIZE];
    private final Paint cellPaint = new Paint();
    private final Paint gridPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint carPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint headingPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint textPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint overlayPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint legendTextPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint scalePaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Path headingPath = new Path();
    private final Path navigationPath = new Path();
    private final ScaleGestureDetector scaleDetector;
    private final GridPathPlanner pathPlanner = new GridPathPlanner();
    private final Paint pathPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint goalPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint vehiclePaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint tagPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint anchorPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint candidatePaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint goalAreaPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint goalAreaBorderPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint goalAreaBlockedPaint = new Paint();
    private final Path vehiclePath = new Path();

    private float carGridX = GRID_CENTER;
    private float carGridY = GRID_CENTER;
    private float headingDegrees = -90f;
    private float scale = 1f;
    private float offsetX;
    private float offsetY;
    private float lastTouchX;
    private float lastTouchY;
    private float downTouchX;
    private float downTouchY;
    private int lastHardwareX = -1;
    private int lastHardwareY = -1;
    private int stepLength = 20;
    private int occupiedCells;
    private int freeCells;
    private int scanCorrections;
    private boolean paused;
    private boolean followCar;
    private boolean fitAnchorFieldPending = true;
    private boolean headingReliable;
    private boolean estimatedPose;
    private boolean motionActive;
    private long lastDetailedScanAtMs;
    private int detailedScanCount;
    private int integratedDetailedPointCount;
    private final int[][] lidarDistanceHistory = new int[3][LidarMapScan.POINT_COUNT];
    private int lidarHistorySlot;
    private int lidarHistoryFrames;
    private boolean onboardMode;
    private int onboardEpoch = -1;
    private int onboardChunkCount;
    private int lastOnboardChunkSequence = -1;
    private OnboardMapStatus onboardMapStatus;
    private float linearMmPerMs = 0.38f;
    private float leftDegreesPerMs = 0.18f;
    private float rightDegreesPerMs = 0.18f;
    private double[][] anchorPositionsM = copyAnchors(VehicleGeometry.ANCHORS_M);
    private String[] anchorDisplayLabels = {"A", "B", "C", "D"};
    private List<GridPathPlanner.Cell> plannedPath = Collections.emptyList();
    private List<GridPathPlanner.Cell> rawPlannedPath = Collections.emptyList();
    private GridPathPlanner.Cell navigationGoal;
    private boolean[] inflatedBlocked;
    private int occupiedClusters;
    private String lastPlanFailure = "";
    private boolean goalSelectionEnabled;
    private boolean[] goalSelectionBlocked;
    private int lastGoalTapX = -1;
    private int lastGoalTapY = -1;
    private boolean lastGoalTapValid;
    private boolean touchMoved;
    private boolean pinchInProgress;
    private GoalSelectionListener goalSelectionListener;
    private boolean worldTransformValid;
    private double worldRotationRad;
    private double worldTranslationXM;
    private double worldTranslationYM;

    public interface GoalSelectionListener {
        void onGoalSelected(boolean pathFound, int goalX, int goalY, int waypointCount);
    }

    public OccupancyGridView(Context context) {
        this(context, null);
    }

    public OccupancyGridView(Context context, @Nullable AttributeSet attrs) {
        super(context, attrs);
        float density = getResources().getDisplayMetrics().density;
        scaleDetector = new ScaleGestureDetector(context, new ScaleGestureDetector.SimpleOnScaleGestureListener() {
            @Override
            public boolean onScaleBegin(ScaleGestureDetector detector) {
                pinchInProgress = true;
                touchMoved = true;
                return true;
            }

            @Override
            public boolean onScale(ScaleGestureDetector detector) {
                fitAnchorFieldPending = false;
                zoomAround(detector.getFocusX(), detector.getFocusY(), detector.getScaleFactor());
                invalidate();
                return true;
            }

            @Override
            public void onScaleEnd(ScaleGestureDetector detector) {
                pinchInProgress = false;
                lastTouchX = detector.getFocusX();
                lastTouchY = detector.getFocusY();
            }
        });
        gridPaint.setColor(Color.rgb(220, 228, 236));
        gridPaint.setStrokeWidth(1f);
        carPaint.setColor(Color.rgb(33, 102, 243));
        headingPaint.setColor(Color.rgb(201, 58, 58));
        headingPaint.setStrokeWidth(4f);
        headingPaint.setStyle(Paint.Style.STROKE);
        textPaint.setColor(Color.rgb(96, 112, 128));
        textPaint.setTextSize(12f * getResources().getDisplayMetrics().scaledDensity);
        legendTextPaint.setColor(Color.rgb(72, 86, 100));
        legendTextPaint.setTextSize(10f * getResources().getDisplayMetrics().scaledDensity);
        scalePaint.setColor(Color.rgb(96, 112, 128));
        scalePaint.setStrokeWidth(2f * getResources().getDisplayMetrics().density);
        pathPaint.setColor(Color.rgb(0, 137, 123));
        pathPaint.setStyle(Paint.Style.STROKE);
        pathPaint.setStrokeWidth(4f * density);
        pathPaint.setStrokeCap(Paint.Cap.ROUND);
        pathPaint.setStrokeJoin(Paint.Join.ROUND);
        goalPaint.setColor(Color.rgb(216, 67, 21));
        goalPaint.setStyle(Paint.Style.STROKE);
        goalPaint.setStrokeWidth(3f * density);
        vehiclePaint.setColor(Color.argb(42, 33, 102, 243));
        vehiclePaint.setStyle(Paint.Style.FILL);
        tagPaint.setColor(Color.rgb(0, 121, 107));
        anchorPaint.setColor(Color.rgb(36, 91, 120));
        candidatePaint.setColor(Color.rgb(164, 171, 177));
        goalAreaPaint.setColor(Color.argb(32, 22, 160, 133));
        goalAreaPaint.setStyle(Paint.Style.FILL);
        goalAreaBorderPaint.setColor(Color.rgb(0, 121, 107));
        goalAreaBorderPaint.setStyle(Paint.Style.STROKE);
        goalAreaBorderPaint.setStrokeWidth(2f * density);
        goalAreaBlockedPaint.setColor(Color.argb(74, 68, 78, 88));
        goalAreaBlockedPaint.setStyle(Paint.Style.FILL);
        setBackgroundColor(Color.WHITE);
    }

    public void setStatusFrame(StatusFrame frame) {
        if (frame == null) return;
        if (onboardMode) return;
        if (frame.getStepLength() > 0) stepLength = frame.getStepLength();
        int x = frame.getNowX();
        int y = frame.getNowY();
        if (lastHardwareX >= 0) {
            int dx = x - lastHardwareX;
            int dy = y - lastHardwareY;
            if (dx != 0 || dy != 0) {
                float cellsPerStep = Math.max(0.5f, stepLength * 10f / RESOLUTION_MM);
                carGridX = clamp(carGridX + dx * cellsPerStep, 1f, GRID_SIZE - 2f);
                carGridY = clamp(carGridY + dy * cellsPerStep, 1f, GRID_SIZE - 2f);
                headingDegrees = NavigationSteering.vehicleHeadingForWorldVector(dx, dy);
                headingReliable = true;
                estimatedPose = false;
            }
        }
        lastHardwareX = x;
        lastHardwareY = y;
        invalidate();
    }

    public void addRadarFrame(RadarFrame frame) {
        if (onboardMode) return;
        if (paused || frame == null || frame.getRadarOnline() != 1) return;
        if (motionActive) return;
        if (System.currentTimeMillis() - lastDetailedScanAtMs <= 2000L) return;
        int[] sectors = frame.getSectorDistancesMm();
        if (occupiedCells >= 18 && estimatedPose) refinePoseWithMap(sectors);
        for (int i = 0; i < sectors.length && i < SECTOR_ANGLES.length; i++) {
            int distance = sectors[i];
            if (distance <= 0 || distance == INVALID_DISTANCE) continue;
            integrateRay(NavigationSteering.toMathematicalHeadingDegrees(headingDegrees)
                    + SECTOR_ANGLES[i], Math.min(distance, MAX_RAY_MM),
                    distance <= MAX_RAY_MM);
        }
        recountCells();
        invalidate();
    }

    /** Immediately records a safety-triggering sector frame before replanning. */
    public void addSafetyObstacleFrame(RadarFrame frame) {
        if (onboardMode || paused || frame == null || frame.getRadarOnline() != 1) return;
        int[] sectors = frame.getSectorDistancesMm();
        int safetyMm = Math.max(150, frame.getSafeDistanceMm());
        for (int i = 0; i < sectors.length && i < SECTOR_ANGLES.length; i++) {
            int distance = sectors[i];
            if (distance <= 0 || distance == INVALID_DISTANCE || distance > safetyMm + 250) continue;
            integrateRay(NavigationSteering.toMathematicalHeadingDegrees(headingDegrees)
                    + SECTOR_ANGLES[i], distance, true);
        }
        recountCells();
        invalidate();
    }

    public void addLidarMapScan(LidarMapScan scan) {
        addLidarMapScan(scan, false);
    }

    /** Integrates a detailed scan while moving only when an absolute pose is fresh. */
    public void addLidarMapScan(LidarMapScan scan, boolean absolutePoseFresh) {
        if (onboardMode) return;
        if (scan == null) return;
        lastDetailedScanAtMs = scan.getReceivedAtMs();
        if (paused || (motionActive && !absolutePoseFresh)) {
            invalidate();
            return;
        }

        java.util.Arrays.fill(lidarDistanceHistory[lidarHistorySlot], 0);
        for (LidarMapPoint point : scan.getPoints()) {
            int bin = point.getBinIndex();
            if (bin >= 0 && bin < LidarMapScan.POINT_COUNT && point.isValid()
                    && point.getQualityClass() >= 2) {
                lidarDistanceHistory[lidarHistorySlot][bin] = point.getDistanceMm();
            }
        }
        lidarHistoryFrames = Math.min(lidarDistanceHistory.length, lidarHistoryFrames + 1);

        int integrated = 0;
        for (LidarMapPoint point : scan.getPoints()) {
            // Low-quality points remain visible on the radar page but are not
            // allowed to permanently alter the occupancy preview.
            if (!point.isValid() || point.getQualityClass() < 2) continue;
            int distance = stableLidarDistance(point.getBinIndex());
            if (distance <= 0) continue;
            int rayDistance = Math.min(distance, MAX_RAY_MM);
            integrateRay(
                    NavigationSteering.toMathematicalHeadingDegrees(headingDegrees)
                            - point.getAngleDegrees(),
                    rayDistance,
                    distance <= MAX_RAY_MM
            );
            integrated++;
        }
        detailedScanCount++;
        integratedDetailedPointCount += integrated;
        lidarHistorySlot = (lidarHistorySlot + 1) % lidarDistanceHistory.length;
        recountCells();
        invalidate();
    }

    /** Returns a median only after at least two recent scans agree in one 5 degree bin. */
    private int stableLidarDistance(int bin) {
        if (bin < 0 || bin >= LidarMapScan.POINT_COUNT || lidarHistoryFrames < 2) return 0;
        int[] values = new int[lidarDistanceHistory.length];
        int count = 0;
        int minimum = Integer.MAX_VALUE;
        int maximum = 0;
        for (int[] history : lidarDistanceHistory) {
            int value = history[bin];
            if (value <= 0) continue;
            values[count++] = value;
            minimum = Math.min(minimum, value);
            maximum = Math.max(maximum, value);
        }
        if (count < 2 || maximum - minimum > LIDAR_TEMPORAL_SPREAD_MM) return 0;
        java.util.Arrays.sort(values, 0, count);
        return values[count / 2];
    }

    private void integrateRay(float angleDegrees, int distanceMm, boolean hasObstacle) {
        double angle = Math.toRadians(angleDegrees);
        float endX = carGridX + (float) Math.cos(angle) * distanceMm / RESOLUTION_MM;
        float endY = carGridY + (float) Math.sin(angle) * distanceMm / RESOLUTION_MM;
        int x0 = Math.round(carGridX);
        int y0 = Math.round(carGridY);
        int x1 = Math.round(endX);
        int y1 = Math.round(endY);
        int dx = Math.abs(x1 - x0);
        int sx = x0 < x1 ? 1 : -1;
        int dy = -Math.abs(y1 - y0);
        int sy = y0 < y1 ? 1 : -1;
        int error = dx + dy;
        int x = x0;
        int y = y0;
        while (true) {
            if (x == x1 && y == y1) break;
            updateCell(x, y, -1);
            int doubled = 2 * error;
            if (doubled >= dy) { error += dy; x += sx; }
            if (doubled <= dx) { error += dx; y += sy; }
        }
        if (hasObstacle) {
            updateCell(x1, y1, 1);
            updateCell(x1 - 1, y1, 1);
            updateCell(x1 + 1, y1, 1);
            updateCell(x1, y1 - 1, 1);
            updateCell(x1, y1 + 1, 1);
        }
    }

    private void updateCell(int x, int y, int delta) {
        if (x < 0 || y < 0 || x >= GRID_SIZE || y >= GRID_SIZE) return;
        // Lidar may observe walls and objects outside the anchor rectangle.
        // Keep those cells in the map; only motion planning is field-bounded.
        int value = occupancy[y][x] + delta;
        occupancy[y][x] = (byte) Math.max(-10, Math.min(10, value));
    }

    public void clearMap() {
        for (int y = 0; y < GRID_SIZE; y++) {
            for (int x = 0; x < GRID_SIZE; x++) occupancy[y][x] = 0;
        }
        carGridX = GRID_CENTER;
        carGridY = GRID_CENTER;
        headingDegrees = -90f;
        lastHardwareX = -1;
        lastHardwareY = -1;
        occupiedCells = 0;
        freeCells = 0;
        scanCorrections = 0;
        headingReliable = false;
        estimatedPose = false;
        motionActive = false;
        lastDetailedScanAtMs = 0L;
        detailedScanCount = 0;
        integratedDetailedPointCount = 0;
        for (int[] history : lidarDistanceHistory) java.util.Arrays.fill(history, 0);
        lidarHistorySlot = 0;
        lidarHistoryFrames = 0;
        clearOnboardCells();
        onboardEpoch = -1;
        onboardMapStatus = null;
        onboardMode = false;
        clearNavigationPath();
        fitAnchorField();
    }

    public void clearOnboardPreview() {
        clearOnboardCells();
        onboardMode = true;
        invalidate();
    }

    private void clearOnboardCells() {
        for (int y = 0; y < ONBOARD_GRID_SIZE; y++) {
            java.util.Arrays.fill(onboardOccupancy[y], (byte) 0);
        }
        java.util.Arrays.fill(onboardChunkSeen, false);
        onboardChunkCount = 0;
        lastOnboardChunkSequence = -1;
    }

    public void applyOnboardMapStatus(OnboardMapStatus status) {
        applyOnboardMapStatus(status, true);
    }

    /** Updates FPGA map metadata and, when requested, its matching local pose. */
    public void applyOnboardMapStatus(OnboardMapStatus status, boolean applyPose) {
        if (status == null) return;
        onboardMapStatus = status;
        onboardEpoch = status.getEpoch();
        paused = status.isMapPaused();
        if (applyPose) {
            carGridX = clamp(status.getPoseXmm() / (float) ONBOARD_RESOLUTION_MM,
                    1f, ONBOARD_GRID_SIZE - 2f);
            carGridY = clamp(status.getPoseYmm() / (float) ONBOARD_RESOLUTION_MM,
                    1f, ONBOARD_GRID_SIZE - 2f);
            headingDegrees = status.getHeadingDegrees();
            headingReliable = (status.getPoseFlags() & 0x03) == 0x03;
            estimatedPose = !headingReliable;
            onboardMode = true;
        }
        invalidate();
    }

    /** Applies FPGA UWB/IMU truth without replacing the phone-owned map. */
    public void applyAbsolutePose(int xMm, int yMm, float heading, boolean reliable) {
        float fieldCenterXmm = (float) (anchorCenterX() * 1000.0);
        float fieldCenterYmm = (float) (anchorCenterY() * 1000.0);
        carGridX = clamp(GRID_CENTER + (xMm - fieldCenterXmm) / RESOLUTION_MM,
                1f, GRID_SIZE - 2f);
        carGridY = clamp(GRID_CENTER + (yMm - fieldCenterYmm) / RESOLUTION_MM,
                1f, GRID_SIZE - 2f);
        headingDegrees = heading;
        headingReliable = reliable;
        estimatedPose = !headingReliable;
        onboardMode = false;
        invalidate();
    }

    /** Overrides only the displayed FPGA-map pose with UWB position + IMU heading. */
    public void applyOnboardFusedPose(int localXmm, int localYmm, float imuHeading,
                                      boolean reliable) {
        carGridX = clamp(localXmm / (float) ONBOARD_RESOLUTION_MM,
                1f, ONBOARD_GRID_SIZE - 2f);
        carGridY = clamp(localYmm / (float) ONBOARD_RESOLUTION_MM,
                1f, ONBOARD_GRID_SIZE - 2f);
        headingDegrees = imuHeading;
        headingReliable = reliable;
        estimatedPose = !reliable;
        onboardMode = true;
        invalidate();
    }

    /** Serializes the phone-owned occupancy map and pose for fast iteration. */
    public String exportPhoneMapSnapshot() {
        ByteBuffer buffer = ByteBuffer.allocate(
                SNAPSHOT_HEADER_BYTES + GRID_SIZE * GRID_SIZE).order(ByteOrder.BIG_ENDIAN);
        buffer.putInt(SNAPSHOT_MAGIC);
        buffer.putInt(GRID_SIZE);
        buffer.putInt(RESOLUTION_MM);
        buffer.putFloat(carGridX);
        buffer.putFloat(carGridY);
        buffer.putFloat(headingDegrees);
        buffer.putInt(occupiedCells);
        buffer.putInt(freeCells);
        buffer.putInt(detailedScanCount);
        buffer.putInt(integratedDetailedPointCount);
        buffer.putInt((headingReliable ? 1 : 0) | (estimatedPose ? 2 : 0));
        for (int y = 0; y < GRID_SIZE; y++) buffer.put(occupancy[y]);
        return Base64.encodeToString(buffer.array(), Base64.NO_WRAP);
    }

    /** Restores a validated phone map snapshot; malformed data is rejected. */
    public boolean importPhoneMapSnapshot(String encoded) {
        if (encoded == null || encoded.isEmpty()) return false;
        try {
            byte[] data = Base64.decode(encoded, Base64.DEFAULT);
            if (data.length != SNAPSHOT_HEADER_BYTES + GRID_SIZE * GRID_SIZE) return false;
            ByteBuffer buffer = ByteBuffer.wrap(data).order(ByteOrder.BIG_ENDIAN);
            if (buffer.getInt() != SNAPSHOT_MAGIC || buffer.getInt() != GRID_SIZE
                    || buffer.getInt() != RESOLUTION_MM) return false;
            carGridX = clamp(buffer.getFloat(), 1f, GRID_SIZE - 2f);
            carGridY = clamp(buffer.getFloat(), 1f, GRID_SIZE - 2f);
            headingDegrees = buffer.getFloat();
            occupiedCells = buffer.getInt();
            freeCells = buffer.getInt();
            detailedScanCount = buffer.getInt();
            integratedDetailedPointCount = buffer.getInt();
            int flags = buffer.getInt();
            headingReliable = (flags & 1) != 0;
            estimatedPose = (flags & 2) != 0;
            for (int y = 0; y < GRID_SIZE; y++) buffer.get(occupancy[y]);
            onboardMode = false;
            paused = true;
            centerOnCar();
            invalidate();
            return true;
        } catch (IllegalArgumentException | java.nio.BufferUnderflowException error) {
            return false;
        }
    }

    public boolean applyOnboardMapChunk(OnboardMapChunk chunk) {
        if (chunk == null) return false;
        int base = chunk.getBaseAddress();
        byte[] cells = chunk.getCellClasses();
        if (base < 0 || base + cells.length > ONBOARD_GRID_SIZE * ONBOARD_GRID_SIZE) {
            return false;
        }
        onboardMode = true;
        for (int offset = 0; offset < cells.length; offset++) {
            int address = base + offset;
            onboardOccupancy[address / ONBOARD_GRID_SIZE][address % ONBOARD_GRID_SIZE] = cells[offset];
        }
        int chunkIndex = base / OnboardMapChunk.CELL_COUNT;
        if (chunkIndex >= 0 && chunkIndex < onboardChunkSeen.length
                && !onboardChunkSeen[chunkIndex]) {
            onboardChunkSeen[chunkIndex] = true;
            onboardChunkCount++;
        }
        lastOnboardChunkSequence = chunk.getSequence();
        invalidate();
        return true;
    }

    public void centerOnCar() {
        followCar = true;
        fitAnchorFieldPending = false;
        offsetX = 0f;
        offsetY = 0f;
        invalidate();
    }

    /** Shows the whole anchor rectangle while retaining map cells outside it. */
    public void fitAnchorField() {
        followCar = false;
        fitAnchorFieldPending = true;
        offsetX = 0f;
        offsetY = 0f;
        invalidate();
    }

    public void setPaused(boolean paused) {
        this.paused = paused;
        invalidate();
    }

    public boolean isPaused() { return paused; }
    public boolean isHeadingReliable() { return headingReliable; }
    public boolean isEstimatedPose() { return estimatedPose; }
    public int getOccupiedCells() {
        return onboardMode && onboardMapStatus != null
                ? onboardMapStatus.getOccupiedCount() : occupiedCells;
    }
    public int getFreeCells() {
        return onboardMode && onboardMapStatus != null
                ? onboardMapStatus.getFreeCount() : freeCells;
    }
    public int getScanCorrections() { return scanCorrections; }
    public int getResolutionMm() { return onboardMode ? ONBOARD_RESOLUTION_MM : RESOLUTION_MM; }
    public int getDetailedScanCount() {
        return onboardMode && onboardMapStatus != null
                ? onboardMapStatus.getScanCount() : detailedScanCount;
    }
    public int getIntegratedDetailedPointCount() { return integratedDetailedPointCount; }
    public boolean isOnboardMode() { return onboardMode; }
    public int getOnboardEpoch() { return onboardEpoch; }
    public int getOnboardChunkCount() { return onboardChunkCount; }
    public int getLastOnboardChunkSequence() { return lastOnboardChunkSequence; }
    public OnboardMapStatus getOnboardMapStatus() { return onboardMapStatus; }
    public float getCarGridX() { return carGridX; }
    public float getCarGridY() { return carGridY; }
    public float getHeadingDegrees() { return headingDegrees; }

    /**
     * Checks a predicted command against the anchor hull after shrinking it by
     * the vehicle circumradius plus 75 mm. Mapping remains unrestricted.
     */
    public boolean canExecuteMotionWithinAnchorBoundary(char command, int durationMs) {
        double worldX = gridToWorldXM(Math.round(carGridX));
        double worldY = gridToWorldYM(Math.round(carGridY));
        double currentClearance = signedAnchorBoundaryClearance(worldX, worldY);
        if (command == 'W' || command == 'S') {
            double direction = command == 'W' ? 1.0 : -1.0;
            double distanceM = Math.max(0, durationMs) * linearMmPerMs / 1000.0;
            double heading = Math.toRadians(
                    NavigationSteering.toMathematicalHeadingDegrees(headingDegrees));
            worldX += Math.cos(heading) * distanceM * direction;
            worldY += Math.sin(heading) * distanceM * direction;
        }
        double predictedClearance = signedAnchorBoundaryClearance(worldX, worldY);
        double requiredClearance = vehicleBoundaryMarginM();
        if (predictedClearance >= requiredClearance) return true;
        // If UWB starts just outside the inset, permit only a command that
        // monotonically increases clearance. All sideways/outward commands
        // remain blocked until the whole vehicle is back in the safe region.
        return currentClearance < requiredClearance
                && predictedClearance > currentClearance + 0.002;
    }

    public float getHeadingErrorToAnchorCenter() {
        double worldX = gridToWorldXM(Math.round(carGridX));
        double worldY = gridToWorldYM(Math.round(carGridY));
        float target = NavigationSteering.vehicleHeadingForWorldVector(
                anchorCenterX() - worldX, anchorCenterY() - worldY);
        return NavigationSteering.normalizeHeadingError(target - headingDegrees);
    }
    public List<GridPathPlanner.Cell> getPlannedPath() {
        return new ArrayList<>(plannedPath);
    }
    public List<GridPathPlanner.Cell> getRawPlannedPath() {
        return new ArrayList<>(rawPlannedPath);
    }
    public GridPathPlanner.Cell getNavigationGoal() { return navigationGoal; }
    public String getLastPlanFailure() { return lastPlanFailure; }
    public int getOccupiedClusters() { return occupiedClusters; }
    public int getDroppedSampleCount() {
        return onboardMode && onboardMapStatus != null
                ? onboardMapStatus.getDroppedCount() : 0;
    }

    /** Installs the rigid transform world = R * FPGA-map + translation. */
    public void setWorldTransform(double rotationRad, double translationXM,
                                  double translationYM, boolean valid) {
        worldRotationRad = rotationRad;
        worldTranslationXM = translationXM;
        worldTranslationYM = translationYM;
        worldTransformValid = valid;
        invalidate();
    }

    public void setGoalSelectionListener(GoalSelectionListener listener) {
        goalSelectionListener = listener;
    }

    public void setGoalSelectionEnabled(boolean enabled) {
        goalSelectionEnabled = enabled;
        lastGoalTapX = -1;
        lastGoalTapY = -1;
        if (enabled && !onboardMode) {
            refreshGoalSelectionMask();
        } else {
            goalSelectionBlocked = null;
        }
        invalidate();
    }

    public boolean isGoalSelectionEnabled() { return goalSelectionEnabled; }

    public boolean planPathTo(int goalX, int goalY) {
        lastPlanFailure = "";
        if (onboardMode) {
            lastPlanFailure = "当前为板载地图预览，尚未同步手机导航地图";
            return false;
        }
        if (!isInsideAnchorBoundaryWorld(gridToWorldXM(goalX), gridToWorldYM(goalY),
                vehicleBoundaryMarginM())) {
            lastPlanFailure = "目标点过于靠近基站边界，车体无法保持安全间距";
            return false;
        }
        navigationGoal = new GridPathPlanner.Cell(goalX, goalY);
        byte[] flat = new byte[GRID_SIZE * GRID_SIZE];
        for (int y = 0; y < GRID_SIZE; y++) {
            System.arraycopy(occupancy[y], 0, flat, y * GRID_SIZE, GRID_SIZE);
        }
        boolean[] allowed = buildAllowedMask();
        int vehicleRadiusCells = Math.max(1, (int) Math.ceil(
                (Math.hypot(VehicleGeometry.TAG_RECTANGLE_LENGTH_M * 0.5,
                        VehicleGeometry.TAG_RECTANGLE_WIDTH_M * 0.5) + 0.075)
                        * 1000.0 / RESOLUTION_MM));
        GridPathPlanner.Result result = pathPlanner.plan(
                flat, GRID_SIZE, GRID_SIZE,
                new GridPathPlanner.Cell(Math.round(carGridX), Math.round(carGridY)),
                navigationGoal, allowed, 3, -2, 2, vehicleRadiusCells);
        plannedPath = result.getPath();
        rawPlannedPath = result.getRawPath();
        inflatedBlocked = result.getBlocked();
        occupiedClusters = result.getOccupiedClusters();
        if (!result.isFound()) {
            switch (result.getFailureReason()) {
                case START_NOT_RECOVERABLE:
                    lastPlanFailure = "当前UWB位置距离安全区域过远，请检查基站布局或先移回场内";
                    break;
                case GOAL_NOT_OPEN:
                    lastPlanFailure = "目标点及其附近被障碍安全区占用";
                    break;
                default:
                    lastPlanFailure = "障碍物或安全边界封闭了可行路径";
                    break;
            }
        }
        invalidate();
        return result.isFound();
    }

    public boolean replanCurrentGoal() {
        return navigationGoal != null && planPathTo(navigationGoal.x, navigationGoal.y);
    }

    public void clearNavigationPath() {
        plannedPath = Collections.emptyList();
        rawPlannedPath = Collections.emptyList();
        navigationGoal = null;
        inflatedBlocked = null;
        occupiedClusters = 0;
        invalidate();
    }

    public double gridToWorldXM(int gridX) {
        return anchorCenterX() + (gridX - GRID_CENTER) * RESOLUTION_MM / 1000.0;
    }

    public double gridToWorldYM(int gridY) {
        return anchorCenterY() + (gridY - GRID_CENTER) * RESOLUTION_MM / 1000.0;
    }

    public void setAnchorPositions(double[][] anchors) {
        if (anchors == null || anchors.length != 4) throw new IllegalArgumentException("four anchors required");
        anchorPositionsM = copyAnchors(anchors);
        clearNavigationPath();
        invalidate();
    }

    public void setAnchorDisplayLabels(String[] labels) {
        if (labels == null || labels.length != 4) {
            throw new IllegalArgumentException("four anchor labels required");
        }
        anchorDisplayLabels = java.util.Arrays.copyOf(labels, labels.length);
        invalidate();
    }

    public double[][] getAnchorPositions() { return copyAnchors(anchorPositionsM); }

    public void setMotionActive(boolean motionActive) {
        this.motionActive = motionActive;
        invalidate();
    }

    public void applyEstimatedMotion(char command, int durationMs) {
        if (onboardMode) return;
        if (command == 'A' || command == 'R') {
            float turnDegrees = Math.max(1f, durationMs
                    * (command == 'A' ? leftDegreesPerMs : rightDegreesPerMs));
            headingDegrees += NavigationSteering.signedTurnDegrees(command, turnDegrees);
        } else if (command == 'W' || command == 'S') {
            float distanceMm = durationMs * linearMmPerMs;
            float direction = command == 'W' ? 1f : -1f;
            double heading = Math.toRadians(
                    NavigationSteering.toMathematicalHeadingDegrees(headingDegrees));
            carGridX = clamp(carGridX + (float) Math.cos(heading) * distanceMm / RESOLUTION_MM * direction,
                    1f, GRID_SIZE - 2f);
            carGridY = clamp(carGridY + (float) Math.sin(heading) * distanceMm / RESOLUTION_MM * direction,
                    1f, GRID_SIZE - 2f);
        }
        estimatedPose = true;
        headingReliable = false;
        invalidate();
    }

    public void setMotionCalibration(float linearMmPerMs, float leftDegreesPerMs,
                                     float rightDegreesPerMs) {
        if (linearMmPerMs > 0f) this.linearMmPerMs = linearMmPerMs;
        if (leftDegreesPerMs > 0f) this.leftDegreesPerMs = leftDegreesPerMs;
        if (rightDegreesPerMs > 0f) this.rightDegreesPerMs = rightDegreesPerMs;
    }

    private void recountCells() {
        int occupied = 0;
        int free = 0;
        for (byte[] row : occupancy) {
            for (byte value : row) {
                if (value >= 3) occupied++;
                else if (value <= -2) free++;
            }
        }
        occupiedCells = occupied;
        freeCells = free;
    }

    private void refinePoseWithMap(int[] sectors) {
        int currentScore = scorePose(carGridX, carGridY, headingDegrees, sectors);
        int bestScore = currentScore;
        float bestX = carGridX;
        float bestY = carGridY;
        float bestHeading = headingDegrees;
        for (int dx = -2; dx <= 2; dx++) {
            for (int dy = -2; dy <= 2; dy++) {
                for (int angle = -20; angle <= 20; angle += 2) {
                    int score = scorePose(carGridX + dx, carGridY + dy,
                            headingDegrees + angle, sectors);
                    if (score > bestScore) {
                        bestScore = score;
                        bestX = carGridX + dx;
                        bestY = carGridY + dy;
                        bestHeading = headingDegrees + angle;
                    }
                }
            }
        }
        if (bestScore >= currentScore + 6) {
            carGridX = clamp(bestX, 1f, GRID_SIZE - 2f);
            carGridY = clamp(bestY, 1f, GRID_SIZE - 2f);
            headingDegrees = bestHeading;
            scanCorrections++;
        }
    }

    private int scorePose(float poseX, float poseY, float poseHeading, int[] sectors) {
        int score = 0;
        int valid = 0;
        for (int i = 0; i < sectors.length && i < SECTOR_ANGLES.length; i++) {
            int distance = sectors[i];
            if (distance <= 0 || distance == INVALID_DISTANCE || distance > MAX_RAY_MM) continue;
            double angle = Math.toRadians(
                    NavigationSteering.toMathematicalHeadingDegrees(poseHeading)
                            + SECTOR_ANGLES[i]);
            int x = Math.round(poseX + (float) Math.cos(angle) * distance / RESOLUTION_MM);
            int y = Math.round(poseY + (float) Math.sin(angle) * distance / RESOLUTION_MM);
            int localBest = 0;
            for (int ox = -1; ox <= 1; ox++) {
                for (int oy = -1; oy <= 1; oy++) {
                    int px = x + ox;
                    int py = y + oy;
                    if (px >= 0 && py >= 0 && px < GRID_SIZE && py < GRID_SIZE) {
                        localBest = Math.max(localBest, occupancy[py][px]);
                    }
                }
            }
            score += Math.max(0, localBest);
            valid++;
        }
        return valid >= 3 ? score : Integer.MIN_VALUE / 4;
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);
        int displayGridSize = onboardMode ? ONBOARD_GRID_SIZE : GRID_SIZE;
        int displayGridCenter = displayGridSize / 2;
        float baseCell = Math.min(getWidth(), getHeight()) / 34f;
        if (fitAnchorFieldPending && baseCell > 0f) {
            double widthCells;
            double heightCells;
            if (onboardMode) {
                widthCells = ONBOARD_GRID_SIZE;
                heightCells = ONBOARD_GRID_SIZE;
            } else {
                double minimumX = Double.POSITIVE_INFINITY;
                double maximumX = Double.NEGATIVE_INFINITY;
                double minimumY = Double.POSITIVE_INFINITY;
                double maximumY = Double.NEGATIVE_INFINITY;
                for (double[] anchor : anchorPositionsM) {
                    minimumX = Math.min(minimumX, anchor[0]);
                    maximumX = Math.max(maximumX, anchor[0]);
                    minimumY = Math.min(minimumY, anchor[1]);
                    maximumY = Math.max(maximumY, anchor[1]);
                }
                // Keep 20 cm outside the anchors visible so lidar observations
                // just beyond the driving boundary remain inspectable.
                widthCells = (maximumX - minimumX + 0.40) * 1000.0 / RESOLUTION_MM;
                heightCells = (maximumY - minimumY + 0.40) * 1000.0 / RESOLUTION_MM;
            }
            scale = clamp((float) Math.min(
                    getWidth() * 0.94 / (widthCells * baseCell),
                    getHeight() * 0.90 / (heightCells * baseCell)), 0.08f, 6.0f);
            fitAnchorFieldPending = false;
        }
        float cell = baseCell * scale;
        float originX = getWidth() / 2f
                - (followCar ? carGridX : displayGridCenter) * cell + offsetX;
        // Grid +Y is a mathematical/world upward axis. Android screen +Y is
        // downward, so the previous implementation mirrored the map against
        // the UWB localization page.
        float originY = getHeight() / 2f
                + (followCar ? carGridY : displayGridCenter) * cell + offsetY;
        int minX = Math.max(0, (int) Math.floor(-originX / cell) - 1);
        int maxX = Math.min(displayGridSize - 1,
                (int) Math.ceil((getWidth() - originX) / cell) + 1);
        int minY = Math.max(0,
                (int) Math.floor((originY - getHeight()) / cell) - 1);
        int maxY = Math.min(displayGridSize - 1,
                (int) Math.ceil(originY / cell) + 1);

        for (int y = minY; y <= maxY; y++) {
            for (int x = minX; x <= maxX; x++) {
                byte value = onboardMode ? onboardOccupancy[y][x] : occupancy[y][x];
                boolean bridge = onboardMode && value == 0 && isOnboardWallBridge(x, y);
                if (value == 0 && !bridge) continue;
                boolean occupiedCell = onboardMode ? value == 2 : value >= 3;
                boolean freeCell = onboardMode ? value == 1 : value <= -2;
                if (bridge) {
                    cellPaint.setColor(Color.rgb(92, 92, 92));
                } else if (occupiedCell) {
                    boolean supported = !onboardMode || hasOnboardOccupiedNeighbor(x, y);
                    int shade = onboardMode ? (supported ? 54 : 164)
                            : Math.max(30, 150 - value * 10);
                    cellPaint.setColor(Color.rgb(shade, shade, shade));
                } else if (freeCell) {
                    cellPaint.setColor(Color.rgb(230, 245, 239));
                } else continue;
                canvas.drawRect(originX + x * cell, originY - (y + 1) * cell,
                        originX + (x + 1) * cell, originY - y * cell, cellPaint);
            }
        }

        if (goalSelectionEnabled && !onboardMode) {
            drawGoalSelectionArea(canvas, originX, originY, cell, minX, maxX, minY, maxY);
        }

        if (cell >= 8f) {
            for (int x = minX; x <= maxX + 1; x++) {
                float px = originX + x * cell;
                canvas.drawLine(px, 0, px, getHeight(), gridPaint);
            }
            for (int y = minY; y <= maxY + 1; y++) {
                float py = originY - y * cell;
                canvas.drawLine(0, py, getWidth(), py, gridPaint);
            }
        }

        float carX = originX + (carGridX + 0.5f) * cell;
        float carY = originY - (carGridY + 0.5f) * cell;
        float radius = Math.max(6f, cell * 0.42f);
        drawWorldAnchors(canvas, originX, originY, cell);
        // Vehicle/control headings increase on a physical clockwise turn.
        // Android's world-to-canvas rotation math is counter-clockwise positive,
        // so convert only at the rendering boundary. Mapping and navigation keep
        // the established vehicle convention.
        float canvasHeadingDegrees =
                NavigationSteering.toCanvasHeadingDegrees(headingDegrees);
        drawVehicleAndTags(canvas, originX, originY, cell, carGridX, carGridY,
                canvasHeadingDegrees);
        double heading = Math.toRadians(canvasHeadingDegrees);
        headingPath.reset();
        headingPath.moveTo(carX, carY);
        headingPath.lineTo(carX + (float) Math.cos(heading) * radius * 2.2f,
                carY - (float) Math.sin(heading) * radius * 2.2f);
        canvas.drawPath(headingPath, headingPaint);
        drawNavigation(canvas, originX, originY, cell);
        drawScaleAndLegend(canvas);
    }

    /** Shows the same safety mask used by path planning while a destination is being selected. */
    private void drawGoalSelectionArea(Canvas canvas, float originX, float originY, float cell,
                                       int minX, int maxX, int minY, int maxY) {
        if (goalSelectionBlocked == null || goalSelectionBlocked.length != GRID_SIZE * GRID_SIZE) {
            refreshGoalSelectionMask();
        }
        if (goalSelectionBlocked == null) return;

        for (int y = minY; y <= maxY; y++) {
            for (int x = minX; x <= maxX; x++) {
                int index = y * GRID_SIZE + x;
                Paint fill = goalSelectionBlocked[index] ? goalAreaBlockedPaint : goalAreaPaint;
                float left = originX + x * cell;
                float top = originY - (y + 1) * cell;
                canvas.drawRect(left, top, left + cell, top + cell, fill);
                if (goalSelectionBlocked[index]) continue;

                float right = left + cell;
                float bottom = top + cell;
                if (x == 0 || goalSelectionBlocked[index - 1])
                    canvas.drawLine(left, top, left, bottom, goalAreaBorderPaint);
                if (x == GRID_SIZE - 1 || goalSelectionBlocked[index + 1])
                    canvas.drawLine(right, top, right, bottom, goalAreaBorderPaint);
                if (y == 0 || goalSelectionBlocked[index - GRID_SIZE])
                    canvas.drawLine(left, bottom, right, bottom, goalAreaBorderPaint);
                if (y == GRID_SIZE - 1 || goalSelectionBlocked[index + GRID_SIZE])
                    canvas.drawLine(left, top, right, top, goalAreaBorderPaint);
            }
        }

        if (lastGoalTapX >= 0 && lastGoalTapY >= 0) {
            float x = originX + (lastGoalTapX + 0.5f) * cell;
            float y = originY - (lastGoalTapY + 0.5f) * cell;
            candidatePaint.setStyle(Paint.Style.STROKE);
            candidatePaint.setStrokeWidth(Math.max(3f, cell * 0.18f));
            candidatePaint.setColor(lastGoalTapValid
                    ? Color.rgb(0, 121, 107) : Color.rgb(214, 69, 69));
            canvas.drawCircle(x, y, Math.max(8f, cell * 0.72f), candidatePaint);
        }

        float density = getResources().getDisplayMetrics().density;
        overlayPaint.setColor(Color.argb(235, 255, 255, 255));
        canvas.drawRoundRect(10f * density, 10f * density, 150f * density, 40f * density,
                6f * density, 6f * density, overlayPaint);
        canvas.drawText("绿色框内可选择目的地", 18f * density, 30f * density, legendTextPaint);
    }

    private void refreshGoalSelectionMask() {
        byte[] flat = new byte[GRID_SIZE * GRID_SIZE];
        for (int y = 0; y < GRID_SIZE; y++) {
            System.arraycopy(occupancy[y], 0, flat, y * GRID_SIZE, GRID_SIZE);
        }
        int vehicleRadiusCells = Math.max(1, (int) Math.ceil(
                vehicleBoundaryMarginM() * 1000.0 / RESOLUTION_MM));
        GridPathPlanner.Result preview = pathPlanner.plan(
                flat, GRID_SIZE, GRID_SIZE,
                new GridPathPlanner.Cell(Math.round(carGridX), Math.round(carGridY)),
                new GridPathPlanner.Cell(GRID_CENTER, GRID_CENTER),
                buildAllowedMask(), 3, -2, 2, vehicleRadiusCells);
        goalSelectionBlocked = preview.getBlocked();
    }

    private void drawNavigation(Canvas canvas, float originX, float originY, float cell) {
        if (!plannedPath.isEmpty()) {
            navigationPath.reset();
            for (int index = 0; index < plannedPath.size(); index++) {
                GridPathPlanner.Cell point = plannedPath.get(index);
                float x = originX + (point.x + 0.5f) * cell;
                float y = originY - (point.y + 0.5f) * cell;
                if (index == 0) navigationPath.moveTo(x, y); else navigationPath.lineTo(x, y);
            }
            canvas.drawPath(navigationPath, pathPaint);
        }
        if (navigationGoal != null) {
            float x = originX + (navigationGoal.x + 0.5f) * cell;
            float y = originY - (navigationGoal.y + 0.5f) * cell;
            float radius = Math.max(8f, cell * 0.7f);
            canvas.drawCircle(x, y, radius, goalPaint);
            canvas.drawLine(x - radius, y, x + radius, y, goalPaint);
            canvas.drawLine(x, y - radius, x, y + radius, goalPaint);
        }
    }

    private boolean hasOnboardOccupiedNeighbor(int x, int y) {
        for (int oy = -1; oy <= 1; oy++) {
            for (int ox = -1; ox <= 1; ox++) {
                if ((ox == 0 && oy == 0) || x + ox < 0 || y + oy < 0
                        || x + ox >= ONBOARD_GRID_SIZE || y + oy >= ONBOARD_GRID_SIZE) continue;
                if (onboardOccupancy[y + oy][x + ox] == 2) return true;
            }
        }
        return false;
    }

    /** Visually closes only a one-cell gap between two confirmed endpoints. */
    private boolean isOnboardWallBridge(int x, int y) {
        if (x <= 0 || y <= 0 || x >= ONBOARD_GRID_SIZE - 1
                || y >= ONBOARD_GRID_SIZE - 1) return false;
        return (onboardOccupancy[y][x - 1] == 2 && onboardOccupancy[y][x + 1] == 2)
                || (onboardOccupancy[y - 1][x] == 2 && onboardOccupancy[y + 1][x] == 2);
    }

    private void drawWorldAnchors(Canvas canvas, float originX, float originY, float cell) {
        for (int index = 0; index < anchorPositionsM.length; index++) {
            double[] grid = worldToDisplayGrid(anchorPositionsM[index][0], anchorPositionsM[index][1]);
            if (grid == null) continue;
            float x = originX + ((float) grid[0] + 0.5f) * cell;
            float y = originY - ((float) grid[1] + 0.5f) * cell;
            float radius = Math.max(5f, cell * 0.30f);
            canvas.drawCircle(x, y, radius, anchorPaint);
            canvas.drawText(anchorDisplayLabels[index],
                    x + radius + 2f, y - radius, legendTextPaint);
        }
    }

    private double[] worldToDisplayGrid(double worldXM, double worldYM) {
        if (!onboardMode) {
            return new double[]{
                    GRID_CENTER + (worldXM - anchorCenterX()) * 1000.0 / RESOLUTION_MM,
                    GRID_CENTER + (worldYM - anchorCenterY()) * 1000.0 / RESOLUTION_MM
            };
        }
        if (!worldTransformValid) return null;
        double dx = worldXM - worldTranslationXM;
        double dy = worldYM - worldTranslationYM;
        double cos = Math.cos(worldRotationRad);
        double sin = Math.sin(worldRotationRad);
        return new double[]{
                (cos * dx + sin * dy) * 1000.0 / ONBOARD_RESOLUTION_MM,
                (-sin * dx + cos * dy) * 1000.0 / ONBOARD_RESOLUTION_MM
        };
    }

    private void drawVehicleAndTags(Canvas canvas, float originX, float originY,
                                    float cell, float centerGridX, float centerGridY,
                                    float vehicleHeadingDegrees) {
        int[] order = {1, 2, 3, 0};
        double heading = Math.toRadians(vehicleHeadingDegrees);
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);
        double resolutionM = (onboardMode ? ONBOARD_RESOLUTION_MM : RESOLUTION_MM) / 1000.0;
        vehiclePath.reset();
        for (int index = 0; index < order.length; index++) {
            double[] offset = VehicleGeometry.TAG_POSITIONS_M[order[index]];
            float gridX = centerGridX + (float) ((cos * offset[0] - sin * offset[1]) / resolutionM);
            float gridY = centerGridY + (float) ((sin * offset[0] + cos * offset[1]) / resolutionM);
            float x = originX + (gridX + 0.5f) * cell;
            float y = originY - (gridY + 0.5f) * cell;
            if (index == 0) vehiclePath.moveTo(x, y); else vehiclePath.lineTo(x, y);
        }
        vehiclePath.close();
        canvas.drawPath(vehiclePath, vehiclePaint);

        for (int tagId : order) {
            double[] offset = VehicleGeometry.TAG_POSITIONS_M[tagId];
            float gridX = centerGridX + (float) ((cos * offset[0] - sin * offset[1]) / resolutionM);
            float gridY = centerGridY + (float) ((sin * offset[0] + cos * offset[1]) / resolutionM);
            float x = originX + (gridX + 0.5f) * cell;
            float y = originY - (gridY + 0.5f) * cell;
            float radius = Math.max(4f, cell * 0.20f);
            canvas.drawCircle(x, y, radius, tagPaint);
            canvas.drawText("T" + tagId, x + radius + 2f, y - radius, legendTextPaint);
        }
    }

    private void drawScaleAndLegend(Canvas canvas) {
        float density = getResources().getDisplayMetrics().density;
        float bottom = getHeight() - 14f * density;
        float scaleStart = 12f * density;
        float scaleEnd = scaleStart + 34f * density;
        float scaleY = bottom - 5f * density;
        canvas.drawLine(scaleStart, scaleY, scaleEnd, scaleY, scalePaint);
        canvas.drawLine(scaleStart, scaleY - 4f * density, scaleStart, scaleY + 4f * density, scalePaint);
        canvas.drawLine(scaleEnd, scaleY - 4f * density, scaleEnd, scaleY + 4f * density, scalePaint);
        canvas.drawText(paused ? "已暂停" : getResolutionMm() + "mm",
                scaleEnd + 7f * density, bottom, textPaint);

        float width = 142f * density;
        float height = 34f * density;
        float left = getWidth() - width - 8f * density;
        float top = getHeight() - height - 8f * density;
        overlayPaint.setColor(Color.argb(232, 255, 255, 255));
        canvas.drawRoundRect(left, top, left + width, top + height,
                6f * density, 6f * density, overlayPaint);

        float centerY = top + height / 2f;
        drawLegendItem(canvas, left + 10f * density, centerY,
                Color.rgb(55, 55, 55), "障碍", false);
        drawLegendItem(canvas, left + 57f * density, centerY,
                Color.rgb(230, 245, 239), "空闲", false);
        drawLegendItem(canvas, left + 104f * density, centerY,
                Color.rgb(33, 102, 243), "车辆", true);
    }

    private void drawLegendItem(Canvas canvas, float x, float centerY,
                                int color, String label, boolean circle) {
        float density = getResources().getDisplayMetrics().density;
        overlayPaint.setColor(color);
        if (circle) {
            canvas.drawCircle(x + 4f * density, centerY, 4f * density, overlayPaint);
        } else {
            canvas.drawRect(x, centerY - 4f * density, x + 8f * density,
                    centerY + 4f * density, overlayPaint);
        }
        canvas.drawText(label, x + 11f * density,
                centerY + 3.5f * density, legendTextPaint);
    }

    @Override
    public boolean onTouchEvent(MotionEvent event) {
        scaleDetector.onTouchEvent(event);
        if (pinchInProgress || event.getPointerCount() > 1) {
            touchMoved = true;
            if (event.getPointerCount() > 0) {
                int pointerIndex = event.getActionMasked() == MotionEvent.ACTION_POINTER_UP
                        && event.getActionIndex() == 0 && event.getPointerCount() > 1 ? 1 : 0;
                lastTouchX = event.getX(pointerIndex);
                lastTouchY = event.getY(pointerIndex);
            }
            return true;
        }
        if (event.getAction() == MotionEvent.ACTION_DOWN) {
            lastTouchX = event.getX();
            lastTouchY = event.getY();
            downTouchX = lastTouchX;
            downTouchY = lastTouchY;
            touchMoved = false;
            return true;
        }
        if (event.getAction() == MotionEvent.ACTION_MOVE) {
            if (Math.hypot(event.getX() - downTouchX, event.getY() - downTouchY) > 6f) {
                touchMoved = true;
            }
            followCar = false;
            fitAnchorFieldPending = false;
            offsetX += event.getX() - lastTouchX;
            offsetY += event.getY() - lastTouchY;
            lastTouchX = event.getX();
            lastTouchY = event.getY();
            invalidate();
            return true;
        }
        if (event.getAction() == MotionEvent.ACTION_UP && goalSelectionEnabled && !touchMoved
                && !onboardMode) {
            float baseCell = Math.min(getWidth(), getHeight()) / 34f;
            float cell = baseCell * scale;
            float originX = getWidth() / 2f - (followCar ? carGridX : GRID_CENTER) * cell + offsetX;
            float originY = getHeight() / 2f + (followCar ? carGridY : GRID_CENTER) * cell + offsetY;
            int goalX = Math.round((event.getX() - originX) / cell - 0.5f);
            int goalY = Math.round((originY - event.getY()) / cell - 0.5f);
            lastGoalTapX = goalX;
            lastGoalTapY = goalY;
            boolean inGrid = goalX >= 0 && goalY >= 0 && goalX < GRID_SIZE && goalY < GRID_SIZE;
            boolean found;
            if (!inGrid) {
                lastPlanFailure = "点击位置超出地图范围";
                found = false;
            } else {
                found = planPathTo(goalX, goalY);
            }
            lastGoalTapValid = found;
            performHapticFeedback(found
                    ? android.view.HapticFeedbackConstants.KEYBOARD_TAP
                    : android.view.HapticFeedbackConstants.LONG_PRESS);
            goalSelectionEnabled = !found;
            if (found) goalSelectionBlocked = null;
            invalidate();
            if (goalSelectionListener != null) {
                goalSelectionListener.onGoalSelected(found, goalX, goalY, plannedPath.size());
            }
            return true;
        }
        return true;
    }

    /** Keeps the world point under the two-finger focus fixed on screen. */
    private void zoomAround(float focusX, float focusY, float scaleFactor) {
        float baseCell = Math.min(getWidth(), getHeight()) / 34f;
        if (baseCell <= 0f) return;
        int displayGridCenter = (onboardMode ? ONBOARD_GRID_SIZE : GRID_SIZE) / 2;
        float centerGridX = followCar ? carGridX : displayGridCenter;
        float centerGridY = followCar ? carGridY : displayGridCenter;
        float oldCell = baseCell * scale;
        float oldOriginX = getWidth() / 2f - centerGridX * oldCell + offsetX;
        float oldOriginY = getHeight() / 2f + centerGridY * oldCell + offsetY;
        float focusGridX = (focusX - oldOriginX) / oldCell;
        float focusGridY = (oldOriginY - focusY) / oldCell;

        scale = clamp(scale * scaleFactor, 0.08f, 6.0f);
        float newCell = baseCell * scale;
        offsetX = focusX - getWidth() / 2f + centerGridX * newCell
                - focusGridX * newCell;
        offsetY = focusY - getHeight() / 2f - centerGridY * newCell
                + focusGridY * newCell;
    }

    private boolean[] buildAllowedMask() {
        boolean[] result = new boolean[GRID_SIZE * GRID_SIZE];
        double marginM = vehicleBoundaryMarginM();
        for (int y = 0; y < GRID_SIZE; y++) {
            for (int x = 0; x < GRID_SIZE; x++) {
                result[y * GRID_SIZE + x] = isInsideAnchorBoundaryWorld(
                        gridToWorldXM(x), gridToWorldYM(y), marginM);
            }
        }
        return result;
    }

    private boolean isInsideAnchorBoundary(int gridX, int gridY) {
        return isInsideAnchorBoundaryWorld(gridToWorldXM(gridX), gridToWorldYM(gridY), 0.0);
    }

    private boolean isInsideAnchorBoundaryWorld(double worldX, double worldY, double marginM) {
        List<double[]> hull = anchorHull();
        boolean inside = false;
        for (int i = 0, j = hull.size() - 1; i < hull.size(); j = i++) {
            double[] a = hull.get(i);
            double[] b = hull.get(j);
            if (((a[1] > worldY) != (b[1] > worldY))
                    && worldX < (b[0] - a[0]) * (worldY - a[1]) / (b[1] - a[1]) + a[0]) {
                inside = !inside;
            }
        }
        if (!inside) return false;
        if (marginM <= 0.0) return true;
        double minimumDistance = Double.POSITIVE_INFINITY;
        for (int i = 0, j = hull.size() - 1; i < hull.size(); j = i++) {
            minimumDistance = Math.min(minimumDistance,
                    distanceToSegment(worldX, worldY, hull.get(j), hull.get(i)));
        }
        return minimumDistance >= marginM;
    }

    /** Positive inside the anchor hull and negative outside it. */
    private double signedAnchorBoundaryClearance(double worldX, double worldY) {
        List<double[]> hull = anchorHull();
        boolean inside = false;
        double minimumDistance = Double.POSITIVE_INFINITY;
        for (int i = 0, j = hull.size() - 1; i < hull.size(); j = i++) {
            double[] a = hull.get(i);
            double[] b = hull.get(j);
            if (((a[1] > worldY) != (b[1] > worldY))
                    && worldX < (b[0] - a[0]) * (worldY - a[1]) / (b[1] - a[1]) + a[0]) {
                inside = !inside;
            }
            minimumDistance = Math.min(minimumDistance,
                    distanceToSegment(worldX, worldY, b, a));
        }
        return inside ? minimumDistance : -minimumDistance;
    }

    private static double distanceToSegment(double x, double y, double[] a, double[] b) {
        double dx = b[0] - a[0];
        double dy = b[1] - a[1];
        double lengthSquared = dx * dx + dy * dy;
        if (lengthSquared <= 1e-12) return Math.hypot(x - a[0], y - a[1]);
        double t = Math.max(0.0, Math.min(1.0,
                ((x - a[0]) * dx + (y - a[1]) * dy) / lengthSquared));
        return Math.hypot(x - (a[0] + t * dx), y - (a[1] + t * dy));
    }

    private static double vehicleBoundaryMarginM() {
        return Math.hypot(VehicleGeometry.TAG_RECTANGLE_LENGTH_M * 0.5,
                VehicleGeometry.TAG_RECTANGLE_WIDTH_M * 0.5) + 0.075;
    }

    private List<double[]> anchorHull() {
        ArrayList<double[]> points = new ArrayList<>();
        Collections.addAll(points, copyAnchors(anchorPositionsM));
        final double centerX = anchorCenterX();
        final double centerY = anchorCenterY();
        points.sort(Comparator.comparingDouble(point -> Math.atan2(point[1] - centerY, point[0] - centerX)));
        return points;
    }

    private double anchorCenterX() {
        double sum = 0.0;
        for (double[] point : anchorPositionsM) sum += point[0];
        return sum / anchorPositionsM.length;
    }

    private double anchorCenterY() {
        double sum = 0.0;
        for (double[] point : anchorPositionsM) sum += point[1];
        return sum / anchorPositionsM.length;
    }

    private static double[][] copyAnchors(double[][] source) {
        double[][] result = new double[source.length][2];
        for (int index = 0; index < source.length; index++) {
            result[index][0] = source[index][0];
            result[index][1] = source[index][1];
        }
        return result;
    }

    private static float clamp(float value, float min, float max) {
        return Math.max(min, Math.min(max, value));
    }
}

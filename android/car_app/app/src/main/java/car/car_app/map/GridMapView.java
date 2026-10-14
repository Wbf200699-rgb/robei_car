package car.car_app.map;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Path;
import android.graphics.RectF;
import android.util.AttributeSet;
import android.view.MotionEvent;
import android.view.View;

import androidx.annotation.Nullable;

import java.util.ArrayList;
import java.util.List;

import car.car_app.bluetooth.RadarFrame;
import car.car_app.bluetooth.StatusFrame;

public class GridMapView extends View {
    public interface OnCellTapListener {
        void onCellTapped(RoutePoint point);
    }

    private static final int DEFAULT_ROWS = 10;
    private static final int DEFAULT_COLUMNS = 10;
    private static final int ROUTE_WORDS = 16;
    private static final float HDMI_WIDTH = 1024f;
    private static final float HDMI_HEIGHT = 600f;
    private static final int HDMI_ORIGIN_X = 675;
    private static final int HDMI_ORIGIN_Y = 370;
    private static final float HDMI_MAP_LEFT = 405f;
    private static final float HDMI_MAP_TOP = 180f;
    private static final float HDMI_MAP_RIGHT = 945f;
    private static final float HDMI_MAP_BOTTOM = 560f;
    private static final int MENU_FOCUS_CHANGE = 0;
    private static final int MENU_IN_CHANGE = 1;
    private static final int MENU_FOCUS_RUN = 2;
    private static final int MENU_IN_RUN = 3;
    private static final int STATE_MENU = 0;
    private static final int STATE_CHANGE = 1;
    private static final int STATE_RUN = 3;
    private static final int STATE_STOP = 4;
    private static final int ROUTE_STOP = 0;
    private static final int ROUTE_RIGHT = 1;
    private static final int ROUTE_LEFT = 2;
    private static final int ROUTE_FORWARD = 3;
    private static final int INVALID_DISTANCE = 0xFFFF;
    private static final int RADAR_SECTOR_COUNT = 8;
    private static final float[] RADAR_RELATIVE_ANGLES = {0f, 45f, 90f, 135f, 180f, -135f, -90f, -45f};
    private static final int DIR_UP = 0;
    private static final int DIR_RIGHT = 1;
    private static final int DIR_DOWN = 2;
    private static final int DIR_LEFT = 3;

    private final Paint gridPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint routePaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint oldRoutePaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint panelPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint borderPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint targetPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint markerPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint stopPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint avoidAreaPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint avoidSafePaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint avoidObstaclePaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint avoidRayPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint avoidPointPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint carPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint carNosePaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint textPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint smallTextPaint = new Paint(Paint.ANTI_ALIAS_FLAG);

    private int rows = DEFAULT_ROWS;
    private int columns = DEFAULT_COLUMNS;
    private RoutePoint carPosition = new RoutePoint(5, 5);
    private CarDirection carDirection = CarDirection.NORTH;
    private List<RoutePoint> route = new ArrayList<>();
    private StatusFrame hardwareStatus;
    private RadarFrame radarFrame;
    private long previousRouteBits;
    private int previousActiveSteps;
    private int previousCompletedSteps;
    private long completedRouteBits;
    private int hardwareDirection = DIR_UP;
    private OnCellTapListener onCellTapListener;

    public GridMapView(Context context) {
        super(context);
        init();
    }

    public GridMapView(Context context, @Nullable AttributeSet attrs) {
        super(context, attrs);
        init();
    }

    public GridMapView(Context context, @Nullable AttributeSet attrs, int defStyleAttr) {
        super(context, attrs, defStyleAttr);
        init();
    }

    private void init() {
        setBackgroundColor(Color.WHITE);

        gridPaint.setColor(Color.rgb(211, 218, 230));
        gridPaint.setStrokeWidth(2f);

        routePaint.setColor(Color.rgb(33, 102, 243));
        routePaint.setStrokeWidth(8f);
        routePaint.setStyle(Paint.Style.STROKE);
        routePaint.setStrokeCap(Paint.Cap.ROUND);
        routePaint.setStrokeJoin(Paint.Join.ROUND);

        oldRoutePaint.setColor(Color.rgb(148, 163, 184));
        oldRoutePaint.setStrokeWidth(4f);
        oldRoutePaint.setStyle(Paint.Style.STROKE);
        oldRoutePaint.setStrokeCap(Paint.Cap.SQUARE);

        panelPaint.setStyle(Paint.Style.FILL);

        borderPaint.setColor(Color.rgb(51, 65, 85));
        borderPaint.setStyle(Paint.Style.STROKE);
        borderPaint.setStrokeWidth(2f);

        targetPaint.setColor(Color.rgb(8, 145, 178));
        targetPaint.setStrokeWidth(4f);
        targetPaint.setStyle(Paint.Style.STROKE);

        markerPaint.setColor(Color.rgb(100, 116, 139));
        markerPaint.setStrokeWidth(4f);
        markerPaint.setStyle(Paint.Style.STROKE);

        stopPaint.setColor(Color.rgb(220, 38, 38));
        stopPaint.setStrokeWidth(4f);
        stopPaint.setStyle(Paint.Style.STROKE);

        avoidAreaPaint.setColor(Color.argb(38, 14, 165, 233));
        avoidAreaPaint.setStyle(Paint.Style.FILL);

        avoidSafePaint.setColor(Color.rgb(14, 116, 144));
        avoidSafePaint.setStrokeWidth(3f);
        avoidSafePaint.setStyle(Paint.Style.STROKE);
        avoidSafePaint.setStrokeCap(Paint.Cap.ROUND);
        avoidSafePaint.setStrokeJoin(Paint.Join.ROUND);

        avoidObstaclePaint.setColor(Color.rgb(220, 38, 38));
        avoidObstaclePaint.setStrokeWidth(5f);
        avoidObstaclePaint.setStyle(Paint.Style.STROKE);
        avoidObstaclePaint.setStrokeCap(Paint.Cap.ROUND);
        avoidObstaclePaint.setStrokeJoin(Paint.Join.ROUND);

        avoidRayPaint.setColor(Color.argb(115, 37, 99, 235));
        avoidRayPaint.setStrokeWidth(2f);
        avoidRayPaint.setStyle(Paint.Style.STROKE);
        avoidRayPaint.setStrokeCap(Paint.Cap.ROUND);

        avoidPointPaint.setColor(Color.rgb(37, 99, 235));
        avoidPointPaint.setStyle(Paint.Style.FILL);

        carPaint.setColor(Color.rgb(31, 41, 51));
        carPaint.setStyle(Paint.Style.FILL);

        carNosePaint.setColor(Color.rgb(250, 204, 21));
        carNosePaint.setStyle(Paint.Style.FILL);

        textPaint.setColor(Color.rgb(15, 23, 42));
        textPaint.setTextAlign(Paint.Align.CENTER);
        textPaint.setTextSize(28f);

        smallTextPaint.setColor(Color.rgb(15, 23, 42));
        smallTextPaint.setTextAlign(Paint.Align.LEFT);
        smallTextPaint.setTextSize(24f);
    }

    public void setOnCellTapListener(OnCellTapListener onCellTapListener) {
        this.onCellTapListener = onCellTapListener;
    }

    public void setRoute(RoutePoint carPosition, List<RoutePoint> route) {
        this.carPosition = carPosition;
        this.route = new ArrayList<>(route);
        invalidate();
    }

    public void setCarState(RoutePoint carPosition, CarDirection carDirection, List<RoutePoint> route) {
        this.carPosition = carPosition;
        this.carDirection = carDirection;
        this.route = new ArrayList<>(route);
        invalidate();
    }

    public void clearRoute(RoutePoint carPosition, CarDirection carDirection) {
        hardwareStatus = null;
        previousRouteBits = 0L;
        previousActiveSteps = 0;
        previousCompletedSteps = 0;
        completedRouteBits = 0L;
        hardwareDirection = DIR_UP;
        radarFrame = null;
        this.carPosition = carPosition;
        this.carDirection = carDirection;
        route.clear();
        route.add(carPosition);
        invalidate();
    }

    public void setHardwareStatus(StatusFrame statusFrame) {
        updateCompletedRouteCache(statusFrame);
        hardwareDirection = calculateDirection(completedRouteBits, statusFrame.getCompletedRouteSteps());
        hardwareStatus = statusFrame;
        previousRouteBits = parseRouteBits(statusFrame.getRouteInfo());
        previousActiveSteps = statusFrame.getActiveRouteSteps();
        previousCompletedSteps = statusFrame.getCompletedRouteSteps();
        invalidate();
    }

    public void setRadarFrame(@Nullable RadarFrame radarFrame) {
        this.radarFrame = radarFrame;
        invalidate();
    }

    private void updateCompletedRouteCache(StatusFrame statusFrame) {
        if (hardwareStatus == null
                || statusFrame.getCompletedRouteSteps() <= 0
                || statusFrame.getCompletedRouteSteps() < previousCompletedSteps) {
            completedRouteBits = 0L;
            hardwareDirection = DIR_UP;
            return;
        }

        int delta = statusFrame.getCompletedRouteSteps() - previousCompletedSteps;
        if (delta <= 0) {
            return;
        }

        int recoverable = Math.min(delta, Math.min(previousActiveSteps, 32 - previousCompletedSteps));
        for (int i = 0; i < recoverable; i++) {
            int command = (int) ((previousRouteBits >> (i * 2)) & 0x3L);
            int completedIndex = previousCompletedSteps + i;
            if (completedIndex >= 0 && completedIndex < 32) {
                completedRouteBits &= ~(0x3L << (completedIndex * 2));
                completedRouteBits |= ((long) command & 0x3L) << (completedIndex * 2);
            }
        }
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);
        if (hardwareStatus != null) {
            drawHardwareMap(canvas);
            return;
        }

        float cellSize = getCellSize();
        float left = getGridLeft(cellSize);
        float top = getGridTop(cellSize);

        drawGrid(canvas, left, top, cellSize);
        drawRoute(canvas, left, top, cellSize);
        drawCar(canvas, left, top, cellSize);
    }

    private void drawHardwareMap(Canvas canvas) {
        float scale = Math.min(getWidth() / HDMI_WIDTH, getHeight() / HDMI_HEIGHT);
        float left = (getWidth() - HDMI_WIDTH * scale) / 2f;
        float top = (getHeight() - HDMI_HEIGHT * scale) / 2f;

        canvas.save();
        canvas.translate(left, top);
        canvas.scale(scale, scale);

        drawHardwarePanels(canvas);
        drawHardwareRoutePanel(canvas);
        drawHardwareText(canvas);

        canvas.restore();
    }

    private void drawHardwarePanels(Canvas canvas) {
        canvas.drawColor(Color.rgb(244, 247, 251));

        panelPaint.setColor(Color.WHITE);
        canvas.drawRect(0f, 0f, HDMI_WIDTH, 112f, panelPaint);

        panelPaint.setColor(Color.rgb(234, 242, 255));
        RectF menuPanel = new RectF(40f, 150f, 330f, 590f);
        canvas.drawRect(menuPanel, panelPaint);

        boolean changeSelected = hardwareStatus.getMenu() == MENU_FOCUS_CHANGE
                || hardwareStatus.getMenu() == MENU_IN_CHANGE;
        boolean runSelected = hardwareStatus.getMenu() == MENU_FOCUS_RUN
                || hardwareStatus.getMenu() == MENU_IN_RUN;
        drawMenuChoiceBox(canvas, new RectF(78f, 230f, 292f, 305f), changeSelected, Color.rgb(29, 78, 216));
        drawMenuChoiceBox(canvas, new RectF(78f, 360f, 292f, 435f), runSelected, Color.rgb(22, 163, 74));

        panelPaint.setColor(Color.rgb(248, 250, 252));
        RectF routePanel = new RectF(360f, 150f, 990f, 590f);
        canvas.drawRect(routePanel, panelPaint);
        canvas.drawRect(routePanel, borderPaint);
    }

    private void drawHardwareRoutePanel(Canvas canvas) {
        RectF mapArea = new RectF(HDMI_MAP_LEFT, HDMI_MAP_TOP, HDMI_MAP_RIGHT, HDMI_MAP_BOTTOM);
        panelPaint.setColor(Color.WHITE);
        canvas.drawRect(mapArea, panelPaint);
        canvas.drawRect(mapArea, borderPaint);

        gridPaint.setColor(Color.rgb(229, 231, 235));
        gridPaint.setStrokeWidth(1f);
        for (float x = HDMI_MAP_LEFT; x <= 925f; x += 40f) {
            canvas.drawLine(x, HDMI_MAP_TOP, x, HDMI_MAP_BOTTOM, gridPaint);
        }
        for (float y = HDMI_MAP_TOP; y <= 540f; y += 40f) {
            canvas.drawLine(HDMI_MAP_LEFT, y, HDMI_MAP_RIGHT, y, gridPaint);
        }

        int stepPix = calcStepPix(hardwareStatus.getActiveRouteSteps());
        HardwareRoute route = buildHardwareRoute(stepPix);
        for (Segment segment : route.oldSegments) {
            canvas.drawLine(segment.startX, segment.startY, segment.endX, segment.endY, oldRoutePaint);
        }
        for (Segment segment : route.segments) {
            canvas.drawLine(segment.startX, segment.startY, segment.endX, segment.endY, routePaint);
        }

        int liveCarX = HDMI_ORIGIN_X + scaledOffset(hardwareStatus.getNowX(), stepPix);
        int liveCarY = HDMI_ORIGIN_Y + scaledOffset(hardwareStatus.getNowY(), stepPix);
        drawTargetMarker(canvas, stepPix);
        drawNowMarker(canvas, stepPix);
        drawAvoidanceOverlay(canvas, liveCarX, liveCarY, route.direction, stepPix);
        drawHardwareCar(canvas, liveCarX, liveCarY, route.direction);
        if (hardwareStatus.getState() == STATE_STOP) {
            drawStopMarker(canvas, liveCarX, liveCarY, route.direction, stepPix);
        }
    }

    private void drawAvoidanceOverlay(Canvas canvas, int carX, int carY, int direction, int stepPix) {
        if (radarFrame == null || radarFrame.getRadarOnline() != 1) {
            return;
        }

        int safeDistance = radarFrame.getSafeDistanceMm();
        if (!isValidDistance(safeDistance)) {
            return;
        }

        float pixelsPerMm = hardwareStatus.getStepLength() > 0
                ? (float) stepPix / hardwareStatus.getStepLength()
                : 0.08f;
        int[] sectors = radarFrame.getSectorDistancesMm();
        float maxDistance = Math.max(safeDistance, maxValidDistance(sectors));
        if (maxDistance <= 0f) {
            return;
        }

        float maxOverlayRadius = Math.max(72f, stepPix * 4.6f);
        float safePx = clamp((safeDistance / maxDistance) * maxOverlayRadius, 22f, maxOverlayRadius);
        canvas.drawCircle(carX, carY, safePx, avoidAreaPaint);
        canvas.drawCircle(carX, carY, safePx, avoidSafePaint);

        Path edgePath = new Path();
        float firstX = 0f;
        float firstY = 0f;
        boolean hasFirstPoint = false;

        for (int i = 0; i < sectors.length && i < RADAR_SECTOR_COUNT; i++) {
            int distance = sectors[i];
            if (!isValidDistance(distance)) {
                continue;
            }

            float radius = clamp((distance / maxDistance) * maxOverlayRadius, 12f, maxOverlayRadius);
            float angle = screenAngleForSector(direction, i);
            double radians = Math.toRadians(angle);
            float x = carX + (float) Math.cos(radians) * radius;
            float y = carY + (float) Math.sin(radians) * radius;
            boolean danger = distance <= safeDistance;

            canvas.drawLine(carX, carY, x, y, danger ? avoidObstaclePaint : avoidRayPaint);
            canvas.drawCircle(x, y, danger ? 7f : 5f, danger ? avoidObstaclePaint : avoidPointPaint);

            if (!hasFirstPoint) {
                edgePath.moveTo(x, y);
                firstX = x;
                firstY = y;
                hasFirstPoint = true;
            } else {
                edgePath.lineTo(x, y);
            }
        }

        if (hasFirstPoint) {
            edgePath.lineTo(firstX, firstY);
            canvas.drawPath(edgePath, avoidObstaclePaint);
        }

        drawFrontDistanceHint(canvas, carX, carY, direction, safePx, maxDistance, maxOverlayRadius);
        smallTextPaint.setTextSize(16f);
        smallTextPaint.setColor(Color.rgb(14, 116, 144));
        canvas.drawText("安全边界 " + safeDistance + "mm", carX + safePx + 8f, carY - safePx - 6f, smallTextPaint);
    }

    private boolean isValidDistance(int distance) {
        return distance > 0 && distance != INVALID_DISTANCE;
    }

    private void drawFrontDistanceHint(Canvas canvas, int carX, int carY, int direction,
                                       float safePx, float maxDistance, float maxOverlayRadius) {
        int frontDistance = radarFrame.getFrontDistance();
        if (!isValidDistance(frontDistance)) {
            return;
        }
        float radius = clamp((frontDistance / maxDistance) * maxOverlayRadius, 12f, maxOverlayRadius);
        float angle = screenAngleForSector(direction, 0);
        double radians = Math.toRadians(angle);
        float x = carX + (float) Math.cos(radians) * radius;
        float y = carY + (float) Math.sin(radians) * radius;
        canvas.drawCircle(x, y, frontDistance <= radarFrame.getSafeDistanceMm() ? 10f : 7f,
                frontDistance <= radarFrame.getSafeDistanceMm() ? avoidObstaclePaint : avoidSafePaint);
        smallTextPaint.setTextSize(15f);
        smallTextPaint.setColor(frontDistance <= radarFrame.getSafeDistanceMm()
                ? Color.rgb(185, 28, 28)
                : Color.rgb(15, 118, 110));
        canvas.drawText("前 " + frontDistance + "mm", x + 8f, y - 8f, smallTextPaint);
    }

    private int maxValidDistance(int[] distances) {
        int max = 0;
        for (int distance : distances) {
            if (isValidDistance(distance)) {
                max = Math.max(max, distance);
            }
        }
        return max;
    }

    private float screenAngleForSector(int direction, int sectorIndex) {
        return directionBaseAngle(direction) + RADAR_RELATIVE_ANGLES[sectorIndex];
    }

    private float directionBaseAngle(int direction) {
        if (direction == DIR_RIGHT) {
            return 0f;
        }
        if (direction == DIR_DOWN) {
            return 90f;
        }
        if (direction == DIR_LEFT) {
            return 180f;
        }
        return -90f;
    }

    private float[] directionVector(int direction) {
        if (direction == DIR_RIGHT) {
            return new float[] {1f, 0f};
        }
        if (direction == DIR_DOWN) {
            return new float[] {0f, 1f};
        }
        if (direction == DIR_LEFT) {
            return new float[] {-1f, 0f};
        }
        return new float[] {0f, -1f};
    }

    private void drawHardwareText(Canvas canvas) {
        drawStateChip(canvas, 52f, "菜单", hardwareStatus.getState() == STATE_MENU, Color.rgb(37, 99, 235));
        drawStateChip(canvas, 192f, "编辑", hardwareStatus.getState() == STATE_CHANGE, Color.rgb(37, 99, 235));
        drawStateChip(canvas, 332f, "运行", hardwareStatus.getState() == STATE_RUN, Color.rgb(22, 163, 74));
        drawStateChip(canvas, 472f, "停止", hardwareStatus.getState() == STATE_STOP, Color.rgb(220, 38, 38));

        drawMenuText(canvas);

        smallTextPaint.setColor(Color.rgb(15, 23, 42));
        smallTextPaint.setTextSize(22f);
        canvas.drawText("路线 " + hardwareStatus.getActiveRouteSteps(), 382f, 170f, smallTextPaint);
        canvas.drawText("完成 " + hardwareStatus.getCompletedRouteSteps(), 482f, 170f, smallTextPaint);
        canvas.drawText("步长 " + formatOptional(hardwareStatus.getStepLength()), 594f, 170f, smallTextPaint);
        canvas.drawText("当前位置 " + hardwareStatus.getNowX() + "," + hardwareStatus.getNowY(), 742f, 170f, smallTextPaint);
        canvas.drawText("目标 " + hardwareStatus.getTargetX() + "," + hardwareStatus.getTargetY(), 742f, 191f, smallTextPaint);

        smallTextPaint.setTextSize(18f);
        canvas.drawText("路线 " + hardwareStatus.getRouteInfo(), 382f, 534f, smallTextPaint);
        canvas.drawText("已完成 " + toFixedRouteHex(completedRouteBits), 382f, 552f, smallTextPaint);
    }

    private void drawStateChip(Canvas canvas, float x, String label, boolean selected, int color) {
        RectF rect = new RectF(x, 18f, x + 120f, 62f);
        panelPaint.setColor(selected ? color : Color.rgb(203, 213, 225));
        canvas.drawRect(rect, panelPaint);
        textPaint.setColor(selected ? Color.WHITE : Color.rgb(51, 65, 85));
        textPaint.setTextSize(25f);
        canvas.drawText(label, rect.centerX(), rect.centerY() - ((textPaint.descent() + textPaint.ascent()) / 2), textPaint);
    }

    private void drawMenuChoiceBox(Canvas canvas, RectF rect, boolean selected, int selectedColor) {
        panelPaint.setColor(selected ? selectedColor : Color.rgb(220, 234, 254));
        canvas.drawRect(rect, panelPaint);
        canvas.drawRect(rect, borderPaint);
    }

    private void drawMenuText(Canvas canvas) {
        boolean changeSelected = hardwareStatus.getMenu() == MENU_FOCUS_CHANGE
                || hardwareStatus.getMenu() == MENU_IN_CHANGE;
        boolean runSelected = hardwareStatus.getMenu() == MENU_FOCUS_RUN
                || hardwareStatus.getMenu() == MENU_IN_RUN;

        textPaint.setTextSize(29f);
        textPaint.setColor(changeSelected ? Color.WHITE : Color.rgb(29, 78, 216));
        canvas.drawText("修改路线", 185f, 272f - ((textPaint.descent() + textPaint.ascent()) / 2), textPaint);
        textPaint.setColor(runSelected ? Color.WHITE : Color.rgb(22, 101, 52));
        canvas.drawText("运行路线", 185f, 402f - ((textPaint.descent() + textPaint.ascent()) / 2), textPaint);

        smallTextPaint.setColor(Color.rgb(15, 23, 42));
        smallTextPaint.setTextSize(22f);
        canvas.drawText("剩余 " + hardwareStatus.getActiveRouteSteps(), 84f, 472f, smallTextPaint);
        canvas.drawText("下一步 " + commandLabel((int) (parseRouteBits(hardwareStatus.getRouteInfo()) & 0x3L)),
                84f, 492f, smallTextPaint);
    }

    private HardwareRoute buildHardwareRoute(int stepPix) {
        HardwareRoute result = new HardwareRoute();
        int curX = HDMI_ORIGIN_X;
        int curY = HDMI_ORIGIN_Y;
        int direction = DIR_UP;

        int completedLimit = Math.min(Math.min(hardwareStatus.getCompletedRouteSteps(), ROUTE_WORDS), 32);
        for (int index = 0; index < completedLimit; index++) {
            int command = (int) ((completedRouteBits >> (index * 2)) & 0x3L);
            if (command == ROUTE_STOP) {
                continue;
            }
            if (command == ROUTE_RIGHT) {
                direction = (direction + 1) & 0x3;
            } else if (command == ROUTE_LEFT) {
                direction = (direction + 3) & 0x3;
            } else {
                int nextX = nextX(curX, direction, stepPix);
                int nextY = nextY(curY, direction, stepPix);
                result.oldSegments.add(new Segment(curX, curY, nextX, nextY));
                curX = nextX;
                curY = nextY;
            }
        }

        result.carX = curX;
        result.carY = curY;
        result.direction = direction;

        long routeBits = parseRouteBits(hardwareStatus.getRouteInfo());
        int limit = Math.min(Math.min(hardwareStatus.getActiveRouteSteps(), ROUTE_WORDS), 32);

        for (int index = 0; index < limit; index++) {
            int command = (int) ((routeBits >> (index * 2)) & 0x3L);
            if (command == ROUTE_STOP) {
                continue;
            }
            if (command == ROUTE_RIGHT) {
                direction = (direction + 1) & 0x3;
            } else if (command == ROUTE_LEFT) {
                direction = (direction + 3) & 0x3;
            } else if (command == ROUTE_FORWARD) {
                int nextX = nextX(curX, direction, stepPix);
                int nextY = nextY(curY, direction, stepPix);
                result.segments.add(new Segment(curX, curY, nextX, nextY));
                curX = nextX;
                curY = nextY;
            }
        }
        return result;
    }

    private int nextX(int currentX, int direction, int stepPix) {
        if (direction == DIR_RIGHT) {
            return currentX + stepPix;
        }
        if (direction == DIR_LEFT) {
            return currentX - stepPix;
        }
        return currentX;
    }

    private int nextY(int currentY, int direction, int stepPix) {
        if (direction == DIR_UP) {
            return currentY - stepPix;
        }
        if (direction == DIR_DOWN) {
            return currentY + stepPix;
        }
        return currentY;
    }

    private void drawHardwareCar(Canvas canvas, float centerX, float centerY, int direction) {
        RectF body = new RectF(centerX - 8f, centerY - 8f, centerX + 8f, centerY + 8f);
        canvas.drawRect(body, carPaint);

        Path arrow = new Path();
        if (direction == DIR_UP) {
            arrow.moveTo(centerX, centerY - 22f);
            arrow.lineTo(centerX - 7f, centerY - 8f);
            arrow.lineTo(centerX + 7f, centerY - 8f);
        } else if (direction == DIR_RIGHT) {
            arrow.moveTo(centerX + 22f, centerY);
            arrow.lineTo(centerX + 8f, centerY - 7f);
            arrow.lineTo(centerX + 8f, centerY + 7f);
        } else if (direction == DIR_DOWN) {
            arrow.moveTo(centerX, centerY + 22f);
            arrow.lineTo(centerX - 7f, centerY + 8f);
            arrow.lineTo(centerX + 7f, centerY + 8f);
        } else {
            arrow.moveTo(centerX - 22f, centerY);
            arrow.lineTo(centerX - 8f, centerY - 7f);
            arrow.lineTo(centerX - 8f, centerY + 7f);
        }
        arrow.close();
        canvas.drawPath(arrow, carNosePaint);
    }

    private void drawTargetMarker(Canvas canvas, int stepPix) {
        float x = HDMI_ORIGIN_X + scaledOffset(hardwareStatus.getTargetX(), stepPix);
        float y = HDMI_ORIGIN_Y + scaledOffset(hardwareStatus.getTargetY(), stepPix);
        canvas.drawLine(x - 8f, y, x + 8f, y, targetPaint);
        canvas.drawLine(x, y - 8f, x, y + 8f, targetPaint);
    }

    private void drawNowMarker(Canvas canvas, int stepPix) {
        float x = HDMI_ORIGIN_X + scaledOffset(hardwareStatus.getNowX(), stepPix);
        float y = HDMI_ORIGIN_Y + scaledOffset(hardwareStatus.getNowY(), stepPix);
        canvas.drawLine(x - 8f, y, x + 8f, y, markerPaint);
        canvas.drawLine(x, y - 8f, x, y + 8f, markerPaint);
    }

    private void drawStopMarker(Canvas canvas, int carX, int carY, int direction, int stepPix) {
        if (direction == DIR_UP) {
            canvas.drawLine(carX - 14f, carY - stepPix, carX + 14f, carY - stepPix, stopPaint);
        } else if (direction == DIR_RIGHT) {
            canvas.drawLine(carX + stepPix, carY - 14f, carX + stepPix, carY + 14f, stopPaint);
        } else if (direction == DIR_DOWN) {
            canvas.drawLine(carX - 14f, carY + stepPix, carX + 14f, carY + stepPix, stopPaint);
        } else {
            canvas.drawLine(carX - stepPix, carY - 14f, carX - stepPix, carY + 14f, stopPaint);
        }
    }

    private int calcStepPix(int steps) {
        if (steps <= 4) {
            return 58;
        }
        if (steps <= 8) {
            return 44;
        }
        if (steps <= 12) {
            return 34;
        }
        if (steps <= 18) {
            return 26;
        }
        if (steps <= 24) {
            return 20;
        }
        return 16;
    }

    private int scaledOffset(int value, int pix) {
        int magnitude = Math.abs(value - 128);
        int scaled = magnitude * pix;
        return value >= 128 ? scaled : -scaled;
    }

    private float clamp(float value, float min, float max) {
        return Math.max(min, Math.min(max, value));
    }

    private long parseRouteBits(String routeInfo) {
        if (routeInfo == null || routeInfo.isEmpty()) {
            return 0L;
        }
        try {
            return Long.parseUnsignedLong(routeInfo, 16);
        } catch (NumberFormatException exception) {
            return 0L;
        }
    }

    private int calculateDirection(long routeBits, int steps) {
        int direction = DIR_UP;
        int limit = Math.min(steps, 32);
        for (int index = 0; index < limit; index++) {
            int command = (int) ((routeBits >> (index * 2)) & 0x3L);
            if (command == ROUTE_RIGHT) {
                direction = (direction + 1) & 0x3;
            } else if (command == ROUTE_LEFT) {
                direction = (direction + 3) & 0x3;
            }
        }
        return direction;
    }

    private String formatOptional(int value) {
        return value >= 0 ? String.valueOf(value) : "?";
    }

    private String toFixedRouteHex(long routeBits) {
        return String.format("%016X", routeBits);
    }

    private String commandLabel(int command) {
        if (command == ROUTE_RIGHT) {
            return "右转";
        }
        if (command == ROUTE_LEFT) {
            return "左转";
        }
        if (command == ROUTE_FORWARD) {
            return "前进";
        }
        return "停止";
    }

    private void drawGrid(Canvas canvas, float left, float top, float cellSize) {
        float width = columns * cellSize;
        float height = rows * cellSize;

        for (int row = 0; row <= rows; row++) {
            float y = top + row * cellSize;
            canvas.drawLine(left, y, left + width, y, gridPaint);
        }

        for (int column = 0; column <= columns; column++) {
            float x = left + column * cellSize;
            canvas.drawLine(x, top, x, top + height, gridPaint);
        }
    }

    private void drawRoute(Canvas canvas, float left, float top, float cellSize) {
        if (route.size() < 2) {
            return;
        }

        RoutePoint firstPoint = route.get(0);
        float previousX = getCenterX(left, cellSize, firstPoint);
        float previousY = getCenterY(top, cellSize, firstPoint);
        for (int index = 1; index < route.size(); index++) {
            RoutePoint point = route.get(index);
            float x = getCenterX(left, cellSize, point);
            float y = getCenterY(top, cellSize, point);
            canvas.drawLine(previousX, previousY, x, y, routePaint);
            previousX = x;
            previousY = y;
        }
    }

    private void drawCar(Canvas canvas, float left, float top, float cellSize) {
        float centerX = getCenterX(left, cellSize, carPosition);
        float centerY = getCenterY(top, cellSize, carPosition);
        float radius = cellSize * 0.32f;
        canvas.drawCircle(centerX, centerY, radius, carPaint);

        Path nose = new Path();
        float tipX = centerX + carDirection.getColumnDelta() * radius * 1.2f;
        float tipY = centerY + carDirection.getRowDelta() * radius * 1.2f;
        float sideX = -carDirection.getRowDelta() * radius * 0.55f;
        float sideY = carDirection.getColumnDelta() * radius * 0.55f;
        float baseX = centerX - carDirection.getColumnDelta() * radius * 0.35f;
        float baseY = centerY - carDirection.getRowDelta() * radius * 0.35f;
        nose.moveTo(tipX, tipY);
        nose.lineTo(baseX + sideX, baseY + sideY);
        nose.lineTo(baseX - sideX, baseY - sideY);
        nose.close();
        canvas.drawPath(nose, carNosePaint);

        canvas.drawText(
                carDirection.getLabel(),
                centerX,
                centerY - ((textPaint.descent() + textPaint.ascent()) / 2),
                textPaint
        );
    }

    @Override
    public boolean onTouchEvent(MotionEvent event) {
        if (event.getAction() != MotionEvent.ACTION_UP) {
            return true;
        }

        float cellSize = getCellSize();
        float left = getGridLeft(cellSize);
        float top = getGridTop(cellSize);
        int column = (int) ((event.getX() - left) / cellSize);
        int row = (int) ((event.getY() - top) / cellSize);

        if (row >= 0 && row < rows && column >= 0 && column < columns && onCellTapListener != null) {
            onCellTapListener.onCellTapped(new RoutePoint(row, column));
        }
        return true;
    }

    private float getCellSize() {
        return Math.min((float) getWidth() / columns, (float) getHeight() / rows);
    }

    private float getGridLeft(float cellSize) {
        return (getWidth() - columns * cellSize) / 2f;
    }

    private float getGridTop(float cellSize) {
        return (getHeight() - rows * cellSize) / 2f;
    }

    private float getCenterX(float left, float cellSize, RoutePoint point) {
        return left + point.getColumn() * cellSize + cellSize / 2f;
    }

    private float getCenterY(float top, float cellSize, RoutePoint point) {
        return top + point.getRow() * cellSize + cellSize / 2f;
    }

    private static class Segment {
        private final int startX;
        private final int startY;
        private final int endX;
        private final int endY;

        private Segment(int startX, int startY, int endX, int endY) {
            this.startX = startX;
            this.startY = startY;
            this.endX = endX;
            this.endY = endY;
        }
    }

    private static class HardwareRoute {
        private final List<Segment> oldSegments = new ArrayList<>();
        private final List<Segment> segments = new ArrayList<>();
        private int carX;
        private int carY;
        private int direction;
    }
}

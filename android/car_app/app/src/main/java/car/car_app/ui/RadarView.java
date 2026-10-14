package car.car_app.ui;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Path;
import android.graphics.RectF;
import android.util.AttributeSet;
import android.view.View;

import androidx.annotation.Nullable;

import car.car_app.bluetooth.LidarMapPoint;
import car.car_app.bluetooth.LidarMapScan;
import car.car_app.bluetooth.RadarFrame;

public class RadarView extends View {
    private static final int INVALID_DISTANCE = 0xFFFF;
    private static final int MAX_DISPLAY_DISTANCE_MM = 5000;
    private static final String[] LABELS = {"前", "右前", "右", "右后", "后", "左后", "左", "左前"};
    private static final float[] ANGLES = {-90f, -45f, 0f, 45f, 90f, 135f, 180f, -135f};

    private final Paint ringPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint sectorPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint dangerPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint edgePaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint pointPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint scanEdgePaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint qualityLowPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint qualityMediumPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint qualityHighPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint safeRingPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint scaleTextPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint textPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint mutedTextPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint carPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private RadarFrame radarFrame;
    private LidarMapScan lidarMapScan;

    public RadarView(Context context) {
        super(context);
        init();
    }

    public RadarView(Context context, @Nullable AttributeSet attrs) {
        super(context, attrs);
        init();
    }

    public RadarView(Context context, @Nullable AttributeSet attrs, int defStyleAttr) {
        super(context, attrs, defStyleAttr);
        init();
    }

    private void init() {
        ringPaint.setColor(Color.rgb(210, 220, 230));
        ringPaint.setStyle(Paint.Style.STROKE);
        ringPaint.setStrokeWidth(2f);

        sectorPaint.setColor(Color.rgb(33, 102, 243));
        sectorPaint.setStyle(Paint.Style.STROKE);
        sectorPaint.setStrokeWidth(3f);
        sectorPaint.setStrokeCap(Paint.Cap.ROUND);

        dangerPaint.setColor(Color.rgb(201, 58, 58));
        dangerPaint.setStyle(Paint.Style.STROKE);
        dangerPaint.setStrokeWidth(5f);
        dangerPaint.setStrokeCap(Paint.Cap.ROUND);

        edgePaint.setColor(Color.rgb(20, 184, 166));
        edgePaint.setStyle(Paint.Style.STROKE);
        edgePaint.setStrokeWidth(5f);
        edgePaint.setStrokeJoin(Paint.Join.ROUND);

        pointPaint.setColor(Color.rgb(33, 102, 243));
        pointPaint.setStyle(Paint.Style.FILL);

        scanEdgePaint.setColor(Color.rgb(20, 184, 166));
        scanEdgePaint.setStyle(Paint.Style.STROKE);
        scanEdgePaint.setStrokeWidth(2f);
        scanEdgePaint.setStrokeJoin(Paint.Join.ROUND);

        qualityLowPaint.setColor(Color.rgb(148, 163, 184));
        qualityLowPaint.setStyle(Paint.Style.FILL);
        qualityMediumPaint.setColor(Color.rgb(33, 102, 243));
        qualityMediumPaint.setStyle(Paint.Style.FILL);
        qualityHighPaint.setColor(Color.rgb(13, 148, 136));
        qualityHighPaint.setStyle(Paint.Style.FILL);

        safeRingPaint.setColor(Color.rgb(201, 58, 58));
        safeRingPaint.setStyle(Paint.Style.STROKE);
        safeRingPaint.setStrokeWidth(2f);
        safeRingPaint.setPathEffect(new android.graphics.DashPathEffect(new float[]{10f, 8f}, 0f));

        scaleTextPaint.setColor(Color.rgb(96, 112, 128));
        scaleTextPaint.setTextAlign(Paint.Align.LEFT);
        scaleTextPaint.setTextSize(18f);

        textPaint.setColor(Color.rgb(24, 33, 43));
        textPaint.setTextAlign(Paint.Align.CENTER);
        textPaint.setTextSize(24f);

        mutedTextPaint.setColor(Color.rgb(96, 112, 128));
        mutedTextPaint.setTextAlign(Paint.Align.CENTER);
        mutedTextPaint.setTextSize(22f);

        carPaint.setColor(Color.rgb(31, 41, 51));
        carPaint.setStyle(Paint.Style.FILL);
    }

    public void setRadarFrame(@Nullable RadarFrame radarFrame) {
        this.radarFrame = radarFrame;
        invalidate();
    }

    public void setLidarMapScan(@Nullable LidarMapScan lidarMapScan) {
        this.lidarMapScan = lidarMapScan;
        invalidate();
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);

        float width = getWidth();
        float height = getHeight();
        float cx = width / 2f;
        float cy = height / 2f;
        float radius = Math.min(width, height) * 0.38f;

        drawRings(canvas, cx, cy, radius);
        if (lidarMapScan != null) {
            drawDetailedScan(canvas, cx, cy, radius);
            drawSafetyOverlay(canvas, cx, cy, radius);
        } else if (radarFrame != null) {
            drawEdgeModel(canvas, cx, cy, radius);
        } else {
            canvas.drawText("暂无雷达数据", cx, cy + radius + 34f, textPaint);
        }
        drawCar(canvas, cx, cy, radius);
    }

    private void drawRings(Canvas canvas, float cx, float cy, float radius) {
        for (int i = 1; i <= 3; i++) {
            canvas.drawCircle(cx, cy, radius * i / 3f, ringPaint);
            canvas.drawText(String.format(java.util.Locale.CHINA, "%.1fm",
                            MAX_DISPLAY_DISTANCE_MM * i / 3000f),
                    cx + 8f, cy - radius * i / 3f + 18f, scaleTextPaint);
        }
        canvas.drawLine(cx, cy - radius, cx, cy + radius, ringPaint);
        canvas.drawLine(cx - radius, cy, cx + radius, cy, ringPaint);
    }

    private void drawDetailedScan(Canvas canvas, float cx, float cy, float radius) {
        Path edgePath = new Path();
        boolean segmentStarted = false;
        LidarMapPoint previous = null;
        float density = getResources().getDisplayMetrics().density;

        for (LidarMapPoint point : lidarMapScan.getPoints()) {
            if (!point.isValid()) {
                segmentStarted = false;
                previous = null;
                continue;
            }
            float normalized = clamp(point.getDistanceMm() / (float) MAX_DISPLAY_DISTANCE_MM, 0f, 1f);
            double radians = Math.toRadians(point.getAngleDegrees() - 90f);
            float x = cx + (float) Math.cos(radians) * normalized * radius;
            float y = cy + (float) Math.sin(radians) * normalized * radius;

            boolean continuous = segmentStarted && previous != null
                    && point.getBinIndex() == previous.getBinIndex() + 1
                    && Math.abs(point.getDistanceMm() - previous.getDistanceMm()) <= 900;
            if (continuous) {
                edgePath.lineTo(x, y);
            } else {
                edgePath.moveTo(x, y);
                segmentStarted = true;
            }

            Paint qualityPaint = point.getQualityClass() >= 3
                    ? qualityHighPaint
                    : (point.getQualityClass() == 2 ? qualityMediumPaint : qualityLowPaint);
            canvas.drawCircle(x, y, (point.getQualityClass() >= 2 ? 2.8f : 2.1f) * density,
                    qualityPaint);
            previous = point;
        }
        canvas.drawPath(edgePath, scanEdgePaint);

        if (radarFrame != null && isValid(radarFrame.getSafeDistanceMm())) {
            float safeRadius = clamp(
                    radarFrame.getSafeDistanceMm() / (float) MAX_DISPLAY_DISTANCE_MM,
                    0f,
                    1f
            ) * radius;
            canvas.drawCircle(cx, cy, safeRadius, safeRingPaint);
        }
    }

    private void drawSafetyOverlay(Canvas canvas, float cx, float cy, float radius) {
        if (radarFrame == null || radarFrame.getObstacleTriggered() != 1) {
            return;
        }
        int[] distances = radarFrame.getSectorDistancesMm();
        int safeDistance = radarFrame.getSafeDistanceMm();
        for (int index = 0; index < distances.length && index < ANGLES.length; index++) {
            int distance = distances[index];
            if (!isValid(distance) || safeDistance <= 0 || distance > safeDistance) {
                continue;
            }
            float pointRadius = clamp(distance / (float) MAX_DISPLAY_DISTANCE_MM, 0f, 1f) * radius;
            double radians = Math.toRadians(ANGLES[index]);
            float x = cx + (float) Math.cos(radians) * pointRadius;
            float y = cy + (float) Math.sin(radians) * pointRadius;
            canvas.drawLine(cx, cy, x, y, dangerPaint);
            canvas.drawCircle(x, y, 8f, dangerPaint);
        }
    }

    private void drawEdgeModel(Canvas canvas, float cx, float cy, float radius) {
        int[] distances = radarFrame.getSectorDistancesMm();
        int safeDistance = radarFrame.getSafeDistanceMm();
        int maxValidDistance = maxValidDistance(distances);
        float maxDistance = Math.max(1f, Math.max(safeDistance, maxValidDistance));
        Path edgePath = new Path();
        boolean hasFirstPoint = false;
        float firstX = 0f;
        float firstY = 0f;

        for (int i = 0; i < distances.length && i < LABELS.length; i++) {
            int distance = distances[i];
            if (!isValid(distance)) {
                drawLabel(canvas, cx, cy, radius, i, "----");
                continue;
            }

            float sectorRadius = clamp(distance / maxDistance, 0.08f, 1f) * radius;
            boolean danger = safeDistance > 0 && distance <= safeDistance;
            double radians = Math.toRadians(ANGLES[i]);
            float x = cx + (float) Math.cos(radians) * sectorRadius;
            float y = cy + (float) Math.sin(radians) * sectorRadius;
            canvas.drawLine(cx, cy, x, y, danger ? dangerPaint : sectorPaint);
            canvas.drawCircle(x, y, danger ? 8f : 6f, danger ? dangerPaint : pointPaint);
            if (!hasFirstPoint) {
                edgePath.moveTo(x, y);
                firstX = x;
                firstY = y;
                hasFirstPoint = true;
            } else {
                edgePath.lineTo(x, y);
            }
            drawLabel(canvas, cx, cy, radius, i, distance + "mm");
        }

        if (hasFirstPoint) {
            edgePath.lineTo(firstX, firstY);
            canvas.drawPath(edgePath, edgePaint);
        }

        if (maxValidDistance <= 0) {
            String line = radarFrame.getRadarOnline() == 1 ? "雷达在线，暂无有效采样" : "雷达未在线或正在启动";
            canvas.drawText(line, cx, cy + radius + 36f, mutedTextPaint);
        }
    }

    private void drawLabel(Canvas canvas, float cx, float cy, float radius, int index, String value) {
        double radians = Math.toRadians(ANGLES[index]);
        float labelRadius = radius + 28f;
        float x = cx + (float) Math.cos(radians) * labelRadius;
        float y = cy + (float) Math.sin(radians) * labelRadius;
        canvas.drawText(LABELS[index], x, y, textPaint);
        canvas.drawText(value, x, y + 22f, textPaint);
    }

    private void drawCar(Canvas canvas, float cx, float cy, float radius) {
        float body = Math.max(18f, radius * 0.13f);
        canvas.drawRoundRect(new RectF(cx - body, cy - body, cx + body, cy + body), 5f, 5f, carPaint);
        Path nose = new Path();
        nose.moveTo(cx, cy - body * 1.8f);
        nose.lineTo(cx - body * 0.55f, cy - body);
        nose.lineTo(cx + body * 0.55f, cy - body);
        nose.close();
        canvas.drawPath(nose, carPaint);
    }

    private int maxValidDistance(int[] distances) {
        int max = 0;
        for (int distance : distances) {
            if (isValid(distance)) {
                max = Math.max(max, distance);
            }
        }
        return max;
    }

    private boolean isValid(int distance) {
        return distance >= 0 && distance != INVALID_DISTANCE;
    }

    private float clamp(float value, float min, float max) {
        return Math.max(min, Math.min(max, value));
    }
}

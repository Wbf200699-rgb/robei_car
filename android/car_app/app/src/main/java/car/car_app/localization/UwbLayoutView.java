package car.car_app.localization;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Path;
import android.util.AttributeSet;
import android.view.View;

import androidx.annotation.Nullable;

import java.util.Locale;
import java.util.Map;

import car.car_app.geometry.VehicleGeometry;
import car.car_app.navigation.NavigationSteering;
import car.car_app.uwb.UwbTagFix;

/** Draws the four fixed LinkPG anchors and four vehicle-mounted tags. */
public final class UwbLayoutView extends View {
    // Physical clockwise order from the vehicle front-left corner.
    private static final int[] TAG_ORDER = {1, 2, 3, 0};

    private final Paint fieldPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint gridPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint anchorPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint tagPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint vehiclePaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint headingPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint labelPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Paint detailPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    private final Path vehiclePath = new Path();

    private AnchorLayout anchorLayout = AnchorLayout.defaultLayout();
    private double vehicleXM = anchorLayout.getWidthM() * 0.5;
    private double vehicleYM = anchorLayout.getHeightM() * 0.5;
    private double vehicleHeadingRad;
    private boolean livePose;
    private boolean poseAvailable;
    private String poseQuality = "几何示意";
    private Map<Integer, UwbTagFix> serverTagFixes = java.util.Collections.emptyMap();

    public UwbLayoutView(Context context) {
        this(context, null);
    }

    public UwbLayoutView(Context context, @Nullable AttributeSet attrs) {
        super(context, attrs);
        float density = getResources().getDisplayMetrics().density;
        fieldPaint.setColor(Color.rgb(245, 248, 251));
        fieldPaint.setStyle(Paint.Style.FILL);
        gridPaint.setColor(Color.rgb(207, 218, 228));
        gridPaint.setStyle(Paint.Style.STROKE);
        gridPaint.setStrokeWidth(density);
        anchorPaint.setColor(Color.rgb(36, 91, 120));
        tagPaint.setColor(Color.rgb(63, 141, 138));
        vehiclePaint.setColor(Color.rgb(35, 43, 52));
        vehiclePaint.setStyle(Paint.Style.FILL);
        headingPaint.setColor(Color.rgb(200, 94, 85));
        headingPaint.setStyle(Paint.Style.STROKE);
        headingPaint.setStrokeWidth(3f * density);
        labelPaint.setColor(Color.rgb(23, 35, 45));
        labelPaint.setTextSize(14f * getResources().getDisplayMetrics().scaledDensity);
        labelPaint.setFakeBoldText(true);
        detailPaint.setColor(Color.rgb(97, 113, 124));
        detailPaint.setTextSize(10f * getResources().getDisplayMetrics().scaledDensity);
        setBackgroundColor(Color.WHITE);
    }

    public void setVehiclePose(double xM, double yM, double headingRad,
                               boolean livePose, String quality) {
        double normalizedHeading = Pose2d.normalizeRadians(headingRad);
        if (!poseAvailable) {
            vehicleXM = xM;
            vehicleYM = yM;
            vehicleHeadingRad = normalizedHeading;
        } else {
            // MainActivity supplies the same filtered UWB position and fused
            // heading to localization and mapping. Do not filter either value
            // again here, otherwise the pages represent different timestamps.
            vehicleXM = xM;
            vehicleYM = yM;
            vehicleHeadingRad = normalizedHeading;
        }
        poseAvailable = true;
        this.livePose = livePose;
        poseQuality = quality == null || quality.trim().isEmpty()
                ? (livePose ? "实时" : "几何示意") : quality;
        invalidate();
    }

    /** Replaces projected tag corners with the latest coordinates reported by the server. */
    public void setServerTagFixes(Map<Integer, UwbTagFix> fixes) {
        serverTagFixes = fixes == null
                ? java.util.Collections.emptyMap()
                : new java.util.HashMap<>(fixes);
        invalidate();
    }

    public void setAnchorLayout(AnchorLayout layout) {
        if (layout == null) return;
        anchorLayout = layout;
        if (!poseAvailable) {
            vehicleXM = layout.getWidthM() * 0.5;
            vehicleYM = layout.getHeightM() * 0.5;
        }
        invalidate();
    }

    public void resetToNominalCenter() {
        poseAvailable = false;
        setVehiclePose(
                anchorLayout.getWidthM() * 0.5,
                anchorLayout.getHeightM() * 0.5,
                0.0,
                false,
                "几何示意"
        );
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);
        detailPaint.setColor(Color.rgb(97, 113, 124));
        float density = getResources().getDisplayMetrics().density;
        float left = 52f * density;
        float right = getWidth() - 28f * density;
        float top = 52f * density;
        float bottom = getHeight() - 82f * density;
        if (right <= left || bottom <= top) return;

        canvas.drawRect(left, top, right, bottom, fieldPaint);
        canvas.drawRect(left, top, right, bottom, gridPaint);
        for (int step = 1; step < 5; step++) {
            float x = left + (right - left) * step / 5f;
            float y = top + (bottom - top) * step / 5f;
            canvas.drawLine(x, top, x, bottom, gridPaint);
            canvas.drawLine(left, y, right, y, gridPaint);
        }

        double[][] cornerPositions = {
                {0.0, 0.0},
                {anchorLayout.getWidthM(), 0.0},
                {anchorLayout.getWidthM(), anchorLayout.getHeightM()},
                {0.0, anchorLayout.getHeightM()}
        };
        for (int corner = 0; corner < cornerPositions.length; corner++) {
            double[] position = cornerPositions[corner];
            float x = worldX(position[0], left, right);
            float y = worldY(position[1], top, bottom);
            float radius = 10f * density;
            canvas.drawCircle(x, y, radius, anchorPaint);
            boolean rightSide = position[0] > anchorLayout.getWidthM() * 0.5;
            float textX = x + (rightSide ? -13f : 13f) * density;
            labelPaint.setTextAlign(rightSide ? Paint.Align.RIGHT : Paint.Align.LEFT);
            detailPaint.setTextAlign(rightSide ? Paint.Align.RIGHT : Paint.Align.LEFT);
            canvas.drawText(anchorLayout.getAnchorNameAtCorner(corner),
                    textX, y - 4f * density, labelPaint);
            canvas.drawText(String.format(Locale.CHINA, "(%.2f, %.2f)m", position[0], position[1]),
                    textX, y + 12f * density, detailPaint);
        }
        labelPaint.setTextAlign(Paint.Align.LEFT);
        detailPaint.setTextAlign(Paint.Align.LEFT);

        drawVehicle(canvas, left, right, top, bottom, density);
        detailPaint.setColor(livePose ? Color.rgb(51, 128, 92) : Color.rgb(97, 113, 124));
        canvas.drawText((livePose ? "实时标签位姿" : "中心示意位姿") + " · " + poseQuality,
                left, getHeight() - 46f * density, detailPaint);
        canvas.drawText(String.format(Locale.CHINA, "车体中心 (%.3f, %.3f)m  航向 %.1f°",
                        vehicleXM, vehicleYM, Math.toDegrees(vehicleHeadingRad)),
                left, getHeight() - 27f * density, detailPaint);
    }

    private void drawVehicle(Canvas canvas, float left, float right, float top,
                             float bottom, float density) {
        double halfLength = VehicleGeometry.TAG_RECTANGLE_LENGTH_M * 0.5;
        double halfWidth = VehicleGeometry.TAG_RECTANGLE_WIDTH_M * 0.5;
        double[][] corners = {
                {halfLength, halfWidth},
                {halfLength, -halfWidth},
                {-halfLength, -halfWidth},
                {-halfLength, halfWidth}
        };
        vehiclePath.reset();
        for (int index = 0; index < corners.length; index++) {
            double[] world = vehicleToWorld(corners[index][0], corners[index][1]);
            float x = worldX(world[0], left, right);
            float y = worldY(world[1], top, bottom);
            if (index == 0) vehiclePath.moveTo(x, y);
            else vehiclePath.lineTo(x, y);
        }
        vehiclePath.close();
        canvas.drawPath(vehiclePath, vehiclePaint);

        double[] nose = vehicleToWorld(halfLength + 0.11, 0.0);
        float centerX = worldX(vehicleXM, left, right);
        float centerY = worldY(vehicleYM, top, bottom);
        canvas.drawLine(centerX, centerY,
                worldX(nose[0], left, right), worldY(nose[1], top, bottom), headingPaint);

        for (int tagId : TAG_ORDER) {
            UwbTagFix fix = serverTagFixes.get(tagId);
            boolean measured = fix != null && fix.isUsable();
            double[] local = VehicleGeometry.TAG_POSITIONS_M[tagId];
            // Once a rigid-body pose exists, render every tag from the known
            // vehicle rectangle. Raw fixes remain available for diagnostics,
            // but cannot visually deform the vehicle or make one corner jump.
            double[] world = poseAvailable
                    ? vehicleToWorld(local[0], local[1])
                    : measured ? new double[] {fix.getXM(), fix.getYM()}
                    : vehicleToWorld(local[0], local[1]);
            float x = worldX(world[0], left, right);
            float y = worldY(world[1], top, bottom);
            canvas.drawCircle(x, y, 7f * density, tagPaint);
            // The real tag rectangle is only 21 x 17.5 cm inside a 1.5 x
            // 1.67 m field. Place labels outside the vehicle silhouette so the
            // four identifiers remain readable without changing geometry.
            boolean front = tagId == 1 || tagId == 2;
            boolean leftSide = tagId == 1 || tagId == 0;
            float labelX = x + (front ? 10f : -10f) * density;
            float labelY = y + (leftSide ? -10f : 19f) * density;
            labelPaint.setTextAlign(front ? Paint.Align.LEFT : Paint.Align.RIGHT);
            canvas.drawLine(x, y, labelX, labelY, gridPaint);
            canvas.drawText("T" + tagId + (measured ? "" : "?"), labelX, labelY, labelPaint);
        }
        labelPaint.setTextAlign(Paint.Align.LEFT);
    }

    private double[] vehicleToWorld(double localX, double localY) {
        // MainActivity stores FPGA/navigation heading as clockwise-positive.
        // World-coordinate rotation is mathematical CCW-positive, so invert
        // the sign only where local vehicle coordinates enter world geometry.
        double worldHeadingRad = Math.toRadians(
                NavigationSteering.toMathematicalHeadingDegrees(
                        (float) Math.toDegrees(vehicleHeadingRad)));
        double cos = Math.cos(worldHeadingRad);
        double sin = Math.sin(worldHeadingRad);
        return new double[] {
                vehicleXM + cos * localX - sin * localY,
                vehicleYM + sin * localX + cos * localY
        };
    }

    private float worldX(double xM, float left, float right) {
        return left + (float) (xM / anchorLayout.getWidthM()) * (right - left);
    }

    private float worldY(double yM, float top, float bottom) {
        return bottom - (float) (yM / anchorLayout.getHeightM()) * (bottom - top);
    }
}

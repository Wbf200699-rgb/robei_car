package car.car_app.localization;

import java.util.Arrays;

/** Rectangular UWB anchor geometry shared by the phone map and server solver. */
public final class AnchorLayout {
    public static final int MIN_DIMENSION_CM = 50;
    public static final int MAX_DIMENSION_CM = 600;
    public static final int CORNER_LOWER_LEFT = 0;
    public static final int CORNER_LOWER_RIGHT = 1;
    public static final int CORNER_UPPER_RIGHT = 2;
    public static final int CORNER_UPPER_LEFT = 3;

    private final int widthCm;
    private final int heightCm;
    private final int[] anchorAtCorner;

    public AnchorLayout(int widthCm, int heightCm, int[] anchorAtCorner) {
        if (widthCm < MIN_DIMENSION_CM || widthCm > MAX_DIMENSION_CM
                || heightCm < MIN_DIMENSION_CM || heightCm > MAX_DIMENSION_CM) {
            throw new IllegalArgumentException("anchor dimensions out of range");
        }
        if (anchorAtCorner == null || anchorAtCorner.length != 4) {
            throw new IllegalArgumentException("four corner labels required");
        }
        boolean[] seen = new boolean[4];
        for (int anchor : anchorAtCorner) {
            if (anchor < 0 || anchor >= 4 || seen[anchor]) {
                throw new IllegalArgumentException("anchor labels must contain A/B/C/D once");
            }
            seen[anchor] = true;
        }
        this.widthCm = widthCm;
        this.heightCm = heightCm;
        this.anchorAtCorner = Arrays.copyOf(anchorAtCorner, 4);
    }

    /** Default competition layout; changing it does not write LinkPG registers. */
    public static AnchorLayout defaultLayout() {
        return new AnchorLayout(150, 170, new int[]{0, 1, 2, 3});
    }

    public int getWidthCm() { return widthCm; }
    public int getHeightCm() { return heightCm; }
    public double getWidthM() { return widthCm / 100.0; }
    public double getHeightM() { return heightCm / 100.0; }
    public int getAnchorAtCorner(int corner) { return anchorAtCorner[corner]; }
    public int[] getCornerAnchors() { return Arrays.copyOf(anchorAtCorner, 4); }

    public String getAnchorNameAtCorner(int corner) {
        return String.valueOf((char) ('A' + anchorAtCorner[corner]));
    }

    /** Coordinates indexed by physical anchor identity A/B/C/D. */
    public int[][] coordinatesByAnchorCm() {
        int[][] corners = {
                {0, 0, 0},
                {widthCm, 0, 0},
                {widthCm, heightCm, 0},
                {0, heightCm, 0}
        };
        int[][] result = new int[4][3];
        for (int corner = 0; corner < 4; corner++) {
            result[anchorAtCorner[corner]] = Arrays.copyOf(corners[corner], 3);
        }
        return result;
    }

    /**
     * Map code historically indexes A=lower-left, B=upper-right,
     * C=lower-right and D=upper-left. Keep that internal order independent of
     * the labels printed on the physical anchors.
     */
    public double[][] canonicalMapAnchorsM() {
        return new double[][]{
                {0.0, 0.0},
                {getWidthM(), getHeightM()},
                {getWidthM(), 0.0},
                {0.0, getHeightM()}
        };
    }

    /** Labels matching canonicalMapAnchorsM()'s internal LL, UR, LR, UL order. */
    public String[] canonicalMapAnchorNames() {
        return new String[]{
                getAnchorNameAtCorner(CORNER_LOWER_LEFT),
                getAnchorNameAtCorner(CORNER_UPPER_RIGHT),
                getAnchorNameAtCorner(CORNER_LOWER_RIGHT),
                getAnchorNameAtCorner(CORNER_UPPER_LEFT)
        };
    }

    public boolean matchesMapAnchors(double[][] anchors, double toleranceM) {
        if (anchors == null || anchors.length != 4) return false;
        double[][] expected = canonicalMapAnchorsM();
        for (int index = 0; index < 4; index++) {
            if (anchors[index] == null || anchors[index].length < 2
                    || Math.abs(anchors[index][0] - expected[index][0]) > toleranceM
                    || Math.abs(anchors[index][1] - expected[index][1]) > toleranceM) {
                return false;
            }
        }
        return true;
    }

    public String cornerCsv() {
        return anchorAtCorner[0] + "," + anchorAtCorner[1] + ","
                + anchorAtCorner[2] + "," + anchorAtCorner[3];
    }

    @Override
    public boolean equals(Object other) {
        if (this == other) return true;
        if (!(other instanceof AnchorLayout)) return false;
        AnchorLayout layout = (AnchorLayout) other;
        return widthCm == layout.widthCm && heightCm == layout.heightCm
                && Arrays.equals(anchorAtCorner, layout.anchorAtCorner);
    }

    @Override
    public int hashCode() {
        int result = 31 * widthCm + heightCm;
        return 31 * result + Arrays.hashCode(anchorAtCorner);
    }
}

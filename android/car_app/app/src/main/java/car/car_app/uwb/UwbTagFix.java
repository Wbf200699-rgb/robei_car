package car.car_app.uwb;

/** One server-validated LinkPG tag coordinate sample. Coordinates are centimeters. */
public final class UwbTagFix {
    private final int tagId;
    private final boolean positionValid;
    private final boolean fresh;
    private final double xCm;
    private final double yCm;
    private final double zCm;

    public UwbTagFix(int tagId, boolean positionValid, boolean fresh,
                     double xCm, double yCm, double zCm) {
        this.tagId = tagId;
        this.positionValid = positionValid;
        this.fresh = fresh;
        this.xCm = xCm;
        this.yCm = yCm;
        this.zCm = zCm;
    }

    public int getTagId() { return tagId; }
    public boolean isUsable() { return positionValid && fresh; }
    public double getXM() { return xCm / 100.0; }
    public double getYM() { return yCm / 100.0; }
    public double getZCm() { return zCm; }
}

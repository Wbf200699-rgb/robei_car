package car.car_app.localization;

/** One engineering-only UWB tag position sample in the shared world frame. */
public final class UwbTagFix {
    private final int tagId;
    private final double xM;
    private final double yM;
    private final long timestampMs;
    private final boolean valid;

    public UwbTagFix(int tagId, double xM, double yM, long timestampMs, boolean valid) {
        this.tagId = tagId;
        this.xM = xM;
        this.yM = yM;
        this.timestampMs = timestampMs;
        this.valid = valid;
    }

    public int getTagId() { return tagId; }
    public double getXM() { return xM; }
    public double getYM() { return yM; }
    public long getTimestampMs() { return timestampMs; }
    public boolean isValid() { return valid; }
}

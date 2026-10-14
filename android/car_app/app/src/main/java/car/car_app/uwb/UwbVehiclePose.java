package car.car_app.uwb;

/** Four-tag rectangular vehicle pose produced by the server. */
public final class UwbVehiclePose {
    private final String quality;
    private final Double xCm;
    private final Double yCm;
    private final Double headingDeg;
    private final int validTagCount;
    private final Double fixSkewMs;
    private final Long updatedAtMs;

    public UwbVehiclePose(String quality, Double xCm, Double yCm, Double headingDeg,
                          int validTagCount, Double fixSkewMs, Long updatedAtMs) {
        this.quality = quality == null ? "unavailable" : quality;
        this.xCm = xCm;
        this.yCm = yCm;
        this.headingDeg = headingDeg;
        this.validTagCount = validTagCount;
        this.fixSkewMs = fixSkewMs;
        this.updatedAtMs = updatedAtMs;
    }

    public String getQuality() { return quality; }
    /** A single valid tag can still provide an approximate vehicle position. */
    public boolean hasPosition() { return xCm != null && yCm != null; }
    public boolean hasCoordinates() { return hasPosition() && headingDeg != null; }
    public boolean isNavigationReliable() {
        return ("full".equals(quality) || "partial_rigid".equals(quality)) && hasCoordinates();
    }
    public double getXM() { return xCm == null ? 0.0 : xCm / 100.0; }
    public double getYM() { return yCm == null ? 0.0 : yCm / 100.0; }
    public float getHeadingDeg() { return headingDeg == null ? 0f : headingDeg.floatValue(); }
    public int getValidTagCount() { return validTagCount; }
    public Double getFixSkewMs() { return fixSkewMs; }
    public Long getUpdatedAtMs() { return updatedAtMs; }
}

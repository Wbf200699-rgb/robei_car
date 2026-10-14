package car.car_app.localization;

/**
 * Rejects isolated UWB position spikes without shrinking normal displacement.
 *
 * <p>The server already median-filters each tag and fits the vehicle rectangle.
 * A second Android EMA made the displayed pose trail the server pose, while an
 * outlier gate measured from that trailing pose could reject every later
 * sample. Plausible samples therefore pass unchanged; a persistent large move
 * is re-acquired after two mutually consistent samples.</p>
 */
public final class UwbPositionFilter {
    private static final double BASE_GATE_M = 0.30;
    private static final double MAX_SPEED_M_PER_S = 3.0;
    private static final double CANDIDATE_GATE_M = 0.45;
    private static final int REACQUIRE_SAMPLE_COUNT = 2;

    private boolean valid;
    private double xM;
    private double yM;
    private long sourceTimestampMs = Long.MIN_VALUE;
    private boolean candidateValid;
    private double candidateXM;
    private double candidateYM;
    private int candidateCount;

    /** Returns true when the published position changed. */
    public boolean update(double sampleXM, double sampleYM, long sampleTimestampMs) {
        if (!Double.isFinite(sampleXM) || !Double.isFinite(sampleYM)) return false;
        if (!valid) {
            accept(sampleXM, sampleYM, sampleTimestampMs);
            return true;
        }
        if (sampleTimestampMs == sourceTimestampMs) return false;
        if (sampleTimestampMs < sourceTimestampMs) {
            reset();
            accept(sampleXM, sampleYM, sampleTimestampMs);
            return true;
        }

        double elapsedSeconds = Math.max(0.02,
                Math.min(3.0, (sampleTimestampMs - sourceTimestampMs) / 1000.0));
        double displacement = Math.hypot(sampleXM - xM, sampleYM - yM);
        if (displacement <= BASE_GATE_M + MAX_SPEED_M_PER_S * elapsedSeconds) {
            accept(sampleXM, sampleYM, sampleTimestampMs);
            return true;
        }

        double candidateDelta = candidateValid
                ? Math.hypot(sampleXM - candidateXM, sampleYM - candidateYM)
                : Double.POSITIVE_INFINITY;
        if (!candidateValid || candidateDelta > CANDIDATE_GATE_M) {
            candidateValid = true;
            candidateXM = sampleXM;
            candidateYM = sampleYM;
            candidateCount = 1;
            return false;
        }
        candidateXM = sampleXM;
        candidateYM = sampleYM;
        candidateCount++;
        if (candidateCount >= REACQUIRE_SAMPLE_COUNT) {
            accept(sampleXM, sampleYM, sampleTimestampMs);
            return true;
        }
        return false;
    }

    public void reset() {
        valid = false;
        sourceTimestampMs = Long.MIN_VALUE;
        clearCandidate();
    }

    public boolean isValid() { return valid; }
    public double getXM() { return xM; }
    public double getYM() { return yM; }

    private void accept(double acceptedXM, double acceptedYM, long timestampMs) {
        xM = acceptedXM;
        yM = acceptedYM;
        sourceTimestampMs = timestampMs;
        valid = true;
        clearCandidate();
    }

    private void clearCandidate() {
        candidateValid = false;
        candidateCount = 0;
    }
}

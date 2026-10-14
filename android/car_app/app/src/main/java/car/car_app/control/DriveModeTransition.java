package car.car_app.control;

/**
 * Keeps an older status snapshot from undoing a mode button immediately after
 * the command has been sent. The FPGA status packet is assembled over time, so
 * a packet received after the ACK can still contain the pre-command mode.
 */
public final class DriveModeTransition {
    public static final int NONE = -1;

    public enum StatusDecision {
        ACCEPTED,
        CONFIRMED,
        STALE_MISMATCH,
        TIMED_OUT
    }

    private final long timeoutMs;
    private int expectedMode = NONE;
    private long requestedAtMs;

    public DriveModeTransition(long timeoutMs) {
        if (timeoutMs <= 0L) throw new IllegalArgumentException("timeoutMs must be positive");
        this.timeoutMs = timeoutMs;
    }

    public void request(int mode, long nowMs) {
        if (mode != 0 && mode != 1) throw new IllegalArgumentException("mode must be 0 or 1");
        expectedMode = mode;
        requestedAtMs = nowMs;
    }

    public StatusDecision onStatus(int reportedMode, long nowMs) {
        if (expectedMode == NONE) return StatusDecision.ACCEPTED;
        if (reportedMode == expectedMode) {
            clear();
            return StatusDecision.CONFIRMED;
        }
        if (nowMs - requestedAtMs < timeoutMs) return StatusDecision.STALE_MISMATCH;
        clear();
        return StatusDecision.TIMED_OUT;
    }

    public int getExpectedMode() {
        return expectedMode;
    }

    public boolean isPending() {
        return expectedMode != NONE;
    }

    public void clear() {
        expectedMode = NONE;
        requestedAtMs = 0L;
    }
}

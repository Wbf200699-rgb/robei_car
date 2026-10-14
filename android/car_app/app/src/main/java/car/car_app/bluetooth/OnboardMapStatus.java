package car.car_app.bluetooth;

/** Parsed FPGA-resident map and configuration-flash state from a fixed @G0 frame. */
public final class OnboardMapStatus {
    public static final int PAYLOAD_LENGTH = 54;

    private final int sequence;
    private final int flags;
    private final int epoch;
    private final int poseXmm;
    private final int poseYmm;
    private final int heading1024;
    private final int scanCount;
    private final int occupiedCount;
    private final int freeCount;
    private final int droppedCount;
    private final int poseFlags;
    private final int flashOperation;
    private final int flashProgress;
    private final int flashError;
    private final int flashJedecId;
    private final long savedSequence;

    private OnboardMapStatus(
            int sequence,
            int flags,
            int epoch,
            int poseXmm,
            int poseYmm,
            int heading1024,
            int scanCount,
            int occupiedCount,
            int freeCount,
            int droppedCount,
            int poseFlags,
            int flashOperation,
            int flashProgress,
            int flashError,
            int flashJedecId,
            long savedSequence
    ) {
        this.sequence = sequence;
        this.flags = flags;
        this.epoch = epoch;
        this.poseXmm = poseXmm;
        this.poseYmm = poseYmm;
        this.heading1024 = heading1024;
        this.scanCount = scanCount;
        this.occupiedCount = occupiedCount;
        this.freeCount = freeCount;
        this.droppedCount = droppedCount;
        this.poseFlags = poseFlags;
        this.flashOperation = flashOperation;
        this.flashProgress = flashProgress;
        this.flashError = flashError;
        this.flashJedecId = flashJedecId;
        this.savedSequence = savedSequence;
    }

    public static OnboardMapStatus parse(String payload) {
        if (payload == null || payload.length() != PAYLOAD_LENGTH || !isHex(payload)) return null;
        try {
            return new OnboardMapStatus(
                    hex(payload, 0, 2),
                    hex(payload, 2, 4),
                    hex(payload, 4, 6),
                    hex(payload, 6, 10),
                    hex(payload, 10, 14),
                    hex(payload, 14, 17),
                    hex(payload, 17, 21),
                    hex(payload, 21, 25),
                    hex(payload, 25, 29),
                    hex(payload, 29, 33),
                    hex(payload, 33, 35),
                    hex(payload, 35, 36),
                    hex(payload, 36, 38),
                    hex(payload, 38, 40),
                    hex(payload, 40, 46),
                    Long.parseLong(payload.substring(46, 54), 16)
            );
        } catch (NumberFormatException error) {
            return null;
        }
    }

    private static int hex(String value, int start, int end) {
        return Integer.parseInt(value.substring(start, end), 16);
    }

    private static boolean isHex(String value) {
        for (int index = 0; index < value.length(); index++) {
            if (Character.digit(value.charAt(index), 16) < 0) return false;
        }
        return true;
    }

    private boolean flag(int bit) {
        return (flags & (1 << bit)) != 0;
    }

    public int getSequence() { return sequence; }
    public int getFlags() { return flags; }
    public int getEpoch() { return epoch; }
    public int getPoseXmm() { return poseXmm; }
    public int getPoseYmm() { return poseYmm; }
    public int getHeading1024() { return heading1024; }
    public float getHeadingDegrees() { return heading1024 * 360f / 1024f; }
    public int getScanCount() { return scanCount; }
    public int getOccupiedCount() { return occupiedCount; }
    public int getFreeCount() { return freeCount; }
    public int getDroppedCount() { return droppedCount; }
    public int getPoseFlags() { return poseFlags; }
    public int getFlashOperation() { return flashOperation; }
    public int getFlashProgress() { return flashProgress; }
    public int getFlashError() { return flashError; }
    public int getFlashJedecId() { return flashJedecId; }
    public long getSavedSequence() { return savedSequence; }

    public boolean isMapActive() { return flag(0); }
    public boolean isMapPaused() { return flag(1); }
    public boolean isMapValid() { return flag(2); }
    public boolean isMapBusy() { return flag(3); }
    public boolean isFlashReady() { return flag(4); }
    public boolean isFlashBusy() { return flag(5); }
    public boolean isLidarOnline() { return flag(6); }
    public boolean areMotionSensorsReady() { return flag(7); }
}

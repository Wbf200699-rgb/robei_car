package car.car_app.bluetooth;

import java.util.ArrayList;
import java.util.List;

public final class LidarMapAssembler {
    public static final long PARTIAL_TIMEOUT_MS = 1500L;
    private static final int COMPLETE_MASK = (1 << LidarMapChunk.PART_COUNT) - 1;

    private final LidarMapChunk[] chunks = new LidarMapChunk[LidarMapChunk.PART_COUNT];
    private int activeSequence = -1;
    private int activeTick10ms = -1;
    private int receivedMask;
    private long lastPartAtMs;
    private int completedScanCount;
    private int incompleteDropCount;
    private int duplicatePartCount;
    private int rejectedPartCount;

    public LidarMapScan add(LidarMapChunk chunk, long nowMs) {
        expire(nowMs);
        if (activeSequence < 0) {
            start(chunk);
        } else if (!matchesActive(chunk)) {
            if (chunk.getPart() != 0) {
                rejectedPartCount++;
                return null;
            }
            dropPartial();
            start(chunk);
        }

        int partMask = 1 << chunk.getPart();
        if ((receivedMask & partMask) != 0) {
            duplicatePartCount++;
        } else {
            receivedMask |= partMask;
        }
        chunks[chunk.getPart()] = chunk;
        lastPartAtMs = nowMs;
        if (receivedMask != COMPLETE_MASK) {
            return null;
        }

        List<LidarMapPoint> points = new ArrayList<>(LidarMapScan.POINT_COUNT);
        for (LidarMapChunk current : chunks) {
            if (current == null || !matchesActive(current)) {
                dropPartial();
                return null;
            }
            points.addAll(current.getPoints());
        }
        if (points.size() != LidarMapScan.POINT_COUNT) {
            dropPartial();
            return null;
        }

        LidarMapScan scan = new LidarMapScan(activeSequence, activeTick10ms, nowMs, points);
        completedScanCount++;
        clearPartial();
        return scan;
    }

    public void expire(long nowMs) {
        if (activeSequence >= 0 && nowMs - lastPartAtMs > PARTIAL_TIMEOUT_MS) {
            dropPartial();
        }
    }

    public void resetSession() {
        clearPartial();
        completedScanCount = 0;
        incompleteDropCount = 0;
        duplicatePartCount = 0;
        rejectedPartCount = 0;
    }

    public int getCompletedScanCount() {
        return completedScanCount;
    }

    public int getIncompleteDropCount() {
        return incompleteDropCount;
    }

    public int getDuplicatePartCount() {
        return duplicatePartCount;
    }

    public int getRejectedPartCount() {
        return rejectedPartCount;
    }

    public int getPendingPartCount() {
        return Integer.bitCount(receivedMask);
    }

    private void start(LidarMapChunk chunk) {
        activeSequence = chunk.getSequence();
        activeTick10ms = chunk.getTick10ms();
    }

    private boolean matchesActive(LidarMapChunk chunk) {
        return chunk.getSequence() == activeSequence && chunk.getTick10ms() == activeTick10ms;
    }

    private void dropPartial() {
        if (receivedMask != 0) {
            incompleteDropCount++;
        }
        clearPartial();
    }

    private void clearPartial() {
        for (int index = 0; index < chunks.length; index++) {
            chunks[index] = null;
        }
        activeSequence = -1;
        activeTick10ms = -1;
        receivedMask = 0;
        lastPartAtMs = 0L;
    }
}

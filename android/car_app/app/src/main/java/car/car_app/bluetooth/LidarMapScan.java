package car.car_app.bluetooth;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public final class LidarMapScan {
    public static final int POINT_COUNT = 72;

    private final int sequence;
    private final int tick10ms;
    private final long receivedAtMs;
    private final List<LidarMapPoint> points;
    private final int validPointCount;
    private final int highQualityPointCount;

    public LidarMapScan(int sequence, int tick10ms, long receivedAtMs, List<LidarMapPoint> points) {
        if (points.size() != POINT_COUNT) {
            throw new IllegalArgumentException("lidar scan must contain 72 points");
        }
        this.sequence = sequence;
        this.tick10ms = tick10ms;
        this.receivedAtMs = receivedAtMs;
        this.points = Collections.unmodifiableList(new ArrayList<>(points));
        int valid = 0;
        int highQuality = 0;
        for (LidarMapPoint point : points) {
            if (point.isValid()) {
                valid++;
                if (point.getQualityClass() >= 2) {
                    highQuality++;
                }
            }
        }
        validPointCount = valid;
        highQualityPointCount = highQuality;
    }

    public int getSequence() {
        return sequence;
    }

    public int getTick10ms() {
        return tick10ms;
    }

    public long getReceivedAtMs() {
        return receivedAtMs;
    }

    public List<LidarMapPoint> getPoints() {
        return points;
    }

    public int getValidPointCount() {
        return validPointCount;
    }

    public int getHighQualityPointCount() {
        return highQualityPointCount;
    }
}

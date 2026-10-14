package car.car_app.localization;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Deque;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Reduces UWB multipath noise before the four-tag rigid-body solve.
 *
 * <p>Each tag is filtered independently with an odd-sized median window. This
 * is intended for stationary samples immediately before and after a motion
 * experiment; it deliberately does not add latency to the vehicle control
 * loop because UWB is engineering truth only.</p>
 */
public final class UwbWindowedPoseEstimator {
    private final int samplesPerTag;
    private final long maximumSampleAgeMs;
    private final UwbRigidBodyEstimator rigidBodyEstimator;
    private final Map<Integer, Deque<UwbTagFix>> windows = new HashMap<>();

    public UwbWindowedPoseEstimator() {
        this(9, 1_500L, new UwbRigidBodyEstimator(0.08, 300L, 1_500L));
    }

    public UwbWindowedPoseEstimator(int samplesPerTag, long maximumSampleAgeMs,
                                    UwbRigidBodyEstimator rigidBodyEstimator) {
        if (samplesPerTag < 3 || (samplesPerTag & 1) == 0)
            throw new IllegalArgumentException("samplesPerTag must be odd and at least three");
        this.samplesPerTag = samplesPerTag;
        this.maximumSampleAgeMs = maximumSampleAgeMs;
        this.rigidBodyEstimator = rigidBodyEstimator;
        for (int tagId = 0; tagId < 4; tagId++) windows.put(tagId, new ArrayDeque<>());
    }

    public void add(UwbTagFix fix) {
        if (fix.getTagId() < 0 || fix.getTagId() > 3 || !fix.isValid()) return;
        Deque<UwbTagFix> window = windows.get(fix.getTagId());
        window.addLast(fix);
        while (window.size() > samplesPerTag) window.removeFirst();
    }

    public UwbPoseEstimate estimate(long nowMs) {
        List<UwbTagFix> medians = new ArrayList<>();
        for (int tagId = 0; tagId < 4; tagId++) {
            Deque<UwbTagFix> window = windows.get(tagId);
            while (!window.isEmpty()
                    && nowMs - window.peekFirst().getTimestampMs() > maximumSampleAgeMs) {
                window.removeFirst();
            }
            if (window.size() < samplesPerTag) continue;
            List<Double> xValues = new ArrayList<>();
            List<Double> yValues = new ArrayList<>();
            long latestTimestamp = 0L;
            for (UwbTagFix fix : window) {
                xValues.add(fix.getXM());
                yValues.add(fix.getYM());
                latestTimestamp = Math.max(latestTimestamp, fix.getTimestampMs());
            }
            medians.add(new UwbTagFix(tagId, median(xValues), median(yValues),
                    latestTimestamp, true));
        }
        return rigidBodyEstimator.estimate(medians, nowMs);
    }

    public void clear() {
        for (Deque<UwbTagFix> window : windows.values()) window.clear();
    }

    private static double median(List<Double> values) {
        Collections.sort(values);
        return values.get(values.size() / 2);
    }
}

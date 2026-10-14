package car.car_app.uwb;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

/** Atomic UWB state delivered to the UI so tags and vehicle pose cannot mix generations. */
public final class UwbServerSnapshot {
    private final boolean gatewayConnected;
    private final Map<Integer, UwbTagFix> tagFixes;
    private final UwbVehiclePose pose;
    private final long receivedAtMs;

    public UwbServerSnapshot(boolean gatewayConnected, Map<Integer, UwbTagFix> tagFixes,
                             UwbVehiclePose pose, long receivedAtMs) {
        this.gatewayConnected = gatewayConnected;
        this.tagFixes = Collections.unmodifiableMap(new HashMap<>(tagFixes));
        this.pose = pose;
        this.receivedAtMs = receivedAtMs;
    }

    public boolean isGatewayConnected() { return gatewayConnected; }
    public Map<Integer, UwbTagFix> getTagFixes() { return tagFixes; }
    public UwbVehiclePose getPose() { return pose; }
    public long getReceivedAtMs() { return receivedAtMs; }
}

package car.car_app.bluetooth;

public final class Vl53Frame {
    public static final int SENSOR_COUNT = 5;
    public static final int INVALID_DISTANCE = -1;
    private static final int PAYLOAD_HEX_LENGTH = 30;

    private final int sequence;
    private final boolean initializationDone;
    private final int onlineMask;
    private final int validMask;
    private final int[] distanceMm;
    private final int[] rawDistanceMm;
    private final int errorCount;
    private final String rawPayload;

    private Vl53Frame(
            int sequence,
            boolean initializationDone,
            int onlineMask,
            int validMask,
            int[] distanceMm,
            int[] rawDistanceMm,
            int errorCount,
            String rawPayload
    ) {
        this.sequence = sequence;
        this.initializationDone = initializationDone;
        this.onlineMask = onlineMask;
        this.validMask = validMask;
        this.distanceMm = distanceMm;
        this.rawDistanceMm = rawDistanceMm;
        this.errorCount = errorCount;
        this.rawPayload = rawPayload;
    }

    public static Vl53Frame parse(String payload) {
        if (payload == null
                || payload.length() != PAYLOAD_HEX_LENGTH
                || !payload.matches("[0-9A-Fa-f]+")) {
            return null;
        }

        int sequence = parseHex(payload, 0, 2);
        int flags = parseHex(payload, 2, 4);
        int validMask = parseHex(payload, 4, 6) & 0x1F;
        int[] distances = new int[SENSOR_COUNT];
        int[] rawDistances = new int[SENSOR_COUNT];
        for (int sensor = 0; sensor < SENSOR_COUNT; sensor++) {
            int start = 6 + sensor * 4;
            int rawDistance = parseHex(payload, start, start + 4);
            rawDistances[sensor] = rawDistance;
            distances[sensor] = ((validMask & (1 << sensor)) != 0
                    && rawDistance != 0xFFFF)
                    ? rawDistance : INVALID_DISTANCE;
        }

        return new Vl53Frame(
                sequence,
                (flags & 0x80) != 0,
                flags & 0x1F,
                validMask,
                distances,
                rawDistances,
                parseHex(payload, 26, 30),
                payload
        );
    }

    public int getSequence() {
        return sequence;
    }

    public boolean isInitializationDone() {
        return initializationDone;
    }

    public int getOnlineMask() {
        return onlineMask;
    }

    public int getValidMask() {
        return validMask;
    }

    public int getOnlineCount() {
        return Integer.bitCount(onlineMask);
    }

    public boolean isOnline(int sensor) {
        checkSensorIndex(sensor);
        return (onlineMask & (1 << sensor)) != 0;
    }

    public boolean isDistanceValid(int sensor) {
        checkSensorIndex(sensor);
        return (validMask & (1 << sensor)) != 0
                && distanceMm[sensor] != INVALID_DISTANCE;
    }

    public int getDistanceMm(int sensor) {
        checkSensorIndex(sensor);
        return distanceMm[sensor];
    }

    public int[] getDistancesMm() {
        return distanceMm.clone();
    }

    public int getRawDistanceMm(int sensor) {
        checkSensorIndex(sensor);
        return rawDistanceMm[sensor];
    }

    public int[] getRawDistancesMm() {
        return rawDistanceMm.clone();
    }

    public int getErrorCount() {
        return errorCount;
    }

    public String getRawPayload() {
        return rawPayload;
    }

    private static int parseHex(String text, int start, int end) {
        return Integer.parseInt(text.substring(start, end), 16);
    }

    private static void checkSensorIndex(int sensor) {
        if (sensor < 0 || sensor >= SENSOR_COUNT) {
            throw new IllegalArgumentException("VL53 sensor index out of range: " + sensor);
        }
    }
}

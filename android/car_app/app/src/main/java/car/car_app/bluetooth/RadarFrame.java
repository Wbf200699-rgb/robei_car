package car.car_app.bluetooth;

public class RadarFrame {
    private static final int UNKNOWN = -1;
    private static final int SECTOR_COUNT = 8;
    private static final int SECTOR_START = 10;
    private static final int SECTOR_HEX_LENGTH = 4;
    private static final int LEGACY_PAYLOAD_HEX_LENGTH = 42;
    private static final int CURRENT_PAYLOAD_HEX_LENGTH = 44;
    private static final int EXTENDED_PAYLOAD_HEX_LENGTH = 53;

    private final int radarOnline;
    private final int obstacleTriggered;
    private final int lidarObstacleTriggered;
    private final int ultrasonicObstacleTriggered;
    private final int motorBlocked;
    private final int ultrasonicValid;
    private final int ultrasonicTimeout;
    private final int ultrasonicDistanceCm;
    private final int safeDistanceMm;
    private final int frontDistance;
    private final int[] sectorDistancesMm;
    private final int sequence;
    private final String rawPayload;

    private RadarFrame(
            int radarOnline,
            int obstacleTriggered,
            int lidarObstacleTriggered,
            int ultrasonicObstacleTriggered,
            int motorBlocked,
            int ultrasonicValid,
            int ultrasonicTimeout,
            int ultrasonicDistanceCm,
            int safeDistanceMm,
            int frontDistance,
            int[] sectorDistancesMm,
            int sequence,
            String rawPayload
    ) {
        this.radarOnline = radarOnline;
        this.obstacleTriggered = obstacleTriggered;
        this.lidarObstacleTriggered = lidarObstacleTriggered;
        this.ultrasonicObstacleTriggered = ultrasonicObstacleTriggered;
        this.motorBlocked = motorBlocked;
        this.ultrasonicValid = ultrasonicValid;
        this.ultrasonicTimeout = ultrasonicTimeout;
        this.ultrasonicDistanceCm = ultrasonicDistanceCm;
        this.safeDistanceMm = safeDistanceMm;
        this.frontDistance = frontDistance;
        this.sectorDistancesMm = sectorDistancesMm;
        this.sequence = sequence;
        this.rawPayload = rawPayload;
    }

    public static RadarFrame parse(String payload) {
        if (payload == null || !isSupportedLength(payload.length()) || !payload.matches("[0-9A-Fa-f]+")) {
            return null;
        }

        boolean extended = payload.length() == EXTENDED_PAYLOAD_HEX_LENGTH;
        int sectorStart = extended ? 19 : SECTOR_START;
        return new RadarFrame(
                parseOptionalHex(payload, 0, 1),
                parseOptionalHex(payload, extended ? 3 : 1, extended ? 4 : 2),
                extended ? parseOptionalHex(payload, 1, 2) : parseOptionalHex(payload, 1, 2),
                extended ? parseOptionalHex(payload, 2, 3) : UNKNOWN,
                extended ? parseOptionalHex(payload, 4, 5) : UNKNOWN,
                extended ? parseOptionalHex(payload, 5, 6) : UNKNOWN,
                extended ? parseOptionalHex(payload, 6, 7) : UNKNOWN,
                extended ? parseOptionalHex(payload, 11, 15) : UNKNOWN,
                parseOptionalHex(payload, extended ? 7 : 2, extended ? 11 : 6),
                parseOptionalHex(payload, extended ? 15 : 6, extended ? 19 : 10),
                parseSectors(payload, sectorStart),
                parseOptionalHex(payload, sectorStart + SECTOR_COUNT * SECTOR_HEX_LENGTH,
                        sectorStart + SECTOR_COUNT * SECTOR_HEX_LENGTH + 2),
                payload
        );
    }

    public int getRadarOnline() {
        return radarOnline;
    }

    public int getObstacleTriggered() {
        return obstacleTriggered;
    }

    public int getLidarObstacleTriggered() { return lidarObstacleTriggered; }
    public int getUltrasonicObstacleTriggered() { return ultrasonicObstacleTriggered; }
    public int getMotorBlocked() { return motorBlocked; }
    public int getUltrasonicValid() { return ultrasonicValid; }
    public int getUltrasonicTimeout() { return ultrasonicTimeout; }
    public int getUltrasonicDistanceCm() { return ultrasonicDistanceCm; }

    public int getSafeDistanceMm() {
        return safeDistanceMm;
    }

    public int getFrontDistance() {
        return frontDistance;
    }

    public int[] getSectorDistancesMm() {
        return sectorDistancesMm.clone();
    }

    public int getSequence() {
        return sequence;
    }

    public String getRawPayload() {
        return rawPayload;
    }

    public boolean hasValue(int value) {
        return value != UNKNOWN;
    }

    private static int[] parseSectors(String payload, int sectorStart) {
        int[] sectors = new int[SECTOR_COUNT];
        for (int i = 0; i < SECTOR_COUNT; i++) {
            int start = sectorStart + i * SECTOR_HEX_LENGTH;
            sectors[i] = parseOptionalHex(payload, start, start + SECTOR_HEX_LENGTH);
        }
        return sectors;
    }

    private static int parseOptionalHex(String hex, int start, int end) {
        if (hex.length() < end) {
            return UNKNOWN;
        }
        return Integer.parseInt(hex.substring(start, end), 16);
    }

    private static boolean isSupportedLength(int length) {
        return length == LEGACY_PAYLOAD_HEX_LENGTH
                || length == CURRENT_PAYLOAD_HEX_LENGTH
                || length == EXTENDED_PAYLOAD_HEX_LENGTH;
    }
}

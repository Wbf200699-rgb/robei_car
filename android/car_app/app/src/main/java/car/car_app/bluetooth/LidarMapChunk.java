package car.car_app.bluetooth;

import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public final class LidarMapChunk {
    public static final int PART_COUNT = 6;
    public static final int POINTS_PER_PART = 12;
    public static final int LINE_LENGTH = 49;
    private static final int INVALID_DISTANCE_CODE = 0x3FF;

    public enum ErrorReason {
        FORMAT,
        CRC
    }

    public static final class ParseException extends Exception {
        private final ErrorReason reason;

        ParseException(ErrorReason reason, String message) {
            super(message);
            this.reason = reason;
        }

        public ErrorReason getReason() {
            return reason;
        }
    }

    private final int sequence;
    private final int tick10ms;
    private final int part;
    private final List<LidarMapPoint> points;
    private final String rawLine;

    private LidarMapChunk(
            int sequence,
            int tick10ms,
            int part,
            List<LidarMapPoint> points,
            String rawLine
    ) {
        this.sequence = sequence;
        this.tick10ms = tick10ms;
        this.part = part;
        this.points = Collections.unmodifiableList(new ArrayList<>(points));
        this.rawLine = rawLine;
    }

    public static LidarMapChunk parse(String line) throws ParseException {
        if (line == null || line.length() != LINE_LENGTH) {
            throw new ParseException(ErrorReason.FORMAT,
                    "@R length=" + (line == null ? -1 : line.length()) + ", expected 49");
        }
        if (!line.startsWith("@R1")) {
            throw new ParseException(ErrorReason.FORMAT, "unsupported @R version");
        }
        for (int index = 3; index < line.length(); index++) {
            if (!isHex(line.charAt(index))) {
                throw new ParseException(ErrorReason.FORMAT,
                        "non-hex @R byte at index " + index);
            }
        }

        int expectedCrc = parseHex(line, 47, 49, "CRC");
        int actualCrc = crc8(line.substring(1, 47).getBytes(StandardCharsets.US_ASCII));
        if (actualCrc != expectedCrc) {
            throw new ParseException(ErrorReason.CRC,
                    String.format("@R CRC got=%02X expected=%02X", expectedCrc, actualCrc));
        }

        int sequence = parseHex(line, 3, 5, "sequence");
        int tick10ms = parseHex(line, 5, 9, "tick");
        int part = parseHex(line, 9, 10, "part");
        int total = parseHex(line, 10, 11, "part count");
        if (total != PART_COUNT || part < 0 || part >= total) {
            throw new ParseException(ErrorReason.FORMAT,
                    "invalid @R part=" + part + " total=" + total);
        }

        List<LidarMapPoint> points = new ArrayList<>(POINTS_PER_PART);
        for (int localIndex = 0; localIndex < POINTS_PER_PART; localIndex++) {
            int start = 11 + localIndex * 3;
            int packed = parseHex(line, start, start + 3, "point");
            int qualityClass = (packed >>> 10) & 0x03;
            int distanceCode = packed & 0x03FF;
            int binIndex = part * POINTS_PER_PART + localIndex;
            int distanceMm = distanceCode == INVALID_DISTANCE_CODE
                    ? LidarMapPoint.INVALID_DISTANCE_MM
                    : distanceCode * 16;
            points.add(new LidarMapPoint(
                    binIndex,
                    binIndex * 5.0f + 2.5f,
                    distanceMm,
                    qualityClass
            ));
        }
        return new LidarMapChunk(sequence, tick10ms, part, points, line);
    }

    public int getSequence() {
        return sequence;
    }

    public int getTick10ms() {
        return tick10ms;
    }

    public int getPart() {
        return part;
    }

    public List<LidarMapPoint> getPoints() {
        return points;
    }

    public String getRawLine() {
        return rawLine;
    }

    public static int crc8(byte[] bytes) {
        int crc = 0;
        for (byte current : bytes) {
            crc ^= current & 0xFF;
            for (int bit = 0; bit < 8; bit++) {
                crc = (crc & 0x80) != 0 ? ((crc << 1) ^ 0x07) & 0xFF : (crc << 1) & 0xFF;
            }
        }
        return crc;
    }

    private static int parseHex(String text, int start, int end, String field)
            throws ParseException {
        try {
            return Integer.parseInt(text.substring(start, end), 16);
        } catch (RuntimeException exception) {
            throw new ParseException(ErrorReason.FORMAT, "invalid @R " + field);
        }
    }

    private static boolean isHex(char value) {
        return (value >= '0' && value <= '9')
                || (value >= 'A' && value <= 'F')
                || (value >= 'a' && value <= 'f');
    }
}

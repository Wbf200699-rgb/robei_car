package car.car_app.bluetooth;

import java.util.ArrayList;
import java.util.List;

public class BleFrameParser {
    private static final int MAX_LINE_LENGTH = 256;
    private final StringBuilder buffer = new StringBuilder();

    public List<BleFrame> append(byte[] data) {
        List<BleFrame> frames = new ArrayList<>();
        for (byte value : data) {
            char character = (char) (value & 0xFF);
            if (character == '\n') {
                flushBuffer(frames);
            } else if (character == '@') {
                flushBuffer(frames);
                buffer.append(character);
            } else if (character != '\r') {
                buffer.append(character);
                if (buffer.length() > MAX_LINE_LENGTH) {
                    buffer.setLength(0);
                }
            }
        }
        return frames;
    }

    public void reset() {
        buffer.setLength(0);
    }

    private void flushBuffer(List<BleFrame> frames) {
        String line = buffer.toString().trim();
        buffer.setLength(0);
        if (!line.isEmpty()) {
            frames.add(parseLine(line));
        }
    }

    private BleFrame parseLine(String line) {
        if ("@PONG".equals(line)) {
            return new BleFrame(BleFrame.Type.PONG, line, "");
        }
        if (line.startsWith("@K") && line.length() >= 3) {
            return new BleFrame(BleFrame.Type.KEY_ACK, line, line.substring(2, 3));
        }
        if (line.startsWith("@S")) {
            return new BleFrame(BleFrame.Type.STATUS, line, line.substring(2));
        }
        if (line.startsWith("@L")) {
            return new BleFrame(BleFrame.Type.RADAR, line, line.substring(2));
        }
        if (line.startsWith("@V")) {
            return new BleFrame(BleFrame.Type.VL53, line, line.substring(2));
        }
        if (line.startsWith("@M2")) {
            return new BleFrame(BleFrame.Type.MOTION, line, line.substring(2));
        }
        if (line.startsWith("@R")) {
            return new BleFrame(BleFrame.Type.LIDAR_MAP, line, line.substring(2));
        }
        if (line.startsWith("@G0")) {
            return new BleFrame(BleFrame.Type.ONBOARD_MAP_STATUS, line, line.substring(3));
        }
        if (line.startsWith("@G1")) {
            return new BleFrame(BleFrame.Type.ONBOARD_MAP_CHUNK, line, line.substring(3));
        }
        return new BleFrame(BleFrame.Type.UNKNOWN, line, line);
    }
}

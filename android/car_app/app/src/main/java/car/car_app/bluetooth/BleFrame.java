package car.car_app.bluetooth;

public class BleFrame {
    public enum Type {
        PONG,
        KEY_ACK,
        STATUS,
        RADAR,
        VL53,
        MOTION,
        LIDAR_MAP,
        ONBOARD_MAP_STATUS,
        ONBOARD_MAP_CHUNK,
        UNKNOWN
    }

    private final Type type;
    private final String rawText;
    private final String value;

    public BleFrame(Type type, String rawText, String value) {
        this.type = type;
        this.rawText = rawText;
        this.value = value;
    }

    public Type getType() {
        return type;
    }

    public String getRawText() {
        return rawText;
    }

    public String getValue() {
        return value;
    }
}

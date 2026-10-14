package car.car_app.bluetooth;

public enum CarCommand {
    UP("W"),
    DOWN("S"),
    LEFT("A"),
    RIGHT("R"),
    CENTER("C"),
    HEARTBEAT("P"),
    REQUEST_STATUS("?"),
    ENTER_MANUAL_MODE("H"),
    ENTER_AUTO_MODE("O"),
    START_LOCAL_CALIBRATION("X"),
    START_ONBOARD_MAPPING("J"),
    PAUSE_ONBOARD_MAPPING("K"),
    CLEAR_ONBOARD_MAP("6"),
    SAVE_ONBOARD_MAP("7"),
    LOAD_ONBOARD_MAP("8"),
    SYNC_ONBOARD_MAP("9"),
    LEARN_STATIC_GYRO_BIAS("Y"),
    ENABLE_LIDAR("E"),
    DISABLE_LIDAR("Q"),
    TOGGLE_SETTING("G"),
    NEXT_PARAMETER("N"),
    PREVIOUS_PARAMETER("B"),
    INCREASE_PARAMETER("+"),
    DECREASE_PARAMETER("-"),
    SELECT_SPEED("V"),
    SELECT_STEP_LENGTH("T"),
    SELECT_LEFT_FRONT_SPEED("1"),
    SELECT_RIGHT_FRONT_SPEED("2"),
    SELECT_LEFT_REAR_SPEED("3"),
    SELECT_RIGHT_REAR_SPEED("4"),
    SELECT_RADAR_DISTANCE("5");

    private final String payload;

    CarCommand(String payload) {
        this.payload = payload;
    }

    public String getPayload() {
        return payload;
    }
}

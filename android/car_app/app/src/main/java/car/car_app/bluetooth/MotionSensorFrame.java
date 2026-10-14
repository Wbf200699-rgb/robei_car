package car.car_app.bluetooth;

public final class MotionSensorFrame {
    public static final int PAYLOAD_LENGTH = 81;
    public static final double ACCEL_LSB_PER_G = 5460.0;
    public static final double GYRO_LSB_PER_DPS = 131.2;

    private static final int STATUS_PMW_ONLINE = 1 << 0;
    private static final int STATUS_BMI0_ONLINE = 1 << 1;
    private static final int STATUS_BMI1_ONLINE = 1 << 2;
    private static final int STATUS_CALIBRATION_DONE = 1 << 3;
    private static final int STATUS_FUSED_DATA_READY = 1 << 4;
    private static final int STATUS_IMU_DEGRADED = 1 << 5;
    private static final int STATUS_IMU_DISAGREEMENT = 1 << 6;
    private static final int STATUS_FLOW_DATA_SEEN = 1 << 7;

    private final int sequence;
    private final int status;
    private final int flowSqual;
    private final int flowDeltaX;
    private final int flowDeltaY;
    private final int flowAccumX;
    private final int flowAccumY;
    private final ImuSample imu0;
    private final ImuSample imu1;

    private MotionSensorFrame(
            int sequence,
            int status,
            int flowSqual,
            int flowDeltaX,
            int flowDeltaY,
            int flowAccumX,
            int flowAccumY,
            ImuSample imu0,
            ImuSample imu1
    ) {
        this.sequence = sequence;
        this.status = status;
        this.flowSqual = flowSqual;
        this.flowDeltaX = flowDeltaX;
        this.flowDeltaY = flowDeltaY;
        this.flowAccumX = flowAccumX;
        this.flowAccumY = flowAccumY;
        this.imu0 = imu0;
        this.imu1 = imu1;
    }

    public static MotionSensorFrame parse(String payload) {
        if (payload == null || payload.length() != PAYLOAD_LENGTH || payload.charAt(0) != '2') {
            return null;
        }
        try {
            int sequence = parseUnsigned(payload, 1, 4);
            int status = parseUnsigned(payload, 5, 2);
            int squal = parseUnsigned(payload, 7, 2);
            int deltaX = parseSigned16(payload, 9);
            int deltaY = parseSigned16(payload, 13);
            int accumX = parseSigned32(payload, 17);
            int accumY = parseSigned32(payload, 25);

            int offset = 33;
            int[] values = new int[12];
            for (int index = 0; index < values.length; index++) {
                values[index] = parseSigned16(payload, offset + index * 4);
            }
            return new MotionSensorFrame(
                    sequence,
                    status,
                    squal,
                    deltaX,
                    deltaY,
                    accumX,
                    accumY,
                    new ImuSample(values[0], values[1], values[2],
                            values[3], values[4], values[5]),
                    new ImuSample(values[6], values[7], values[8],
                            values[9], values[10], values[11])
            );
        } catch (NumberFormatException | IndexOutOfBoundsException exception) {
            return null;
        }
    }

    private static int parseUnsigned(String text, int offset, int length) {
        return Integer.parseInt(text.substring(offset, offset + length), 16);
    }

    private static int parseSigned16(String text, int offset) {
        return (short) Integer.parseInt(text.substring(offset, offset + 4), 16);
    }

    private static int parseSigned32(String text, int offset) {
        return (int) Long.parseLong(text.substring(offset, offset + 8), 16);
    }

    private boolean hasStatus(int mask) {
        return (status & mask) != 0;
    }

    public int getSequence() {
        return sequence;
    }

    public int getStatus() {
        return status;
    }

    public int getFlowSqual() {
        return flowSqual;
    }

    public int getFlowDeltaX() {
        return flowDeltaX;
    }

    public int getFlowDeltaY() {
        return flowDeltaY;
    }

    public int getFlowAccumX() {
        return flowAccumX;
    }

    public int getFlowAccumY() {
        return flowAccumY;
    }

    public ImuSample getImu0() {
        return imu0;
    }

    public ImuSample getImu1() {
        return imu1;
    }

    public boolean isPmw3901Online() {
        return hasStatus(STATUS_PMW_ONLINE);
    }

    public boolean isBmi0Online() {
        return hasStatus(STATUS_BMI0_ONLINE);
    }

    public boolean isBmi1Online() {
        return hasStatus(STATUS_BMI1_ONLINE);
    }

    public boolean isCalibrationDone() {
        return hasStatus(STATUS_CALIBRATION_DONE);
    }

    public boolean isFusedDataReady() {
        return hasStatus(STATUS_FUSED_DATA_READY);
    }

    public boolean isImuDegraded() {
        return hasStatus(STATUS_IMU_DEGRADED);
    }

    public boolean isImuDisagreement() {
        return hasStatus(STATUS_IMU_DISAGREEMENT);
    }

    public boolean hasFlowData() {
        return hasStatus(STATUS_FLOW_DATA_SEEN);
    }

    public static final class ImuSample {
        private final int accelX;
        private final int accelY;
        private final int accelZ;
        private final int gyroX;
        private final int gyroY;
        private final int gyroZ;

        private ImuSample(int accelX, int accelY, int accelZ,
                          int gyroX, int gyroY, int gyroZ) {
            this.accelX = accelX;
            this.accelY = accelY;
            this.accelZ = accelZ;
            this.gyroX = gyroX;
            this.gyroY = gyroY;
            this.gyroZ = gyroZ;
        }

        public int getAccelX() { return accelX; }
        public int getAccelY() { return accelY; }
        public int getAccelZ() { return accelZ; }
        public int getGyroX() { return gyroX; }
        public int getGyroY() { return gyroY; }
        public int getGyroZ() { return gyroZ; }

        public double getAccelXG() { return accelX / ACCEL_LSB_PER_G; }
        public double getAccelYG() { return accelY / ACCEL_LSB_PER_G; }
        public double getAccelZG() { return accelZ / ACCEL_LSB_PER_G; }
        public double getGyroXDps() { return gyroX / GYRO_LSB_PER_DPS; }
        public double getGyroYDps() { return gyroY / GYRO_LSB_PER_DPS; }
        public double getGyroZDps() { return gyroZ / GYRO_LSB_PER_DPS; }
    }
}

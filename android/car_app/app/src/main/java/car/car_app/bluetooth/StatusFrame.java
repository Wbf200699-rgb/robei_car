package car.car_app.bluetooth;

public class StatusFrame {
    private static final int BASE_PAYLOAD_HEX_LENGTH = 31;
    private static final int LEGACY_PAYLOAD_HEX_LENGTH = 43;
    private static final int CURRENT_PAYLOAD_HEX_LENGTH = 45;
    private static final int EXTENDED_PAYLOAD_HEX_LENGTH = 64;
    private static final int C5_DIAGNOSTIC_PAYLOAD_HEX_LENGTH = 83;
    private static final int LOCAL_CALIBRATION_PAYLOAD_HEX_LENGTH = 96;
    private static final int UNKNOWN = -1;

    private final int state;
    private final int menu;
    private final int activeRouteSteps;
    private final int completedRouteSteps;
    private final int nowX;
    private final int nowY;
    private final int targetX;
    private final int targetY;
    private final String routeInfo;
    private final int stepLength;
    private final int speed;
    private final int settingMode;
    private final int selectedParameter;
    private final int radarOnline;
    private final int manualMode;
    private final int lidarEnabled;
    private final int leftFrontSpeed;
    private final int rightFrontSpeed;
    private final int leftRearSpeed;
    private final int rightRearSpeed;
    private final int safeDistanceMm;
    private final int manualCommand;
    private final int watchdogTriggered;
    private final int motorBlocked;
    private final int c5FrameCount;
    private final int c5LeftFrontPulse;
    private final int c5RightFrontPulse;
    private final int c5LeftRearPulse;
    private final int c5RightRearPulse;
    private final int stopCompleted;
    private final int stopFramesRemaining;
    private final int c5Busy;
    private final int sequence;
    private final int localCalibrationState;
    private final int localCalibrationFlags;
    private final int learnedYawTrim;
    private final int yawCorrection;
    private final int calibrationGyroPeak;
    private final String extraPayload;

    private StatusFrame(
            int state,
            int menu,
            int activeRouteSteps,
            int completedRouteSteps,
            int nowX,
            int nowY,
            int targetX,
            int targetY,
            String routeInfo,
            int stepLength,
            int speed,
            int settingMode,
            int selectedParameter,
            int radarOnline,
            int manualMode,
            int lidarEnabled,
            int leftFrontSpeed,
            int rightFrontSpeed,
            int leftRearSpeed,
            int rightRearSpeed,
            int safeDistanceMm,
            int manualCommand,
            int watchdogTriggered,
            int motorBlocked,
            int c5FrameCount,
            int c5LeftFrontPulse,
            int c5RightFrontPulse,
            int c5LeftRearPulse,
            int c5RightRearPulse,
            int stopCompleted,
            int stopFramesRemaining,
            int c5Busy,
            int sequence,
            int localCalibrationState,
            int localCalibrationFlags,
            int learnedYawTrim,
            int yawCorrection,
            int calibrationGyroPeak,
            String extraPayload
    ) {
        this.state = state;
        this.menu = menu;
        this.activeRouteSteps = activeRouteSteps;
        this.completedRouteSteps = completedRouteSteps;
        this.nowX = nowX;
        this.nowY = nowY;
        this.targetX = targetX;
        this.targetY = targetY;
        this.routeInfo = routeInfo;
        this.stepLength = stepLength;
        this.speed = speed;
        this.settingMode = settingMode;
        this.selectedParameter = selectedParameter;
        this.radarOnline = radarOnline;
        this.manualMode = manualMode;
        this.lidarEnabled = lidarEnabled;
        this.leftFrontSpeed = leftFrontSpeed;
        this.rightFrontSpeed = rightFrontSpeed;
        this.leftRearSpeed = leftRearSpeed;
        this.rightRearSpeed = rightRearSpeed;
        this.safeDistanceMm = safeDistanceMm;
        this.manualCommand = manualCommand;
        this.watchdogTriggered = watchdogTriggered;
        this.motorBlocked = motorBlocked;
        this.c5FrameCount = c5FrameCount;
        this.c5LeftFrontPulse = c5LeftFrontPulse;
        this.c5RightFrontPulse = c5RightFrontPulse;
        this.c5LeftRearPulse = c5LeftRearPulse;
        this.c5RightRearPulse = c5RightRearPulse;
        this.stopCompleted = stopCompleted;
        this.stopFramesRemaining = stopFramesRemaining;
        this.c5Busy = c5Busy;
        this.sequence = sequence;
        this.localCalibrationState = localCalibrationState;
        this.localCalibrationFlags = localCalibrationFlags;
        this.learnedYawTrim = learnedYawTrim;
        this.yawCorrection = yawCorrection;
        this.calibrationGyroPeak = calibrationGyroPeak;
        this.extraPayload = extraPayload;
    }

    public static StatusFrame parse(String payload) {
        if (payload == null || !isSupportedLength(payload.length())) {
            return null;
        }
        if (!payload.matches("[0-9A-Fa-f]+")) {
            return null;
        }
        String hex = payload.substring(0, BASE_PAYLOAD_HEX_LENGTH);
        String extra = payload.substring(BASE_PAYLOAD_HEX_LENGTH);
        boolean localCalibration =
                payload.length() == LOCAL_CALIBRATION_PAYLOAD_HEX_LENGTH;
        boolean c5Diagnostic = payload.length() == C5_DIAGNOSTIC_PAYLOAD_HEX_LENGTH
                || localCalibration;
        boolean extended = payload.length() == EXTENDED_PAYLOAD_HEX_LENGTH || c5Diagnostic;
        return new StatusFrame(
                parseHex(hex, 0, 2),
                parseHex(hex, 2, 3),
                parseHex(hex, 3, 5),
                parseHex(hex, 5, 7),
                parseHex(hex, 7, 9),
                parseHex(hex, 9, 11),
                parseHex(hex, 11, 13),
                parseHex(hex, 13, 15),
                hex.substring(15, 31),
                parseOptionalHex(extra, 0, 4),
                parseOptionalHex(extra, 4, 7),
                parseOptionalHex(extra, extended ? 23 : 7, extended ? 24 : 8),
                parseOptionalHex(extra, extended ? 24 : 8, extended ? 25 : 9),
                parseOptionalHex(extra, extended ? 25 : 9, extended ? 26 : 10),
                parseOptionalHex(extra, extended ? 26 : 10, extended ? 27 : 11),
                parseOptionalHex(extra, extended ? 27 : 11, extended ? 28 : 12),
                extended ? parseOptionalHex(extra, 7, 10) : UNKNOWN,
                extended ? parseOptionalHex(extra, 10, 13) : UNKNOWN,
                extended ? parseOptionalHex(extra, 13, 16) : UNKNOWN,
                extended ? parseOptionalHex(extra, 16, 19) : UNKNOWN,
                extended ? parseOptionalHex(extra, 19, 23) : UNKNOWN,
                extended ? parseOptionalHex(extra, 28, 29) : UNKNOWN,
                extended ? parseOptionalHex(extra, 29, 30) : UNKNOWN,
                extended ? parseOptionalHex(extra, 30, 31) : UNKNOWN,
                c5Diagnostic ? parseOptionalHex(extra, 31, 35) : UNKNOWN,
                c5Diagnostic ? parseOptionalHex(extra, 35, 38) : UNKNOWN,
                c5Diagnostic ? parseOptionalHex(extra, 38, 41) : UNKNOWN,
                c5Diagnostic ? parseOptionalHex(extra, 41, 44) : UNKNOWN,
                c5Diagnostic ? parseOptionalHex(extra, 44, 47) : UNKNOWN,
                c5Diagnostic ? parseOptionalHex(extra, 47, 48) : UNKNOWN,
                c5Diagnostic ? parseOptionalHex(extra, 48, 49) : UNKNOWN,
                c5Diagnostic ? parseOptionalHex(extra, 49, 50) : UNKNOWN,
                parseOptionalHex(extra, c5Diagnostic ? 50 : (extended ? 31 : 12),
                        c5Diagnostic ? 52 : (extended ? 33 : 14)),
                localCalibration ? parseOptionalHex(extra, 52, 53) : UNKNOWN,
                localCalibration ? parseOptionalHex(extra, 53, 55) : UNKNOWN,
                localCalibration ? parseSigned12(extra, 55, 58) : UNKNOWN,
                localCalibration ? parseSigned12(extra, 58, 61) : UNKNOWN,
                localCalibration ? parseOptionalHex(extra, 61, 65) : UNKNOWN,
                extra
        );
    }

    public int getState() {
        return state;
    }

    public int getMenu() {
        return menu;
    }

    public int getActiveRouteSteps() {
        return activeRouteSteps;
    }

    public int getCompletedRouteSteps() {
        return completedRouteSteps;
    }

    public int getNowX() {
        return nowX;
    }

    public int getNowY() {
        return nowY;
    }

    public int getTargetX() {
        return targetX;
    }

    public int getTargetY() {
        return targetY;
    }

    public String getRouteInfo() {
        return routeInfo;
    }

    public int getStepLength() {
        return stepLength;
    }

    public int getSpeed() {
        return speed;
    }

    public int getSettingMode() {
        return settingMode;
    }

    public int getSelectedParameter() {
        return selectedParameter;
    }

    public int getRadarOnline() {
        return radarOnline;
    }

    public int getManualMode() {
        return manualMode;
    }

    public int getLidarEnabled() {
        return lidarEnabled;
    }

    public int getLeftFrontSpeed() { return leftFrontSpeed; }
    public int getRightFrontSpeed() { return rightFrontSpeed; }
    public int getLeftRearSpeed() { return leftRearSpeed; }
    public int getRightRearSpeed() { return rightRearSpeed; }
    public int getSafeDistanceMm() { return safeDistanceMm; }
    public int getManualCommand() { return manualCommand; }
    public int getWatchdogTriggered() { return watchdogTriggered; }
    public int getMotorBlocked() { return motorBlocked; }
    public int getC5FrameCount() { return c5FrameCount; }
    public int getC5LeftFrontPulse() { return c5LeftFrontPulse; }
    public int getC5RightFrontPulse() { return c5RightFrontPulse; }
    public int getC5LeftRearPulse() { return c5LeftRearPulse; }
    public int getC5RightRearPulse() { return c5RightRearPulse; }
    public int getStopCompleted() { return stopCompleted; }
    public int getStopFramesRemaining() { return stopFramesRemaining; }
    public int getC5Busy() { return c5Busy; }
    public int getLocalCalibrationState() { return localCalibrationState; }
    public int getLocalCalibrationFlags() { return localCalibrationFlags; }
    public int getLearnedYawTrim() { return learnedYawTrim; }
    public int getYawCorrection() { return yawCorrection; }
    public int getCalibrationGyroPeak() { return calibrationGyroPeak; }

    public boolean isLocalCalibrationBusy() {
        return localCalibrationFlags >= 0 && (localCalibrationFlags & 0x01) != 0;
    }

    public boolean isLocalCalibrationResultValid() {
        return localCalibrationFlags >= 0 && (localCalibrationFlags & 0x02) != 0;
    }

    public boolean isLocalCalibrationFailed() {
        return localCalibrationFlags >= 0 && (localCalibrationFlags & 0x04) != 0;
    }

    public boolean isLocalCorrectionActive() {
        return localCalibrationFlags >= 0 && (localCalibrationFlags & 0x08) != 0;
    }

    public boolean isGyroPolarityValid() {
        return localCalibrationFlags >= 0 && (localCalibrationFlags & 0x10) != 0;
    }

    public boolean isCalibrationSensorsReady() {
        return localCalibrationFlags >= 0 && (localCalibrationFlags & 0x40) != 0;
    }

    public boolean isCalibrationObstacleActive() {
        return localCalibrationFlags >= 0 && (localCalibrationFlags & 0x80) != 0;
    }

    public int getSequence() {
        return sequence;
    }

    public String getExtraPayload() {
        return extraPayload;
    }

    private static int parseHex(String hex, int start, int end) {
        return Integer.parseInt(hex.substring(start, end), 16);
    }

    private static int parseOptionalHex(String hex, int start, int end) {
        if (hex.length() < end) {
            return UNKNOWN;
        }
        return parseHex(hex, start, end);
    }

    private static int parseSigned12(String hex, int start, int end) {
        int value = parseOptionalHex(hex, start, end);
        if (value < 0) return UNKNOWN;
        return (value & 0x800) != 0 ? value - 0x1000 : value;
    }

    private static boolean isSupportedLength(int length) {
        return length == BASE_PAYLOAD_HEX_LENGTH
                || length == LEGACY_PAYLOAD_HEX_LENGTH
                || length == CURRENT_PAYLOAD_HEX_LENGTH
                || length == EXTENDED_PAYLOAD_HEX_LENGTH
                || length == C5_DIAGNOSTIC_PAYLOAD_HEX_LENGTH
                || length == LOCAL_CALIBRATION_PAYLOAD_HEX_LENGTH;
    }
}

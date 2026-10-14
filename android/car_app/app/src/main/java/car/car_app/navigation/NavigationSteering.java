package car.car_app.navigation;

/** Shared heading convention for path following and local motion estimation. */
public final class NavigationSteering {
    private NavigationSteering() {}

    /** The vehicle/map convention uses increasing heading for a physical right turn. */
    public static char commandForHeadingError(float errorDegrees) {
        return errorDegrees >= 0f ? 'R' : 'A';
    }

    /** Returns a signed heading delta in the established vehicle/map convention. */
    public static float signedTurnDegrees(char command, float magnitudeDegrees) {
        float magnitude = Math.abs(magnitudeDegrees);
        if (command == 'A') return -magnitude;
        if (command == 'R') return magnitude;
        return 0f;
    }

    public static float normalizeHeadingError(float angle) {
        while (angle > 180f) angle -= 360f;
        while (angle <= -180f) angle += 360f;
        return angle;
    }

    /** Rebases the board's clockwise-positive heading onto an aligned world heading. */
    public static float rebaseBoardHeading(float worldReferenceDegrees,
                                           float boardReferenceDegrees,
                                           float boardCurrentDegrees) {
        return normalize360(worldReferenceDegrees
                + normalizeHeadingError(boardCurrentDegrees - boardReferenceDegrees));
    }

    /**
     * Converts a direction vector in the A-C/A-D world frame into the vehicle
     * heading convention. {@link Math#atan2(double, double)} is CCW-positive,
     * while a physical right turn increases the FPGA/App vehicle heading.
     */
    public static float vehicleHeadingForWorldVector(double dx, double dy) {
        return fromMathematicalHeadingDegrees((float) Math.toDegrees(Math.atan2(dy, dx)));
    }

    /** Converts a mathematical CCW-positive world angle to vehicle heading. */
    public static float fromMathematicalHeadingDegrees(float worldHeadingDegrees) {
        return normalize360(-worldHeadingDegrees);
    }

    /** Converts vehicle clockwise-positive heading to mathematical world angle. */
    public static float toMathematicalHeadingDegrees(float vehicleHeadingDegrees) {
        return normalize360(-vehicleHeadingDegrees);
    }

    /** Converts the vehicle's clockwise-positive heading for CCW-positive canvas math. */
    public static float toCanvasHeadingDegrees(float vehicleHeadingDegrees) {
        return toMathematicalHeadingDegrees(vehicleHeadingDegrees);
    }

    /**
     * Produces a deliberately short closed-loop turn pulse. The 0.4 gain
     * leaves room for C5 UART latency and wheel inertia; the next pulse is
     * chosen only after a fresh settled heading is available.
     */
    public static int boundedTurnPulseDurationMs(float errorDegrees,
                                                 float calibratedDegreesPerMs) {
        float rate = calibratedDegreesPerMs > 0f ? calibratedDegreesPerMs : 0.18f;
        int proportional = Math.round(Math.abs(errorDegrees) / rate * 0.4f);
        return Math.max(80, Math.min(180, proportional));
    }

    private static float normalize360(float angle) {
        angle %= 360f;
        return angle < 0f ? angle + 360f : angle;
    }
}

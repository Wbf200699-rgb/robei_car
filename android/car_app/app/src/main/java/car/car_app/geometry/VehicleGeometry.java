package car.car_app.geometry;

/** Generated from config/vehicle_geometry.json. Do not edit by hand. */
public final class VehicleGeometry {
    public static final int SCHEMA_VERSION = 1;

    // Anchor row order is A, B, C, D; world +X is A->C and +Y is A->D.
    public static final double[][] ANCHORS_M = {
            {0, 0, 0},
            {1.5, 1.67, 0},
            {1.5, 0, 0},
            {0, 1.67, 0}
    };
    public static final double ANCHOR_RECTANGLE_WIDTH_M = 1.5;
    public static final double ANCHOR_RECTANGLE_HEIGHT_M = 1.67;

    public static final int FRONT_LEFT_TAG = 1;
    public static final int FRONT_RIGHT_TAG = 2;
    public static final int REAR_LEFT_TAG = 0;
    public static final int REAR_RIGHT_TAG = 3;

    // Row index is tag ID. Vehicle +X is forward and +Y is left.
    public static final double[][] TAG_POSITIONS_M = {
            {-0.11, 0.1, 0},
            {0.11, 0.1, 0},
            {0.11, -0.1, 0},
            {-0.11, -0.1, 0}
    };
    public static final double TAG_RECTANGLE_LENGTH_M = 0.22;
    public static final double TAG_RECTANGLE_WIDTH_M = 0.2;
    public static final double DIMENSION_TOLERANCE_M = 0.05;
    public static final double MAXIMUM_FIX_AGE_S = 0.6;
    public static final double MAXIMUM_FIX_SKEW_S = 0.15;

    public static final boolean UWB_EXTERNAL_TRUTH_ONLY = true;
    public static final double[] PMW3901_POSITION_M = {0, 0, 0.105};
    public static final double PMW3901_CENTER_TO_TAIL_M = 0.2;
    public static final int[] VL53L1X_I2C_ADDRESSES_7BIT = {48, 49, 50, 51, 52};

    private VehicleGeometry() {}
}

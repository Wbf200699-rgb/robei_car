package car.car_app.map;

public enum CarDirection {
    NORTH(0, -1, "北"),
    EAST(1, 0, "东"),
    SOUTH(0, 1, "南"),
    WEST(-1, 0, "西");

    private final int columnDelta;
    private final int rowDelta;
    private final String label;

    CarDirection(int columnDelta, int rowDelta, String label) {
        this.columnDelta = columnDelta;
        this.rowDelta = rowDelta;
        this.label = label;
    }

    public int getColumnDelta() {
        return columnDelta;
    }

    public int getRowDelta() {
        return rowDelta;
    }

    public String getLabel() {
        return label;
    }

    public CarDirection turnLeft() {
        switch (this) {
            case NORTH:
                return WEST;
            case WEST:
                return SOUTH;
            case SOUTH:
                return EAST;
            case EAST:
            default:
                return NORTH;
        }
    }

    public CarDirection turnRight() {
        switch (this) {
            case NORTH:
                return EAST;
            case EAST:
                return SOUTH;
            case SOUTH:
                return WEST;
            case WEST:
            default:
                return NORTH;
        }
    }
}

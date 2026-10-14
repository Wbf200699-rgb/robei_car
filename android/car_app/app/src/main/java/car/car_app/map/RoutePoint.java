package car.car_app.map;

public class RoutePoint {
    private final int row;
    private final int column;

    public RoutePoint(int row, int column) {
        this.row = row;
        this.column = column;
    }

    public int getRow() {
        return row;
    }

    public int getColumn() {
        return column;
    }

    @Override
    public boolean equals(Object object) {
        if (this == object) {
            return true;
        }
        if (!(object instanceof RoutePoint)) {
            return false;
        }
        RoutePoint that = (RoutePoint) object;
        return row == that.row && column == that.column;
    }

    @Override
    public int hashCode() {
        int result = row;
        result = 31 * result + column;
        return result;
    }
}

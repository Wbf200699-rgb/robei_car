package car.car_app.map;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class RouteManager {
    private static final int DEFAULT_ROWS = 10;
    private static final int DEFAULT_COLUMNS = 10;

    private final List<RoutePoint> route = new ArrayList<>();
    // 与 route 并行，记录每个路径点时小车的朝向，用于后退时同步恢复方向
    private final List<CarDirection> directionHistory = new ArrayList<>();
    private RoutePoint carPosition;
    private RoutePoint targetPosition;
    private CarDirection carDirection = CarDirection.NORTH;

    public RouteManager(int startRow, int startColumn) {
        carPosition = new RoutePoint(startRow, startColumn);
        targetPosition = new RoutePoint(startRow, startColumn);
        route.add(carPosition);
        directionHistory.add(carDirection);
    }

    public RoutePoint getCarPosition() {
        return carPosition;
    }

    public CarDirection getCarDirection() {
        return carDirection;
    }

    public List<RoutePoint> getRoute() {
        return Collections.unmodifiableList(route);
    }

    public RoutePoint getTargetPosition() {
        return targetPosition;
    }

    public void setTarget(RoutePoint target) {
        targetPosition = target;
        route.clear();
        route.addAll(buildSimplePath(carPosition, target));
    }

    public void setCarAndTarget(RoutePoint carPosition, RoutePoint targetPosition) {
        this.carPosition = carPosition;
        this.targetPosition = targetPosition;
        route.clear();
        directionHistory.clear();
        route.add(carPosition);
        directionHistory.add(carDirection);
    }

    public void moveTarget(int rowDelta, int columnDelta) {
        int row = clamp(targetPosition.getRow() + rowDelta, 0, DEFAULT_ROWS - 1);
        int column = clamp(targetPosition.getColumn() + columnDelta, 0, DEFAULT_COLUMNS - 1);
        setTarget(new RoutePoint(row, column));
    }

    public void clearRoute() {
        route.clear();
        directionHistory.clear();
        targetPosition = carPosition;
        route.add(carPosition);
        directionHistory.add(carDirection);
    }

    public void setCarPosition(RoutePoint carPosition) {
        this.carPosition = carPosition;
        this.targetPosition = carPosition;
        route.clear();
        directionHistory.clear();
        route.add(carPosition);
        directionHistory.add(carDirection);
    }

    public void moveForward() {
        moveCar(carDirection.getRowDelta(), carDirection.getColumnDelta());
    }

    public void moveBackward() {
        // 回退到路径中的上一个位置，同时恢复对应的朝向
        if (route.size() >= 2) {
            route.remove(route.size() - 1);
            directionHistory.remove(directionHistory.size() - 1);
            carPosition = route.get(route.size() - 1);
            carDirection = directionHistory.get(directionHistory.size() - 1);
            targetPosition = carPosition;
        }
    }

    public void turnLeft() {
        carDirection = carDirection.turnLeft();
    }

    public void turnRight() {
        carDirection = carDirection.turnRight();
    }

    private void moveCar(int rowDelta, int columnDelta) {
        int row = clamp(carPosition.getRow() + rowDelta, 0, DEFAULT_ROWS - 1);
        int column = clamp(carPosition.getColumn() + columnDelta, 0, DEFAULT_COLUMNS - 1);
        RoutePoint nextPosition = new RoutePoint(row, column);
        if (!nextPosition.equals(carPosition)) {
            route.add(nextPosition);
            directionHistory.add(carDirection);
        }
        carPosition = nextPosition;
        targetPosition = nextPosition;
    }

    private List<RoutePoint> buildSimplePath(RoutePoint start, RoutePoint target) {
        List<RoutePoint> path = new ArrayList<>();
        int row = start.getRow();
        int column = start.getColumn();

        while (column != target.getColumn()) {
            column += column < target.getColumn() ? 1 : -1;
            path.add(new RoutePoint(row, column));
        }

        while (row != target.getRow()) {
            row += row < target.getRow() ? 1 : -1;
            path.add(new RoutePoint(row, column));
        }

        return path;
    }

    private int clamp(int value, int min, int max) {
        return Math.max(min, Math.min(max, value));
    }
}

package car.car_app.navigation;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class GridPathPlannerTest {
    @Test
    public void mergesNearbyPointsInflatesVehicleAndFindsDetour() {
        int size = 41;
        byte[] map = new byte[size * size];
        boolean[] allowed = new boolean[map.length];
        java.util.Arrays.fill(allowed, true);
        java.util.Arrays.fill(map, (byte) -4);
        for (int y = 13; y <= 27; y += 2) map[y * size + 20] = 8;

        GridPathPlanner.Result result = new GridPathPlanner().plan(
                map, size, size,
                new GridPathPlanner.Cell(5, 20), new GridPathPlanner.Cell(35, 20),
                allowed, 3, -2, 1, 2);

        assertTrue(result.isFound());
        assertTrue(result.getOccupiedClusters() == 1);
        for (GridPathPlanner.Cell cell : result.getPath()) {
            assertFalse(result.getBlocked()[cell.y * size + cell.x]);
        }
    }

    @Test
    public void permitsUnknownInsideBoundaryButRejectsOutsideCells() {
        int size = 21;
        byte[] map = new byte[size * size];
        boolean[] allowed = new boolean[map.length];
        for (int y = 5; y <= 15; y++) {
            for (int x = 5; x <= 15; x++) allowed[y * size + x] = true;
        }
        GridPathPlanner.Result result = new GridPathPlanner().plan(
                map, size, size,
                new GridPathPlanner.Cell(10, 10), new GridPathPlanner.Cell(14, 14),
                allowed, 3, -2, 1, 2);
        assertTrue(result.isFound());
        for (GridPathPlanner.Cell cell : result.getRawPath()) {
            assertTrue(allowed[cell.y * size + cell.x]);
        }
    }

    @Test
    public void prefersConfirmedFreeCorridorOverShortUnknownRoute() {
        int size = 21;
        byte[] map = new byte[size * size];
        boolean[] allowed = new boolean[map.length];
        java.util.Arrays.fill(allowed, true);
        for (int x = 2; x <= 18; x++) map[6 * size + x] = -5;
        for (int y = 6; y <= 10; y++) {
            map[y * size + 2] = -5;
            map[y * size + 18] = -5;
        }

        GridPathPlanner.Result result = new GridPathPlanner().plan(
                map, size, size,
                new GridPathPlanner.Cell(2, 10), new GridPathPlanner.Cell(18, 10),
                allowed, 3, -2, 0, 0);

        assertTrue(result.isFound());
        assertTrue(result.getRawPath().stream().anyMatch(cell -> cell.y == 6));
    }

    @Test
    public void stopsWhenInflatedWallSealsAnchorRegion() {
        int size = 31;
        byte[] map = new byte[size * size];
        boolean[] allowed = new boolean[map.length];
        java.util.Arrays.fill(map, (byte) -5);
        for (int y = 3; y <= 27; y++) {
            for (int x = 3; x <= 27; x++) allowed[y * size + x] = true;
        }
        for (int y = 3; y <= 27; y++) map[y * size + 15] = 8;

        GridPathPlanner.Result result = new GridPathPlanner().plan(
                map, size, size,
                new GridPathPlanner.Cell(8, 15), new GridPathPlanner.Cell(22, 15),
                allowed, 3, -2, 1, 2);

        assertFalse(result.isFound());
    }

    @Test
    public void projectsFiveBySixMetreEdgeUwbStartIntoSafeInset() {
        int size = 241;
        byte[] map = new byte[size * size];
        boolean[] allowed = new boolean[map.length];
        // 5 m x 6 m anchor rectangle on a 25 mm grid after the 224 mm
        // vehicle-circumradius safety inset used by OccupancyGridView.
        for (int y = 9; y <= 231; y++) {
            for (int x = 29; x <= 211; x++) allowed[y * size + x] = true;
        }

        GridPathPlanner.Result result = new GridPathPlanner().plan(
                map, size, size,
                new GridPathPlanner.Cell(120, 0),
                new GridPathPlanner.Cell(120, 120),
                allowed, 3, -2, 0, 0);

        assertTrue(result.isFound());
        assertTrue(result.getRawPath().get(0).y >= 9);
        assertTrue(result.getFailureReason() == GridPathPlanner.FailureReason.NONE);
    }
}

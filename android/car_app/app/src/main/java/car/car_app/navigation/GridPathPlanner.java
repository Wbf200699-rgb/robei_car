package car.car_app.navigation;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.PriorityQueue;

/** A* planner over a bounded occupancy grid with vehicle-footprint inflation. */
public final class GridPathPlanner {
    public enum FailureReason {
        NONE,
        START_NOT_RECOVERABLE,
        GOAL_NOT_OPEN,
        NO_ROUTE
    }

    public static final class Cell {
        public final int x;
        public final int y;

        public Cell(int x, int y) { this.x = x; this.y = y; }

        @Override public boolean equals(Object value) {
            if (!(value instanceof Cell)) return false;
            Cell other = (Cell) value;
            return x == other.x && y == other.y;
        }

        @Override public int hashCode() { return 31 * x + y; }
    }

    public static final class Result {
        private final List<Cell> path;
        private final List<Cell> rawPath;
        private final boolean[] blocked;
        private final int occupiedClusters;
        private final FailureReason failureReason;

        Result(List<Cell> path, List<Cell> rawPath, boolean[] blocked, int occupiedClusters,
               FailureReason failureReason) {
            this.path = path;
            this.rawPath = rawPath;
            this.blocked = blocked;
            this.occupiedClusters = occupiedClusters;
            this.failureReason = failureReason;
        }

        public List<Cell> getPath() { return path; }
        public List<Cell> getRawPath() { return rawPath; }
        public boolean[] getBlocked() { return blocked.clone(); }
        public int getOccupiedClusters() { return occupiedClusters; }
        public FailureReason getFailureReason() { return failureReason; }
        public boolean isFound() { return !path.isEmpty(); }
    }

    // The chassis executes forward and in-place quarter turns. Cardinal A*
    // therefore produces a path that can be translated into real commands
    // without pretending that the vehicle can move diagonally.
    private static final int[][] NEIGHBORS = {
            {0, -1}, {-1, 0}, {1, 0}, {0, 1}
    };

    private static final class Node implements Comparable<Node> {
        final int index;
        final int score;
        Node(int index, int score) { this.index = index; this.score = score; }
        @Override public int compareTo(Node other) { return Integer.compare(score, other.score); }
    }

    public Result plan(byte[] occupancy, int width, int height, Cell start, Cell goal,
                       boolean[] allowed, int occupiedThreshold, int freeThreshold,
                       int mergeRadiusCells, int inflateRadiusCells) {
        if (occupancy.length != width * height || allowed.length != occupancy.length) {
            throw new IllegalArgumentException("grid dimensions do not match");
        }
        boolean[] occupied = new boolean[occupancy.length];
        for (int i = 0; i < occupancy.length; i++) {
            occupied[i] = allowed[i] && occupancy[i] >= occupiedThreshold;
        }
        boolean[] merged = dilate(occupied, width, height, Math.max(0, mergeRadiusCells), allowed);
        int clusters = countClusters(merged, width, height);
        boolean[] blocked = dilate(merged, width, height, Math.max(0, inflateRadiusCells), allowed);
        for (int i = 0; i < blocked.length; i++) {
            // Unknown cells remain traversable at a higher A* cost. This lets a
            // partially explored map reach a selected goal while still
            // preferring rays that lidar has already confirmed as free.
            if (!allowed[i]) blocked[i] = true;
        }

        // UWB can put the center a few decimetres outside the vehicle-safe
        // inset near a field edge. Search far enough to project that measured
        // start back into the nearest safe cell, but keep the goal strict.
        int startRecoveryRadius = Math.min(32, Math.max(width, height) / 3);
        Cell safeStart = nearestOpen(start, blocked, width, height,
                Math.max(8, startRecoveryRadius));
        Cell safeGoal = nearestOpen(goal, blocked, width, height, 12);
        if (safeStart == null) {
            return new Result(Collections.emptyList(), Collections.emptyList(), blocked, clusters,
                    FailureReason.START_NOT_RECOVERABLE);
        }
        if (safeGoal == null) {
            return new Result(Collections.emptyList(), Collections.emptyList(), blocked, clusters,
                    FailureReason.GOAL_NOT_OPEN);
        }
        List<Cell> raw = aStar(safeStart, safeGoal, blocked, occupancy,
                freeThreshold, width, height);
        FailureReason reason = raw.isEmpty() ? FailureReason.NO_ROUTE : FailureReason.NONE;
        return new Result(simplify(raw, blocked, width, height), raw, blocked, clusters, reason);
    }

    private static boolean[] dilate(boolean[] source, int width, int height, int radius,
                                    boolean[] allowed) {
        boolean[] result = source.clone();
        if (radius == 0) return result;
        int radiusSquared = radius * radius;
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                if (!source[y * width + x]) continue;
                for (int dy = -radius; dy <= radius; dy++) {
                    for (int dx = -radius; dx <= radius; dx++) {
                        if (dx * dx + dy * dy > radiusSquared) continue;
                        int nx = x + dx;
                        int ny = y + dy;
                        if (nx >= 0 && ny >= 0 && nx < width && ny < height
                                && allowed[ny * width + nx]) result[ny * width + nx] = true;
                    }
                }
            }
        }
        return result;
    }

    private static int countClusters(boolean[] occupied, int width, int height) {
        boolean[] seen = new boolean[occupied.length];
        int[] queue = new int[occupied.length];
        int count = 0;
        for (int origin = 0; origin < occupied.length; origin++) {
            if (!occupied[origin] || seen[origin]) continue;
            count++;
            int head = 0;
            int tail = 0;
            queue[tail++] = origin;
            seen[origin] = true;
            while (head < tail) {
                int current = queue[head++];
                int x = current % width;
                int y = current / width;
                for (int[] offset : NEIGHBORS) {
                    int nx = x + offset[0];
                    int ny = y + offset[1];
                    if (nx < 0 || ny < 0 || nx >= width || ny >= height) continue;
                    int next = ny * width + nx;
                    if (occupied[next] && !seen[next]) {
                        seen[next] = true;
                        queue[tail++] = next;
                    }
                }
            }
        }
        return count;
    }

    private static Cell nearestOpen(Cell requested, boolean[] blocked, int width, int height,
                                    int maximumRadius) {
        for (int radius = 0; radius <= maximumRadius; radius++) {
            Cell best = null;
            int bestDistance = Integer.MAX_VALUE;
            for (int dy = -radius; dy <= radius; dy++) {
                for (int dx = -radius; dx <= radius; dx++) {
                    int x = requested.x + dx;
                    int y = requested.y + dy;
                    if (x < 0 || y < 0 || x >= width || y >= height) continue;
                    if (blocked[y * width + x]) continue;
                    int distance = dx * dx + dy * dy;
                    if (distance < bestDistance) {
                        bestDistance = distance;
                        best = new Cell(x, y);
                    }
                }
            }
            if (best != null) return best;
        }
        return null;
    }

    private static List<Cell> aStar(Cell start, Cell goal, boolean[] blocked,
                                    byte[] occupancy, int freeThreshold,
                                    int width, int height) {
        int total = width * height;
        int[] cost = new int[total];
        int[] previous = new int[total];
        Arrays.fill(cost, Integer.MAX_VALUE);
        Arrays.fill(previous, -1);
        int startIndex = start.y * width + start.x;
        int goalIndex = goal.y * width + goal.x;
        PriorityQueue<Node> open = new PriorityQueue<>();
        cost[startIndex] = 0;
        open.add(new Node(startIndex, heuristic(start.x, start.y, goal.x, goal.y)));
        while (!open.isEmpty()) {
            Node node = open.poll();
            if (node.index == goalIndex) break;
            int x = node.index % width;
            int y = node.index / width;
            for (int[] offset : NEIGHBORS) {
                int nx = x + offset[0];
                int ny = y + offset[1];
                if (nx < 0 || ny < 0 || nx >= width || ny >= height) continue;
                int next = ny * width + nx;
                if (blocked[next]) continue;
                int evidence = occupancy[next];
                int traversalCost;
                if (evidence <= freeThreshold) {
                    traversalCost = 10;
                } else if (evidence < 0) {
                    traversalCost = 14;
                } else if (evidence == 0) {
                    traversalCost = 28;
                } else {
                    traversalCost = 45;
                }
                int nextCost = cost[node.index] + traversalCost;
                if (nextCost >= cost[next]) continue;
                cost[next] = nextCost;
                previous[next] = node.index;
                open.add(new Node(next, nextCost + heuristic(nx, ny, goal.x, goal.y)));
            }
        }
        if (cost[goalIndex] == Integer.MAX_VALUE) return Collections.emptyList();
        ArrayList<Cell> path = new ArrayList<>();
        for (int current = goalIndex; current >= 0; current = previous[current]) {
            path.add(new Cell(current % width, current / width));
            if (current == startIndex) break;
        }
        Collections.reverse(path);
        return path;
    }

    private static int heuristic(int x, int y, int gx, int gy) {
        int dx = Math.abs(gx - x);
        int dy = Math.abs(gy - y);
        return 10 * (dx + dy);
    }

    private static List<Cell> simplify(List<Cell> path, boolean[] blocked, int width, int height) {
        if (path.size() <= 2) return path;
        ArrayList<Cell> result = new ArrayList<>();
        int anchor = 0;
        result.add(path.get(0));
        while (anchor < path.size() - 1) {
            int target = path.size() - 1;
            while (target > anchor + 1
                    && !lineClear(path.get(anchor), path.get(target), blocked, width, height)) {
                target--;
            }
            result.add(path.get(target));
            anchor = target;
        }
        return result;
    }

    private static boolean lineClear(Cell start, Cell end, boolean[] blocked,
                                     int width, int height) {
        int x = start.x;
        int y = start.y;
        int dx = Math.abs(end.x - x);
        int sx = x < end.x ? 1 : -1;
        int dy = -Math.abs(end.y - y);
        int sy = y < end.y ? 1 : -1;
        int error = dx + dy;
        while (true) {
            if (blocked[y * width + x] && !(x == start.x && y == start.y)) return false;
            if (x == end.x && y == end.y) return true;
            int doubled = 2 * error;
            if (doubled >= dy) { error += dy; x += sx; }
            if (doubled <= dx) { error += dx; y += sy; }
        }
    }
}

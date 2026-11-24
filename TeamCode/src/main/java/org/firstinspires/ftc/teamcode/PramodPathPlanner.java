
package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;

import java.util.ArrayList;
import java.util.List;

public class PramodPathPlanner {

    // =========================================
    // HIVE BOUNDARIES
    // =========================================

    private static final double HIVE_MIN_X = 36.77;
    private static final double HIVE_MAX_X = 107.23;

    private static final double HIVE_MIN_Y = 42.00;
    private static final double HIVE_MAX_Y = 102.00;

    private static final double CORNER_MARGIN = 1.0;

    // =========================================
    // DANIEL'S WAYPOINTS
    // =========================================

    private static final Pose DANIEL_DRIVE_START =
            new Pose(99.651, 115.099, 0);

    private static final Pose DANIEL_FOURTH =
            new Pose(125, 94.293, 0);

    private static final Pose DANIEL_SIXTH =
            new Pose(105.082, 21.135, 0);

    private static final Pose DANIEL_EIGHTH =
            new Pose(94.859, 17.045, 0);

    private static final Pose DANIEL_TENTH =
            new Pose(87.208, 32.554, 0);

    private static final Pose DANIEL_TWELFTH =
            new Pose(115, 51, 0);

    private static final Pose DANIEL_THIRTEENTH =
            new Pose(108.446, 55.198, 0);

    private static final Pose DANIEL_FOURTEENTH =
            new Pose(109.051, 86.086, 0);

    private static final Pose DANIEL_FIFTEENTH =
            new Pose(109.051, 96.086, 0);

    private static final Pose DANIEL_SIXTEENTH =
            new Pose(96.236, 117.378, 0);

    private static final Pose DANIEL_EIGHTEENTH =
            new Pose(124.840, 47.484, 0);

    // =========================================
    // DISTANCE CALCULATION
    // =========================================

    private static double distance(Pose a, Pose b) {

        double dx = b.getX() - a.getX();
        double dy = b.getY() - a.getY();

        return Math.sqrt(dx * dx + dy * dy);
    }

    // =========================================
    // HIVE COLLISION CHECK
    // =========================================

    private static boolean insideHive(double x, double y) {

        return x >= HIVE_MIN_X
                && x <= HIVE_MAX_X
                && y >= HIVE_MIN_Y
                && y <= HIVE_MAX_Y;
    }

    private static boolean isPoseSafe(Pose pose) {

        return !insideHive(
                pose.getX(),
                pose.getY()
        );
    }

    // =========================================
    // STRAIGHT PATH COLLISION CHECK
    // =========================================

    private static boolean isPathSafe(Pose start, Pose end) {

        double length = distance(start, end);

        int samples = Math.max(
                1,
                (int) Math.ceil(length * 2)
        );

        for (int i = 0; i <= samples; i++) {

            double t = (double) i / samples;

            double x = start.getX()
                    + (end.getX() - start.getX()) * t;

            double y = start.getY()
                    + (end.getY() - start.getY()) * t;

            if (insideHive(x, y)) {
                return false;
            }
        }

        return true;
    }

    // =========================================
    // DIJKSTRA SHORTEST SAFE PATH
    // =========================================

    public static List<Pose> findRoute(Pose start, Pose goal) {

        List<Pose> nodes = new ArrayList<>();

        nodes.add(start);
        nodes.add(goal);

        // Expanded hive corners

        nodes.add(new Pose(
                HIVE_MIN_X - CORNER_MARGIN,
                HIVE_MIN_Y - CORNER_MARGIN,
                0
        ));

        nodes.add(new Pose(
                HIVE_MIN_X - CORNER_MARGIN,
                HIVE_MAX_Y + CORNER_MARGIN,
                0
        ));

        nodes.add(new Pose(
                HIVE_MAX_X + CORNER_MARGIN,
                HIVE_MIN_Y - CORNER_MARGIN,
                0
        ));

        nodes.add(new Pose(
                HIVE_MAX_X + CORNER_MARGIN,
                HIVE_MAX_Y + CORNER_MARGIN,
                0
        ));

        // Daniel's waypoints

        nodes.add(DANIEL_DRIVE_START);
        nodes.add(DANIEL_FOURTH);
        nodes.add(DANIEL_SIXTH);
        nodes.add(DANIEL_EIGHTH);
        nodes.add(DANIEL_TENTH);
        nodes.add(DANIEL_TWELFTH);
        nodes.add(DANIEL_THIRTEENTH);
        nodes.add(DANIEL_FOURTEENTH);
        nodes.add(DANIEL_FIFTEENTH);
        nodes.add(DANIEL_SIXTEENTH);
        nodes.add(DANIEL_EIGHTEENTH);

        int n = nodes.size();

        double[] distances = new double[n];

        int[] previous = new int[n];

        boolean[] visited = new boolean[n];

        for (int i = 0; i < n; i++) {

            distances[i] = Double.POSITIVE_INFINITY;

            previous[i] = -1;
        }

        // Validate start and destination

        if (!isPoseSafe(start) || !isPoseSafe(goal)) {
            return new ArrayList<>();
        }

        distances[0] = 0;

        // Dijkstra main loop

        for (int count = 0; count < n; count++) {

            int current = -1;

            double smallestDistance =
                    Double.POSITIVE_INFINITY;

            for (int i = 0; i < n; i++) {

                if (!visited[i]
                        && distances[i] < smallestDistance) {

                    smallestDistance = distances[i];

                    current = i;
                }
            }

            if (current == -1) {
                break;
            }

            if (current == 1) {
                break;
            }

            visited[current] = true;

            for (int next = 0; next < n; next++) {

                if (visited[next] || next == current) {
                    continue;
                }

                Pose a = nodes.get(current);
                Pose b = nodes.get(next);

                if (!isPoseSafe(b)) {
                    continue;
                }

                if (!isPathSafe(a, b)) {
                    continue;
                }

                double edgeDistance = distance(a, b);

                double newDistance =
                        distances[current] + edgeDistance;

                if (newDistance < distances[next]) {

                    distances[next] = newDistance;

                    previous[next] = current;
                }
            }
        }

        // Reconstruct route

        List<Pose> route = new ArrayList<>();

        if (!Double.isFinite(distances[1])) {
            return route;
        }

        int current = 1;

        while (current != -1) {

            route.add(0, nodes.get(current));

            current = previous[current];
        }

        return route;
    }
}

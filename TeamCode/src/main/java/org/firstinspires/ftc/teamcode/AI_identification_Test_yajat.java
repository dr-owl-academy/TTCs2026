
package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "AI_Identification_test_yajat")
public class AI_identification_Test_yajat extends OpMode {

    private Follower follower;
    private Limelight3A limelight;

    private boolean aiming = false;
    private boolean lastA = false;
    private boolean autoRunning = false;

    private String lastAIClass = "None";
    private double lastAIConfidence = 0;

    private final int AI_PIPELINE = 0;

    // BLUE ALLIANCE
    private final boolean blueTeam = true;

    // =========================================
    // DANIEL'S AUTONOMOUS COORDINATES
    // =========================================

    private static final Pose START_POSE =
            new Pose(99.651, 132.274, Math.toRadians(-90));

    private static final Pose DANIEL_WAYPOINT_1 =
            new Pose(125, 94.293, Math.toRadians(-1));

    private static final Pose DANIEL_WAYPOINT_2 =
            new Pose(105.082, 21.135, Math.toRadians(-108));

    private static final Pose SHOOTING_POSITION =
            new Pose(86, 30, Math.toRadians(90));

    // =========================================
    // HIVE OBSTACLE BOUNDARIES
    // =========================================

    private static final double HIVE_MIN_X = 36.77;
    private static final double HIVE_MAX_X = 107.23;

    private static final double HIVE_MIN_Y = 42.00;
    private static final double HIVE_MAX_Y = 102.00;

    private static final double TARGET_HEADING =
            Math.toRadians(90);

    // =========================================
    // AUTONOMOUS PATH STORAGE
    // =========================================

    private List<PathChain> autonomousPaths =
            new ArrayList<>();

    private int currentPathIndex = 0;

    // =========================================
    // INITIALIZATION
    // =========================================

    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(START_POSE);

        limelight = hardwareMap.get(
                Limelight3A.class,
                "limelight"
        );

        limelight.pipelineSwitch(AI_PIPELINE);
        limelight.start();

        telemetry.addLine("AI Hive Switching Ready");
        telemetry.update();
    }

    @Override
    public void start() {

        follower.startTeleopDrive();
        follower.update();
    }

    // =========================================
    // DISTANCE CALCULATION
    // =========================================

    private double distance(Pose a, Pose b) {

        double dx = b.getX() - a.getX();
        double dy = b.getY() - a.getY();

        return Math.sqrt(dx * dx + dy * dy);
    }

    // =========================================
    // CHECK WHETHER A PATH CROSSES THE HIVE
    // =========================================

    private boolean isPathSafe(Pose start, Pose end) {

        double length = distance(start, end);

        int samples = Math.max(
                1,
                (int) Math.ceil(length)
        );

        for (int i = 0; i <= samples; i++) {

            double t = (double) i / samples;

            double x = start.getX()
                    + (end.getX() - start.getX()) * t;

            double y = start.getY()
                    + (end.getY() - start.getY()) * t;

            boolean insideHive =
                    x > HIVE_MIN_X
                            && x < HIVE_MAX_X
                            && y > HIVE_MIN_Y
                            && y < HIVE_MAX_Y;

            if (insideHive) {
                return false;
            }
        }

        return true;
    }

    // =========================================
    // DIJKSTRA SHORTEST PATH ALGORITHM
    // =========================================

    private List<Pose> dijkstra(Pose start, Pose goal) {

        List<Pose> nodes = new ArrayList<>();

        nodes.add(start);
        nodes.add(goal);

        // Four corners of the expanded hive obstacle

        nodes.add(new Pose(
                HIVE_MIN_X,
                HIVE_MIN_Y,
                TARGET_HEADING
        ));

        nodes.add(new Pose(
                HIVE_MIN_X,
                HIVE_MAX_Y,
                TARGET_HEADING
        ));

        nodes.add(new Pose(
                HIVE_MAX_X,
                HIVE_MIN_Y,
                TARGET_HEADING
        ));

        nodes.add(new Pose(
                HIVE_MAX_X,
                HIVE_MAX_Y,
                TARGET_HEADING
        ));

        int n = nodes.size();

        double[] distances = new double[n];

        int[] previous = new int[n];

        boolean[] visited = new boolean[n];

        for (int i = 0; i < n; i++) {
            distances[i] = Double.POSITIVE_INFINITY;
            previous[i] = -1;
        }

        // Cannot plan a route if either endpoint
        // already overlaps the expanded obstacle.

        if (start.getX() > HIVE_MIN_X
                && start.getX() < HIVE_MAX_X
                && start.getY() > HIVE_MIN_Y
                && start.getY() < HIVE_MAX_Y) {
            return new ArrayList<>();
        }

        if (goal.getX() > HIVE_MIN_X
                && goal.getX() < HIVE_MAX_X
                && goal.getY() > HIVE_MIN_Y
                && goal.getY() < HIVE_MAX_Y) {
            return new ArrayList<>();
        }

        distances[0] = 0;

        // Dijkstra's algorithm

        for (int count = 0; count < n; count++) {

            int current = -1;

            double smallestDistance =
                    Double.POSITIVE_INFINITY;

            // Find the unvisited node with
            // the smallest known distance.

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

            // Examine every possible neighboring node.

            for (int next = 0; next < n; next++) {

                if (visited[next] || next == current) {
                    continue;
                }

                Pose a = nodes.get(current);
                Pose b = nodes.get(next);

                // Only connect nodes if the
                // straight segment is collision-free.

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

        // Reconstruct shortest path.

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

    // =========================================
    // CONVERT DIJKSTRA ROUTE INTO PEDRO PATHS
    // =========================================

    private void addRouteLeg(Pose start, Pose goal) {

        List<Pose> route = dijkstra(start, goal);

        if (route.size() < 2) {

            telemetry.addData(
                    "Dijkstra Error",
                    "No safe route found"
            );

            return;
        }

        for (int i = 0; i < route.size() - 1; i++) {

            Pose pathStart = route.get(i);
            Pose pathEnd = route.get(i + 1);

            PathChain path = follower.pathBuilder()

                    .addPath(new BezierLine(
                            pathStart,
                            pathEnd
                    ))

                    .setLinearHeadingInterpolation(
                            pathStart.getHeading(),
                            TARGET_HEADING
                    )

                    .build();

            autonomousPaths.add(path);
        }
    }

    // =========================================
    // BUILD COMPLETE BLUE UPPER ROUTE
    // =========================================

    private void buildSwitchSideRoute() {

        autonomousPaths.clear();

        Pose currentPose = follower.getPose();

        if (currentPose == null) {
            telemetry.addLine("Pose unavailable");
            return;
        }

        // First: current position to Daniel waypoint 1.

        addRouteLeg(
                currentPose,
                DANIEL_WAYPOINT_1
        );

        // Second: Daniel waypoint 1 to waypoint 2.

        addRouteLeg(
                DANIEL_WAYPOINT_1,
                DANIEL_WAYPOINT_2
        );

        // Third: waypoint 2 to shooting position.

        addRouteLeg(
                DANIEL_WAYPOINT_2,
                SHOOTING_POSITION
        );

        if (autonomousPaths.isEmpty()) {

            telemetry.addLine("No valid autonomous route");
            return;
        }

        currentPathIndex = 0;

        follower.setMaxPower(0.5);

        follower.followPath(
                autonomousPaths.get(currentPathIndex),
                true
        );

        autoRunning = true;
        aiming = false;

        telemetry.addLine("Safe route started");
    }

    // =========================================
    // AUTONOMOUS PATH MANAGEMENT
    // =========================================

    private void updateAutonomous() {

        if (!autoRunning) {
            return;
        }

        if (!follower.isBusy()) {

            currentPathIndex++;

            if (currentPathIndex < autonomousPaths.size()) {

                follower.followPath(
                        autonomousPaths.get(currentPathIndex),
                        true
                );

            } else {

                autoRunning = false;

                follower.setMaxPower(1.0);

                follower.startTeleopDrive();

                telemetry.addLine(
                        "Finished switching sides"
                );
            }
        }
    }

    // =========================================
    // LIMELIGHT CLASSIFICATION
    // =========================================

    private void detectHiveTilt() {

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {

            telemetry.addLine("Waiting for AI detection");
            return;
        }

        List<LLResultTypes.ClassifierResult> detections =
                result.getClassifierResults();

        if (detections == null || detections.isEmpty()) {

            telemetry.addLine("No classifications");
            return;
        }

        LLResultTypes.ClassifierResult best =
                detections.get(0);

        for (LLResultTypes.ClassifierResult detection
                : detections) {

            if (detection.getConfidence()
                    > best.getConfidence()) {

                best = detection;
            }
        }

        lastAIClass = best.getClassName();

        lastAIConfidence = best.getConfidence();

        telemetry.addData("AI Class", lastAIClass);

        telemetry.addData(
                "Confidence",
                lastAIConfidence
        );

        // HIVE TILTED TOWARDS ROBOT:
        // Switch to the other shooting side.

        if (lastAIClass.equalsIgnoreCase(
                "0 Tilt Towards Blue")) {

            telemetry.addLine("Decision: SWITCH SIDES");

            buildSwitchSideRoute();
        }

        // HIVE TILTED AWAY:
        // Remain at the current shooting position.

        else if (lastAIClass.equalsIgnoreCase(
                "1 Tilt Away Blue")) {

            telemetry.addLine("Decision: STAY");

            aiming = false;
        }
    }

    // =========================================
    // MAIN LOOP
    // =========================================

    @Override
    public void loop() {

        follower.update();

        Pose currentPose = follower.getPose();

        if (currentPose == null) {

            telemetry.addLine("Pedro pose unavailable");

            telemetry.update();
            return;
        }

        telemetry.addData("Robot X", currentPose.getX());
        telemetry.addData("Robot Y", currentPose.getY());

        telemetry.addData(
                "Heading",
                Math.toDegrees(currentPose.getHeading())
        );

        // Detect A button rising edge.

        if (gamepad1.a && !lastA && !autoRunning) {

            aiming = !aiming;
        }

        lastA = gamepad1.a;

        // =====================================
        // AUTONOMOUS MODE
        // =====================================

        if (autoRunning) {

            updateAutonomous();

            telemetry.addData(
                    "Mode",
                    "Autonomous"
            );

            telemetry.addData(
                    "Path Index",
                    currentPathIndex
            );

            telemetry.addData(
                    "Path Count",
                    autonomousPaths.size()
            );
        }

        // =====================================
        // AI DETECTION MODE
        // =====================================

        else if (aiming) {

            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true
            );

            detectHiveTilt();

            telemetry.addLine("AI Detection Mode");
        }

        // =====================================
        // NORMAL DRIVER CONTROL
        // =====================================

        else {

            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true
            );

            telemetry.addLine("Driver Control");
        }

        telemetry.addData("Last AI Class", lastAIClass);

        telemetry.addData(
                "Last AI Confidence",
                String.format(
                        "%.2f%%",
                        lastAIConfidence * 100
                )
        );

        telemetry.update();
    }
}

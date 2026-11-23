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

@TeleOp(name = "AI_Hive_SwitchingTheSecond")
public class Dijkstracode2 extends OpMode {

    private Follower follower;
    private Limelight3A limelight;

    // =========================================
    // CONTROL VARIABLES
    // =========================================

    private boolean aiming = false;
    private boolean lastA = false;
    private boolean autoRunning = false;

    private boolean selectedUpperStart = true;
    private boolean appliedUpperStart = true;

    private String lastAIClass = "None";
    private double lastAIConfidence = 0;

    private final int AI_PIPELINE = 0;

    // =========================================
    // BLUE ALLIANCE STARTING POSITIONS
    // =========================================

    // Daniel's original starting position.
    private static final Pose BLUE_UPPER_START =
            new Pose(
                    99.651,
                    132.274,
                    Math.toRadians(-90)
            );

    // Mirrored starting position across field center.
    // Change this if your actual lower starting position differs.
    private static final Pose BLUE_LOWER_START =
            new Pose(
                    99.651,
                    11.726,
                    Math.toRadians(90)
            );

    // =========================================
    // BLUE SHOOTING POSITIONS
    // =========================================

    private static final Pose BLUE_UPPER_SHOOTING =
            new Pose(
                    84,
                    118,
                    Math.toRadians(270)
            );

    private static final Pose BLUE_LOWER_SHOOTING =
            new Pose(
                    86,
                    30,
                    Math.toRadians(90)
            );

    // =========================================
    // HIVE OBSTACLE BOUNDARIES
    // =========================================

    // Expanded for the 17 x 17 inch robot
    // and additional clearance.

    private static final double HIVE_MIN_X = 36.77;
    private static final double HIVE_MAX_X = 107.23;

    private static final double HIVE_MIN_Y = 42.00;
    private static final double HIVE_MAX_Y = 102.00;

    private static final double CORNER_MARGIN = 1.0;

    // =========================================
    // DANIEL'S AUTONOMOUS WAYPOINTS
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
    // PATH STORAGE
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

        follower.setStartingPose(BLUE_UPPER_START);

        limelight = hardwareMap.get(
                Limelight3A.class,
                "limelight"
        );

        limelight.pipelineSwitch(AI_PIPELINE);
        limelight.start();

        telemetry.addLine("AI Hive Switching Ready");
        telemetry.addLine("D-pad UP: Upper start");
        telemetry.addLine("D-pad DOWN: Lower start");

        telemetry.update();
    }

    // =========================================
    // SELECT STARTING SIDE
    // =========================================

    @Override
    public void init_loop() {

        if (gamepad1.dpad_up) {
            selectedUpperStart = true;
        }

        else if (gamepad1.dpad_down) {
            selectedUpperStart = false;
        }

        // Only reset the starting pose when
        // the selected side actually changes.

        if (selectedUpperStart != appliedUpperStart) {

            if (selectedUpperStart) {

                follower.setStartingPose(BLUE_UPPER_START);

            } else {

                follower.setStartingPose(BLUE_LOWER_START);
            }

            appliedUpperStart = selectedUpperStart;
        }

        telemetry.addData(
                "Selected Start",
                selectedUpperStart ? "BLUE UPPER" : "BLUE LOWER"
        );

        telemetry.addData(
                "Starting X",
                selectedUpperStart
                        ? BLUE_UPPER_START.getX()
                        : BLUE_LOWER_START.getX()
        );

        telemetry.addData(
                "Starting Y",
                selectedUpperStart
                        ? BLUE_UPPER_START.getY()
                        : BLUE_LOWER_START.getY()
        );

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
    // CHECK WHETHER A POSITION OVERLAPS HIVE
    // =========================================

    private boolean insideHive(double x, double y) {

        return x >= HIVE_MIN_X
                && x <= HIVE_MAX_X
                && y >= HIVE_MIN_Y
                && y <= HIVE_MAX_Y;
    }

    // =========================================
    // CHECK WHETHER A POSITION IS SAFE
    // =========================================

    private boolean isPoseSafe(Pose pose) {

        return !insideHive(
                pose.getX(),
                pose.getY()
        );
    }

    // =========================================
    // CHECK WHETHER A STRAIGHT PATH IS SAFE
    // =========================================

    private boolean isPathSafe(Pose start, Pose end) {

        double length = distance(start, end);

        // Check every half inch along the path.

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
    // DIJKSTRA SHORTEST PATH ALGORITHM
    // =========================================

    private List<Pose> dijkstra(Pose start, Pose goal) {

        List<Pose> nodes = new ArrayList<>();

        // Node 0 = starting position.
        // Node 1 = destination.

        nodes.add(start);
        nodes.add(goal);

        // =====================================
        // EXPANDED HIVE CORNERS
        // =====================================

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

        // =====================================
        // DANIEL'S AUTONOMOUS WAYPOINTS
        // =====================================

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

        // =====================================
        // VALIDATE START AND DESTINATION
        // =====================================

        if (!isPoseSafe(start)) {

            telemetry.addLine(
                    "ERROR: Starting position overlaps hive boundary"
            );

            return new ArrayList<>();
        }

        if (!isPoseSafe(goal)) {

            telemetry.addLine(
                    "ERROR: Destination overlaps hive boundary"
            );

            return new ArrayList<>();
        }

        distances[0] = 0;

        // =====================================
        // DIJKSTRA MAIN LOOP
        // =====================================

        for (int count = 0; count < n; count++) {

            int current = -1;

            double smallestDistance =
                    Double.POSITIVE_INFINITY;

            // Find closest unvisited node.

            for (int i = 0; i < n; i++) {

                if (!visited[i]
                        && distances[i] < smallestDistance) {

                    smallestDistance = distances[i];

                    current = i;
                }
            }

            // No reachable nodes remain.

            if (current == -1) {
                break;
            }

            // Destination reached.

            if (current == 1) {
                break;
            }

            visited[current] = true;

            // =================================
            // EXAMINE NEIGHBORING NODES
            // =================================

            for (int next = 0; next < n; next++) {

                if (visited[next] || next == current) {
                    continue;
                }

                Pose a = nodes.get(current);
                Pose b = nodes.get(next);

                if (!isPoseSafe(b)) {
                    continue;
                }

                // Reject paths that intersect hive.

                if (!isPathSafe(a, b)) {
                    continue;
                }

                double edgeDistance = distance(a, b);

                double newDistance =
                        distances[current] + edgeDistance;

                // Update shortest known distance.

                if (newDistance < distances[next]) {

                    distances[next] = newDistance;

                    previous[next] = current;
                }
            }
        }

        // =====================================
        // RECONSTRUCT ROUTE
        // =====================================

        List<Pose> route = new ArrayList<>();

        if (!Double.isFinite(distances[1])) {

            telemetry.addLine(
                    "Dijkstra: No safe route found"
            );

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
    // BUILD ROUTE FOR EITHER BLUE SIDE
    // =========================================

    private void buildSwitchSideRoute() {

        autonomousPaths.clear();

        Pose currentPose = follower.getPose();

        if (currentPose == null) {

            telemetry.addLine("Pose unavailable");

            return;
        }

        // =====================================
        // DETERMINE CURRENT FIELD SIDE
        // =====================================

        boolean currentlyUpper =
                currentPose.getY() > 72;

        Pose destination;

        if (currentlyUpper) {

            // Upper BLUE -> Lower BLUE

            destination = BLUE_LOWER_SHOOTING;

            telemetry.addLine(
                    "BLUE UPPER -> BLUE LOWER"
            );

        } else {

            // Lower BLUE -> Upper BLUE

            destination = BLUE_UPPER_SHOOTING;

            telemetry.addLine(
                    "BLUE LOWER -> BLUE UPPER"
            );
        }

        telemetry.addData(
                "Current Y",
                currentPose.getY()
        );

        telemetry.addData(
                "Destination X",
                destination.getX()
        );

        telemetry.addData(
                "Destination Y",
                destination.getY()
        );

        // =====================================
        // CALCULATE DIJKSTRA ROUTE
        // =====================================

        List<Pose> route = dijkstra(
                currentPose,
                destination
        );

        if (route.size() < 2) {

            telemetry.addLine(
                    "ERROR: Could not create safe route"
            );

            aiming = false;

            return;
        }

        // =====================================
        // CONVERT ROUTE INTO PEDRO PATHS
        // =====================================

        for (int i = 0; i < route.size() - 1; i++) {

            Pose pathStart = route.get(i);

            Pose pathEnd = route.get(i + 1);

            PathChain path = follower.pathBuilder()

                    .addPath(
                            new BezierLine(
                                    pathStart,
                                    pathEnd
                            )
                    )

                    .setLinearHeadingInterpolation(
                            pathStart.getHeading(),
                            destination.getHeading()
                    )

                    .build();

            autonomousPaths.add(path);
        }

        // =====================================
        // START AUTONOMOUS MOVEMENT
        // =====================================

        if (autonomousPaths.isEmpty()) {

            telemetry.addLine("No paths generated");

            aiming = false;

            return;
        }

        currentPathIndex = 0;

        follower.setMaxPower(0.5);

        follower.followPath(
                autonomousPaths.get(0),
                true
        );

        autoRunning = true;

        aiming = false;

        telemetry.addLine(
                "Safe autonomous route started"
        );

        telemetry.addData(
                "Number of paths",
                autonomousPaths.size()
        );
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

            // More path segments remain.

            if (currentPathIndex < autonomousPaths.size()) {

                follower.followPath(
                        autonomousPaths.get(currentPathIndex),
                        true
                );
            }

            // All paths completed.

            else {

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
    // LIMELIGHT AI DETECTION
    // =========================================

    private void detectHiveTilt() {

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {

            telemetry.addLine(
                    "Waiting for valid Limelight result"
            );

            return;
        }

        List<LLResultTypes.ClassifierResult> detections =
                result.getClassifierResults();

        if (detections == null || detections.isEmpty()) {

            telemetry.addLine(
                    "No AI classifications"
            );

            return;
        }

        // =====================================
        // FIND HIGHEST CONFIDENCE CLASS
        // =====================================

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

        telemetry.addData(
                "AI Class",
                lastAIClass
        );

        telemetry.addData(
                "AI Confidence",
                lastAIConfidence
        );

        // =====================================
        // TILT TOWARDS
        // =====================================

        if (lastAIClass.equalsIgnoreCase(
                "0 Tilt Towards Blue")) {

            telemetry.addLine(
                    "DECISION: SWITCH SIDES"
            );

            buildSwitchSideRoute();
        }

        // =====================================
        // TILT AWAY
        // =====================================

        else if (lastAIClass.equalsIgnoreCase(
                "1 Tilt Away Blue")) {

            telemetry.addLine(
                    "DECISION: STAY IN POSITION"
            );

            // No movement.
            // Robot remains at its current position.

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

            telemetry.addLine(
                    "Pedro Pathing pose unavailable"
            );

            telemetry.update();

            return;
        }

        // =====================================
        // ROBOT POSITION TELEMETRY
        // =====================================

        telemetry.addData(
                "Robot X",
                currentPose.getX()
        );

        telemetry.addData(
                "Robot Y",
                currentPose.getY()
        );

        telemetry.addData(
                "Robot Heading",
                Math.toDegrees(
                        currentPose.getHeading()
                )
        );

        telemetry.addData(
                "Current Side",
                currentPose.getY() > 72
                        ? "BLUE UPPER"
                        : "BLUE LOWER"
        );

        // =====================================
        // A BUTTON ACTIVATION
        // =====================================

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
                    "Total Paths",
                    autonomousPaths.size()
            );
        }

        // =====================================
        // AI DETECTION MODE
        // =====================================

        else if (aiming) {

            // Allow driver to move while
            // AI detection is active.

            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true
            );

            detectHiveTilt();

            telemetry.addLine(
                    "AI Detection Mode"
            );
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

            telemetry.addLine(
                    "Driver Control"
            );
        }

        // =====================================
        // LAST AI INFORMATION
        // =====================================

        telemetry.addData(
                "Last AI Class",
                lastAIClass
        );

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
package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Red River AI Autonomous")
public class Karimsautonirun extends OpMode {

    private Follower follower;
    private Limelight3A limelight;
    private final ElapsedTime timer = new ElapsedTime();

    // =========================================
    // FIELD POSITIONS
    // =========================================

    private static final Pose START =
            new Pose(66.5, 132.6, Math.toRadians(270));

    private static final Pose UPPER_HIVE =
            new Pose(47.4, 121.3, Math.toRadians(-70));

    private static final Pose FIRST_FLOWER =
            new Pose(53.2, 125.9, Math.toRadians(108));

    private static final Pose SECOND_FLOWER =
            new Pose(14.8, 40.7, Math.toRadians(157));

    private static final Pose GARDEN =
            new Pose(10.5, 10.8, Math.toRadians(-90));

    private static final Pose LOWER_HIVE =
            new Pose(42.6, 29.0, Math.toRadians(425));

    private static final Pose PARK =
            new Pose(10.67, 107.47, Math.toRadians(180));

    // =========================================
    // MANUAL HIVE-AVOIDANCE WAYPOINTS
    // =========================================

    // These keep the robot to the LEFT of the hive
    // when traveling between its upper and lower sides.

    private static final double LEFT_X = 35.0;
    private static final double LOWER_Y = 41.0;
    private static final double UPPER_Y = 103.0;

    // =========================================
    // LIMELIGHT LABELS
    // =========================================

    private static final String TILT_AWAY =
            "0 Tilt AWAY Red";

    private static final String TILT_TOWARDS =
            "1 Tilt TOWARDS Red";

    // =========================================
    // SETTINGS
    // =========================================

    private static final int MAX_SHOTS = 4;

    private static final double SHOOT_WAIT = 3.0;
    private static final double COLLECT_WAIT = 2.0;
    private static final double DETECTION_WAIT = 0.5;

    // =========================================
    // STATE MACHINE
    // =========================================

    private enum State {
        MOVING,
        WAITING,
        DETECTING,
        DONE
    }

    private enum Action {
        DETECT,
        SHOOT,
        SHOOT_FINISHED,
        COLLECT,
        COLLECT_FINISHED,
        PARK
    }

    private State state = State.DONE;
    private Action nextAction;

    private double waitDuration;

    private final List<PathChain> activePaths =
            new ArrayList<>();

    private int pathIndex = 0;

    // =========================================
    // CYCLE TRACKING
    // =========================================

    private int shotsCompleted = 0;

    // true = upper hive
    // false = lower hive

    private boolean currentlyAtUpperHive = true;

    private final Pose[] collectionPoints = {
            FIRST_FLOWER,
            SECOND_FLOWER,
            GARDEN
    };

    private final boolean[] collected = {
            false,
            false,
            false
    };

    private int activeCollectionIndex = -1;

    // =========================================
    // INITIALIZATION
    // =========================================

    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(START);
        follower.setMaxPower(0.5);

        limelight = hardwareMap.get(
                Limelight3A.class,
                "limelight"
        );

        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addLine("Red River Autonomous Ready");
        telemetry.update();
    }

    // =========================================
    // START
    // =========================================

    @Override
    public void start() {

        // Robot already has balls loaded.
        // Travel to upper hive and DETECT FIRST.

        currentlyAtUpperHive = true;

        moveTo(
                UPPER_HIVE,
                Action.DETECT,
                DETECTION_WAIT
        );
    }

    // =========================================
    // MAIN LOOP
    // =========================================

    @Override
    public void loop() {

        follower.update();

        switch (state) {

            case MOVING:

                if (!follower.isBusy()) {

                    pathIndex++;

                    if (pathIndex < activePaths.size()) {

                        follower.followPath(
                                activePaths.get(pathIndex),
                                true
                        );

                    } else {

                        beginWait(
                                waitDuration,
                                nextAction
                        );
                    }
                }

                break;

            case WAITING:

                if (timer.seconds() >= waitDuration) {
                    performAction(nextAction);
                }

                break;

            case DETECTING:

                detectHive();
                break;

            case DONE:
                break;
        }

        telemetry.addData("State", state);
        telemetry.addData("Shots", shotsCompleted);

        telemetry.addData(
                "Current Hive",
                currentlyAtUpperHive ? "UPPER" : "LOWER"
        );

        telemetry.addData("First Flower", collected[0]);
        telemetry.addData("Second Flower", collected[1]);
        telemetry.addData("Garden", collected[2]);

        telemetry.update();
    }

    // =========================================
    // BUILD A ROUTE WITHOUT PRAMOD PATH PLANNER
    // =========================================

    private List<Pose> buildRoute(Pose start, Pose goal) {

        List<Pose> route = new ArrayList<>();

        route.add(start);

        boolean goingFromTopToBottom =
                start.getY() > UPPER_Y &&
                        goal.getY() < LOWER_Y;

        boolean goingFromBottomToTop =
                start.getY() < LOWER_Y &&
                        goal.getY() > UPPER_Y;

        if (goingFromTopToBottom) {

            route.add(new Pose(
                    LEFT_X,
                    UPPER_Y,
                    start.getHeading()
            ));

            route.add(new Pose(
                    LEFT_X,
                    LOWER_Y,
                    goal.getHeading()
            ));
        }

        else if (goingFromBottomToTop) {

            route.add(new Pose(
                    LEFT_X,
                    LOWER_Y,
                    start.getHeading()
            ));

            route.add(new Pose(
                    LEFT_X,
                    UPPER_Y,
                    goal.getHeading()
            ));
        }

        route.add(goal);

        return route;
    }

    // =========================================
    // MOVEMENT
    // =========================================

    private void moveTo(
            Pose destination,
            Action action,
            double waitAfterArrival
    ) {

        List<Pose> route = buildRoute(
                follower.getPose(),
                destination
        );

        activePaths.clear();

        for (int i = 0; i < route.size() - 1; i++) {

            Pose startPose = route.get(i);
            Pose endPose = route.get(i + 1);

            double distance = Math.hypot(
                    endPose.getX() - startPose.getX(),
                    endPose.getY() - startPose.getY()
            );

            if (distance < 0.1) {
                continue;
            }

            PathChain path = follower.pathBuilder()
                    .addPath(
                            new BezierLine(startPose, endPose)
                    )
                    .setLinearHeadingInterpolation(
                            startPose.getHeading(),
                            endPose.getHeading()
                    )
                    .build();

            activePaths.add(path);
        }

        nextAction = action;
        waitDuration = waitAfterArrival;
        pathIndex = 0;

        if (activePaths.isEmpty()) {

            beginWait(waitAfterArrival, action);
            return;
        }

        state = State.MOVING;

        follower.followPath(
                activePaths.get(pathIndex),
                true
        );
    }

    // =========================================
    // WAITING
    // =========================================

    private void beginWait(
            double seconds,
            Action action
    ) {

        waitDuration = seconds;
        nextAction = action;

        timer.reset();
        state = State.WAITING;
    }

    // =========================================
    // ACTION HANDLER
    // =========================================

    private void performAction(Action action) {

        switch (action) {

            case DETECT:

                state = State.DETECTING;
                break;

            case SHOOT:

                shoot();

                shotsCompleted++;

                beginWait(
                        SHOOT_WAIT,
                        Action.SHOOT_FINISHED
                );

                break;

            case SHOOT_FINISHED:

                if (shotsCompleted >= MAX_SHOTS) {

                    moveTo(
                            PARK,
                            Action.PARK,
                            0
                    );

                } else {

                    activeCollectionIndex =
                            getNearestUncollectedSource();

                    if (activeCollectionIndex == -1) {

                        moveTo(
                                PARK,
                                Action.PARK,
                                0
                        );

                    } else {

                        moveTo(
                                collectionPoints[activeCollectionIndex],
                                Action.COLLECT,
                                0
                        );
                    }
                }

                break;

            case COLLECT:

                collect();

                collected[activeCollectionIndex] = true;

                beginWait(
                        COLLECT_WAIT,
                        Action.COLLECT_FINISHED
                );

                break;

            case COLLECT_FINISHED:

                // Return to the hive where the robot
                // most recently shot.

                moveTo(
                        currentlyAtUpperHive
                                ? UPPER_HIVE
                                : LOWER_HIVE,
                        Action.DETECT,
                        DETECTION_WAIT
                );

                break;

            case PARK:

                state = State.DONE;
                break;
        }
    }

    // =========================================
    // CHOOSE NEAREST UNUSED COLLECTION SOURCE
    // =========================================

    private int getNearestUncollectedSource() {

        Pose currentHive = currentlyAtUpperHive
                ? UPPER_HIVE
                : LOWER_HIVE;

        int bestIndex = -1;
        double shortestDistance = 999999999.0;

        for (int i = 0; i < collectionPoints.length; i++) {

            if (collected[i]) {
                continue;
            }

            Pose point = collectionPoints[i];

            double distance = Math.hypot(
                    point.getX() - currentHive.getX(),
                    point.getY() - currentHive.getY()
            );

            if (distance < shortestDistance) {

                shortestDistance = distance;
                bestIndex = i;
            }
        }

        return bestIndex;
    }

    // =========================================
    // LIMELIGHT DETECTION
    // =========================================

    private void detectHive() {

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return;
        }

        List<LLResultTypes.ClassifierResult> results =
                result.getClassifierResults();

        if (results == null || results.isEmpty()) {
            return;
        }

        LLResultTypes.ClassifierResult best = null;
        double highestConfidence = -1;

        for (LLResultTypes.ClassifierResult classification : results) {

            if (classification.getConfidence() > highestConfidence) {

                highestConfidence =
                        classification.getConfidence();

                best = classification;
            }
        }

        if (best == null) {
            return;
        }

        String label = best.getClassName();

        telemetry.addData("Detected Label", label);
        telemetry.addData("Confidence", highestConfidence);

        // TOWARDS:
        // Switch to the opposite hive.
        // Do not shoot at the detection position.

        if (label.equalsIgnoreCase(TILT_TOWARDS)) {

            currentlyAtUpperHive = !currentlyAtUpperHive;

            Pose oppositeHive = currentlyAtUpperHive
                    ? UPPER_HIVE
                    : LOWER_HIVE;

            moveTo(
                    oppositeHive,
                    Action.SHOOT,
                    0
            );
        }

        // AWAY:
        // Stay at the current hive and shoot.

        else if (label.equalsIgnoreCase(TILT_AWAY)) {

            moveTo(
                    currentlyAtUpperHive
                            ? UPPER_HIVE
                            : LOWER_HIVE,
                    Action.SHOOT,
                    0
            );
        }
    }

    // =========================================
    // MECHANISM METHODS
    // =========================================

    private void shoot() {

        // INSERT YOUR ACTUAL SHOOTER CODE HERE.

    }

    private void collect() {

        // INSERT YOUR ACTUAL INTAKE CODE HERE.

    }

    // =========================================
    // STOP
    // =========================================

    @Override
    public void stop() {

        if (limelight != null) {
            limelight.stop();
        }
    }
}
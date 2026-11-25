
package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Auton Blue Lower")
public class Auton_Blue_Lower extends OpMode {

    // =========================================
    // AUTONOMOUS STATES
    // =========================================

    private enum AutoState {

        INITIAL_AI,

        NEW_POSITION_PATH,
        WAIT_NEW_POSITION_PATH,
        WAIT_INITIAL_WAYPOINT,
        WAIT_BLUE_LOWER_POSITION,
        WAIT_SECOND_AI_POSITION,

        SECOND_AI,

        BRANCH1_CONNECT,
        WAIT_BRANCH1_CONNECT,

        PATH1_TO_BRANCH,
        WAIT_BRANCH1_CONNECTOR,

        TURN_TO_125,
        WAIT_TURN_125,

        THIRD_AI,

        BRANCH1_1_PATH,
        WAIT_BRANCH1_1_PATH,
        WAIT_BRANCH1_1_TO_SHARED,

        THIRD_BRANCH1_2_PATH,
        WAIT_THIRD_BRANCH1_2_PATH,

        BRANCH2_PATH,
        WAIT_BRANCH2_PATH,

        SHARED_PATH1,
        WAIT_SHARED_PATH1,
        WAIT_SHARED_PATH2,
        WAIT_SHARED_PATH3,
        WAIT_SHARED_PATH4,

        FOURTH_AI,

        UPPER_SHOOTING_PATH,
        WAIT_UPPER_SHOOTING_PATH,

        REVERSE_PATH1,
        WAIT_REVERSE_PATH1,
        WAIT_REVERSE_PATH2,

        COMPLETE
    }

    // =========================================
    // ROBOT AND LIMELIGHT
    // =========================================

    private Follower follower;
    private Limelight3A limelight;

    private AutoState autoState = AutoState.INITIAL_AI;

    private static final String TILT_AWAY_BLUE =
            "1 Tilt Away Blue";

    private static final String TILT_TOWARD_BLUE =
            "0 Tilt Towards Blue";

    private String secondAI = "";
    private String thirdAI = "";
    private String fourthAI = "";

    // =========================================
    // ROBOT POSES
    // =========================================

    private static final Pose START_POSE =
            new Pose(117, 8, Math.toRadians(90));

    private static final Pose NEW_POSITION =
            new Pose(103, 30, Math.toRadians(-90));

    private static final Pose INITIAL_WAYPOINT =
            new Pose(94, 13, Math.toRadians(-90));

    private static final Pose BLUE_LOWER_SHOOTING_POSITION =
            new Pose(85, 29, Math.toRadians(90));

    private static final Pose SECOND_AI_POSITION =
            new Pose(94, 19, Math.toRadians(90));

    private static final Pose BRANCH1_POSITION =
            new Pose(121, 131, Math.toRadians(0));

    private static final Pose THIRD_AI_POSITION =
            new Pose(121, 131, Math.toRadians(-125));

    private static final Pose SHARED_POSITION1 =
            new Pose(107, 41, Math.toRadians(0));

    private static final Pose SHARED_POSITION2 =
            new Pose(107, 101, Math.toRadians(0));

    private static final Pose BLUE_UPPER_SHOOTING_POSITION =
            new Pose(85, 114, Math.toRadians(-90));

    private static final Pose SHARED_POSITION3 =
            new Pose(128, 94, Math.toRadians(0));

    private static final Pose FOURTH_AI_POSITION =
            new Pose(120, 94, Math.toRadians(170));

    // =========================================
    // PATH ARRAY
    // =========================================

    private PathChain[] paths = new PathChain[19];

    // =========================================
    // INIT
    // =========================================

    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);

        limelight = hardwareMap.get(
                Limelight3A.class,
                "limelight"
        );

        limelight.pipelineSwitch(0);
        limelight.start();

        follower.setStartingPose(START_POSE);

        buildPaths();

        autoState = AutoState.INITIAL_AI;

        telemetry.addLine("Autonomous ready");
        telemetry.update();
    }

    @Override
    public void start() {
        autoState = AutoState.INITIAL_AI;
    }

    // =========================================
    // MAIN LOOP
    // =========================================

    @Override
    public void loop() {

        follower.update();

        autonomousPathUpdate();

        Pose currentPose = follower.getPose();

        telemetry.addData("X", currentPose.getX());
        telemetry.addData("Y", currentPose.getY());

        telemetry.addData(
                "Heading",
                Math.toDegrees(currentPose.getHeading())
        );

        telemetry.addData("State", autoState);
        telemetry.addData("Follower Busy", follower.isBusy());

        telemetry.addData("Second AI", secondAI);
        telemetry.addData("Third AI", thirdAI);
        telemetry.addData("Fourth AI", fourthAI);

        if (autoState == AutoState.COMPLETE) {
            telemetry.addLine("Autonomous complete");
        }

        telemetry.update();
    }

    // =========================================
    // BUILD ALL PATHS
    // =========================================

    private void buildPaths() {

        // Initial route

        paths[0] = makePath(START_POSE, NEW_POSITION);

        paths[1] = makePath(NEW_POSITION, INITIAL_WAYPOINT);

        paths[2] = makePath(
                INITIAL_WAYPOINT,
                BLUE_LOWER_SHOOTING_POSITION
        );

        paths[3] = makePath(
                BLUE_LOWER_SHOOTING_POSITION,
                SECOND_AI_POSITION
        );


        // Branch 1 connection

        paths[4] = makePath(
                SECOND_AI_POSITION,
                SHARED_POSITION1
        );


        // Third AI Branch 1-2

        paths[6] = makePath(
                THIRD_AI_POSITION,
                SHARED_POSITION2
        );


        // Branch 2

        paths[8] = makePath(
                SECOND_AI_POSITION,
                SHARED_POSITION1
        );


        // Path 1

        paths[9] = makePath(
                SHARED_POSITION1,
                SHARED_POSITION2
        );


        // Normal shared route

        paths[10] = makePath(
                SHARED_POSITION2,
                BLUE_UPPER_SHOOTING_POSITION
        );

        paths[11] = makePath(
                BLUE_UPPER_SHOOTING_POSITION,
                SHARED_POSITION3
        );

        paths[12] = makePath(
                SHARED_POSITION3,
                FOURTH_AI_POSITION
        );


        // Fourth AI branches

        paths[13] = makePath(
                FOURTH_AI_POSITION,
                BLUE_UPPER_SHOOTING_POSITION
        );

        paths[14] = makePath(
                FOURTH_AI_POSITION,
                SHARED_POSITION2
        );


        // Path 1 Reversed

        paths[15] = makePath(
                SHARED_POSITION2,
                SHARED_POSITION1
        );

        paths[16] = makePath(
                SHARED_POSITION1,
                BLUE_LOWER_SHOOTING_POSITION
        );


        // Connect Path 1 to Branch 1

        paths[17] = makePath(
                SHARED_POSITION2,
                BRANCH1_POSITION
        );


        // Branch 1-1:
        // Third AI directly to Blue Upper Shooting

        paths[18] = makePath(
                THIRD_AI_POSITION,
                BLUE_UPPER_SHOOTING_POSITION
        );
    }

    // =========================================
    // PATH BUILDER
    // =========================================

    private PathChain makePath(Pose start, Pose end) {

        return follower.pathBuilder()
                .addPath(new BezierLine(start, end))
                .setLinearHeadingInterpolation(
                        start.getHeading(),
                        end.getHeading()
                )
                .build();
    }

    // =========================================
    // AUTONOMOUS STATE MACHINE
    // =========================================

    private void autonomousPathUpdate() {

        switch (autoState) {

            // =================================
            // INITIAL AI
            // =================================

            case INITIAL_AI: {

                String initialAI = readClassifier();

                if (initialAI.equals(TILT_AWAY_BLUE)) {
                    autoState = AutoState.NEW_POSITION_PATH;
                }

                break;
            }

            // =================================
            // NEW POSITION
            // =================================

            case NEW_POSITION_PATH:

                follower.followPath(paths[0], true);
                autoState = AutoState.WAIT_NEW_POSITION_PATH;

                break;

            case WAIT_NEW_POSITION_PATH:

                if (!follower.isBusy()) {
                    follower.followPath(paths[1], true);
                    autoState = AutoState.WAIT_INITIAL_WAYPOINT;
                }

                break;

            // =================================
            // INITIAL WAYPOINT
            // =================================

            case WAIT_INITIAL_WAYPOINT:

                if (!follower.isBusy()) {
                    follower.followPath(paths[2], true);
                    autoState = AutoState.WAIT_BLUE_LOWER_POSITION;
                }

                break;

            // =================================
            // BLUE LOWER SHOOTING POSITION
            // =================================

            case WAIT_BLUE_LOWER_POSITION:

                if (!follower.isBusy()) {
                    follower.followPath(paths[3], true);
                    autoState = AutoState.WAIT_SECOND_AI_POSITION;
                }

                break;

            // =================================
            // SECOND AI POSITION
            // =================================

            case WAIT_SECOND_AI_POSITION:

                if (!follower.isBusy()) {
                    autoState = AutoState.SECOND_AI;
                }

                break;

            // =================================
            // SECOND AI
            // =================================

            case SECOND_AI:

                secondAI = readClassifier();

                if (secondAI.equals(TILT_AWAY_BLUE)) {
                    autoState = AutoState.BRANCH1_CONNECT;
                }
                else if (secondAI.equals(TILT_TOWARD_BLUE)) {
                    autoState = AutoState.BRANCH2_PATH;
                }

                break;

            // =================================
            // BRANCH 1
            // =================================

            case BRANCH1_CONNECT:

                follower.followPath(paths[4], true);
                autoState = AutoState.WAIT_BRANCH1_CONNECT;

                break;

            case WAIT_BRANCH1_CONNECT:

                if (!follower.isBusy()) {
                    follower.followPath(paths[9], true);
                    autoState = AutoState.PATH1_TO_BRANCH;
                }

                break;

            case PATH1_TO_BRANCH:

                if (!follower.isBusy()) {
                    follower.followPath(paths[17], true);
                    autoState = AutoState.WAIT_BRANCH1_CONNECTOR;
                }

                break;

            case WAIT_BRANCH1_CONNECTOR:

                if (!follower.isBusy()) {
                    autoState = AutoState.TURN_TO_125;
                }

                break;

            // =================================
            // TURN TO THIRD AI (-125 DEGREES)
            // =================================

            case TURN_TO_125:

                follower.turnTo(Math.toRadians(-125));
                autoState = AutoState.WAIT_TURN_125;

                break;

            case WAIT_TURN_125:

                if (!follower.isBusy()) {
                    autoState = AutoState.THIRD_AI;
                }

                break;

            // =================================
            // THIRD AI
            // =================================

            case THIRD_AI:

                thirdAI = readClassifier();

                if (thirdAI.equals(TILT_AWAY_BLUE)) {
                    autoState = AutoState.BRANCH1_1_PATH;
                }
                else if (thirdAI.equals(TILT_TOWARD_BLUE)) {
                    autoState = AutoState.THIRD_BRANCH1_2_PATH;
                }

                break;

            // =================================
            // BRANCH 1-1
            // TILT AWAY BLUE
            // =================================

            case BRANCH1_1_PATH:

                follower.followPath(paths[18], true);
                autoState = AutoState.WAIT_BRANCH1_1_PATH;

                break;

            case WAIT_BRANCH1_1_PATH:

                if (!follower.isBusy()) {
                    follower.followPath(paths[11], true);
                    autoState = AutoState.WAIT_BRANCH1_1_TO_SHARED;
                }

                break;

            case WAIT_BRANCH1_1_TO_SHARED:

                if (!follower.isBusy()) {
                    follower.followPath(paths[12], true);
                    autoState = AutoState.WAIT_SHARED_PATH4;
                }

                break;

            // =================================
            // BRANCH 1-2
            // TILT TOWARDS BLUE
            // =================================

            case THIRD_BRANCH1_2_PATH:

                follower.followPath(paths[6], true);
                autoState = AutoState.WAIT_THIRD_BRANCH1_2_PATH;

                break;

            case WAIT_THIRD_BRANCH1_2_PATH:

                if (!follower.isBusy()) {
                    follower.followPath(paths[15], true);
                    autoState = AutoState.WAIT_REVERSE_PATH2;
                }

                break;

            // =================================
            // BRANCH 2
            // =================================

            case BRANCH2_PATH:

                follower.followPath(paths[8], true);
                autoState = AutoState.WAIT_BRANCH2_PATH;

                break;

            case WAIT_BRANCH2_PATH:

                if (!follower.isBusy()) {
                    autoState = AutoState.SHARED_PATH1;
                }

                break;

            // =================================
            // NORMAL SHARED ROUTE
            // =================================

            case SHARED_PATH1:

                follower.followPath(paths[9], true);
                autoState = AutoState.WAIT_SHARED_PATH1;

                break;

            case WAIT_SHARED_PATH1:

                if (!follower.isBusy()) {
                    follower.followPath(paths[10], true);
                    autoState = AutoState.WAIT_SHARED_PATH2;
                }

                break;

            case WAIT_SHARED_PATH2:

                if (!follower.isBusy()) {
                    follower.followPath(paths[11], true);
                    autoState = AutoState.WAIT_SHARED_PATH3;
                }

                break;

            case WAIT_SHARED_PATH3:

                if (!follower.isBusy()) {
                    follower.followPath(paths[12], true);
                    autoState = AutoState.WAIT_SHARED_PATH4;
                }

                break;

            case WAIT_SHARED_PATH4:

                if (!follower.isBusy()) {
                    autoState = AutoState.FOURTH_AI;
                }

                break;

            // =================================
            // FOURTH AI
            // =================================

            case FOURTH_AI:

                fourthAI = readClassifier();

                if (fourthAI.equals(TILT_AWAY_BLUE)) {
                    autoState = AutoState.UPPER_SHOOTING_PATH;
                }
                else if (fourthAI.equals(TILT_TOWARD_BLUE)) {
                    autoState = AutoState.REVERSE_PATH1;
                }

                break;

            // =================================
            // FOURTH AI: BLUE UPPER
            // =================================

            case UPPER_SHOOTING_PATH:

                follower.followPath(paths[13], true);
                autoState = AutoState.WAIT_UPPER_SHOOTING_PATH;

                break;

            case WAIT_UPPER_SHOOTING_PATH:

                if (!follower.isBusy()) {
                    autoState = AutoState.COMPLETE;
                }

                break;

            // =================================
            // FOURTH AI: PATH 1 REVERSED
            // =================================

            case REVERSE_PATH1:

                follower.followPath(paths[14], true);
                autoState = AutoState.WAIT_REVERSE_PATH1;

                break;

            case WAIT_REVERSE_PATH1:

                if (!follower.isBusy()) {
                    follower.followPath(paths[15], true);
                    autoState = AutoState.WAIT_REVERSE_PATH2;
                }

                break;

            // =================================
            // FINISH REVERSED ROUTE
            // =================================

            case WAIT_REVERSE_PATH2:

                if (!follower.isBusy()) {
                    follower.followPath(paths[16], true);
                    autoState = AutoState.COMPLETE;
                }

                break;

            // =================================
            // COMPLETE
            // =================================

            case COMPLETE:

                break;
        }
    }

    // =========================================
    // LIMELIGHT CLASSIFIER
    // =========================================

    private String readClassifier() {

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return "";
        }

        if (result.getClassifierResults().isEmpty()) {
            return "";
        }

        LLResultTypes.ClassifierResult best =
                result.getClassifierResults().get(0);

        return best.getClassName();
    }
}

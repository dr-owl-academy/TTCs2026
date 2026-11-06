package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "DanielSimpleAutontest1")
public class DanielAuton1V2 extends OpMode {

    private enum AutoState {
        ROTATE,
        WAIT_FOR_ROTATION,
        DRIVE,
        WAIT_FOR_DRIVE,
        COMPLETE
    }

    private Follower follower;

    private AutoState autoState = AutoState.ROTATE;

    // =========================================
    // ROBOT STARTING POSITION
    // =========================================

    private static final Pose START_POSE =
            new Pose(104, 132, Math.toRadians(-90));

    // =========================================
    // DESTINATION COORDINATES
    // Original X and Y coordinates preserved.
    // Headings are calculated automatically.
    // =========================================

    private final Pose[] waypoints = {
            new Pose(129.587, 130.006, 0),
            new Pose(110.236, 112.652, 0),
            new Pose(124.583, 87.536, 0),
            new Pose(97.819, 109.770, 0),
            new Pose(83.722, 105.466, 0),
            new Pose(85.444, 123.492, 0),
            new Pose(85.697, 103.772, 0),
            new Pose(126.679, 47.832, 0)
    };

    private PathChain[] paths =
            new PathChain[waypoints.length];

    private double[] targetHeadings =
            new double[waypoints.length];

    private int currentPathIndex = 0;

    private double currentTargetHeading = 0;

    // =========================================
    // INITIALIZATION
    // =========================================

    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(START_POSE);

        // Reduced power for initial testing.
        follower.setMaxPower(0.5);

        buildPaths();

        autoState = AutoState.ROTATE;

        telemetry.addLine("Autonomous initialized");
        telemetry.addData("Number of paths", paths.length);
        telemetry.update();
    }

    @Override
    public void start() {

        currentPathIndex = 0;
        autoState = AutoState.ROTATE;
    }

    // =========================================
    // MAIN LOOP
    // =========================================

    @Override
    public void loop() {

        // Always update Pedro Pathing.
        follower.update();

        autonomousPathUpdate();

        Pose currentPose = follower.getPose();

        telemetry.addData("State", autoState);

        telemetry.addData(
                "Path Number",
                currentPathIndex + 1
        );

        telemetry.addData("X", currentPose.getX());
        telemetry.addData("Y", currentPose.getY());

        telemetry.addData(
                "Heading Degrees",
                Math.toDegrees(currentPose.getHeading())
        );

        telemetry.addData(
                "Target Heading Degrees",
                Math.toDegrees(currentTargetHeading)
        );

        telemetry.addData(
                "Follower Busy",
                follower.isBusy()
        );

        if (autoState == AutoState.COMPLETE) {
            telemetry.addLine("AUTONOMOUS COMPLETE");
        }

        telemetry.update();
    }

    // =========================================
    // BUILD ALL STRAIGHT PATHS
    // =========================================

    private void buildPaths() {

        Pose previousPose = START_POSE;

        for (int i = 0; i < waypoints.length; i++) {

            Pose targetPose = waypoints[i];

            double deltaX =
                    targetPose.getX() - previousPose.getX();

            double deltaY =
                    targetPose.getY() - previousPose.getY();

            // Calculate direction toward destination.
            double heading =
                    Math.atan2(deltaY, deltaX);

            targetHeadings[i] = heading;

            // Both endpoints have the SAME heading.
            // This prevents turning while driving.

            Pose pathStart = new Pose(
                    previousPose.getX(),
                    previousPose.getY(),
                    heading
            );

            Pose pathEnd = new Pose(
                    targetPose.getX(),
                    targetPose.getY(),
                    heading
            );

            paths[i] = follower.pathBuilder()
                    .addPath(
                            new BezierLine(pathStart, pathEnd)
                    )
                    .setLinearHeadingInterpolation(
                            heading,
                            heading
                    )
                    .build();

            previousPose = targetPose;
        }
    }

    // =========================================
    // ROTATE AND DRIVE STATE MACHINE
    // =========================================

    private void autonomousPathUpdate() {

        switch (autoState) {

            case ROTATE:

                if (currentPathIndex >= paths.length) {
                    autoState = AutoState.COMPLETE;
                    break;
                }

                currentTargetHeading =
                        targetHeadings[currentPathIndex];

                // Rotate to face the next destination.
                follower.turnTo(currentTargetHeading);

                autoState = AutoState.WAIT_FOR_ROTATION;

                break;

            case WAIT_FOR_ROTATION:

                // Wait until rotation is finished.
                if (!follower.isBusy()) {

                    autoState = AutoState.DRIVE;
                }

                break;

            case DRIVE:

                // Drive straight with constant heading.
                follower.followPath(
                        paths[currentPathIndex],
                        true
                );

                autoState = AutoState.WAIT_FOR_DRIVE;

                break;

            case WAIT_FOR_DRIVE:

                // Wait until the robot reaches its destination.
                if (!follower.isBusy()) {

                    currentPathIndex++;

                    if (currentPathIndex >= paths.length) {

                        autoState = AutoState.COMPLETE;

                    } else {

                        autoState = AutoState.ROTATE;
                    }
                }

                break;

            case COMPLETE:

                // Nothing else to do.
                break;
        }
    }

    @Override
    public void stop() {

        // Stop the follower when autonomous ends.
        if (follower != null) {
            follower.breakFollowing();
        }
    }
}
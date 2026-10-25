package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "DanielSimpleAuton")
public class DanielAuton1 extends OpMode {

    private enum AutoState {
        PATH1,
        PATH2,
        PATH3,
        PATH4,
        PATH5,
        PATH6,
        PATH7,
        PATH8,
        PATH9,
        PATH10,
        COMPLETE
    }

    private Follower follower;

    private PathChain[] paths = new PathChain[9]; //sets up pathchain for multiple paths
//start FSM state
    private AutoState autoState = AutoState.PATH1;

    // =========================================
    // ROBOT POSES for FSM
    // Pose format: X, Y, heading in radians
    // =========================================

    private static final Pose START_POSE =
            new Pose(104, 132, Math.toRadians(-90));

    private static final Pose DRIVE_START_POSE =
            new Pose(116, 131, Math.toRadians(103)); //accidentally added extra

    private static final Pose TARGET_POSE =
            new Pose(129.587, 130.006, Math.toRadians(103));

    private static final Pose FOURTH_POSE =
            new Pose(110.236, 112.652, Math.toRadians(42));

    private static final Pose FIFTH_POSE =
            new Pose(124.583, 87.536, Math.toRadians(-144));

    private static final Pose SIXTH_POSE =
            new Pose(97.819, 109.77, Math.toRadians(140));

    private static final Pose SEVENTH_POSE =
            new Pose(83.722, 105.466, Math.toRadians(-163));

    private static final Pose EIGHTH_POSE =
            new Pose(85.444, 123.492, Math.toRadians(-95));

    private static final Pose NINTH_POSE =
            new Pose(85.697, 103.772, Math.toRadians(-89));

    private static final Pose TENTH_POSE =
            new Pose(126.679, 47.832, Math.toRadians(-82));


    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(START_POSE);

        // Reduced power for testing.
        follower.setMaxPower(0.5);

        buildPaths();

        autoState = AutoState.PATH1;

        telemetry.addLine("Autonomous ready");
        telemetry.update();
    }


    @Override
    public void start() {
        autoState = AutoState.PATH1;
    } //redundant line, only used to reset state when auton begins


    @Override
    public void loop() {

        // Update Pedro Pathing every loop.
        follower.update();

        // Run autonomous state machine.
        autonomousPathUpdate();

        Pose currentPose = follower.getPose();

        telemetry.addData("X", currentPose.getX());
        telemetry.addData("Y", currentPose.getY());

        telemetry.addData(
                "Heading (degrees)",
                Math.toDegrees(currentPose.getHeading())
        );

        telemetry.addData("State", autoState);
        telemetry.addData("Follower Busy", follower.isBusy());
//helps with debug, shows if robot is moving or not or if it is still running the path
        if (autoState == AutoState.COMPLETE) {
            telemetry.addLine("Autonomous complete");
        }

        telemetry.update();
    }


    @Override
    public void stop() {
        // No additional cleanup required.
    }


    // =========================================
    // BUILD ALL PATHS
    // Each path gets its own starting and ending
    // heading from its Pose coordinates.
    // =========================================

    private void buildPaths() {

        paths[0] = makePath(START_POSE, DRIVE_START_POSE);

        paths[1] = makePath(DRIVE_START_POSE, TARGET_POSE);

        paths[2] = makePath(TARGET_POSE, FOURTH_POSE);

        paths[3] = makePath(FOURTH_POSE, FIFTH_POSE);

        paths[4] = makePath(FIFTH_POSE, SIXTH_POSE);

        paths[5] = makePath(SIXTH_POSE, SEVENTH_POSE);

        paths[6] = makePath(SEVENTH_POSE, EIGHTH_POSE);

        paths[7] = makePath(EIGHTH_POSE, NINTH_POSE);

        paths[8] = makePath(NINTH_POSE, TENTH_POSE);
    } //builds all the paths in order for robot to move to correct heading while driving to next path


    // Creates a path that interpolates heading between poses.
    private PathChain makePath(Pose start, Pose end) {

        return follower.pathBuilder()
                .addPath(new BezierLine(start, end))
                .setLinearHeadingInterpolation(
                        start.getHeading(),
                        end.getHeading()
                )
                .build();
    }//designs the path builder for auton instead of writing same code for every path


    // =========================================
    // AUTONOMOUS STATE MACHINE
    // =========================================

    private void autonomousPathUpdate() {

        switch (autoState) {

            case PATH1:
                follower.followPath(paths[0], true);
                autoState = AutoState.PATH2;
                break;

            case PATH2:
                if (!follower.isBusy()) {
                    follower.followPath(paths[1], true);
                    autoState = AutoState.PATH3;
                }
                break;

            case PATH3:
                if (!follower.isBusy()) {
                    follower.followPath(paths[2], true);
                    autoState = AutoState.PATH4;
                }
                break;

            case PATH4:
                if (!follower.isBusy()) {
                    follower.followPath(paths[3], true);
                    autoState = AutoState.PATH5;
                }
                break;

            case PATH5:
                if (!follower.isBusy()) {
                    follower.followPath(paths[4], true);
                    autoState = AutoState.PATH6;
                }
                break;

            case PATH6:
                if (!follower.isBusy()) {
                    follower.followPath(paths[5], true);
                    autoState = AutoState.PATH7;
                }
                break;

            case PATH7:
                if (!follower.isBusy()) {
                    follower.followPath(paths[6], true);
                    autoState = AutoState.PATH8;
                }
                break;

            case PATH8:
                if (!follower.isBusy()) {
                    follower.followPath(paths[7], true);
                    autoState = AutoState.PATH9;
                }
                break;

            case PATH9:
                if (!follower.isBusy()) {
                    follower.followPath(paths[8], true);
                    autoState = AutoState.PATH10;
                }
                break;

            case PATH10:
                if (!follower.isBusy()) {
                    autoState = AutoState.COMPLETE;
                }
                break;

            case COMPLETE:
                // Autonomous has finished.
                break;
        }
    }
}
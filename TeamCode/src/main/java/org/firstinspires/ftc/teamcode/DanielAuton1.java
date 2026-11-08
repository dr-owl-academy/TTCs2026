package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "DanielSimpleAuton1")
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
        PATH11,
        PATH12,
        PATH13,
        PATH14,
        PATH15,
        PATH16,
        PATH17,
        PATH18,
        COMPLETE
    }

    private Follower follower;
    private Pose debugTarget = START_POSE;
    private String debugAction = "Waiting to Start";

    private PathChain[] paths = new PathChain[18]; //sets up pathchain for multiple paths
//start FSM state
    private AutoState autoState = AutoState.PATH1;

    // =========================================
    // ROBOT POSES for FSM
    // Pose format: X, Y, heading in radians
    // =========================================

    private static final Pose START_POSE =
            new Pose(99.651, 132.274, Math.toRadians(-90));

    private static final Pose DRIVE_START_POSE =
            new Pose(99.651, 115.099, Math.toRadians(-90)); //accidentally added extra

    private static final Pose TARGET_POSE =
            new Pose(99.651, 115.099, Math.toRadians(-1));

    private static final Pose FOURTH_POSE =
            new Pose(129.44, 94.293, Math.toRadians(-1));

    private static final Pose FIFTH_POSE =
            new Pose(129.44, 94.293, Math.toRadians(-108));

    private static final Pose SIXTH_POSE =
            new Pose(105.082, 21.135, Math.toRadians(-108));

    private static final Pose SEVENTH_POSE =
            new Pose(105.082, 21.135, Math.toRadians(113));

    private static final Pose EIGHTH_POSE =
            new Pose(94.859, 17.045, Math.toRadians(113));

    private static final Pose NINTH_POSE =
            new Pose(94.859, 17.045, Math.toRadians(116));

    private static final Pose TENTH_POSE =
            new Pose(87.208, 32.554, Math.toRadians(116));
    private static final Pose ELEVENTH_POSE =
            new Pose(87.208, 32.554, Math.toRadians(70));
    private static final Pose TWELFTH_POSE =
            new Pose(108.446, 55.198, Math.toRadians(70));
    private static final Pose THIRTEENTH_POSE =
            new Pose(108.446, 55.198, Math.toRadians(104));
    private static final Pose FOURTEENTH_POSE =
            new Pose(109.051, 86.086, Math.toRadians(104));
    private static final Pose FIFTEENTH_POSE =
            new Pose(109.051, 96.086, Math.toRadians(119));
    private static final Pose SIXTEENTH_POSE =
            new Pose(96.236, 117.378, Math.toRadians(119));
    private static final Pose SEVENTEENTH_POSE =
            new Pose(96.236, 117.378, Math.toRadians(-68));
    private static final Pose EIGHTEENTH_POSE =
            new Pose(124.840, 47.484, Math.toRadians(-68));





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

        paths[9] = makePath(TENTH_POSE, ELEVENTH_POSE);

        paths[10] = makePath(ELEVENTH_POSE, TWELFTH_POSE);

        paths[11] = makePath(TWELFTH_POSE, THIRTEENTH_POSE);

        paths[12] = makePath(THIRTEENTH_POSE, FOURTEENTH_POSE);

        paths[13] = makePath(FOURTEENTH_POSE, FIFTEENTH_POSE);

        paths[14] = makePath(FIFTEENTH_POSE, SIXTEENTH_POSE);

        paths[15] = makePath(SIXTEENTH_POSE, SEVENTEENTH_POSE);

        paths[16] = makePath(SEVENTEENTH_POSE, EIGHTEENTH_POSE);






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
                    follower.turnTo(Math.toRadians(-1));
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
                    follower.turnTo(Math.toRadians(-108));
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
                    follower.turnTo(Math.toRadians(113));
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
                    follower.turnTo(Math.toRadians(116));
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
                    follower.turnTo(Math.toRadians(70));
                    autoState = AutoState.PATH11;
                }
                break;
            case PATH11:
                if (!follower.isBusy()) {
                    follower.followPath(paths[10], true);
                    autoState = AutoState.PATH12;
                }
                break;
            case PATH12:
                if (!follower.isBusy()) {
                    follower.turnTo(Math.toRadians(104));
                    autoState = AutoState.PATH13;
                }
                break;
            case PATH13:
                if (!follower.isBusy()) {
                    follower.followPath(paths[12], true);
                    autoState = AutoState.PATH14;
                }
                break;
                case PATH14:
                if (!follower.isBusy()) {
                    follower.turnTo(Math.toRadians(119));
                    autoState = AutoState.PATH15;
                }
                break;
                case PATH15:
                if (!follower.isBusy()) {
                    follower.followPath(paths[14], true);
                    autoState = AutoState.PATH16;
                }
                break;
                case PATH16:
                if (!follower.isBusy()) {
                    follower.turnTo(Math.toRadians(-68));
                    autoState = AutoState.PATH17;
                }
                break;
                case PATH17:
                if (!follower.isBusy()) {
                    follower.followPath(paths[16], true);
                    autoState = AutoState.PATH18;
                }
                break;



            case PATH18:
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
package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "River Auton Path")
public class river_auton_path extends OpMode {

    private enum AutoState {
        PATH1, WAIT1,
        PATH2, WAIT2, AI_AFTER_2,
        PATH3, WAIT3,
        PATH4, WAIT4, AI_AFTER_4,
        PATH5, WAIT5,
        PATH6, WAIT6, AI_AFTER_6,
        PATH7, WAIT7, AI_AFTER_7,
        COMPLETE
    }

    private Follower follower;
    private PathChain[] paths = new PathChain[7];
    private AutoState autoState = AutoState.PATH1;
    private long waitStart = 0;

    // ----- Poses (unchanged from your code) -----
    private static final Pose START =
            new Pose(51.6289, 6.8244, Math.toRadians(90));
    private static final Pose GOTO_SHOOT =
            new Pose(52.8867, 13.7422, Math.toRadians(83.9918));
    private static final Pose GOTO_SHOOT_CONTROL =
            new Pose(52.6119, 11.1311, Math.toRadians(0));
    private static final Pose GET_POLLEN_GARDEN =
            new Pose(10.7319, 8.457, Math.toRadians(-172.8538));
    private static final Pose GOTO_SHOOT_2 =
            new Pose(38.0341, 120.1244, Math.toRadians(76.261));
    private static final Pose GET_POLLEN_FLOWER =
            new Pose(47.4193, 131.8578, Math.toRadians(51.3446));
    private static final Pose SHOOT2 =
            new Pose(57.4474, 114.0252, Math.toRadians(119.3512));
    private static final Pose GET_POLLEN_FLOWER_2 =
            new Pose(9.2244, 52.6148, Math.toRadians(-128.141));
    private static final Pose PARK =
            new Pose(12.3437, 95.4274, Math.toRadians(-94.1671));

    // Alternate points used when hive is tilted away
    // (replace these coordinates with the real alternate poses you want)
    private static final Pose ALT_AFTER_2 =
            new Pose(20.0, 20.0, Math.toRadians(90));
    private static final Pose ALT_AFTER_4 =
            new Pose(40.0, 40.0, Math.toRadians(45));
    private static final Pose ALT_AFTER_6 =
            new Pose(30.0, 60.0, Math.toRadians(0));
    private static final Pose ALT_AFTER_7 =
            new Pose(15.0, 80.0, Math.toRadians(-90));

    private PathChain altAfter2, altAfter4, altAfter6, altAfter7;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START);
        follower.setMaxPower(0.5);
        buildPaths();
        buildAlternatePaths();
        autoState = AutoState.PATH1;
        telemetry.addLine("Autonomous ready");
        telemetry.update();
    }

    @Override
    public void start() {
        autoState = AutoState.PATH1;
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        Pose p = follower.getPose();
        telemetry.addData("X", p.getX());
        telemetry.addData("Y", p.getY());
        telemetry.addData("Heading", Math.toDegrees(p.getHeading()));
        telemetry.addData("State", autoState);
        telemetry.addData("Busy", follower.isBusy());
        telemetry.update();
    }

    private void buildPaths() {
        paths[0] = follower.pathBuilder()
                .addPath(new BezierCurve(START, GOTO_SHOOT_CONTROL, GOTO_SHOOT))
                .setLinearHeadingInterpolation(START.getHeading(), GOTO_SHOOT.getHeading())
                .build();

        paths[1] = makeLine(GOTO_SHOOT, GET_POLLEN_GARDEN);
        paths[2] = makeLine(GET_POLLEN_GARDEN, GOTO_SHOOT_2);
        paths[3] = makeLine(GOTO_SHOOT_2, GET_POLLEN_FLOWER);
        paths[4] = makeLine(GET_POLLEN_FLOWER, SHOOT2);
        paths[5] = makeLine(SHOOT2, GET_POLLEN_FLOWER_2);
        paths[6] = makeLine(GET_POLLEN_FLOWER_2, PARK);
    }

    private void buildAlternatePaths() {
        // These are simple lines to the alternate points.
        // Change them to whatever paths you actually want when the hive is tilted away.
        altAfter2 = makeLine(GET_POLLEN_GARDEN, ALT_AFTER_2);
        altAfter4 = makeLine(GET_POLLEN_FLOWER, ALT_AFTER_4);
        altAfter6 = makeLine(GET_POLLEN_FLOWER_2, ALT_AFTER_6);
        altAfter7 = makeLine(PARK, ALT_AFTER_7);
    }

    private PathChain makeLine(Pose start, Pose end) {
        return follower.pathBuilder()
                .addPath(new BezierLine(start, end))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .build();
    }

    private void autonomousPathUpdate() {
        switch (autoState) {

            // ---------- PATH 1 ----------
            case PATH1:
                follower.followPath(paths[0], true);
                autoState = AutoState.WAIT1;
                break;
            case WAIT1:
                if (!follower.isBusy()) {
                    waitStart = System.currentTimeMillis();
                    autoState = AutoState.PATH2;
                }
                break;

            // ---------- PATH 2 + AI ----------
            case PATH2:
                if (System.currentTimeMillis() - waitStart >= 1800) {
                    follower.followPath(paths[1], true);
                    autoState = AutoState.WAIT2;
                }
                break;
            case WAIT2:
                if (!follower.isBusy()) {
                    autoState = AutoState.AI_AFTER_2;   // check after Path 2
                }
                break;
            case AI_AFTER_2:
                // Call PromodPathPlaner here
                boolean tiltedAway = River_Path_Planer.isHiveTiltedAway();  // <-- method call

                if (tiltedAway) {
                    // Hive tilted away → go to alternate point
                    follower.followPath(altAfter2, true);
                    autoState = AutoState.WAIT3;   // or whatever state continues after the alt path
                } else {
                    // Hive toward the path → continue main path
                    autoState = AutoState.PATH3;
                }
                break;

            // ---------- PATH 3 ----------
            case PATH3:
                if (System.currentTimeMillis() - waitStart >= 100) {
                    follower.followPath(paths[2], true);
                    autoState = AutoState.WAIT3;
                }
                break;
            case WAIT3:
                if (!follower.isBusy()) {
                    waitStart = System.currentTimeMillis();
                    autoState = AutoState.PATH4;
                }
                break;

            // ---------- PATH 4 + AI ----------
            case PATH4:
                if (System.currentTimeMillis() - waitStart >= 1250) {
                    follower.followPath(paths[3], true);
                    autoState = AutoState.WAIT4;
                }
                break;
            case WAIT4:
                if (!follower.isBusy()) {
                    autoState = AutoState.AI_AFTER_4;
                }
                break;
            case AI_AFTER_4:
                boolean tiltedAway4 = River_Path_Planer.isHiveTiltedAway();

                if (tiltedAway4) {
                    follower.followPath(altAfter4, true);
                    autoState = AutoState.WAIT5;
                } else {
                    autoState = AutoState.PATH5;
                }
                break;

            // ---------- PATH 5 ----------
            case PATH5:
                if (System.currentTimeMillis() - waitStart >= 1000) {
                    follower.followPath(paths[4], true);
                    autoState = AutoState.WAIT5;
                }
                break;
            case WAIT5:
                if (!follower.isBusy()) {
                    waitStart = System.currentTimeMillis();
                    autoState = AutoState.PATH6;
                }
                break;

            // ---------- PATH 6 + AI ----------
            case PATH6:
                if (System.currentTimeMillis() - waitStart >= 1800) {
                    follower.followPath(paths[5], true);
                    autoState = AutoState.WAIT6;
                }
                break;
            case WAIT6:
                if (!follower.isBusy()) {
                    autoState = AutoState.AI_AFTER_6;
                }
                break;
            case AI_AFTER_6:
                boolean tiltedAway6 = River_Path_Planer.isHiveTiltedAway();

                if (tiltedAway6) {
                    follower.followPath(altAfter6, true);
                    autoState = AutoState.WAIT7;
                } else {
                    autoState = AutoState.PATH7;
                }
                break;

            // ---------- PATH 7 + AI ----------
            case PATH7:
                if (System.currentTimeMillis() - waitStart >= 500) {
                    follower.followPath(paths[6], true);
                    autoState = AutoState.WAIT7;
                }
                break;
            case WAIT7:
                if (!follower.isBusy()) {
                    autoState = AutoState.AI_AFTER_7;
                }
                break;
            case AI_AFTER_7:
                boolean tiltedAway7 = River_Path_Planer.isHiveTiltedAway();

                if (tiltedAway7) {
                    follower.followPath(altAfter7, true);
                    // after alternate, go to complete (or add more states if needed)
                    autoState = AutoState.COMPLETE;
                } else {
                    autoState = AutoState.COMPLETE;
                }
                break;

            case COMPLETE:
                break;
        }
    }
}
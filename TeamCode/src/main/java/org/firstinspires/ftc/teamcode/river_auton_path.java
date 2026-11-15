package org.firstinspires.ftc.teamcode;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "River Red Auton", group = "Red")
public class river_auton_path extends OpMode {

    private enum AutoState {
        PATH1, PAUSE1, TURN1,
        PATH2, PAUSE2, TURN2,
        PATH3, PAUSE3, TURN3,
        PATH4, PAUSE4, TURN4,
        PATH5, PAUSE5, TURN5,
        PATH6, PAUSE6, TURN6,
        PATH7, PAUSE7, TURN7,
        COMPLETE
    }

    private Follower follower;
    private PathChain[] paths = new PathChain[7];
    private AutoState autoState = AutoState.PATH1;
    private long pauseStart = 0;

    private static final Pose[] POINTS = {
            new Pose(46.4165, 8.3509, Math.toRadians(90)),
            new Pose(46.3568, 23.9845, Math.toRadians(90.2191)),
            new Pose(14.815, 12.9082, Math.toRadians(-160.6506)),
            new Pose(26.0055, 31.3329, Math.toRadians(58.7271)),
            new Pose(22.1045, 45.9476, Math.toRadians(104.9449)),
            new Pose(38.3489, 31.2244, Math.toRadians(-42.1877)),
            new Pose(17.2518, 100.395, Math.toRadians(106.9617)),
            new Pose(11.5333, 98.7444, Math.toRadians(-163.9003))
    };

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        // Faster turning
        follower.setHeadingPIDFCoefficients(new PIDFCoefficients(3.0, 0, 0.02, 0.05));

        follower.setStartingPose(POINTS[0]);
        follower.setMaxPower(0.35);
        buildPaths();
        autoState = AutoState.PATH1;

        telemetry.addLine("Ready");
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
        telemetry.addData("X", "%.1f", p.getX());
        telemetry.addData("Y", "%.1f", p.getY());
        telemetry.addData("Heading", "%.1f", Math.toDegrees(p.getHeading()));
        telemetry.addData("State", autoState);
        telemetry.addData("Busy", follower.isBusy());
        telemetry.update();
    }

    private void buildPaths() {
        for (int i = 0; i < 7; i++) {
            paths[i] = makePath(POINTS[i], POINTS[i + 1]);
        }
    }

    private PathChain makePath(Pose start, Pose end) {
        return follower.pathBuilder()
                .addPath(new BezierLine(start, end))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .build();
    }

    // Short path that only turns (same position, new heading)
    private PathChain makeTurnPath(Pose current, double targetHeading) {
        Pose end = new Pose(current.getX(), current.getY(), targetHeading);
        return follower.pathBuilder()
                .addPath(new BezierLine(current, end))
                .setLinearHeadingInterpolation(current.getHeading(), targetHeading)
                .build();
    }

    private void autonomousPathUpdate() {
        Pose current = follower.getPose();

        switch (autoState) {

            case PATH1:
                follower.followPath(paths[0], true);
                autoState = AutoState.PAUSE1;
                break;
            case PAUSE1:
                if (!follower.isBusy()) {
                    pauseStart = System.currentTimeMillis();
                    autoState = AutoState.TURN1;
                }
                break;
            case TURN1:
                if (System.currentTimeMillis() - pauseStart >= 1000) {
                    follower.followPath(makeTurnPath(current, POINTS[1].getHeading()), true);
                    autoState = AutoState.PATH2;
                }
                break;

            case PATH2:
                if (!follower.isBusy()) {
                    follower.followPath(paths[1], true);
                    autoState = AutoState.PAUSE2;
                }
                break;
            case PAUSE2:
                if (!follower.isBusy()) {
                    pauseStart = System.currentTimeMillis();
                    autoState = AutoState.TURN2;
                }
                break;
            case TURN2:
                if (System.currentTimeMillis() - pauseStart >= 1000) {
                    follower.followPath(makeTurnPath(current, POINTS[2].getHeading()), true);
                    autoState = AutoState.PATH3;
                }
                break;

            case PATH3:
                if (!follower.isBusy()) {
                    follower.followPath(paths[2], true);
                    autoState = AutoState.PAUSE3;
                }
                break;
            case PAUSE3:
                if (!follower.isBusy()) {
                    pauseStart = System.currentTimeMillis();
                    autoState = AutoState.TURN3;
                }
                break;
            case TURN3:
                if (System.currentTimeMillis() - pauseStart >= 1000) {
                    follower.followPath(makeTurnPath(current, POINTS[3].getHeading()), true);
                    autoState = AutoState.PATH4;
                }
                break;

            case PATH4:
                if (!follower.isBusy()) {
                    follower.followPath(paths[3], true);
                    autoState = AutoState.PAUSE4;
                }
                break;
            case PAUSE4:
                if (!follower.isBusy()) {
                    pauseStart = System.currentTimeMillis();
                    autoState = AutoState.TURN4;
                }
                break;
            case TURN4:
                if (System.currentTimeMillis() - pauseStart >= 1000) {
                    follower.followPath(makeTurnPath(current, POINTS[4].getHeading()), true);
                    autoState = AutoState.PATH5;
                }
                break;

            case PATH5:
                if (!follower.isBusy()) {
                    follower.followPath(paths[4], true);
                    autoState = AutoState.PAUSE5;
                }
                break;
            case PAUSE5:
                if (!follower.isBusy()) {
                    pauseStart = System.currentTimeMillis();
                    autoState = AutoState.TURN5;
                }
                break;
            case TURN5:
                if (System.currentTimeMillis() - pauseStart >= 1000) {
                    follower.followPath(makeTurnPath(current, POINTS[5].getHeading()), true);
                    autoState = AutoState.PATH6;
                }
                break;

            case PATH6:
                if (!follower.isBusy()) {
                    follower.followPath(paths[5], true);
                    autoState = AutoState.PAUSE6;
                }
                break;
            case PAUSE6:
                if (!follower.isBusy()) {
                    pauseStart = System.currentTimeMillis();
                    autoState = AutoState.TURN6;
                }
                break;
            case TURN6:
                if (System.currentTimeMillis() - pauseStart >= 1000) {
                    follower.followPath(makeTurnPath(current, POINTS[6].getHeading()), true);
                    autoState = AutoState.PATH7;
                }
                break;

            case PATH7:
                if (!follower.isBusy()) {
                    follower.followPath(paths[6], true);
                    autoState = AutoState.PAUSE7;
                }
                break;
            case PAUSE7:
                if (!follower.isBusy()) {
                    pauseStart = System.currentTimeMillis();
                    autoState = AutoState.TURN7;
                }
                break;
            case TURN7:
                if (System.currentTimeMillis() - pauseStart >= 1000) {
                    follower.followPath(makeTurnPath(current, POINTS[7].getHeading()), true);
                    autoState = AutoState.COMPLETE;
                }
                break;

            case COMPLETE:
                break;
        }
    }
}
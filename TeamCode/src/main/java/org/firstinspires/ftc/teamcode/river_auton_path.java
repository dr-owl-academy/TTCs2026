package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "River Auton Path")
public class river_auton_path extends OpMode {

    private enum AutoState {
        PATH1, PATH2, PATH3, PATH4, PATH5, PATH6, PATH7, COMPLETE
    }

    private Follower follower;
    private PathChain[] paths = new PathChain[7];
    private AutoState autoState = AutoState.PATH1;

    // New points
    private static final Pose START =
            new Pose(46.4165, 8.3509, Math.toRadians(90));
    private static final Pose POINT1 =
            new Pose(46.3568, 23.9845, Math.toRadians(90.2191));
    private static final Pose POINT2 =
            new Pose(14.815, 12.9082, Math.toRadians(-160.6506));
    private static final Pose POINT3 =
            new Pose(26.0055, 31.3329, Math.toRadians(58.7271));
    private static final Pose POINT4 =
            new Pose(22.1045, 45.9476, Math.toRadians(104.9449));
    private static final Pose POINT5 =
            new Pose(35.0986, 31.9073, Math.toRadians(-47.2162));
    private static final Pose POINT6 =
            new Pose(28.2347, 98.8582, Math.toRadians(95.8536));
    private static final Pose POINT7 =
            new Pose(15.3938, 98.7703, Math.toRadians(-179.6077));

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START);
        follower.setMaxPower(0.5);
        buildPaths();
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

        Pose currentPose = follower.getPose();
        telemetry.addData("X", currentPose.getX());
        telemetry.addData("Y", currentPose.getY());
        telemetry.addData("Heading (degrees)", Math.toDegrees(currentPose.getHeading()));
        telemetry.addData("State", autoState);
        telemetry.addData("Follower Busy", follower.isBusy());

        if (autoState == AutoState.COMPLETE) {
            telemetry.addLine("Autonomous complete");
        }
        telemetry.update();
    }

    private void buildPaths() {
        paths[0] = makePath(START, POINT1);
        paths[1] = makePath(POINT1, POINT2);
        paths[2] = makePath(POINT2, POINT3);
        paths[3] = makePath(POINT3, POINT4);
        paths[4] = makePath(POINT4, POINT5);
        paths[5] = makePath(POINT5, POINT6);
        paths[6] = makePath(POINT6, POINT7);
    }

    private PathChain makePath(Pose start, Pose end) {
        return follower.pathBuilder()
                .addPath(new BezierLine(start, end))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .build();
    }

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
                    autoState = AutoState.COMPLETE;
                }
                break;

            case COMPLETE:
                break;
        }
    }
}
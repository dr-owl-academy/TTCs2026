package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "River Red Auton", group = "Red")
public class river_auton_path extends OpMode {

    private Follower follower;
    private PathChain redRoute;

    private static final Pose[] POINTS = {
            new Pose(32.3, 6.3, Math.toRadians(90)),
            new Pose(32.6, 10.3, Math.toRadians(86)),
            new Pose(34.0, 12.1, Math.toRadians(175)),   // farther from wall
            new Pose(26.1, 32.7, Math.toRadians(57)),
            new Pose(21.8, 48.9, Math.toRadians(105)),
            new Pose(38.3, 31.2, Math.toRadians(-47)),
            new Pose(17.3, 100.4, Math.toRadians(107)),
            new Pose(11.5, 98.7, Math.toRadians(-164))
    };

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(0.25);
        follower.setStartingPose(POINTS[0]);
        buildRoute();

        telemetry.addLine("Ready - place robot on first point");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        follower.update();
        Pose p = follower.getPose();
        telemetry.addData("X", "%.1f", p.getX());
        telemetry.addData("Y", "%.1f", p.getY());
        telemetry.addData("Heading", "%.1f", Math.toDegrees(p.getHeading()));
        telemetry.update();
    }

    @Override
    public void start() {
        follower.setStartingPose(POINTS[0]);
        follower.followPath(redRoute, true);
    }

    @Override
    public void loop() {
        follower.update();
        Pose p = follower.getPose();
        telemetry.addData("Status", follower.isBusy() ? "Running" : "Done");
        telemetry.addData("X", "%.1f", p.getX());
        telemetry.addData("Y", "%.1f", p.getY());
        telemetry.addData("Heading", "%.1f", Math.toDegrees(p.getHeading()));
        telemetry.update();
    }

    private void buildRoute() {
        redRoute = follower.pathBuilder()
                .addPath(new BezierLine(POINTS[0], POINTS[1]))
                .setLinearHeadingInterpolation(POINTS[0].getHeading(), POINTS[1].getHeading())

                .addPath(new BezierLine(POINTS[1], POINTS[2]))
                .setConstantHeadingInterpolation(POINTS[1].getHeading())

                .addPath(new BezierLine(POINTS[2], POINTS[3]))
                .setLinearHeadingInterpolation(POINTS[2].getHeading(), POINTS[3].getHeading())

                .addPath(new BezierLine(POINTS[3], POINTS[4]))
                .setLinearHeadingInterpolation(POINTS[3].getHeading(), POINTS[4].getHeading())

                .addPath(new BezierLine(POINTS[4], POINTS[5]))
                .setLinearHeadingInterpolation(POINTS[4].getHeading(), POINTS[5].getHeading())

                .addPath(new BezierLine(POINTS[5], POINTS[6]))
                .setLinearHeadingInterpolation(POINTS[5].getHeading(), POINTS[6].getHeading())

                .addPath(new BezierLine(POINTS[6], POINTS[7]))
                .setLinearHeadingInterpolation(POINTS[6].getHeading(), POINTS[7].getHeading())

                .build();
    }
}
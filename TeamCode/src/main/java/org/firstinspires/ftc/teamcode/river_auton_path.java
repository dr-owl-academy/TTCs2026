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
    private PathChain[] paths;
    private int currentPath = 0;

    private static final Pose[] POINTS = {
            new Pose(32.3, 6.3, Math.toRadians(90)),
            new Pose(32.6, 10.3, Math.toRadians(86)),
            new Pose(34.0, 12.1, Math.toRadians(175)),
            new Pose(26.1, 32.7, Math.toRadians(57)),
            new Pose(21.8, 48.9, Math.toRadians(105)),
            new Pose(38.3, 31.2, Math.toRadians(-47)),
            new Pose(17.3, 100.4, Math.toRadians(107)),
            new Pose(11.5, 98.7, Math.toRadians(-164))
    };

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(0.2);          // slower so it stops better
        follower.setStartingPose(POINTS[0]);
        buildPaths();

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
        currentPath = 0;
        follower.followPath(paths[currentPath], true);
    }

    @Override
    public void loop() {
        follower.update();

        // When the current path finishes, start the next one
        if (!follower.isBusy() && currentPath < paths.length - 1) {
            currentPath++;
            follower.followPath(paths[currentPath], true);
        }

        Pose p = follower.getPose();
        telemetry.addData("Path", currentPath + 1 + " / " + paths.length);
        telemetry.addData("Status", follower.isBusy() ? "Running" : "Done");
        telemetry.addData("X", "%.1f", p.getX());
        telemetry.addData("Y", "%.1f", p.getY());
        telemetry.addData("Heading", "%.1f", Math.toDegrees(p.getHeading()));
        telemetry.update();
    }

    private void buildPaths() {
        paths = new PathChain[POINTS.length - 1];

        for (int i = 0; i < paths.length; i++) {
            paths[i] = follower.pathBuilder()
                    .addPath(new BezierLine(POINTS[i], POINTS[i + 1]))
                    .setLinearHeadingInterpolation(
                            POINTS[i].getHeading(),
                            POINTS[i + 1].getHeading()
                    )
                    .build();
        }

        // Special case: leftward move uses constant heading so it strafes
        paths[1] = follower.pathBuilder()
                .addPath(new BezierLine(POINTS[1], POINTS[2]))
                .setConstantHeadingInterpolation(POINTS[1].getHeading())
                .build();
    }
}
package org.firstinspires.ftc.teamcode;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.FilteredPIDFCoefficients;
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
            new Pose(32.5277, 8.4566, Math.toRadians(90)),
            new Pose(32.5607, 22.0115, Math.toRadians(89.8608)),
            new Pose(12.7213, 12.091, Math.toRadians(-153.4332)),
            new Pose(26.1111, 32.723, Math.toRadians(57.017)),
            new Pose(21.8256, 48.832, Math.toRadians(104.8973)),
            new Pose(38.3489, 31.2244, Math.toRadians(-46.8197)),
            new Pose(17.2518, 100.395, Math.toRadians(106.9617)),
            new Pose(11.5333, 98.7444, Math.toRadians(-163.9003))
    };

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        // Set PIDF directly in the autonomous so it corrects while following
        follower.setTranslationalPIDFCoefficients(new PIDFCoefficients(0.09, 0, 0.0034, 0.07));
        follower.setHeadingPIDFCoefficients(new PIDFCoefficients(1.0, 0, 0.01, 0.02));
        follower.setDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.025, 0, 0.0001, 0.6, 0.01));

        follower.setMaxPower(0.2);
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
        follower.update();   // this is what makes the PIDF correct the robot

        if (!follower.isBusy() && currentPath < paths.length - 1) {
            currentPath++;
            follower.followPath(paths[currentPath], true);
        }

        Pose p = follower.getPose();
        telemetry.addData("Path", (currentPath + 1) + " / " + paths.length);
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
    }
}
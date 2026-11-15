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
    private PathChain path;

    // Target point (first point in your route)
    private final Pose target = new Pose(32.5607, 22.0115, Math.toRadians(89.8608));

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        follower.setTranslationalPIDFCoefficients(new PIDFCoefficients(0.09, 0, 0.0034, 0.07));
        follower.setHeadingPIDFCoefficients(new PIDFCoefficients(2.5, 0, 0.02, 0.04));
        follower.setDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.025, 0, 0.0001, 0.6, 0.01));

        follower.setMaxPower(0.4);

        // Set a known pose on Init
        follower.setStartingPose(new Pose(72, 72, Math.toRadians(90)));

        telemetry.addLine("Init done. Move the robot to the real start position.");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // Keep updating so moving the robot by hand updates the pose
        follower.update();

        Pose p = follower.getPose();
        telemetry.addData("X", "%.1f", p.getX());
        telemetry.addData("Y", "%.1f", p.getY());
        telemetry.addData("Heading", "%.1f", Math.toDegrees(p.getHeading()));
        telemetry.addLine("Move robot to start, then press play");
        telemetry.update();
    }

    @Override
    public void start() {
        // Use whatever pose the robot is at right now as the start
        Pose current = follower.getPose();

        path = follower.pathBuilder()
                .addPath(new BezierLine(current, target))
                .setLinearHeadingInterpolation(current.getHeading(), target.getHeading())
                .build();

        follower.followPath(path, true);
    }

    @Override
    public void loop() {
        follower.update();

        // Hard stop when it finishes
        if (!follower.isBusy()) {
            follower.breakFollowing();
        }

        Pose p = follower.getPose();
        telemetry.addData("Status", follower.isBusy() ? "Running" : "Stopped");
        telemetry.addData("X", "%.1f", p.getX());
        telemetry.addData("Y", "%.1f", p.getY());
        telemetry.addData("Heading", "%.1f", Math.toDegrees(p.getHeading()));
        telemetry.update();
    }
}
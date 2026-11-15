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
    private boolean stopped = false;

    // New points (start farther away)
    private static final Pose[] POINTS = {
            new Pose(46.4165, 8.3509, Math.toRadians(90)),
            new Pose(46.3568, 23.9845, Math.toRadians(90.2191)),
            new Pose(12.7213, 12.091, Math.toRadians(-160.5264)),
            new Pose(26.1111, 32.723, Math.toRadians(57.017)),
            new Pose(21.8256, 48.832, Math.toRadians(104.8973)),
            new Pose(38.3489, 31.2244, Math.toRadians(-46.8197)),
            new Pose(17.2518, 100.395, Math.toRadians(106.9617)),
            new Pose(11.5333, 98.7444, Math.toRadians(-163.9003))
    };

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        follower.setTranslationalPIDFCoefficients(new PIDFCoefficients(0.09, 0, 0.0034, 0.07));
        follower.setHeadingPIDFCoefficients(new PIDFCoefficients(2.5, 0, 0.02, 0.04));
        follower.setDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.025, 0, 0.0001, 0.6, 0.01));

        follower.setMaxPower(0.35);
        follower.setStartingPose(POINTS[0]);
        buildPaths();

        telemetry.addLine("Ready - place robot on new start point");
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
        stopped = false;
        follower.followPath(paths[currentPath], true);
    }

    @Override
    public void loop() {
        follower.update();

        Pose p = follower.getPose();
        Pose target = POINTS[currentPath + 1];

        // Distance to the current target
        double dx = p.getX() - target.getX();
        double dy = p.getY() - target.getY();
        double distance = Math.sqrt(dx * dx + dy * dy);

        // Hard stop if close enough OR path is done
        if ((distance < 2.0 || !follower.isBusy()) && !stopped) {
            follower.breakFollowing();
            stopped = true;

            if (currentPath < paths.length - 1) {
                currentPath++;
                stopped = false;
                follower.followPath(paths[currentPath], true);
            }
        }

        telemetry.addData("Path", (currentPath + 1) + " / " + paths.length);
        telemetry.addData("Distance", "%.1f", distance);
        telemetry.addData("Status", stopped ? "Stopped" : "Running");
        telemetry.addData("X", "%.1f", p.getX());
        telemetry.addData("Y", "%.1f", p.getY());
        telemetry.addData("Heading", "%.1f", Math.toDegrees(p.getHeading()));
        telemetry.update();
    }

    private void buildPaths() {
        // Only first path active for testing
        paths = new PathChain[1];

        paths[0] = follower.pathBuilder()
                .addPath(new BezierLine(POINTS[0], POINTS[1]))
                .setLinearHeadingInterpolation(POINTS[0].getHeading(), POINTS[1].getHeading())
                .build();

        /*
        // Uncomment and change array size when ready
        paths[1] = follower.pathBuilder()
                .addPath(new BezierLine(POINTS[1], POINTS[2]))
                .setLinearHeadingInterpolation(POINTS[1].getHeading(), POINTS[2].getHeading())
                .build();
        */
    }
}
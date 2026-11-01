package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "River_auton_path")
public class river_auton_path extends OpMode {


    private Follower follower;

    private Pose[] route;
    private int currentPath = 0;
    private Pose pathStart;
    private Pose pathEnd;
    private int activePath = -1;



    // All route coordinates, in order
    private void buildPaths() {
        route = new Pose[] {
                new Pose(32.3119, 6.323, Math.toRadians(90)),
                new Pose(32.5607, 10.2601, Math.toRadians(86.3837)),
                new Pose(7.5644, 9.5585, Math.toRadians(-178.3922)),
                new Pose(26.1111, 32.723, Math.toRadians(51.3174)),
                new Pose(8.72, 47.8896, Math.toRadians(138.9085)),
                new Pose(38.3489, 31.2244, Math.toRadians(-29.3562)),
                new Pose(9.0037, 99.4074, Math.toRadians(113.2865)),
                new Pose(11.5333, 98.7444, Math.toRadians(-14.6857))
        };
    }
    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);

       buildPaths();

        // Reduced power for testing.
        follower.setStartingPose(route[0]);
        follower.setMaxPower(0.5);


        currentPath = 0;

        telemetry.addLine("Autonomous ready");
        telemetry.update();
    }


    @Override
    public void start() {

        currentPath = 0;
        activePath = -1;
        pathStart = null;
        pathEnd = null;
    }


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

        telemetry.addData("Path segment", currentPath);
        telemetry.addData("Total segments", route.length - 1);
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("Active Path", activePath);

        if (pathStart != null && pathEnd != null) {
            telemetry.addData("Start X", pathStart.getX());
            telemetry.addData("Start Y", pathStart.getY());

            telemetry.addData("End X", pathEnd.getX());
            telemetry.addData("End Y", pathEnd.getY());
        }
//helps with debug, shows if robot is moving or not or if it is still running the path
        if (currentPath >= route.length - 1
            && !follower.isBusy()) {
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
        if (follower.isBusy()) {
            return;
        }

        if (currentPath >= route.length - 1) {
            return;
        }
        activePath = currentPath;
        pathStart = route[currentPath];
        pathEnd = route[currentPath + 1];
        PathChain nextPath = follower.pathBuilder()
                .addPath(new BezierLine(
                        route[currentPath],
                        route[currentPath + 1]
                ))
                .setLinearHeadingInterpolation(
                        route[currentPath].getHeading(),
                        route[currentPath + 1].getHeading()
                )
                .build();

        follower.followPath(nextPath, true);
        currentPath++;
    }
}
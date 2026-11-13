package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "River Red Auton")
public class river_auton_path extends OpMode {

    // The robot's PedroPathing follower.
    private Follower follower;

    // All eight red-side positions, in the order the robot visits them.
    // Pose format: X position, Y position, heading in radians.
    private static final Pose[] RED_SIDE_POINTS = {

            new Pose(32.3119, 6.323, Math.toRadians(90)),

            new Pose(32.5607, 10.2601, Math.toRadians(86.3837)),

            new Pose(12.7213, 12.091, Math.toRadians(174.7275)),

            new Pose(26.1111, 32.723, Math.toRadians(57.017)),

            new Pose(21.8256, 48.855, Math.toRadians(104.8771)),

            new Pose(38.3489, 31.2244, Math.toRadians(-46.8569)),

            new Pose(17.2518, 100.395, Math.toRadians(106.9617)),

            new Pose(11.5333, 98.7444, Math.toRadians(-163.9003))
    };

    // There is one path between each pair of consecutive points.
    private PathChain[] paths =
            new PathChain[RED_SIDE_POINTS.length - 1];

    // Tracks which path should be started next.
    private int nextPathIndex = 0;

    // Becomes true after the final path finishes.
    private boolean autoComplete = false;


    @Override
    public void init() {

        // Create the follower using your existing Constants.java.
        follower = Constants.createFollower(hardwareMap);

        // Tell PedroPathing where the robot starts.
        follower.setStartingPose(RED_SIDE_POINTS[0]);

        // Keep the reduced power setting for initial testing.
        follower.setMaxPower(0.5);

        // Create all seven paths.
        buildPaths();

        telemetry.addLine("Red Autonomous Ready");
        telemetry.addData("Number of Points", RED_SIDE_POINTS.length);
        telemetry.addData("Number of Paths", paths.length);
        telemetry.update();
    }
    @Override
    public void init_loop() {

        telemetry.addData("Before Update X", follower.getPose().getX());
        telemetry.addData("Before Update Y", follower.getPose().getY());

        follower.update();

        telemetry.addData("After Update X", follower.getPose().getX());
        telemetry.addData("After Update Y", follower.getPose().getY());

        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));

        telemetry.update();
    }


    @Override
    public void start() {

        // Reset the autonomous tracking variables.
        nextPathIndex = 0;
        autoComplete = false;

        // Start the first path.
        if (paths.length > 0) {
            follower.followPath(paths[nextPathIndex], true);
            nextPathIndex++;
        }
    }


    @Override
    public void loop() {

        // Update PedroPathing every loop.
        follower.update();

        // If the current path has finished, start the next one.
        if (!autoComplete && !follower.isBusy()) {

            if (nextPathIndex < paths.length) {

                follower.followPath(paths[nextPathIndex], true);
                nextPathIndex++;

            } else {

                autoComplete = true;
            }
        }

        // Get the robot's current estimated position.
        Pose currentPose = follower.getPose();

        // Display useful information on the Driver Hub.
        telemetry.addData("X", currentPose.getX());
        telemetry.addData("Y", currentPose.getY());

        telemetry.addData(
                "Heading (degrees)",
                Math.toDegrees(currentPose.getHeading())
        );

        telemetry.addData(
                "Paths Started",
                nextPathIndex + " / " + paths.length
        );

        telemetry.addData("Follower Busy", follower.isBusy());

        telemetry.addData("Autonomous Complete", autoComplete);

        telemetry.update();
    }


    // Builds every path using the points in RED_SIDE_POINTS.
    private void buildPaths() {

        for (int i = 0; i < paths.length; i++) {

            paths[i] = makePath(
                    RED_SIDE_POINTS[i],
                    RED_SIDE_POINTS[i + 1]
            );
        }
    }


    // Creates one straight Bezier path between two poses.
    private PathChain makePath(Pose start, Pose end) {

        return follower.pathBuilder()
                .addPath(new BezierLine(start, end))
                .setLinearHeadingInterpolation(
                        start.getHeading(),
                        end.getHeading()
                )
                .build();
    }
}
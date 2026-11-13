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

    // PedroPathing follower controls the robot's movement.
    private Follower follower;

    // All of the red-side positions, in order.
    // Heading values must be in radians.
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

    // One path connects each point to the next point.
    private PathChain[] paths;

    // Tracks which path should start next.
    private int nextPathIndex = 0;

    // Becomes true after every path has finished.
    private boolean autoComplete = false;

    @Override
    public void init() {

        // Create the PedroPathing follower using your Constants file.
        follower = Constants.createFollower(hardwareMap);

        // Limit the robot's maximum movement power.
        follower.setMaxPower(0.5);

        // Build all paths before autonomous starts.
        buildPaths();

        telemetry.addLine("Red Autonomous Ready");
        telemetry.addData("Points", RED_SIDE_POINTS.length);
        telemetry.addData("Paths", paths.length);
        telemetry.update();
    }

    @Override
    public void init_loop() {

        // Keep the localizer updated while waiting for PLAY.
        follower.update();
    }

    @Override
    public void start() {

        // Update the localizer once more before starting movement.
        follower.update();

        // Set the robot's known starting position after localization updates.
        follower.setStartingPose(RED_SIDE_POINTS[0]);

        // Reset the autonomous progress.
        nextPathIndex = 0;
        autoComplete = false;

        // Start the first path.
        startNextPath();
    }

    @Override
    public void loop() {

        // Update PedroPathing every active loop.
        follower.update();

        // Only start another path after the current one finishes.
        if (!autoComplete && !follower.isBusy()) {

            if (nextPathIndex < paths.length) {

                startNextPath();

            } else {

                // No paths remain.
                autoComplete = true;
            }
        }

        telemetry.addData(
                "Autonomous",
                autoComplete ? "Complete" : "Running"
        );

        telemetry.addData(
                "Paths Started",
                nextPathIndex + " / " + paths.length
        );

        telemetry.update();
    }

    // Starts the next path in the array.
    private void startNextPath() {

        if (nextPathIndex < paths.length) {

            follower.followPath(paths[nextPathIndex], true);

            nextPathIndex++;
        }
    }

    // Creates a path between every pair of consecutive points.
    private void buildPaths() {

        paths = new PathChain[RED_SIDE_POINTS.length - 1];

        for (int i = 0; i < paths.length; i++) {

            Pose startPoint = RED_SIDE_POINTS[i];
            Pose endPoint = RED_SIDE_POINTS[i + 1];

            paths[i] = makePath(startPoint, endPoint);
        }
    }

    // Creates one straight-line path with a gradual heading change.
    private PathChain makePath(Pose startPoint, Pose endPoint) {

        return follower.pathBuilder()

                // Draw a straight line between the two positions.
                .addPath(new BezierLine(startPoint, endPoint))

                // Gradually rotate from the starting heading to the ending heading.
                .setLinearHeadingInterpolation(
                        startPoint.getHeading(),
                        endPoint.getHeading()
                )

                .build();
    }
}
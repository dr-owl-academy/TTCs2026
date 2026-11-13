
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

    private boolean autoComplete = false;

    // All original red-side points. Headings are in radians.
    private static final Pose[] POINTS = {
            new Pose(32.3119, 6.323, Math.toRadians(90)),
            new Pose(32.5607, 10.2601, Math.toRadians(86.3837)),
            new Pose(12.7213, 12.091, Math.toRadians(174.7275)),
            new Pose(26.1111, 32.723, Math.toRadians(57.017)),
            new Pose(21.8256, 48.855, Math.toRadians(104.8771)),
            new Pose(38.3489, 31.2244, Math.toRadians(-46.8569)),
            new Pose(17.2518, 100.395, Math.toRadians(106.9617)),
            new Pose(11.5333, 98.7444, Math.toRadians(-163.9003))
    };

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        follower.setMaxPower(0.5);
        follower.setStartingPose(POINTS[0]);

        buildRoute();

        telemetry.addLine("Red Autonomous Ready");
        telemetry.addData("Number of Points", POINTS.length);
        telemetry.update();
    }

    @Override
    public void init_loop() {
        follower.update();
    }

    @Override
    public void start() {
        autoComplete = false;
        follower.followPath(redRoute, true);
    }

    @Override
    public void loop() {
        follower.update();

        if (!follower.isBusy()) {
            autoComplete = true;
        }

        Pose current = follower.getPose();

        telemetry.addData("Status",
                autoComplete ? "Complete" : "Running");

        telemetry.addData("X", "%.1f", current.getX());
        telemetry.addData("Y", "%.1f", current.getY());

        telemetry.addData("Heading (deg)", "%.1f",
                Math.toDegrees(current.getHeading()));

        telemetry.addData("Busy", follower.isBusy());

        telemetry.update();
    }

    private void buildRoute() {
        redRoute = follower.pathBuilder()

                .addPath(new BezierLine(POINTS[0], POINTS[1]))
                .setLinearHeadingInterpolation(
                        POINTS[0].getHeading(), POINTS[1].getHeading())

                .addPath(new BezierLine(POINTS[1], POINTS[2]))
                .setLinearHeadingInterpolation(
                        POINTS[1].getHeading(), POINTS[2].getHeading())

                .addPath(new BezierLine(POINTS[2], POINTS[3]))
                .setLinearHeadingInterpolation(
                        POINTS[2].getHeading(), POINTS[3].getHeading())

                .addPath(new BezierLine(POINTS[3], POINTS[4]))
                .setLinearHeadingInterpolation(
                        POINTS[3].getHeading(), POINTS[4].getHeading())

                .addPath(new BezierLine(POINTS[4], POINTS[5]))
                .setLinearHeadingInterpolation(
                        POINTS[4].getHeading(), POINTS[5].getHeading())

                .addPath(new BezierLine(POINTS[5], POINTS[6]))
                .setLinearHeadingInterpolation(
                        POINTS[5].getHeading(), POINTS[6].getHeading())

                .addPath(new BezierLine(POINTS[6], POINTS[7]))
                .setLinearHeadingInterpolation(
                        POINTS[6].getHeading(), POINTS[7].getHeading())

                .build();
    }
}

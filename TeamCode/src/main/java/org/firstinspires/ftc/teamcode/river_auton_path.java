package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Auton Blue Lower - Fixed Points")
public class river_auton_path extends OpMode {

    private Follower follower;
    private Limelight3A limelight;

    private static final int TILT_TOWARD_BLUE = 0;
    private static final int TILT_AWAY_BLUE = 1;

    private int state = 0;
    private boolean pathStarted = false;

    private PathChain currentPath;

    // Your starting pose
    private static final Pose START = new Pose(52.7667, 8.8808, Math.toRadians(90));

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        follower.setStartingPose(START);
    }

    @Override
    public void start() {
        state = 0;
    }

    @Override
    public void loop() {
        follower.update();

        switch (state) {

            // Initial path to first shoot + garden
            case 0:
                runPath(
                        START,
                        new Pose(52.8867, 13.7422, Math.toRadians(88.5864)),
                        new Pose(14.8454, 14.5299, Math.toRadians(180))
                );
                if (pathFinished()) state = 1;
                break;

            // Second AI read (after garden / point ~2-3)
            case 1:
                int ai = readClassifier();
                if (ai == TILT_AWAY_BLUE) {
                    state = 10;   // go to miss-after-point-4 path
                } else {
                    state = 2;    // continue normal (you can fill this later)
                }
                break;

            // ========== MISS AFTER POINT 4 PATH ==========
            case 10:
                runPath(
                        new Pose(14.8454, 14.5299, Math.toRadians(180)),
                        new Pose(32.0611, 27.209, Math.toRadians(36.371)),
                        new Pose(14.8158, 44.3489, Math.toRadians(135.1755)),
                        new Pose(66.3695, 26.2173, Math.toRadians(160.6231)),
                        new Pose(67.1092, 123.9801, Math.toRadians(89.5665)),
                        new Pose(47.3004, 125.8486, Math.toRadians(-5.3888)),
                        new Pose(48.3276, 109.2928, Math.toRadians(-86.4495)),
                        new Pose(15.9648, 111.2813, Math.toRadians(-3.5162))
                );
                if (pathFinished()) state = 99;
                break;

            // ========== MISS AFTER POINT 6 PATH ==========
            // (call this from another AI check if you want)
            case 20:
                runPath(
                        new Pose(14.8454, 14.5299, Math.toRadians(180)),
                        new Pose(44.3715, 28.5071, Math.toRadians(25.3321)),
                        new Pose(14.1395, 47.2571, Math.toRadians(148.1927)),
                        new Pose(61.6249, 30.7197, Math.toRadians(160.7987)),
                        new Pose(59.2415, 126.6525, Math.toRadians(91.4232)),
                        new Pose(48.4612, 127.0181, Math.toRadians(-1.9424)),
                        new Pose(48.5842, 111.0631, Math.toRadians(-89.5582)),
                        new Pose(11.3629, 109.5779, Math.toRadians(2.285))
                );
                if (pathFinished()) state = 99;
                break;

            // Normal / toward path placeholder
            case 2:
                // TODO: put your normal (tilt toward) path here
                state = 99;
                break;

            case 99:
                telemetry.addLine("Done");
                break;
        }

        telemetry.addData("State", state);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    private int readClassifier() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid() || result.getClassifierResults().isEmpty()) {
            return -1;
        }
        try {
            return Integer.parseInt(result.getClassifierResults().get(0).getClassName());
        } catch (NumberFormatException e) {
            return -1;
        }
    }

    private void runPath(Pose... poses) {
        if (!pathStarted) {
            PathBuilder builder = follower.pathBuilder();
            for (int i = 0; i < poses.length - 1; i++) {
                builder.addPath(new BezierLine(poses[i], poses[i + 1]))
                        .setLinearHeadingInterpolation(poses[i].getHeading(), poses[i + 1].getHeading());
            }
            currentPath = builder.build();
            follower.followPath(currentPath);
            pathStarted = true;
        }
    }

    private boolean pathFinished() {
        if (pathStarted && !follower.isBusy()) {
            pathStarted = false;
            return true;
        }
        return false;
    }
}
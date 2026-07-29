package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Hanming_Simple_Auton")
public class Hanming_Simple_Auton extends OpMode {

    private enum AutoState {
        Init_turn,
        Wait_turn,
        Init_drive,
        Wait_drive,
        Complete
    }

    private Follower follower;
    private PathChain driveToTarget;

    private AutoState autoState = AutoState.Init_turn;

    // Robot begins at (72, 72), facing 90 degrees.
    private static final Pose START_POSE =
            new Pose(72, 72, Math.toRadians(90));

    // Same position after turning to 180 degrees.
    private static final Pose DRIVE_START_POSE =
            new Pose(72, 72, Math.toRadians(180));

    // Move 24 inches in the negative X direction.
    private static final Pose TARGET_POSE =
            new Pose(48, 72, Math.toRadians(180));

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(START_POSE);
        follower.setMaxPower(0.5);

        buildPath();

        telemetry.addLine("Autonomous ready");
        telemetry.addData("Starting pose", START_POSE);
        telemetry.update();
    }

    @Override
    public void start() {
        autoState = AutoState.Init_turn;
    }

    @Override
    public void loop() {
        // Must be called every loop.
        follower.update();

        autonomousPathUpdate();

        Pose currentPose = follower.getPose();

        telemetry.addData("State", autoState);
        telemetry.addData("Busy", follower.isBusy());
        telemetry.addData("X", currentPose.getX());
        telemetry.addData("Y", currentPose.getY());
        telemetry.addData(
                "Heading",
                Math.toDegrees(currentPose.getHeading())
        );

        if (autoState == AutoState.Complete) {
            telemetry.addLine("Autonomous complete");
        }

        telemetry.update();
    }

    private void buildPath() {
        driveToTarget = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                DRIVE_START_POSE,
                                TARGET_POSE
                        )
                )
                .setConstantHeadingInterpolation(
                        Math.toRadians(180)
                )
                .build();
    }

    private void autonomousPathUpdate() {
        switch (autoState) {

            case Init_turn:
                // Start turning from 90 degrees to 180 degrees.
                follower.turnTo(Math.toRadians(180));

                // Immediately switch to the waiting state so
                // turnTo() is not restarted every loop.
                autoState = AutoState.Wait_turn;
                break;

            case Wait_turn:
                // Wait until the turn has finished.
                if (!follower.isBusy()) {
                    autoState = AutoState.Init_drive;
                }
                break;

            case Init_drive:
                // Begin driving from X=72 to X=48.
                follower.followPath(driveToTarget, true);
                autoState = AutoState.Wait_drive;
                break;

            case Wait_drive:
                // Wait until the 24-inch drive has finished.
                if (!follower.isBusy()) {
                    autoState = AutoState.Complete;
                }
                break;

            case Complete:
                // Autonomous is finished.
                break;
        }
    }

    @Override
    public void stop() {
    }
}
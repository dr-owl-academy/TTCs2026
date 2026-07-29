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
    private static final Pose START_POSE = new Pose(72, 72, Math.toRadians(90));
    private static final Pose DRIVE_START_POSE = new Pose(72, 72, Math.toRadians(180));
    private static final Pose TARGET_POSE = new Pose(24, 72, Math.toRadians(180));

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START_POSE);
        follower.setMaxPower(0.5);

        buildPath();

        telemetry.addLine("Auton ready");
        telemetry.update();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        Pose currentPose = follower.getPose();
        telemetry.addData("X", currentPose.getX());
        telemetry.addData("Y", currentPose.getY());
        telemetry.addData(
                "Heading",
                Math.toDegrees(currentPose.getHeading())
        );
        telemetry.addData("State", autoState);
        if (autoState == AutoState.Complete) {
            telemetry.addLine("Autonomous complete");
        }

        telemetry.update();
    }

    @Override
    public void stop() {
    }

    private void buildPath() {
        driveToTarget = follower.pathBuilder()
                .addPath(new BezierLine(
                        DRIVE_START_POSE,
                        TARGET_POSE
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }

    private void autonomousPathUpdate() {
        switch (autoState) {

            case Init_turn:
                follower.turnTo(Math.toRadians(180));
                autoState = AutoState.Wait_turn;
                break;

            case Wait_turn:
                if (!follower.isBusy()) {
                    autoState = AutoState.Init_drive;
                }
                break;

            case Init_drive:
                follower.followPath(driveToTarget, true);
                autoState = AutoState.Wait_drive;
                break;

            case Wait_drive:
                if (!follower.isBusy()) {
                    autoState = AutoState.Complete;
                }
                break;

            case Complete:
                break;
        }
    }
}
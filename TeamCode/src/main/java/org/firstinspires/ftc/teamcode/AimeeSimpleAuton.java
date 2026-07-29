package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Aimee Simple Auton")
public class AimeeSimpleAuton extends OpMode {

    private enum AutoState {

        START_TO_TURN_180,
        WAIT_FOR_TURN_180,
        START_TO_DRIVE_TO_TARGET,
        WAIT_TO_DRIVE_TO_TARGET,
        DONE
    }

    //
    private Follower follower;

    //you can use the PathChain to make paths for each part of the run (segment)
    private PathChain driveToTarget;

    //this part starts the FSM state (think of the bubbles and how one leads to another)
    private AutoState autoState = AutoState.START_TO_TURN_180;


    private static final Pose START_POSE = new Pose(72,72, Math.toRadians(90));

    private static final Pose DRIVE_START_POSE = new Pose(72,72, Math.toRadians(180));

    private static final Pose TARGET_POSE = new Pose(24,72, Math.toRadians(180));

    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(START_POSE);

        //this is to reduce the power
        follower.setMaxPower(0.5);

        buildPath();

        telemetry.addLine("Autonomous ready");
        telemetry.update();
    }

    @Override
    public void loop() {

        //the pedropathing has to update every single cycle
        follower.update();

        //
        autonomousPathUpdate();

        Pose currentPose = follower.getPose();

        telemetry.addData("X",currentPose.getX());

        telemetry.addData("Y", currentPose.getY());

        telemetry.addData("Heading", Math.toDegrees(currentPose.getHeading()));

        telemetry.addData("State", autoState);

        if (autoState == AutoState.DONE) {

            telemetry.addLine("Autonomous complete");
        }

        telemetry.update();
    }

    @Override
    public void stop() {
    }

}

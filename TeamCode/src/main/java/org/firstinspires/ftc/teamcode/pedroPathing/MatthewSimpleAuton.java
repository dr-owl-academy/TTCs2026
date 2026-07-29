package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;


public class MatthewSimpleAuton {

    private enum Autostate{
        START_TURN_TO_180,
        WAIT_FOR_TURN_TO_180,
        START_DRIVE_TO_TARGET,
        WAIT_FOR_DRIVE_TO_TARGET,
        COMPLETE

    }
    private Follower follower;

    //use PathChain for each path segment
    private PathChain driveToTarget;

    // Starting FSM state.
    private Autostate autoState = Autostate.START_TURN_TO_180;

    private static final Pose START_POSE = new Pose(72,72,Math.toRadians(90) );
    private static final Pose DRIVE_START_POSE = new Pose(72,72,Math.toRadians(180));
    private static final Pose TARGET_POSE = new Pose(24,72, Math.toRadians(180));

    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(START_POSE);

        follower.setMaxPower(0.5);

        buildPath();

        telemetry.addLine("Autonomous ready");
        telemetry.update();

    }
    @Override
    public void loop() {

        // Pedro must update every loop.
        follower.update();

        // Update the autonomous FSM.
        autonomousPathUpdate();

        Pose currentPose = follower.getPose();

        telemetry.addData("X", currentPose.getX() );

        telemetry.addData("Y", currentPose.getY() );

        telemetry.addData("Heading", Math.toDegrees(currentPose.getHeading()));

        telemetry.addData("State", autoState );

        if (autoState == Autostate.COMPLETE) {

            telemetry.addLine("Autonomous complete" );
        }

        telemetry.update();
    }
}

package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Aimee Simple Auton")
public class AimeeSimpleAuton extends OpMode {

    private static final double INTAKE_VELOCITY = 1500; //eat
    private DcMotorEx LeIntake;

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

    //private static final
    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(START_POSE);

        //this is to reduce the power
        follower.setMaxPower(0.5);

        LeIntake = (DcMotorEx) hardwareMap.get(DcMotor.class, "intakemotor"); //continue this later!!
        LeIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        LeIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        buildPath();

        telemetry.addLine("Autonomous ready");
        telemetry.update();
    }

    @Override
    public void loop() {

        //the pedropathing has to update every single cycle
        follower.update();

        //this is to update the FSM
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

    //this will build all the PathChains that are going to be used

    private void buildPath() {
        driveToTarget = follower.pathBuilder()
                .addPath(new BezierLine(DRIVE_START_POSE, TARGET_POSE))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }

    //this updates the FSM
    private void autonomousPathUpdate() {

        switch (autoState) {

            case START_TO_TURN_180:
                //start turning from 90 degrees all the way to 0 degrees
                follower.turnTo(Math.toRadians(180));
                autoState = AutoState.WAIT_FOR_TURN_180;
                break;

            case WAIT_FOR_TURN_180:
                //this waits, very patiently, for the turn to be done
                if(!follower.isBusy()) {
                    autoState = AutoState.START_TO_DRIVE_TO_TARGET;
                }
                break;

            case START_TO_DRIVE_TO_TARGET:
                //starts driving to the target location and then stops and holds its position
                follower.followPath(driveToTarget,true);
                autoState = AutoState.WAIT_TO_DRIVE_TO_TARGET;
                break;

            case WAIT_TO_DRIVE_TO_TARGET:
                //wait for the robot to stop driving before starting the next part
                if (!follower.isBusy()) {
                    autoState = AutoState.DONE;
                }
                break;

            case DONE:
                break;
        }
    }
}

package org.firstinspires.ftc.teamcode;


import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Simple Auton Code Yajat")
public class YajatSimpleAuton extends OpMode {

    private DcMotorEx intakeMotor;
    private static final double INTAKE_VELOCITY = 2000;

    //now we put the things in order ig
    private enum AutoState {
        START_TURN_TO_180,
        WAIT_FOR_TURN,
        START_DRIVE_TO_TARGET,
        WAIT_FOR_DRIVE_TO_TARGET,
        START_DRIVE_2,
        WAIT_FOR_DRIVE2,
        COMPLETE
    }


    private Follower follower;

    //use pathchaining
    private PathChain driveToTarget;
    private AutoState autoState = AutoState.START_TURN_TO_180;

//did somthing i think. also lots of errors


    private static final Pose START_POSE = new Pose(72, 72, Math.toRadians(90));

    private static final Pose DRIVE_START_POSE = new Pose(72, 72, 180);

    private static final Pose TARGET_POSE = new Pose(24, 72, 180);

//i think i did something so the bot knows where to go

    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(START_POSE);

        intakeMotor =
                hardwareMap.get(DcMotorEx.class, "intakemotor");

        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setVelocity(0);



        // Reduced power for initial testing.
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

        if (autoState == AutoState.COMPLETE) {

            telemetry.addLine("Autonomous complete" );
        }

        telemetry.update();
    }

    @Override
    public void stop() {
    }

    // Builds all PathChains used by this autonomous.
    private void buildPath() {

        driveToTarget = follower.pathBuilder()
                .addPath(new BezierLine(DRIVE_START_POSE,TARGET_POSE))
                .setConstantHeadingInterpolation( Math.toRadians(180))
                .build();
    }

    // Updates the autonomous finite state machine.
    private void autonomousPathUpdate() {

        switch (autoState) {

            case START_TURN_TO_180:
                // Turn in place from 90 degrees to 0 degrees.
                follower.turnTo(Math.toRadians(180));
                autoState = AutoState.WAIT_FOR_TURN;
                break;

            case WAIT_FOR_TURN:
                // Wait for the turn to finish.
                if (!follower.isBusy()) {
                    autoState = AutoState.START_DRIVE_TO_TARGET;
                }
                break;

            case START_DRIVE_TO_TARGET:
                /* Start driving to the target.
                 * true tells Pedro to hold the final pose.
                 */
                follower.followPath(driveToTarget, true);
                autoState = AutoState.WAIT_FOR_DRIVE_TO_TARGET;
                break;

            case WAIT_FOR_DRIVE_TO_TARGET:
                // Wait for the driving path to finish.
               //intake start ig
                intakeMotor.setVelocity(-INTAKE_VELOCITY);

                if (!follower.isBusy()) {
                    autoState = AutoState.COMPLETE;

                }
                break;

            case START_DRIVE_2:



            case COMPLETE:
                break;
        }


    }
}
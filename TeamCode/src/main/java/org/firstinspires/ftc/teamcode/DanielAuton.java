/*
package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous(name = "Daniel Auton")
public class DanielAuton extends OpMode {

    private DcMotorEx IntakeMotor; //Shows that intake motor is connected to DCMotorEx

    //What is stated in Finite State Machine (Commands such as Start, wait, turn, etc.)


    //tells if action is done or not done
    private enum Autostate {
        START_TURN_TO_180,
        WAIT_FOR_TURN_TO_180,
        START_DRIVE_TO_TARGET,
        WAIT_FOR_DRIVE_TO_TARGET,
        COMPLETE
    }



    private Follower follower;

    //Pathchain links paths, use for each segment
    private PathChain driveToTarget;

    //the first action or starts the finite state machine
    private Autostate autoState = Autostate.START_TURN_TO_180;

    IntakeMotor = hardwareMap.get(DcMotor.class,"intake")

    IntakeMotor.setdirection(DcMotor.Direction.FORWARD);
    IntakeMotor.ZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    //Sets what positions to move to during FSM
    private static final Pose START_POSE = new Pose(72, 72, Math.toRadians(90)); //original direction
    private static final Pose DRIVE_START_POSE = new Pose(72, 72, Math.toRadians(180)); //turn 90 degrees
    private static final Pose TARGET_POSE = new Pose(24, 72, Math.toRadians(180)); //move x
    private static final double INTAKE_VELOCITY = -2000; //setting intake speed


    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(START_POSE);

        //Reducing the power for initial testing (can change later)
        follower.setMaxPower(0.5);

        buildPath();

        //shows robot is ready to run
        telemetry.addLine("Autonomous ready");
        telemetry.update();
    }

    @Override
    public void loop() {

        //Updates the PedroPathing
        follower.update();

        //Updates the autonomous for FSM
        autonomousPathUpdate();

        //Gets the pose for follower (know what coordinate it is)
        Pose currentPose = follower.getPose();

        //telemetry gives me the x and y positions on screen and other info
        telemetry.addData("X", currentPose.getX());

        telemetry.addData("X", currentPose.getY());

        telemetry.addData("Heading", Math.toDegrees(currentPose.getHeading()));

        telemetry.addData("State", autoState);

        if (autoState == Autostate.COMPLETE) {

            telemetry.addLine("Autonomous complete"); //auto is complete after action is complete
        }

        telemetry.update();
    }

    @Override
    public void stop() {

    }

    //Here is where you build or put all the pathchains used for the autonomous
    private void buildPath() {

        driveToTarget = follower.pathBuilder()
                .addPath(new BezierLine(DRIVE_START_POSE, TARGET_POSE))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }

    // Makes the updates for Finite State Machine
    private void autonomousPathUpdate() {

        switch (autoState) {

            case START_TURN_TO_180:
                //turn in place from 90 to 180 degrees
                follower.turnTo(Math.toRadians(180));
                autoState = Autostate.WAIT_FOR_TURN_TO_180;
                break;

            case WAIT_FOR_TURN_TO_180:
                //Wait for turn to finish, follower is busy means it is finished
                if (!follower.isBusy()) {
                    autoState = Autostate.START_DRIVE_TO_TARGET;
                }
                break;

            case START_DRIVE_TO_TARGET:
                //Drive to the target, and the true tells the pedropathing to not turn (hold final pose)
                follower.followPath(driveToTarget, true);
                autoState = Autostate.START_DRIVE_TO_TARGET;
                break;

            case WAIT_FOR_DRIVE_TO_TARGET:
                //Wait for driving path to finish
                if (!follower.isBusy()) {
                    autoState = Autostate.COMPLETE;
                }
                break;
            case COMPLETE:
                break;
        }

    }
}*/
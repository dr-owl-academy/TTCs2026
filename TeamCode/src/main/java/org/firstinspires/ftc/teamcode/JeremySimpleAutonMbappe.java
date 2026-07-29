// vocab:
// private: belongs only to this script
// static: belongs to the hole class not one object
// final: doesn't change after init
// void: doesn't return anything
// double: pos/neg integer

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

@Autonomous(name = "JeremySimpleAutonMbappe") // tells the robot the code is autonomous
public class JeremySimpleAutonMbappe extends OpMode {

    private static final double INTAKE_VELOCITY = 2000; // make the speed of mbappes legs
    private DcMotorEx mbappesLegs; // create the motor

    private enum AutoState { // create enums for the tasks (finite state machine [fsm])
        START_TURNING_180,
        WAIT_TO_TURN_180,
        START_DRIVING_TO_TARGET,
        WAIT_TO_DRIVE_TO_TARGET,
        COMPLETE
    }

    private Follower simpleMbappeSpecial; // define a follower to do the tasks

    private PathChain driveToTarget; // make a path chain for each task segment

    private AutoState autoState = AutoState.START_TURNING_180; // start running fsm

    // making each chain segment of the fsm; radians are used in trig. to calculate things easier (other degrees)
    private static final Pose START_POSE = new Pose(72, 72, Math.toRadians(90));
    private static final Pose DRIVING_POSE = new Pose(72, 72, Math.toRadians(180));
    private static final Pose TARGET_POSE = new Pose(24, 72, Math.toRadians(180));

    @Override
    public void init(){
        simpleMbappeSpecial = Constants.createFollower(hardwareMap); // create the follower

        simpleMbappeSpecial.setStartingPose(START_POSE); // make mbappe start the special

        simpleMbappeSpecial.setMaxPower(1); // set mbappe's strength

        mbappesLegs = (DcMotorEx) hardwareMap.get(DcMotor.class, "intakemotor"); // set mbappe's legs to the intake motor

        // set up intake motor
        mbappesLegs.setDirection(DcMotorSimple.Direction.FORWARD); // make the intake spin forward
        mbappesLegs.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // make the motor brake when not spinning
        mbappesLegs.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mbappesLegs.setVelocity(0); // dont make mbappe drift

        buildPath();

        // output to express how ready mbappe is
        telemetry.addLine("Mbappe Ready!");
        telemetry.update(); // update telemetry
    }

    @Override
    public void loop(){
        simpleMbappeSpecial.update(); // update the follower every cycle

        autonomousPathUpdate(); // update the fsm

        Pose curPose = simpleMbappeSpecial.getPose();

        // add data to the telemetry: display the position
        telemetry.addData("X: ", curPose.getX());
        telemetry.addData("Y: ", curPose.getY());
        telemetry.addData("Heading: ", Math.toDegrees(curPose.getHeading()));
        telemetry.addData("State: ", autoState);

        if(autoState == AutoState.COMPLETE){ // when we are done say "we are done"
            telemetry.addLine("Mbappe Special is done; sometimes i get i little competitive, but its good");
        }

        telemetry.update();
    }

    @Override
    public void stop(){} // stop the bot while keeping pedropath

    private void buildPath(){ // build mbappes path
        driveToTarget = simpleMbappeSpecial.pathBuilder() // making a path from one point to another
                .addPath(new BezierLine(DRIVING_POSE, TARGET_POSE))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }

    private void autonomousPathUpdate(){ // write when to switch to next fsm segment
        switch(autoState){
            case START_TURNING_180: // first step: turn 180 degrees
                simpleMbappeSpecial.turnTo(Math.toRadians(180)); // start turning
                autoState = AutoState.WAIT_TO_TURN_180;
                break;

            case WAIT_TO_TURN_180: // second step: wait to turn 180
                if(!simpleMbappeSpecial.isBusy()){ // check if we are still turning
                    autoState = AutoState.START_DRIVING_TO_TARGET; // if not then move on
                }
                break;

            case START_DRIVING_TO_TARGET: // third step: move 2 tiles
                simpleMbappeSpecial.followPath(driveToTarget, true); // start moving
                mbappesLegs.setVelocity(INTAKE_VELOCITY); // start moving intake
                autoState = AutoState.WAIT_TO_DRIVE_TO_TARGET;
                break;

            case WAIT_TO_DRIVE_TO_TARGET: //  fourth step: wait to move 2 tiles
                if(!simpleMbappeSpecial.isBusy()){
                    autoState = AutoState.COMPLETE;
                }
                break;

            case COMPLETE: // fifth: finish
                break;

        }
    }





}

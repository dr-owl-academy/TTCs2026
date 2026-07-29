package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "EasonSimpleAuton")
public class EasonSimpleAuton extends OpMode {

    //Machine States
    private enum AutoState {
        START_TURN_1,
        WAIT_FOR_TURN_1,
        START_DRIVE_TO_TARGET_1,
        WAIT_FOR_DRIVE_TO_TARGET,
        COMPLETE
    }
    //Naming Follower
    private Follower follower;
    //Naming PathChain
    private PathChain driveToTarget;
    //Start State
    private AutoState autoState = AutoState.START_TURN_1;
    //Show Action
    private static final Pose START_POSE = new Pose(72,72,Math.toRadians(90));
    private static final Pose DRIVE_START_POSE = new Pose(72,72,Math.toRadians(180));
    private static final Pose TARGET_POSE = new Pose(24,72,Math.toRadians(180));

    @Override
    public void init(){
        follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(START_POSE);

        //Less Power
        follower.setMaxPower(0.5);

        buildPath();
        //Show String
        telemetry.addLine("Auton Ready");
        telemetry.update();
    }

    @Override
    public void loop(){
        //update Pedro Pathing
        follower.update();

        //Update the Machine States
        autonomousPathUpdate();
        //Show Current Position
        Pose currentPose = follower.getPose();

        telemetry.addData("X",currentPose.getX());

        telemetry.addData("X",currentPose.getX());

        telemetry.addData("Heading",Math.toDegrees(currentPose.getHeading()));

        telemetry.addData("State", autoState);

        if (autoState== AutoState.COMPLETE) {
            telemetry.addLine("Autonomous complete");
        }

        telemetry.update();
    }
    @Override
    public void stop(){
    }
    private void buildPath(){
        driveToTarget=follower.pathBuilder()
                //Straight Line
                .addPath(new BezierLine(DRIVE_START_POSE,TARGET_POSE))
                //Maintain Heading
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }
    //Update Machine States
    private void autonomousPathUpdate() {
        switch (autoState) {
            case START_TURN_1:
                //Turn to 180 degrees in PedroPathing
                follower.turnTo( Math.toRadians(180));
                autoState = AutoState.WAIT_FOR_TURN_1;
                break;

            case WAIT_FOR_TURN_1:
                //Wait Until Robot is Facing 180 Degrees
                if (!follower.isBusy()){
                    autoState = AutoState.START_DRIVE_TO_TARGET_1;
                }
                break;

            case START_DRIVE_TO_TARGET_1:
                //Driving To Target
                follower.followPath(driveToTarget,true);
                autoState = AutoState.WAIT_FOR_DRIVE_TO_TARGET;
                break;

            case WAIT_FOR_DRIVE_TO_TARGET:
                //Wait Until Done With Driving
                if (!follower.isBusy()){
                    autoState = AutoState.COMPLETE;
                }
                break;

            case COMPLETE:
                //Hold Position
                break;
        }

    }
}
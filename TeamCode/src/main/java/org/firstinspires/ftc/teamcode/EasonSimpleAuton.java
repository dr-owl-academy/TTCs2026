package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
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
        follower - Constants.createFollower(hardwareMap);

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

        if (autoState== AutoState.COMPLETE) {}


    }
}
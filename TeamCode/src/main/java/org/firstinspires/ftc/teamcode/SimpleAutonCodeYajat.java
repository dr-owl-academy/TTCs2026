package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
@Autonomous(name = "Simple Auton Code Yajat")
public class simpleAutonCodeYajat extends OpMode {
  //now we put the things in order ig
    private enum AutoState{
        START_TURN_TO_180,

        WAIT_FOR_TURN,
        START_DRIVE_TO_TARGET,
        WAIT_FOR_TARGET,
        COMPLETE
    }
}

private Follower follower;

//use pathchaining
private PathChain driveToTarget;

//did somthing i think. also lots of errors

private YajatSimpleAuton.AutoState = YajatSimpleAuton.AutoState.START_TURN_TO_180;

private static final pose START_POSE = new Pose(72,72,Math.toRadians());

private static final pose DRIVE_START_POSE = new Pose(72,72,180);

private static final pose TARGET_POSE = new Pose(24,72,180);

//i think i did something so the bot knows where to go

@Override
public void init() {}
package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous(name = "Daniel Auton")
public class DanielAuton extends OpMode {
}

//What is stated in Finite State Machine (Commands such as Start, wait, turn, etc.)
private enum Autostate {
    START_TURN_TO_180,
    WAIT_FOR_TURN_TO_180,
    START_DRIVE_TO_TARGET,
    WAIT_FOR_DRIVE_TO_TARGET,
    COMPLETE
}

//tells if action is done or not done
private Follower follower;

//Pathchain links paths, use for each segment
private PathChain driveToTarget;

//the first action or starts the finite state machine
private Autostate autoState = Autostate.START_TURN_TO_180;

//Sets what positions to to move to during FSM
private static final Pose START_POSE = new Pose(72,72,Math.toRadians(90));
private static final Pose DRIVE_START_POSE = new Pose(72,72,Math.toRadians(180));
private static final Pose TARGET_POSE = new Pose(72,72,Math.toRadians(180));

@Override
public void init() {

    follower = Constants.createFollower(hardwareMap);

}
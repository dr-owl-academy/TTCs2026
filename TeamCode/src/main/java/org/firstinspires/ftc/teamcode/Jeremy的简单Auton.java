// vocab:
// private: belongs only to this script
// static: belongs to the hole class not one object
// final: doesn't change after init
// void: doesn't return anything

package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Jeremy的简单Auton") // tells the robot the code is autonomous
public class Jeremy的简单Auton {

    private enum AutoState { // create enums for the tasks (finite state machine [fsm])
        START_TURNING_180,
        WAIT_TO_TURN_180,
        START_DRIVING_TO_TARGET,
        WAIT_TO_DRIVE_TO_TARGET,
        COMPLETE;
    }

    private Follower simpleMbappeSpecial; // make a follower to do the tasks

    private PathChain driveToTarget; // make a path chain for each task segment

    private AutoState autoState = AutoState.START_TURNING_180; // start running fsm

    // making each chain segment of the fsm; radians are used in trig. to calculate things easier (other degrees)
    private static final Pose START_POSE = new Pose(72, 72, Math.toRadians(90));
    private static final Pose DRIVING_POSE = new Pose(72, 72, Math.toRadians(180));
    private static final Pose TARGET_POSE = new Pose(24, 72, Math.toRadians(180));

    @Override
    private void init(){
        simpleMbappeSpecial = Constants.createFollower(hardwareMap);

    }




}

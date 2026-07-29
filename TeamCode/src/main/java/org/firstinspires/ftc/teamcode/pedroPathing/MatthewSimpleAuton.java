package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


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
}

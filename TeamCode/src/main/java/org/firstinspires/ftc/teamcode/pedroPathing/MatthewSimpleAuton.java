package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;

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

}

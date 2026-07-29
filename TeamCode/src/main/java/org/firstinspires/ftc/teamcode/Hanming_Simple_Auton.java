package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Hanming_Simple_Auton")
public class Hanming_Simple_Auton {
}

    private enum Autostate {
        Init_turn,
        Wait_turn,
        Init_drive,
        Wait_drive,
        Complete
}

    private Follower follower;
    private path pathChain;
    private AutoState autoState = AutoState.Init_turn;
    private static final Pose Start_Pose = new Pose(72,72,Math.toRadians(90) );
    private static final Pose Drive_Start_Pose = new Pose(72,72,Math.toRadians(180));
    private static final Pose Complete_Pose = new Pose(24,72, Math.toRadians(180));

    @Override
    public void init() {

        follower =
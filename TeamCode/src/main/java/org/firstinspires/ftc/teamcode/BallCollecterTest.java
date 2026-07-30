package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.lang.reflect.Parameter;

@Autonomous
public class BallCollecterTest extends OpMode {

    private Follower follower;

    public enum PathState {
    /*
    START POS - MID FIRST BALL
    MID FIRST BALL - FIRST BALL
    FIRST BALL - MID SECOND BALL
    MID SECOND BALL - SECOND BALL
    SECOND BALL - THIRD BALL
    THIRD BALL - DEPOSIT
      */



    }

    PathState pathState;
//start with middle pose since we know where to start it
    private final Pose startingPose = new Pose(34.40354090354091,133.77777777777777,Math.toRadians(270));
    private final Pose mid1Pose = new Pose(101.7844932844933,85.06715506715508,Math.toRadians(270));
    private final Pose ball1Pose = new Pose(77.4977220676374,71.41611221732335,Math.toRadians(180));
    private final Pose mid2Pose = new Pose(79.84632554962174,24.328636451650116,Math.toRadians(180));
    private final Pose ball2Pose = new Pose(27.4282145960422,21.987112844302306,Math.toRadians(180));

    private PathChain startPoseMid1Pose;

    public void buildPaths() {

        startPoseMid1Pose = follower.pathBuilder()
                .addPath(new BezierCurve(startingPose,mid1Pose))
                .setLinearHeadingInterpolation(startingPose.getHeading(),mid1Pose.getHeading())
                .build();
    }



    @Override
    public void init() {


    }

    @Override
    public void loop() {

    }
}

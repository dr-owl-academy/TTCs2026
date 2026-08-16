package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

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
        STARTPOS_1MID,
        MID1_BALL1,
        BALL1_MID2,
        MID2_BALL2,
        BALL2_BALL3,
        BALL3_DEPOSIT,
        WAIT_FOR_DEPOSIT,
        COMPLETE

    }

    PathState pathState;
//start with middle pose since we know where to start it
    private final Pose startingPose = new Pose(34.40354090354091,133.77777777777777,Math.toRadians(270));
    private final Pose mid1Pose = new Pose(101.7844932844933,85.06715506715508,Math.toRadians(270));
    private final Pose ball1Pose = new Pose(77.4977220676374,71.41611221732335,Math.toRadians(180));
    private final Pose mid2Pose = new Pose(79.84632554962174,24.328636451650116,Math.toRadians(180));
    private final Pose ball2Pose = new Pose(27.4282145960422,21.987112844302306,Math.toRadians(180));
    private final Pose ball3Pose = new Pose(134.43833943833945,35.81623931623931,Math.toRadians(0));
    private final Pose deposit = new Pose(132.04214983276103,23.63395836848312,Math.toRadians(270));

    private PathChain startPoseMid1Pose;
    private PathChain Mid1toBall1;
    private PathChain Ball1toMid2;
    private PathChain Mid2toBall2;
    private PathChain Ball2toBall3;
    private PathChain Ball3toDeposit;

    public void buildPaths() {

        startPoseMid1Pose = follower.pathBuilder()
                .addPath(new BezierLine(startingPose,mid1Pose))
                .setLinearHeadingInterpolation(startingPose.getHeading(),mid1Pose.getHeading())
                .build();

        Mid1toBall1 = follower.pathBuilder()
                .addPath(new BezierLine(mid1Pose,ball1Pose))
                .setLinearHeadingInterpolation(mid1Pose.getHeading(),ball1Pose.getHeading())
                .build();

        Ball1toMid2 = follower.pathBuilder()
                .addPath(new BezierLine(ball1Pose,mid2Pose))
                .setLinearHeadingInterpolation(ball1Pose.getHeading(),mid2Pose.getHeading())
                .build();

        Mid2toBall2 = follower.pathBuilder()
                .addPath(new BezierLine(mid2Pose,ball2Pose))
                .setLinearHeadingInterpolation(mid2Pose.getHeading(),ball2Pose.getHeading())
                .build();

        Ball2toBall3 = follower.pathBuilder()
                .addPath(new BezierLine(ball2Pose,ball3Pose))
                .setLinearHeadingInterpolation(ball2Pose.getHeading(),ball3Pose.getHeading())
                .build();

        Ball3toDeposit = follower.pathBuilder()
                .addPath(new BezierLine(ball3Pose,deposit))
                .setLinearHeadingInterpolation(ball3Pose.getHeading(),deposit.getHeading())
                .build();

    }

    public void statePathUpdate() {
        switch (pathState) {
            case STARTPOS_1MID:
                follower.followPath(startPoseMid1Pose, true);
                pathState = PathState.MID1_BALL1;
                break;

            case MID1_BALL1:
                if (!follower.isBusy()) {
                    follower.followPath(Mid1toBall1, true);
                    pathState = PathState.BALL1_MID2;
                }
                break;

            case BALL1_MID2:
                if (!follower.isBusy()) {
                    follower.followPath(Ball1toMid2);
                    pathState = PathState.MID2_BALL2;
                }
                break;

            case MID2_BALL2:
                if (!follower.isBusy()) {
                    follower.followPath(Mid2toBall2);
                    pathState = PathState.BALL2_BALL3;
                }
                break;

            case BALL2_BALL3:
                if (!follower.isBusy()) {
                    follower.followPath(Ball2toBall3);
                    pathState = PathState.BALL3_DEPOSIT;
                }
                break;

            case BALL3_DEPOSIT:
                if (!follower.isBusy()){
                    follower.followPath(Ball3toDeposit);
                pathState = PathState.WAIT_FOR_DEPOSIT;
                }
                break;

            case WAIT_FOR_DEPOSIT:
                if(!follower.isBusy()){
                    pathState = PathState.COMPLETE;
                }

            case COMPLETE:
                break;




        }
    }


    @Override
    public void init() {
        pathState=PathState.STARTPOS_1MID;

        follower= Constants.createFollower(hardwareMap);

        follower.setStartingPose(startingPose);

        buildPaths();

    }

    public void start() {


    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();
    }
}

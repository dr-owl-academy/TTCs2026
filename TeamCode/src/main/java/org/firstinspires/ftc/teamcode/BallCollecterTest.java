package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.lang.reflect.Parameter;

@Autonomous
public class BallCollecterTest extends OpMode {

    private Follower follower;

    private DcMotor Intake;
    private ElapsedTime motorTimer = new ElapsedTime();

    private boolean motorRunning = false;
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
    private final Pose ball3Pose = new Pose(124.34002433090023,50.002433090024326,Math.toRadians(0));
    private final Pose deposit = new Pose(123.19884169884172,25.482625482625487,Math.toRadians(270));

    private PathChain startPoseMid1Pose;
    private PathChain Mid1toBall1;
    private PathChain Ball1toMid2;
    private PathChain Mid2toBall2;
    private PathChain Ball2toBall3;
    private PathChain Ball3toDeposit;

    private double motorPower;

    private void startMotor(double power){
        motorTimer.reset();
        motorRunning = true;
        motorPower = power;
        Intake.setPower(power);
    }

    private void updateMotor() {
        if (motorRunning) {
            if (motorTimer.seconds()<2.0){
                Intake.setPower(motorPower);
            } else {
                Intake.setPower(0);
                motorRunning = false;
            }
        }
    }
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
                startMotor(1.0);
                follower.followPath(startPoseMid1Pose, true);
                pathState = PathState.MID1_BALL1;
                break;

            case MID1_BALL1:
                if (!follower.isBusy()) {
                    startMotor(1.0);
                    follower.followPath(Mid1toBall1, true);
                    pathState = PathState.BALL1_MID2;
                }
                break;

            case BALL1_MID2:
                if (!follower.isBusy()) {
                    startMotor(1.0);
                    follower.followPath(Ball1toMid2, true);
                    pathState = PathState.MID2_BALL2;
                }
                break;

            case MID2_BALL2:
                if (!follower.isBusy()) {
                    startMotor(1.0);
                    follower.followPath(Mid2toBall2, true);
                    pathState = PathState.BALL2_BALL3;
                }
                break;

            case BALL2_BALL3:
                if (!follower.isBusy()) {
                    startMotor(1.0);
                    follower.followPath(Ball2toBall3,true);
                    pathState = PathState.BALL3_DEPOSIT;
                }
                break;

            case BALL3_DEPOSIT:
                if (!follower.isBusy()){
                    follower.followPath(Ball3toDeposit,true);
                pathState = PathState.WAIT_FOR_DEPOSIT;
                }
                break;

            case WAIT_FOR_DEPOSIT:
                if(!follower.isBusy()){
                    startMotor(-1.0);
                    pathState = PathState.COMPLETE;
                }
                break;

            case COMPLETE:
                break;




        }
    }


    @Override
    public void init() {


        pathState=PathState.STARTPOS_1MID;

        follower= Constants.createFollower(hardwareMap);

        follower.setStartingPose(startingPose);

        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Intake.setPower(0);

        buildPaths();

    }

    public void start() {


    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();
        updateMotor();
    }
}

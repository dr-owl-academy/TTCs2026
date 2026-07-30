package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.JeremySimpleAutonMbappe;

@Autonomous
public class BallCollectorAuton extends OpMode {

    private static final double INTAKE_VELOCITY = -2000; // make the speed of intake motor
    private DcMotorEx intakeMotor; // create the motor

    private Follower boiledPotatoe; // make a new follower

    private enum AutonState { // each action
        PICK_UP_FIRST_BALL,
        /*MOVE_RIGHT,
        PICK_UP_SECOND_BALL,
        PICK_UP_THIRD_BALL,
        DEPOSIT_BALL*/
    }

    private PathChain driveToFirstBall; // make the collection path chain

    private BallCollectorAuton.AutonState autonState = AutonState.PICK_UP_FIRST_BALL; // start running fsm

    // create poses
    private static final Pose START_POSE = new Pose(37.75, 133.5, Math.toRadians(270));
    private static final Pose BALL_ONE_POSE = new Pose(24, 24, Math.toRadians(270));
    private static final Pose INTERMEDIATE_POSE = new Pose(84, 24, Math.toRadians(0));
    private static final Pose BALL_TWO_POSE = new Pose(77, 72, Math.toRadians(90));
    private static final Pose BALL_THREE_POSE = new Pose(134, 48, Math.toRadians(0));
    private static final Pose DEPOSIT_POSE = new Pose(132, 20, Math.toRadians(270));

    @Override
    public void init() {
        boiledPotatoe = Constants.createFollower(hardwareMap); //create the follower hdwr map
        boiledPotatoe.setStartingPose(START_POSE); // set the first pose
        boiledPotatoe.setMaxPower(1); // robot's max power

        buildPath(); // build the bezier line path
    }


    @Override
    public void loop() {
        // update
        boiledPotatoe.update(); // update the follower
        autonomousPathUpdate(); // update the fsm

        Pose curPose = boiledPotatoe.getPose(); // the robot's pose

        // add data to the telemetry: display the position
        telemetry.addData("X: ", curPose.getX());
        telemetry.addData("Y: ", curPose.getY());
        telemetry.addData("Heading: ", Math.toDegrees(curPose.getHeading()));
        telemetry.addData("State: ", autonState);
        telemetry.update();

    }

    @Override
    public void stop() {
    }

    private void buildPath() { // build the robot's path using bezier lines
        driveToFirstBall = boiledPotatoe.pathBuilder() // first path to pick up first ball
                .addPath(new BezierLine(START_POSE, BALL_ONE_POSE))
                .setLinearHeadingInterpolation(START_POSE.getHeading(), BALL_ONE_POSE.getHeading())
                .build();
    }

    private void autonomousPathUpdate() {
        switch (autonState) {
            case PICK_UP_FIRST_BALL: // first step: turn 180 degrees
                boiledPotatoe.followPath(driveToFirstBall); // start going to the first ball
                intakeMotor.setVelocity(INTAKE_VELOCITY);


                break;
        }
    }
}

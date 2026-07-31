package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous
public class BallCollectorAuton extends OpMode {

    private static final double INTAKE_VELOCITY = -2000; // make the speed of intake motor
    private DcMotorEx intakeMotor; // create the motor

    private Follower boiledPotatoe; // make a new follower

    private enum AutonState { // each action
        PICK_UP_FIRST_BALL,
        MOVE_RIGHT,
        PICK_UP_SECOND_BALL,
        PICK_UP_THIRD_BALL,
        DEPOSIT_BALL,
        COMPLETE
    }

    private PathChain driveToFirstBall;
    private PathChain driveToIntermediatePath;
    private PathChain driveToSecondBall;
    private PathChain driveToThirdBall;
    private PathChain driveToBallDeposit;

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
                .addPath(new BezierLine(START_POSE, BALL_ONE_POSE)) // create the bezier line from one point to another
                .setLinearHeadingInterpolation(START_POSE.getHeading(), BALL_ONE_POSE.getHeading()) // make heading linear so we can turn while moving (only works with mecanum)
                .build();
        driveToIntermediatePath = boiledPotatoe.pathBuilder() // set up to pick up second ball
                .addPath(new BezierLine(BALL_ONE_POSE, INTERMEDIATE_POSE))
                .setLinearHeadingInterpolation(BALL_ONE_POSE.getHeading(), INTERMEDIATE_POSE.getHeading())
                .build();
        driveToSecondBall = boiledPotatoe.pathBuilder() // pick up second bal,
                .addPath(new BezierLine(INTERMEDIATE_POSE, BALL_TWO_POSE))
                .setLinearHeadingInterpolation(INTERMEDIATE_POSE.getHeading(), BALL_TWO_POSE.getHeading())
                .build();
        driveToThirdBall = boiledPotatoe.pathBuilder() // pick up third ball
                .addPath(new BezierLine(BALL_TWO_POSE, BALL_THREE_POSE))
                .setLinearHeadingInterpolation(BALL_TWO_POSE.getHeading(), BALL_THREE_POSE.getHeading())
                .build();
        driveToBallDeposit = boiledPotatoe.pathBuilder() // deposit balls
                .addPath(new BezierLine(BALL_THREE_POSE, DEPOSIT_POSE))
                .setLinearHeadingInterpolation(BALL_THREE_POSE.getHeading(), DEPOSIT_POSE.getHeading())
                .build();
    }

    private void autonomousPathUpdate() {
        switch (autonState) {
            case PICK_UP_FIRST_BALL: // go pick up first ball
                boiledPotatoe.followPath(driveToFirstBall); // start going to the first ball
                intakeMotor.setVelocity(INTAKE_VELOCITY);

                if(!boiledPotatoe.isBusy()){ // when we are done
                    intakeMotor.setVelocity(0); // turn off intake motor
                    autonState = AutonState.MOVE_RIGHT;
                    break;
                }

            case MOVE_RIGHT: // set up to pick up second ball
                boiledPotatoe.followPath(driveToIntermediatePath); // start going to the intermediate

                if(!boiledPotatoe.isBusy()){ // when we are done
                    autonState = AutonState.PICK_UP_SECOND_BALL;
                    break;
                }

            case PICK_UP_SECOND_BALL: // pick up second ball
                boiledPotatoe.followPath(driveToSecondBall); // start going to the second ball
                intakeMotor.setVelocity(INTAKE_VELOCITY);

                if(!boiledPotatoe.isBusy()){ // when we are done
                    intakeMotor.setVelocity(0); // turn off intake motor
                    autonState = AutonState.PICK_UP_THIRD_BALL;
                    break;
                }

            case PICK_UP_THIRD_BALL: // first step: turn 180 degrees
                boiledPotatoe.followPath(driveToThirdBall); // start going to the third ball
                intakeMotor.setVelocity(INTAKE_VELOCITY);

                if(!boiledPotatoe.isBusy()){ // when we are done
                    intakeMotor.setVelocity(0); // turn off intake motor
                    autonState = AutonState.DEPOSIT_BALL;
                    break;
                }

            case DEPOSIT_BALL: // deposit the balls
                boiledPotatoe.followPath(driveToBallDeposit); // start going to the deposit bin

                if(!boiledPotatoe.isBusy()){ // when we are done
                    autonState = AutonState.COMPLETE;
                    break;
                }

            case COMPLETE: // when we are done
                intakeMotor.setVelocity(-INTAKE_VELOCITY);  // spin the motor backwards to outake balls
                break;

        }
    }
}

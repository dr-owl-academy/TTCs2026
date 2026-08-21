package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;



@Autonomous(name = "AutoSensor")
public class Sensorpracticestuff extends OpMode {
    private Follower follower;
    private PathChain driveToTarget;
    private PathChain drivetoball1;
    private PathChain drivetoball2;
    private PathChain drivetoball3;
    private PathChain Dropoffballscomplete;
    private PathState pathState = PathState.Drive_to_Ball1;


    private final Pose START_POSE = new Pose(37, 134, Math.toRadians(-90));
    private final Pose BALL1POSE = new Pose(24, 24, Math.toRadians(264));
    private final Pose BALL2POSE = new Pose(121, 48, Math.toRadians(16));
    private final Pose BALL3POSE = new Pose(79, 69, Math.toRadians(158));
    private final Pose TARGET_POSE = new Pose(125, 30, Math.toRadians(315));

    public enum PathState { //manage pathstates
        Drive_to_Ball1,
        Drive_to_Ball2,
        Drive_to_Ball3,
        Drive_to_Target,
        Complete
    }


    @Override
    public void init() {
follower = Constants.createFollower(hardwareMap); //initiate follower object
        follower.setStartingPose(START_POSE); //set initial position
        follower.setMaxPower(1); //setting max speed
        buildPath(); //build the bezier line path down there

    }

    @Override
    public void loop() {

        //Updates the PedroPathing
        follower.update();

        //Updates the autonomous for FSM
        autonomousPathUpdate();

        //Gets the pose for follower (know what coordinate it is)
        Pose currentPose = follower.getPose();

        //telemetry gives me the x and y positions on screen and other info
        telemetry.addData("X", currentPose.getX());

        telemetry.addData("Y", currentPose.getY());
        telemetry.update();


    }

    private void buildPath() {


        drivetoball1 = follower.pathBuilder()
                .addPath(new BezierLine(START_POSE, BALL1POSE))
                        .setLinearHeadingInterpolation(START_POSE.getHeading(), BALL1POSE.getHeading())
                                .build();
        drivetoball2 = follower.pathBuilder()
                .addPath(new BezierLine(BALL1POSE, BALL2POSE))
                .setLinearHeadingInterpolation(BALL1POSE.getHeading(), BALL2POSE.getHeading())
                .build();
        drivetoball3 = follower.pathBuilder()
                .addPath(new BezierLine(BALL2POSE, BALL3POSE))
                .setLinearHeadingInterpolation(BALL2POSE.getHeading(), BALL3POSE.getHeading())
                .build();
        driveToTarget = follower.pathBuilder()
                .addPath(new BezierLine(BALL3POSE, TARGET_POSE))
                .setLinearHeadingInterpolation(BALL3POSE.getHeading(), TARGET_POSE.getHeading())
                .build();

    }

    private void autonomousPathUpdate() {
        switch (pathState) {

            case Drive_to_Ball1:
                if (!follower.isBusy()) {
                    follower.followPath(drivetoball1);
                    pathState = PathState.Drive_to_Ball2;
                }
                break;
            case Drive_to_Ball2:
                if (!follower.isBusy()) {
                    follower.followPath(drivetoball2);
                    pathState = PathState.Drive_to_Ball3;
                }
                break;
            case Drive_to_Ball3:
                if (!follower.isBusy()) {
                    follower.followPath(drivetoball3);
                    pathState = PathState.Drive_to_Target;
                }
            case Drive_to_Target:
                if (!follower.isBusy()) {
                    follower.followPath(driveToTarget);
                    pathState = PathState.Complete;
                }
                break;
            case Complete:
                break;
        }
    }
}
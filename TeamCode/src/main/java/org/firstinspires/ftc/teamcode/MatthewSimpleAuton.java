package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


public class MatthewSimpleAuton extends OpMode{

    private enum Autostate{
        START_TURN_TO_180,
        WAIT_FOR_TURN_TO_180,
        START_DRIVE_TO_TARGET,
        WAIT_FOR_DRIVE_TO_TARGET,
        START_DRIVE_2,
        WAIT_FOR_DRIVE_2,
        COMPLETE,

    }
    private Follower follower;

    //use PathChain for each path segment
    private PathChain driveToTarget;

    // Starting FSM state.
    private Autostate autoState = Autostate.START_TURN_TO_180;

    private static final Pose START_POSE = new Pose(72,72,Math.toRadians(90) );
    private static final Pose DRIVE_START_POSE = new Pose(72,72,Math.toRadians(180));
    private static final Pose TARGET_POSE = new Pose(24,72, Math.toRadians(180));
    private static final Pose TARGET_POSE2 = new Pose(24,72, Math.toRadians(-60));
    private static final Pose INTAKE = new Pose();

    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(START_POSE);

        follower.setMaxPower(0.5);

        buildPath();

        telemetry.addLine("Autonomous ready");
        telemetry.update();

    }
    @Override
    public void loop() {

        // Pedro must update every loop.
        follower.update();

        // Update the autonomous FSM.
        follower.update();

        Pose currentPose = follower.getPose();

        telemetry.addData("X", currentPose.getX() );

        telemetry.addData("Y", currentPose.getY() );

        telemetry.addData("Heading", Math.toDegrees(currentPose.getHeading()));

        telemetry.addData("State", autoState );

        if (autoState == autoState.COMPLETE) {

            telemetry.addLine("Autonomous complete" );
        }

        telemetry.update();
    }
    @Override
    public void stop() {
    }

    // Builds all PathChains used by this autonomous.
    private void buildPath() {

        driveToTarget = follower.pathBuilder()
                .addPath(new BezierLine(DRIVE_START_POSE,TARGET_POSE))
                .setConstantHeadingInterpolation( Math.toRadians(180))
                .build();
    }

    // Updates the autonomous finite state machine.

    public void autonomousPathUpdate(){
        switch (autoState) {

            case START_TURN_TO_180:
                // Turn in place from 90 degrees to 0 degrees.
                follower.turnTo(Math.toRadians(180));
                autoState = autoState.WAIT_FOR_TURN_TO_180;
                break;

            case WAIT_FOR_TURN_TO_180:
                // Wait for the turn to finish.
                if (!follower.isBusy()) {
                    autoState = autoState.START_DRIVE_TO_TARGET;
                }
                break;

            case START_DRIVE_TO_TARGET:
                /* Start driving to the target.
                 * true tells Pedro to hold the final pose.
                 */
                follower.followPath(driveToTarget, true);
                autoState = autoState.WAIT_FOR_DRIVE_TO_TARGET;
                break;

            case WAIT_FOR_DRIVE_TO_TARGET:
                // Wait for the driving path to finish.
                if (!follower.isBusy()) {
                    autoState = autoState.COMPLETE;
                }
                break;

            case COMPLETE:
                break;
        }
    }
}


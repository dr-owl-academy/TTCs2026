package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Coach Simple Auton")
public class CoachSimpleAuton extends OpMode {

    // Finite state machine states.
    private enum AutoState {
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
    private AutoState autoState = AutoState.START_TURN_TO_180;

    private static final Pose START_POSE = new Pose(72,72,Math.toRadians(90) );
    private static final Pose DRIVE_START_POSE = new Pose(72,72,Math.toRadians(180));
    private static final Pose TARGET_POSE = new Pose(24,72, Math.toRadians(180));

    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(START_POSE);

        // Reduced power for initial testing.
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
        autonomousPathUpdate();

        Pose currentPose = follower.getPose();

        telemetry.addData("X", currentPose.getX() );

        telemetry.addData("Y", currentPose.getY() );

        telemetry.addData("Heading", Math.toDegrees(currentPose.getHeading()));

        telemetry.addData("State", autoState );

        if (autoState == AutoState.COMPLETE) {

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
    private void autonomousPathUpdate() {

        switch (autoState) {

            case START_TURN_TO_180:
                // Turn in place from 90 degrees to 0 degrees.
                follower.turnTo( Math.toRadians(180));
                autoState = AutoState.WAIT_FOR_TURN_TO_180;
                break;

            case WAIT_FOR_TURN_TO_180:
                // Wait for the turn to finish.
                if (!follower.isBusy()) {
                    autoState = AutoState.START_DRIVE_TO_TARGET;
                }
                break;

            case START_DRIVE_TO_TARGET:
                /* Start driving to the target.
                 * true tells Pedro to hold the final pose.
                 */
                follower.followPath(driveToTarget,true);
                autoState = AutoState.WAIT_FOR_DRIVE_TO_TARGET;
                break;

            case WAIT_FOR_DRIVE_TO_TARGET:
                // Wait for the driving path to finish.
                if (!follower.isBusy()) {
                    autoState = AutoState.COMPLETE;
                }
                break;

            case COMPLETE:
                break;
        }
    }
}
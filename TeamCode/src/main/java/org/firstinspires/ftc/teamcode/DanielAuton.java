package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "DanielSimpleAuton")
public class DanielAuton extends OpMode {

    // Finite state machine states.
    private enum AutoState {
        PATH1,
        PATH2,
        PATH3,
        PATH4,
        COMPLETE
    }

    private Follower follower;

    //use PathChain for each path segment
    private PathChain driveToTarget;

    // Starting FSM state.
    private AutoState autoState = AutoState.PATH1;
// Movements for FSM
    private static final Pose START_POSE = new Pose(104,132,Math.toRadians(-90) );
    private static final Pose DRIVE_START_POSE = new Pose(72,72,Math.toRadians(-90));
    private static final Pose TARGET_POSE = new Pose(129.587,130.006, Math.toRadians(103));
    private static final Pose FOURTH_POSE = new Pose(129.587,130.006, Math.toRadians(103));
    private static final Pose FIFTH_POSE = new Pose(129.587,130.006, Math.toRadians(103));
    private static final Pose SIXTH_POSE = new Pose(129.587,130.006, Math.toRadians(103));
    private static final Pose SEVENTH_POSE = new Pose(129.587,130.006, Math.toRadians(103));
    private static final Pose EIGHTH_POSE = new Pose(129.587,130.006, Math.toRadians(103));
    private static final Pose NINTH_POSE = new Pose(129.587,130.006, Math.toRadians(103));
    private static final Pose TENTH_POSE = new Pose(129.587,130.006, Math.toRadians(103));

    @Override
    public void init() {
// Hardware Map appears in init
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

        telemetry.update(); //all the telemetry shown on the driver hub
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

            case PATH1:
                // Turn in place from 90 degrees to 0 degrees.
                follower.turnTo( Math.toRadians(180));
                autoState = AutoState.PATH2;
                break;

            case PATH2:
                // Wait for the turn to finish.
                if (!follower.isBusy()) {
                    autoState = AutoState.PATH3;
                }
                break;

            case PATH3:
                /* Start driving to the target.
                 * true tells Pedro to hold the final pose.
                 */
                follower.followPath(driveToTarget,true);
                autoState = AutoState.PATH4;
                break;

            case PATH4:
                // Wait for the driving path to finish.
                if (!follower.isBusy()) {
                    autoState = AutoState.COMPLETE;
                }
                break;

            case COMPLETE:
                //Path is now complete
                break;
        }
    }
}
/*package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous
public class Daniel_YellowBall_Limelight extends OpMode {

    private Follower follower;
    private Limelight3A limelight;

    private final Pose startPose = new Pose(72, 72, Math.PI / 2);
    //limelight pipeline used to tune green ball
    private static final int YELLOW_BALL_PIPELINE = 2;

    // CAMERA GEOMETRY
    //relative height camera to ball center
    private static final double HEIGHT_DIFFERENCE = 9.5;   // 3.6 inches

    private static final double CAMERA_DOWN_ANGLE = 45.0;

    private static final double STOP_DISTANCE = 4.5;


    // MOVEMENT SETTINGS

    private static final double SEARCH_TURN_POWER = 0.20;

    private static final double FAST_FORWARD = 0.25;

    private static final double SLOW_FORWARD = 0.12;

    private static final double TURN_KP = 0.015;

    private static final double MAX_TURN_POWER = 0.20;
    private double maxTargetAreaFound = 0;   // Peak 'ta' spotted during sweep
    private double bestHeadingHeading = 0;   // Saved heading where peak 'ta' occurred
    private double lastHeading = 0;         // Tracks robot heading from previous loop frame
    private double totalRotatedAngle = 0;   // Accumulates total radians turned

    // LIMELIGHT DATA

    private boolean targetDetected = false;

    private double tx = 0;
    private double ty = 0;
    private double ta = 0;

    private double horizontalDistance = Double.POSITIVE_INFINITY;


    // STATE MACHINE

    private enum State {
        SEARCH,
        APPROACH,
        STOP,
        START_SEARCH_360,     // <--- ADD THIS
        SCANNING_360,         // <--- ADD THIS
        ALIGN_TO_BEST_TARGET, // <--- ADD THIS

    }

    private State state = State.SEARCH;


    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(startPose);


        limelight = hardwareMap.get( Limelight3A.class,"limelight" );

        limelight.pipelineSwitch(YELLOW_BALL_PIPELINE);


        telemetry.setMsTransmissionInterval(20);

        telemetry.addData("Status", "Initialized");

        telemetry.addData("Start Pose", startPose );
    }


    @Override
    public void start() {

        limelight.start();

        follower.startTeleopDrive();

        state = State.SEARCH;
        state = State.START_SEARCH_360;
    }


    @Override
    public void loop() {

        // Read latest camera data
        updateLimelight();

        // Run autonomous state machine
        updateStateMachine(); {
            switch (state) {
                // Setup state - resets tracking variables before starting the spin
                case START_SEARCH_360:
                    maxTargetAreaFound = 0;
                    totalRotatedAngle = 0;
                    lastHeading = follower.getPose().getHeading();
                    state = State.SCANNING_360;
                    break;

                case SCANNING_360:
                    follower.setTeleOpDrive(0, 0, SEARCH_TURN_POWER, true);

                    double currentHeading = follower.getPose().getHeading();

                    // 1. Calculate frame-to-frame delta
                    double deltaHeading = currentHeading - lastHeading;

                    // 2. Wrap angle boundary [-PI, PI]
                    deltaHeading = Math.atan2(Math.sin(deltaHeading), Math.cos(deltaHeading));

                    // 3. Accumulate absolute rotation amount
                    totalRotatedAngle += Math.abs(deltaHeading);
                    lastHeading = currentHeading;

                    // 4. Save peak target area & heading
                    if (targetDetected && ta > maxTargetAreaFound) {
                        maxTargetAreaFound = ta;
                        bestHeadingHeading = currentHeading;
                    }

                    // 5. Complete sweep at 2*PI radians (360 degrees)
                    if (totalRotatedAngle >= (2.0 * Math.PI)) {
                        follower.setTeleOpDrive(0, 0, 0, true);

                        if (maxTargetAreaFound > 0) {
                            state = State.ALIGN_TO_BEST_TARGET;
                        } else {
                            state = State.START_SEARCH_360; // Retry if no target seen
                        }
                    }
                    break;
            }
        }

        // Pedro updates motors + localization
        follower.update();

        // Driver Station telemetry
        telemetry.addData(
                "State",
                state
        );

        telemetry.addData("Ball Detected",targetDetected );


        if (targetDetected) {
            telemetry.addData("tx",  tx  );

            telemetry.addData("ty", ty  );

            telemetry.addData("Horizontal Distance", horizontalDistance );

        }
        else {

            telemetry.addData("Horizontal Distance","No target" );
        }


        telemetry.addData("Pose", follower.getPose());


        telemetry.update();
    }


    // =====================================================
    // LIMELIGHT
    // =====================================================

    private void updateLimelight() {

        LLResult result = limelight.getLatestResult();


        targetDetected = false;

        tx = 0;
        ty = 0;
        ta = 0;

        horizontalDistance = Double.POSITIVE_INFINITY;


        if (result != null && result.isValid() && !result.getColorResults().isEmpty()) {

            targetDetected = true;

            tx = result.getTx();

            ty = result.getTy();

            horizontalDistance = calculateHorizontalDistance(ty);
        }
    }


    // STATE MACHINE

    private void updateStateMachine() {

        switch (state) {


            // =============================================
            // SEARCH FOR BALL
            // =============================================

            case SEARCH:


                if (targetDetected) {

                    // Stop spinning
                    follower.setTeleOpDrive(0,0,0,true );

                    state = State.APPROACH;

                } else {

                    // Spin counterclockwise
                    follower.setTeleOpDrive(0,0, SEARCH_TURN_POWER,true );
                }

                break;

            // DRIVE TOWARD BALL
            case APPROACH:

                // Ball disappeared
                if (!targetDetected) {

                    state = State.SEARCH;

                    follower.setTeleOpDrive(0,0, SEARCH_TURN_POWER,true);

                    break;
                }


                // Stop 6 inches away horizontally
                if (horizontalDistance <= STOP_DISTANCE) {

                    follower.setTeleOpDrive(0,0,0,true );

                    state = State.STOP;

                    break;
                }


                // -------------------------
                // STEERING
                // -------------------------

                double turnPower = TURN_KP * tx;


                turnPower = Math.max(-MAX_TURN_POWER, Math.min( MAX_TURN_POWER, turnPower ) );


                // -------------------------
                // FORWARD SPEED
                // -------------------------

                double forwardPower;


                if (horizontalDistance > 12.0) {

                    forwardPower =  FAST_FORWARD;

                } else {

                    forwardPower =  SLOW_FORWARD;
                }


                follower.setTeleOpDrive(
                        forwardPower,
                        0,
                        turnPower,
                        true
                );

                break;


            // =============================================
            // STOP
            // =============================================

            case STOP:

                follower.setTeleOpDrive(0,0,0,true );

                break;
        }
    }


    // =====================================================
    // DISTANCE CALCULATION
    // =====================================================

    private double calculateHorizontalDistance( double tyDegrees) {

        double downwardAngle = CAMERA_DOWN_ANGLE - tyDegrees;

        if (downwardAngle <= 0 || downwardAngle >= 90) {

            return Double.POSITIVE_INFINITY;
        }


        return HEIGHT_DIFFERENCE / Math.tan( Math.toRadians( downwardAngle ));
    }

    @Override
    public void stop() {

        follower.setTeleOpDrive( 0, 0,0,true );

        follower.update();

        limelight.stop();
    }
}*/

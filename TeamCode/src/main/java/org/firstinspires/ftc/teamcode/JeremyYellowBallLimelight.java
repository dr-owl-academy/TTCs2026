package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous
public class JeremyYellowBallLimelight extends OpMode {

    private Follower follower;
    private Limelight3A limelight;

    private final Pose startPose = new Pose(72, 72, Math.PI / 2);
    //limelight pipeline used to tune green ball
    private static final int YELLOW_BALL_PIPELINE = 2;

    // CAMERA GEOMETRY: relative height camera to ball center
    private static final double HEIGHT_DIFFERENCE = 5.0;   // 3.6 inches

    private static final double CAMERA_DOWN_ANGLE = 45.0;

    private static final double STOP_DISTANCE = 6.0;


    // MOVEMENT SETTINGS

    private static final double SEARCH_TURN_POWER = 0.20;

    private static final double FAST_FORWARD = 0.25;

    private static final double SLOW_FORWARD = 0.12;

    private static final double TURN_KP = 0.015; // turn magnitude

    private static final double MAX_TURN_POWER = 0.20;

    // LIMELIGHT DATA

    private boolean targetDetected = false;

    private double tx = 0;
    private double ty = 0;

    private double horizontalDistance = Double.POSITIVE_INFINITY; // set the distance to a large number to reduce later


    // STATE MACHINE
    private enum State {
        SEARCH,
        APPROACH,
        STOP
    }

    private State state = State.SEARCH;


    @Override
    public void init() {

        // set up the follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);


        // define the limelight and the pipeline
        limelight = hardwareMap.get( Limelight3A.class,"limelight" );
        limelight.pipelineSwitch(YELLOW_BALL_PIPELINE);

        // telemetry setup
        telemetry.setMsTransmissionInterval(20);
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Start Pose", startPose );
    }

    @Override
    public void start() {

        limelight.start();

        follower.startTeleopDrive();

        state = State.SEARCH;
    }


    @Override
    public void loop() {
        updateLimelight();  // Read latest camera data

        updateStateMachine(); // Run autonomous state machine

        follower.update(); // Pedro updates motors + localization

        telemetry.addData( // Driver Station telemetry
                "State",
                state
        );

        telemetry.addData("Ball Detected",targetDetected );


        if (targetDetected) { // if we found the target then find the distance
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

        LLResult result = limelight.getLatestResult(); // read the limelight

        targetDetected = false;

        // reset distance variables
        tx = 0;
        ty = 0;

        horizontalDistance = Double.POSITIVE_INFINITY;


        if (result != null && result.isValid() && !result.getColorResults().isEmpty()) { // if we found the ball and the color matches
            targetDetected = true;

            // get parameters
            tx = result.getTx();
            ty = result.getTy();

            horizontalDistance = calculateHorizontalDistance(ty); // calculate distance
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
                    follower.setTeleOpDrive(0,0,0,true);

                    state = State.APPROACH;

                } else {

                    // Spin counterclockwise
                    follower.setTeleOpDrive(0,0, SEARCH_TURN_POWER,true);
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
}


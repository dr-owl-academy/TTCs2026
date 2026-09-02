package org.firstinspires.ftc.teamcode;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import java.util.List;
import java.util.ArrayList;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
// wsp
@Autonomous
public class river_yellowball_limelight extends OpMode {

    private Follower follower;
    private Limelight3A limelight;
    private List<Double> fieldAngles = new ArrayList<>();
    private double lastSampleHeading = 0;
    private double scanStartHeading = 0;
    private final Pose startPose = new Pose(72, 72, Math.PI / 2);
    //limelight pipeline used to tune green ball
    private static final int YELLOW_BALL_PIPELINE = 9;
    private double totalTurned = 0;
    private double previousHeading = 0;

    // CAMERA GEOMETRY
    //relative height camera to ball center
    private static final double HEIGHT_DIFFERENCE = 3.6;   // 3.6 inches

    private static final double CAMERA_DOWN_ANGLE = 45.0;

    private static final double STOP_DISTANCE = 6.0;


    // MOVEMENT SETTINGS

    private static final double SEARCH_TURN_POWER = 0.20;

    private static final double FAST_FORWARD = 0.25;

    private static final double SLOW_FORWARD = 0.12;

    private static final double TURN_KP = 0.015;

    private static final double MAX_TURN_POWER = 0.20;

    // LIMELIGHT DATA

    private boolean targetDetected = false;

    private double tx = 0;
    private double ty = 0;
    private double ta = 0;
    private double bestArea = 0;
    private double bestFieldAngle = 0;
    private double targetFieldAngle = 0;

    private double horizontalDistance = Double.POSITIVE_INFINITY;


    // STATE MACHINE

    private enum State {
        SEARCH,
        TURN_TO_TARGET,
        APPROACH,
        STOP
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
    }


    @Override
    public void loop() {

        // Read latest camera data
        updateLimelight();

        // Run autonomous state machine
        updateStateMachine();

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

        horizontalDistance = Double.POSITIVE_INFINITY;


        if (result != null && result.isValid() && !result.getColorResults().isEmpty()) {

            targetDetected = true;

            tx = result.getTx();

            ty = result.getTy();

            ta = result.getTa();

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
                double heading = Math.toDegrees(follower.getPose().getHeading());

                if (fieldAngles.isEmpty() && lastSampleHeading == 0) {
                    previousHeading = heading;
                    totalTurned = 0;
                    bestArea = 0;
                    bestFieldAngle = 0;
                }
                double delta = heading - previousHeading;

                if (delta > 180) delta -= 360;
                if (delta < -180) delta += 360;

                totalTurned += Math.abs(delta);
                previousHeading = heading;

                if (Math.abs(heading - lastSampleHeading) >= 10) {

                    if (targetDetected) {
                        double ballLocation = heading + tx;
                        fieldAngles.add(ballLocation);

                        if (ta > bestArea) {
                            bestArea = ta;
                            bestFieldAngle = ballLocation;
                        }
                    }

                    lastSampleHeading = heading;
                }

                if (totalTurned < 360) {

                        follower.setTeleOpDrive(0, 0, SEARCH_TURN_POWER, true);

                } else {

                    follower.setTeleOpDrive(0, 0, 0, true);

                    if (bestArea > 0) {

                        targetFieldAngle = bestFieldAngle;
                        state = State.TURN_TO_TARGET;

                    } else {

                            state = State.STOP;
                    }
                }
                break;

            case TURN_TO_TARGET:

                double currentHeading = Math.toDegrees((follower.getPose().getHeading()));
                double error = targetFieldAngle - currentHeading;

                while (error > 180) error -= 360;
                while (error < -180) error += 360;

                if (Math.abs(error) < 3) {
                    follower.setTeleOpDrive(0,0,0, true);
                    state = State.APPROACH;
                } else {
                    double turn = Math.max(-MAX_TURN_POWER, Math.min(MAX_TURN_POWER, TURN_KP * error * 10));
                    follower.setTeleOpDrive(0, 0, turn, true);
                }

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

package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous
public class yajat_yellow_ball_test extends OpMode {

    private Follower follower;
    private Limelight3A limelight;

    private final Pose startPose = new Pose(72, 72, Math.PI / 2);
    //limelight pipeline used to tune green ball
    private static final int YELLOW_BALL_PIPELINE = 9;

    // CAMERA GEOMETRY
    //relative height camera to ball center
    private static final double HEIGHT_DIFFERENCE = 9.5;   // 9.5 inches

    private static final double CAMERA_DOWN_ANGLE = 45.0;

    private static final double STOP_DISTANCE = 4.5;


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

    private double horizontalDistance = Double.POSITIVE_INFINITY;

  //best cluster
    private double BestArea = 0;
    private double BestDistance = Double.POSITIVE_INFINITY;
    private Pose BestPose = null;

    //cluster location
    private double clusterX = 0;
    private double clusterY = 0;

    //360 scan
    private double lastHeading = 0;
    private double accumalatedRotation = 0;


    // STATE MACHINE

    private enum State {
        SPIN,
        TURN_TO_TARGET,
        APPROACH,
        STOP
    }

    private State state = State.SPIN;


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

        state = State.SPIN;
        lastHeading = follower.getPose().getHeading();
        accumalatedRotation = 0;
        BestArea = 0;
        BestDistance = Double.POSITIVE_INFINITY;
        BestPose = null;
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

            horizontalDistance = calculateHorizontalDistance(ty);
        }
    }


    // STATE MACHINE

    private void updateStateMachine() {

        switch (state) {


            // =============================================
            // SEARCH FOR BALL
            // =============================================

            case SPIN:
                //how much has the bot rotated
               double currentHeading = follower.getPose().getHeading();

               double delta = currentHeading - lastHeading;
               //deal with the rotation wrapping
                while (delta > Math.PI){
                    delta-= 2 * Math.PI;
                }
                while (delta < Math.PI){
                    delta+= 2 * Math.PI;
                }
                accumalatedRotation += Math.abs(delta);
                lastHeading = currentHeading;

                //look for clusters
                LLResult result = limelight.getLatestResult();
                if(result !=null && result.isValid() && !result.getColorResults().isEmpty()) {
                    for (LLResultTypes.ColorResult color : result.getColorResults() ) {
                        double area = color.getTargetArea();
                        double distance =
                                calculateHorizontalDistance(color.getTargetYDegrees());
                            //is this the most grande cluster?
                        if(area > BestArea && distance != Double.POSITIVE_INFINITY) {
                            BestArea = area;
                            BestDistance = distance;
                            //save bot position when we saw it
                            BestPose = follower.getPose();
                        }
                    }
                }
                // KEEP SPINNING
                follower.setTeleOpDrive(
                        0,
                        0,
                        SEARCH_TURN_POWER,
                        true
                );

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

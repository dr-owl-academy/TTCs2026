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
    private double BestTx = 0;
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
        DriveToCluster,
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
        BestTx = 0;
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

            case SPIN: {
                //how much has the bot rotated
                double currentHeading = follower.getPose().getHeading();

                double delta = currentHeading - lastHeading;
                //deal with the rotation wrapping
                while (delta > Math.PI) {
                    delta -= 2 * Math.PI;
                }
                while (delta < -Math.PI) {
                    delta += 2 * Math.PI;
                }
                accumalatedRotation += Math.abs(delta);
                lastHeading = currentHeading;

                //look for clusters
                LLResult result = limelight.getLatestResult();
                if (result != null && result.isValid() && !result.getColorResults().isEmpty()) {
                    for (LLResultTypes.ColorResult color : result.getColorResults()) {
                        double area = color.getTargetArea();
                        double distance =
                                calculateHorizontalDistance(color.getTargetYDegrees());
                        //is this the most grande cluster?
                        if (area > BestArea && distance != Double.POSITIVE_INFINITY) {
                            BestArea = area;
                            BestDistance = distance;
                            BestTx = color.getTargetXDegrees();
                            //save bot position when we saw it
                            BestPose = new Pose(
                                    follower.getPose().getX(),
                                    follower.getPose().getY(),
                                    follower.getPose().getHeading()
                            );
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
                //have WE completed a full rotation
                if (accumalatedRotation >= 2 * Math.PI) {

                    follower.setTeleOpDrive(
                            0,
                            0,
                            0,
                            true
                    );

                    if (BestPose != null) {
                        calculateClusterPosition();

                        state = State.TURN_TO_TARGET;
                    }
                }
                break;
            }

                //turn to the ball
            case TURN_TO_TARGET: {
                Pose currentpose = follower.getPose();

                double dx = clusterX - currentpose.getX();
                double dy = clusterY - currentpose.getY();

                double targetHeading = Math.atan2(dy, dx);

                double headingError = targetHeading - currentpose.getHeading();

                while (headingError > Math.PI)
                    headingError -= 2 * Math.PI;

                while (headingError < -Math.PI)
                    headingError += 2 * Math.PI;

                double turnPower =
                        TURN_KP * Math.toDegrees(headingError);
                turnPower = Math.max(
                        -MAX_TURN_POWER, Math.min(MAX_TURN_POWER, turnPower)
                );

                if (Math.abs(Math.toDegrees(headingError)) < 2.0) {
                    follower.setTeleOpDrive(0, 0, 0, true);
                    state = State.DriveToCluster;
                    break;
                }

                follower.setTeleOpDrive(
                        0,
                        0,
                        turnPower,
                        true

                );

                break;
            }

            // DRIVE TOWARD BALL
            case DriveToCluster:
                Pose currentpose = follower.getPose();
                //difference between bot and target
                double dx = clusterX - currentpose.getX();
                double dy = clusterY - currentpose.getY();
                double distanceToCluster = Math.hypot(dx, dy);

                //is the bot close enough yet
                if(distanceToCluster <= STOP_DISTANCE) {
                    follower.setTeleOpDrive(
                            0,
                            0,
                            0,
                            true
                    );

                    state= State.STOP;
                    break;
                }


                //convert the field coords to robot coords
                double heading = currentpose.getHeading();
                double forward =
                        dx * Math.cos(heading)
                        + dy * Math.sin(heading);
                double strafe =
                        -dx * Math.sin(heading)
                        + dy * Math.cos(heading);

                //normalize movement
                double magnitude = Math.hypot(forward,strafe);
                if(magnitude > 0){
                    forward /= magnitude;
                    strafe /= magnitude;
                }
                //robot go vroom now
                double drivePower;
                if(distanceToCluster > 12.0) {
                    drivePower = FAST_FORWARD;
                } else {
                    drivePower = SLOW_FORWARD;
                }

                follower.setTeleOpDrive(
                        forward * drivePower,
                        strafe * drivePower,
                        0,
                        true
                );

                break;





            // =============================================
            // STOP
            // =============================================

            case STOP:

                follower.setTeleOpDrive(
                        0,
                        0,
                        0,
                        true
                );

                break;
        }
    }

    private void calculateClusterPosition() {
        double robotX = BestPose.getX();
        double roboty = BestPose.getY();
        double robotHeading = BestPose.getHeading();

        double targetbearing =
                robotHeading + Math.toRadians(BestTx);

        clusterX =
                robotX
                + BestDistance * Math.cos(targetbearing);
        clusterY =
                roboty
                + BestDistance * Math.sin(targetbearing);
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

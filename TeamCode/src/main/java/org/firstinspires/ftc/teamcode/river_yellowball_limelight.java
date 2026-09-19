package org.firstinspires.ftc.teamcode;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//
@Autonomous
public class river_yellowball_limelight extends OpMode {

    private Follower follower;
    private Limelight3A limelight;
    private double lastSampleHeading = 0;
    private long turnStartTimeNs = 0;
    private final Pose startPose = new Pose(72, 72, Math.PI / 2);
    //limelight pipeline used to tune green ball
    private static final int YELLOW_BALL_PIPELINE = 9;
    private double totalTurned = 0;
    private double previousHeading = 0;
    private int missedFrames = 0;
    private static final int MISSED_FRAMES_LIMIT = 40;

    // CAMERA GEOMETRY
    //relative height camera to ball center
    private static final double HEIGHT_DIFFERENCE = 3.6;   // 3.6 inches

    private static final double CAMERA_DOWN_ANGLE = 45.0;

    private static final double STOP_DISTANCE = 2.0;

    private static final double CAMERA_HEADING_OFFSET = -42;

    // MOVEMENT SETTINGS

    private static final double SEARCH_TURN_POWER = 0.7;

    private static final double FAST_FORWARD = 0.25;

    private static final double SLOW_FORWARD = 0.16;

    private static final double TURN_KP = 0.015;

    private static final double MAX_TURN_POWER = 0.20;
    private static final double MIN_TURN_POWER = 0.08;

    // LIMELIGHT DATA

    private boolean targetDetected = false;

    private double tx = 0;
    private double ty = 0;
    private double ta = 0;
    private double bestArea = 0;

    private double horizontalDistance = Double.POSITIVE_INFINITY;

    // BEST BALL OBSERVATION (captured during SEARCH)
    private double bestDistance = Double.POSITIVE_INFINITY;
    private double bestTx = 0;
    private Pose bestPose = null;

    // CALCULATED FIELD TARGET
    private double targetX = 0;
    private double targetY = 0;


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
            telemetry.addData("ta", ta);

            telemetry.addData("Horizontal Distance", horizontalDistance );

        }
        else {

            telemetry.addData("Horizontal Distance","No target" );
        }


        telemetry.addData("Pose", follower.getPose());

        telemetry.addData("Total Turned", totalTurned);
        telemetry.addData("Best Area", bestArea);


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

                if (totalTurned == 0 && lastSampleHeading == 0) {
                    previousHeading = heading;
                    bestArea = 0;
                    bestPose = null;
                }
                double delta = heading - previousHeading;

                if (delta > 180) delta -= 360;
                if (delta < -180) delta += 360;

                totalTurned += Math.abs(delta);
                previousHeading = heading;

                if (Math.abs(heading - lastSampleHeading) >= 1) {

                    if (targetDetected) {

                        LLResult scanResult = limelight.getLatestResult();

                        if (scanResult != null && scanResult.isValid()) {

                            for (LLResultTypes.ColorResult color : scanResult.getColorResults()) {

                                double candidateArea = color.getTargetArea();
                                double candidateDistance = calculateHorizontalDistance(color.getTargetYDegrees());

                                if (candidateArea > bestArea && candidateDistance != Double.POSITIVE_INFINITY) {
                                    bestArea = candidateArea;
                                    bestTx = color.getTargetXDegrees();
                                    bestDistance = candidateDistance;
                                    bestPose = new Pose(
                                            follower.getPose().getX(),
                                            follower.getPose().getY(),
                                            follower.getPose().getHeading()
                                    );
                                }

                            };
                        }
                    }

                    lastSampleHeading = heading;
                }

                if (totalTurned < 360) {

                    follower.setTeleOpDrive(0, 0, SEARCH_TURN_POWER, true);

                } else {

                    follower.setTeleOpDrive(0, 0, 0, true);

                    if (bestArea > 0 && bestPose != null) {

                        calculateTargetPosition();
                        turnStartTimeNs = System.nanoTime();
                        state = State.TURN_TO_TARGET;

                    } else {

                        state = State.STOP;
                    }
                }
                break;

            case TURN_TO_TARGET: {

                Pose currentPose = follower.getPose();
                double dx = targetX - currentPose.getX();
                double dy = targetY - currentPose.getY();

                double targetHeadingRad = Math.atan2(dy, dx);
                double headingErrorRad = targetHeadingRad - currentPose.getHeading();

                while (headingErrorRad > Math.PI) headingErrorRad -= 2 * Math.PI;
                while (headingErrorRad < -Math.PI) headingErrorRad += 2 * Math.PI;

                double error = Math.toDegrees(headingErrorRad);
                double turnElapsedSec = (System.nanoTime() - turnStartTimeNs) / 1e9;
                telemetry.addData("TT Error", error);

                if (Math.abs(error) < 3) {
                    follower.setTeleOpDrive(0,0,0, true);
                    state = State.APPROACH;
                    break;
                }

                if (turnElapsedSec > 4.0) {
                    follower.setTeleOpDrive(0,0,0, true);
                    state = State.SEARCH;
                    totalTurned = 0;
                    bestArea = 0;
                    bestPose = null;
                    lastSampleHeading = 0;
                    break;
                }

                double turn;
                if (Math.abs(error) < 1) {
                    turn = 0;
                } else {
                    double rawTurn = TURN_KP * error * 4;
                    if (Math.abs(rawTurn) < MIN_TURN_POWER) {
                        turn = Math.copySign(MIN_TURN_POWER, error);
                    } else {
                        turn = Math.max(-MAX_TURN_POWER, Math.min(MAX_TURN_POWER, rawTurn));
                    }
                }

                follower.setTeleOpDrive(0,0,turn, true);

                break;
            }

            // DRIVE TOWARD BALL
            case APPROACH: {

                if (!targetDetected) {
                    missedFrames++;

                    if (missedFrames > 10) {
                        follower.setTeleOpDrive(0, 0, 0, true);
                        state = State.SEARCH;
                        totalTurned = 0;
                        bestArea = 0;
                        bestPose = null;
                        lastSampleHeading = 0;
                        missedFrames = 0;
                        break;
                    }
                } else {
                    missedFrames = 0;
                }

                if (horizontalDistance <= STOP_DISTANCE) {
                    follower.setTeleOpDrive(0, 0, 0, true);
                    state = State.STOP;
                    break;
                }

                double forwardPower =
                        (horizontalDistance > 12) ? FAST_FORWARD : SLOW_FORWARD;

                double turn = -TURN_KP * tx * 4;

                turn = Math.max(-MAX_TURN_POWER,
                        Math.min(MAX_TURN_POWER, turn));

                follower.setTeleOpDrive(
                        forwardPower, 0, turn, true);

                break;
            }


            // =============================================
            // STOP
            // =============================================

            case STOP:

                follower.setTeleOpDrive(0,0,0,true );

                break;
        }
    }

    // =====================================================
    // TARGET POSITION CALCULATION
    // =====================================================

    private void calculateTargetPosition() {
        double robotHeadingDeg = Math.toDegrees(bestPose.getHeading());
        double bearingDeg = robotHeadingDeg + bestTx + CAMERA_HEADING_OFFSET;
        double bearingRad = Math.toRadians(bearingDeg);

        targetX = bestPose.getX() + bestDistance * Math.cos(bearingRad);
        targetY = bestPose.getY() + bestDistance * Math.sin(bearingRad);
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

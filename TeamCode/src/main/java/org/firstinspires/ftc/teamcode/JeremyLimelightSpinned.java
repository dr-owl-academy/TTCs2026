package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class JeremyLimelightSpinned extends OpMode {

    private Follower follower;
    private Limelight3A limelight;

    private final Pose startPose = new Pose(72, 72, Math.PI / 2);
    private static final int YELLOW_BALL_PIPELINE = 9;

    // CAMERA GEOMETRY
    private static final double HEIGHT_DIFFERENCE = 9.5;
    private static final double CAMERA_DOWN_ANGLE = 45.0;
    private static final double STOP_DISTANCE = 4.5;

    // MOVEMENT SETTINGS
    private static final double SEARCH_TURN_POWER = 0.20;
    private static final double FAST_FORWARD = 0.25;
    private static final double SLOW_FORWARD = 0.12;
    private static final double TURN_KP = 0.015;       // For Limelight tx (degrees)
    private static final double HEADING_KP = 0.8;      // For Pose heading (radians)
    private static final double MAX_TURN_POWER = 0.20;
    private static final double MIN_TURN_POWER = 0.05;

    // LIMELIGHT DATA
    private boolean targetDetected = false;
    private double tx = 0;
    private double ty = 0;
    private double ta = 0;
    private double horizontalDistance = Double.POSITIVE_INFINITY;

    // ROTATION TRACKING & BEST TARGET
    private double lastHeading = 0;
    private double accumulatedHeading = 0;
    private boolean searchCircleComplete = false;

    private double maxArea = 0;
    private double bestHeading = 0;
    private boolean foundAnyBall = false;

    // STATE MACHINE
    private enum State {
        SEARCH,
        TURN_TO_BEST,
        APPROACH,
        STOP
    }

    private State state = State.SEARCH;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(YELLOW_BALL_PIPELINE);

        telemetry.setMsTransmissionInterval(20);
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Start Pose", startPose);
    }

    @Override
    public void start() {
        limelight.start();
        follower.startTeleopDrive();

        resetSearchSweep();
        state = State.SEARCH;
    }

    @Override
    public void loop() {
        updateLimelight();
        updateRotationTracking();
        updateStateMachine();
        follower.update();

        // Telemetry
        telemetry.addData("State", state);
        telemetry.addData("Full Circle Done", searchCircleComplete);
        telemetry.addData("Max Area Found", maxArea);
        telemetry.addData("Best Target Heading (deg)", Math.toDegrees(bestHeading));

        telemetry.addData("Ball Detected", targetDetected);
        if (targetDetected) {
            telemetry.addData("tx", tx);
            telemetry.addData("ty", ty);
            telemetry.addData("ta", ta);
            telemetry.addData("Horizontal Distance", horizontalDistance);
        } else {
            telemetry.addData("Horizontal Distance", "No target");
        }

        telemetry.addData("Current Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    private void resetSearchSweep() {
        lastHeading = follower.getPose().getHeading();
        accumulatedHeading = 0;
        searchCircleComplete = false;

        maxArea = 0;
        bestHeading = 0;
        foundAnyBall = false;
    }

    private void updateRotationTracking() {
        if (searchCircleComplete) return;

        double currentHeading = follower.getPose().getHeading();
        double deltaHeading = currentHeading - lastHeading;

        // Normalize delta to be between -PI and PI
        while (deltaHeading > Math.PI) deltaHeading -= 2 * Math.PI;
        while (deltaHeading < -Math.PI) deltaHeading += 2 * Math.PI;

        accumulatedHeading += Math.abs(deltaHeading);
        lastHeading = currentHeading;

        if (accumulatedHeading >= 2 * Math.PI) {
            searchCircleComplete = true;
        }
    }

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
            ta = result.getTa(); // Area of the target
            horizontalDistance = calculateHorizontalDistance(ty);
        }
    }

    private void updateStateMachine() {
        switch (state) {

            // =============================================
            // SEARCH FOR BALL (FULL 360)
            // =============================================
            case SEARCH:
                // Record the largest ball seen during the sweep
                if (targetDetected) {
                    if (ta > maxArea) {
                        maxArea = ta;
                        // Calculate absolute field heading of the target
                        bestHeading = follower.getPose().getHeading() - Math.toRadians(tx);
                        foundAnyBall = true;
                    }
                }

                if (searchCircleComplete) {
                    follower.setTeleOpDrive(0, 0, 0, true);

                    if (foundAnyBall) {
                        state = State.TURN_TO_BEST;
                    } else {
                        // If no ball was seen during the entire 360, reset and sweep again
                        resetSearchSweep();
                    }
                } else {
                    // Spin counterclockwise
                    follower.setTeleOpDrive(0, 0, SEARCH_TURN_POWER, true);
                }
                break;

            // =============================================
            // TURN TO THE BEST BALL FOUND
            // =============================================
            case TURN_TO_BEST:
                double currentHeading = follower.getPose().getHeading();
                double error = bestHeading - currentHeading;

                // Normalize error to find the shortest turn path
                while (error > Math.PI) error -= 2 * Math.PI;
                while (error < -Math.PI) error += 2 * Math.PI;

                // If we are facing the target within ~3 degrees, approach it
                if (Math.abs(error) < Math.toRadians(3.0)) {
                    follower.setTeleOpDrive(0, 0, 0, true);
                    state = State.APPROACH;
                } else {
                    double turnPower = error * HEADING_KP;

                    // Cap max turn power
                    turnPower = Math.max(-MAX_TURN_POWER, Math.min(MAX_TURN_POWER, turnPower));

                    // Prevent stalling when very close to the target heading
                    if (Math.abs(turnPower) < MIN_TURN_POWER) {
                        turnPower = Math.signum(turnPower) * MIN_TURN_POWER;
                    }

                    follower.setTeleOpDrive(0, 0, turnPower, true);
                }
                break;

            // =============================================
            // DRIVE TOWARD BALL
            // =============================================
            case APPROACH:
                // Ball disappeared, lost tracking
                if (!targetDetected) {
                    resetSearchSweep();
                    state = State.SEARCH;
                    break;
                }

                // Stop 4.5 inches away horizontally
                if (horizontalDistance <= STOP_DISTANCE) {
                    follower.setTeleOpDrive(0, 0, 0, true);
                    state = State.STOP;
                    break;
                }

                // STEERING
                double turnPower = TURN_KP * tx;
                turnPower = Math.max(-MAX_TURN_POWER, Math.min(MAX_TURN_POWER, turnPower));

                // FORWARD SPEED
                double forwardPower;
                if (horizontalDistance > 12.0) {
                    forwardPower = FAST_FORWARD;
                } else {
                    forwardPower = SLOW_FORWARD;
                }

                follower.setTeleOpDrive(forwardPower, 0, turnPower, true);
                break;

            // =============================================
            // STOP
            // =============================================
            case STOP:
                follower.setTeleOpDrive(0, 0, 0, true);
                break;
        }
    }

    private double calculateHorizontalDistance(double tyDegrees) {
        double downwardAngle = CAMERA_DOWN_ANGLE - tyDegrees;
        if (downwardAngle <= 0 || downwardAngle >= 90) {
            return Double.POSITIVE_INFINITY;
        }
        return HEIGHT_DIFFERENCE / Math.tan(Math.toRadians(downwardAngle));
    }

    @Override
    public void stop() {
        follower.setTeleOpDrive(0, 0, 0, true);
        follower.update();
        limelight.stop();
    }
}
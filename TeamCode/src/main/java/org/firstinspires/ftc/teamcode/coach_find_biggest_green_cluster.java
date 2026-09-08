package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Uses Pedro Pathing and a Limelight color pipeline to:
 *  1. rotate continuously while measuring a wrapped 360-degree sweep,
 *  2. remember the green cluster with the greatest target area,
 *  3. use Pedro's heading PIDF to turn back toward it,
 *  4. approach while Limelight proportionally keeps it centered, and
 *  5. restart the scan if the target is lost during the approach.
 *
 * Pipeline 9 setup matters:
 *  - Use a Color/Retroreflective pipeline tuned for green.
 *  - Use Smart Target Grouping if several neighboring balls should be one cluster.
 *  - Set Sort Mode to Largest. The code reads only Limelight's primary target.
 *  - Set Output -> Targeting Region to the BOTTOM of the target. This makes ty
 *    describe a point on the floor, which is required by the range calculation.
 *
 * This class targets the Pedro Pathing 2.x API.
 */
@Autonomous(name = "coach find biggest green cluster", group = "Pedro")
public class coach_find_biggest_green_cluster extends OpMode {

    private enum AutoState {
        IDLE,
        START_SCANNING,
        SCANNING,
        TURNING_TO_BEST,
        DRIVING_TO_TARGET,
        DONE,
        NO_TARGET
    }

    private static final Pose START_POSE = new Pose(72, 72, Math.PI / 2.0);

    private static final int LIMELIGHT_PIPELINE = 9;
    private static final long MAX_RESULT_STALENESS_MS = 100;

    // The scan is driven manually because a target heading of start + 2*pi is
    // equivalent to the starting heading after angle wrapping. Positive power
    // normally turns counterclockwise; reverse the sign to scan clockwise.
    private static final double FULL_SCAN_RADIANS = 2.0 * Math.PI;
    private static final double SCAN_TURN_POWER = 0.50;

    // follower.turnTo() uses Pedro's configured heading PIDF. This only limits
    // the maximum drivetrain output that PIDF may request for the return turn.
    private static final double RETURN_TURN_MAX_POWER = 0.90;

    // Limelight approach controller. tx is measured in degrees.
    private static final double APPROACH_TURN_KP = 0.025;
    private static final double APPROACH_MAX_TURN_POWER = 0.45;
    private static final double APPROACH_CENTER_DEADBAND_DEGREES = 0.75;

    // Forward power is proportional near the target but capped for speed.
    private static final double APPROACH_DISTANCE_KP = 0.05;
    private static final double APPROACH_MIN_FORWARD_POWER = 0.15;
    private static final double APPROACH_MAX_FORWARD_POWER = 0.45;

    private static final double CAMERA_HEIGHT_IN = 5.4;
    private static final double CAMERA_PITCH_DOWN_RADIANS = Math.toRadians(19.9);

    // Zero is correct when pipeline 9's Targeting Region is the bottom of the blob.
    // If ty represents the center of a ball instead, change this to the height of
    // that center above the floor.
    private static final double TARGET_POINT_HEIGHT_IN = 0.0;

    // Positive means the camera points counterclockwise from the robot's forward axis.
    private static final double CAMERA_YAW_OFFSET_RADIANS = 0.0;

    // This is the final horizontal distance from the camera lens to the target point.
    private static final double STOP_DISTANCE_IN = 6.0;
    private static final double MAX_USABLE_RANGE_IN = 120.0;
    private static final double MIN_CLUSTER_AREA = 0.01;

    private static final double TARGET_LOST_GRACE_SECONDS = 0.15;

    private Follower follower;
    private Limelight3A limelight;

    private AutoState state = AutoState.IDLE;
    private final ElapsedTime targetLostTimer = new ElapsedTime();

    private double previousScanHeadingRadians;
    private double accumulatedScanRotationRadians;

    private TargetObservation bestScanObservation;
    private TargetObservation latestObservation;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START_POSE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(LIMELIGHT_PIPELINE);
        limelight.start();

        telemetry.setMsTransmissionInterval(50);
        telemetry.addLine("Ready to scan for green clusters.");
        telemetry.addLine("Pipeline 9 must target the bottom of each grouped cluster.");
        telemetry.update();
    }

    @Override
    public void start() {
        setState(AutoState.START_SCANNING);
    }

    @Override
    public void loop() {
        // Pedro localization and motor control must be updated every loop.
        follower.update();

        switch (state) {
            case START_SCANNING:
                // One-time entry actions for every new 360-degree scan.
                follower.setMaxPower(1.0);
                follower.startTeleopDrive();

                previousScanHeadingRadians = follower.getPose().getHeading();
                accumulatedScanRotationRadians = 0.0;
                bestScanObservation = null;
                latestObservation = null;

                follower.setTeleOpDrive(0.0, 0.0, SCAN_TURN_POWER, true);
                setState(AutoState.SCANNING);
                break;

            case SCANNING: {
                double currentHeadingRadians = follower.getPose().getHeading();

                // Wrap only the small change since the previous loop so crossing
                // from +179 degrees to -179 degrees counts as +2 degrees.
                double headingChangeRadians = wrapRadians(
                        currentHeadingRadians - previousScanHeadingRadians);

                accumulatedScanRotationRadians += headingChangeRadians;
                previousScanHeadingRadians = currentHeadingRadians;

                TargetObservation observation = getPrimaryFreshObservation();
                if (observation != null) {
                    latestObservation = observation;

                    if (bestScanObservation == null
                            || observation.area > bestScanObservation.area) {
                        bestScanObservation = observation;
                    }
                }

                if (Math.abs(accumulatedScanRotationRadians) < FULL_SCAN_RADIANS) {
                    follower.setTeleOpDrive(0.0, 0.0, SCAN_TURN_POWER, true);
                } else {
                    // The measured rotation has reached one full revolution.
                    follower.setTeleOpDrive(0.0, 0.0, 0.0, true);

                    if (bestScanObservation == null) {
                        follower.holdPoint(follower.getPose());
                        setState(AutoState.NO_TARGET);
                    } else {
                        // turnTo() uses the heading PIDF configured in Constants.
                        follower.setMaxPower(RETURN_TURN_MAX_POWER);
                        follower.turnTo(bestScanObservation.fieldBearingRadians);
                        setState(AutoState.TURNING_TO_BEST);
                    }
                }
                break;
            }

            case TURNING_TO_BEST:
                if (!follower.isBusy()) {
                    // Leave Pedro's PIDF turn mode. DRIVING_TO_TARGET will use
                    // the next fresh Limelight frame before commanding motion.
                    follower.setMaxPower(1.0);
                    follower.startTeleopDrive();
                    follower.setTeleOpDrive(0.0, 0.0, 0.0, true);
                    targetLostTimer.reset();
                    setState(AutoState.DRIVING_TO_TARGET);
                }
                break;

            case DRIVING_TO_TARGET: {
                TargetObservation observation = getPrimaryFreshObservation();

                if (observation == null) {
                    // Stop immediately instead of continuing with stale vision.
                    follower.setTeleOpDrive(0.0, 0.0, 0.0, true);

                    if (targetLostTimer.seconds() >= TARGET_LOST_GRACE_SECONDS) {
                        setState(AutoState.START_SCANNING);
                    }
                } else {
                    targetLostTimer.reset();
                    latestObservation = observation;

                    if (observation.rangeInches <= STOP_DISTANCE_IN) {
                        follower.setTeleOpDrive(0.0, 0.0, 0.0, true);
                        follower.holdPoint(follower.getPose());
                        setState(AutoState.DONE);
                    } else {
                        driveTowardTarget(observation);
                    }
                }
                break;
            }

            case IDLE:
            case DONE:
            case NO_TARGET:
                break;
        }

        updateTelemetry();
    }

    /** Drives forward while a proportional Limelight correction keeps tx near zero. */
    private void driveTowardTarget(TargetObservation observation) {
        double distanceErrorInches = observation.rangeInches - STOP_DISTANCE_IN;

        double forwardPower = clamp(APPROACH_DISTANCE_KP * distanceErrorInches, APPROACH_MIN_FORWARD_POWER, APPROACH_MAX_FORWARD_POWER
        );

        double turnPower = 0.0;
        if (Math.abs(observation.txDegrees) > APPROACH_CENTER_DEADBAND_DEGREES) {
            // Limelight tx is positive to the right, while positive Pedro turn
            // power is counterclockwise, so the correction uses a negative sign.
            turnPower = clamp(-APPROACH_TURN_KP * observation.txDegrees, -APPROACH_MAX_TURN_POWER, APPROACH_MAX_TURN_POWER);
        }

        follower.setTeleOpDrive(forwardPower, 0.0, turnPower, true);
    }

    /**
     * Returns Limelight's primary target. Pipeline 9 must use Sort Mode = Largest,
     * so this is already the largest valid grouped cluster in the current frame.
     */
    private TargetObservation getPrimaryFreshObservation() {
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid() || result.getPipelineIndex() != LIMELIGHT_PIPELINE || result.getStaleness() > MAX_RESULT_STALENESS_MS) {
            return null;
        }

        return makeObservation(result.getTa(), result.getTxNC(), result.getTyNC());
    }

    /**
     * Converts Limelight angles to a ray/ground-plane intersection.
     *
     * Limelight angles are positive right and positive up. Pedro headings are
     * positive counterclockwise, so the horizontal camera angle is subtracted.
     */
    private TargetObservation makeObservation(double area, double txDegrees, double tyDegrees) {
        if (!Double.isFinite(area) || !Double.isFinite(txDegrees) || !Double.isFinite(tyDegrees) || area < MIN_CLUSTER_AREA) {
            return null;
        }

        double heightDifference = CAMERA_HEIGHT_IN - TARGET_POINT_HEIGHT_IN;
        if (heightDifference <= 0.0) {
            return null;
        }

        double tx = Math.toRadians(txDegrees);
        double ty = Math.toRadians(tyDegrees);
        double tanTx = Math.tan(tx);
        double tanTy = Math.tan(ty);

        double sinPitch = Math.sin(CAMERA_PITCH_DOWN_RADIANS);
        double cosPitch = Math.cos(CAMERA_PITCH_DOWN_RADIANS);

        // Ray components after rotating the optical axis down by the mount pitch.
        double forwardComponent = cosPitch + sinPitch * tanTy;
        double rightComponent = tanTx;
        double downComponent = sinPitch - cosPitch * tanTy;

        // Reject targets at/above the horizon or behind the camera.
        if (downComponent <= 0.02 || forwardComponent <= 0.0) {
            return null;
        }

        double rayScale = heightDifference / downComponent;
        double forwardInches = rayScale * forwardComponent;
        double rightInches = rayScale * rightComponent;
        double rangeInches = Math.hypot(forwardInches, rightInches);

        if (!Double.isFinite(rangeInches) || rangeInches <= 0.0 || rangeInches > MAX_USABLE_RANGE_IN) {
            return null;
        }

        double clockwiseCameraBearing = Math.atan2(rightInches, forwardInches);
        double fieldBearing = wrapRadians(follower.getPose().getHeading() + CAMERA_YAW_OFFSET_RADIANS - clockwiseCameraBearing);

        return new TargetObservation(area, txDegrees, tyDegrees, rangeInches, fieldBearing);
    }

    private void setState(AutoState newState) {
        state = newState;
    }

    private void updateTelemetry() {
        Pose pose = follower.getPose();

        telemetry.addData("State", state);
        telemetry.addData("Pose", "x %.1f, y %.1f, h %.1f deg", pose.getX(), pose.getY(), Math.toDegrees(pose.getHeading()));
        telemetry.addData("Scan rotation", "%.1f / 360.0 deg", Math.toDegrees(Math.abs(accumulatedScanRotationRadians)));

        if (latestObservation != null) {
            telemetry.addData("Current target", "area %.3f, tx %.1f, ty %.1f", latestObservation.area, latestObservation.txDegrees, latestObservation.tyDegrees);
            telemetry.addData("Current range", "%.1f in", latestObservation.rangeInches);
        }

        if (bestScanObservation != null) {
            telemetry.addData("Best cluster area", "%.3f", bestScanObservation.area);
            telemetry.addData("Best bearing", "%.1f deg", Math.toDegrees(bestScanObservation.fieldBearingRadians));
            telemetry.addData("Best scan range", "%.1f in", bestScanObservation.rangeInches);
        }

        if (state == AutoState.NO_TARGET) {
            telemetry.addLine("Stopped: no fresh target from pipeline 9.");
        }

        telemetry.update();
    }

    private static double wrapRadians(double angle) {
        while (angle <= -Math.PI) {
            angle += 2.0 * Math.PI;
        }
        while (angle > Math.PI) {
            angle -= 2.0 * Math.PI;
        }
        return angle;
    }

    private static double clamp(double value, double minimum, double maximum) {
        return Math.max(minimum, Math.min(maximum, value));
    }

    @Override
    public void stop() {
        if (follower != null) {
            follower.breakFollowing();
        }
        if (limelight != null) {
            limelight.stop();
        }
    }

    private static final class TargetObservation {
        final double area;
        final double txDegrees;
        final double tyDegrees;
        final double rangeInches;
        final double fieldBearingRadians;

        TargetObservation(double area, double txDegrees, double tyDegrees, double rangeInches, double fieldBearingRadians) {
            this.area = area;
            this.txDegrees = txDegrees;
            this.tyDegrees = tyDegrees;
            this.rangeInches = rangeInches;
            this.fieldBearingRadians = fieldBearingRadians;
        }
    }
}

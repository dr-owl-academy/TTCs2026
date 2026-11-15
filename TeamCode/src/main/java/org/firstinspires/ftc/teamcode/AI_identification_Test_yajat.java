
package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@TeleOp(name = "AI_identification_and_auton_Test_yajat")
public class AI_identification_Test_yajat extends OpMode {

    Follower follower;
    Limelight3A limelight;

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    boolean aiming = false;
    boolean LastA = false;
    boolean autoRunning = false;

    // Remember the last AI classification
    String lastAIClass = "None";
    double lastAIConfidence = 0.0;

    final int AI_pipeline = 0;

    // SELECT YOUR ALLIANCE
    final boolean blueTeam = true;

    PathChain switchSidePath;

    // =====================================
    // ROUTE CONFIGURATION
    // =====================================

    static class Route {

        double waypoint1X, waypoint1Y;
        double waypoint2X, waypoint2Y;
        double targetX, targetY, targetHeading;

        Route(double w1x, double w1y,
              double w2x, double w2y,
              double tx, double ty, double tHeading) {

            waypoint1X = w1x;
            waypoint1Y = w1y;

            waypoint2X = w2x;
            waypoint2Y = w2y;

            targetX = tx;
            targetY = ty;
            targetHeading = tHeading;
        }

        boolean isConfigured() {
            return Double.isFinite(waypoint1X)
                    && Double.isFinite(waypoint1Y)
                    && Double.isFinite(waypoint2X)
                    && Double.isFinite(waypoint2Y)
                    && Double.isFinite(targetX)
                    && Double.isFinite(targetY)
                    && Double.isFinite(targetHeading);
        }
    }

    // REPLACE THESE WITH ACTUAL FIELD COORDINATES

    final Route BLUE_UPPER = new Route(
            112, 99,
            112, 30,
            86, 30, 90
    );

    final Route BLUE_LOWER = new Route(
            107, 39,
            107, 98,
            86, 127, 180
    );

    final Route RED_UPPER = new Route(
            Double.NaN, Double.NaN,
            Double.NaN, Double.NaN,
            Double.NaN, Double.NaN, Double.NaN
    );

    final Route RED_LOWER = new Route(
            Double.NaN, Double.NaN,
            Double.NaN, Double.NaN,
            Double.NaN, Double.NaN, Double.NaN
    );

    // =====================================
    // INITIALIZATION
    // =====================================

    @Override
    public void init() {

        frontLeft = hardwareMap.get(DcMotor.class, "motorLF");
        frontRight = hardwareMap.get(DcMotor.class, "motorRF");
        backLeft = hardwareMap.get(DcMotor.class, "motorLB");
        backRight = hardwareMap.get(DcMotor.class, "motorRB");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(new Pose(72, 72, Math.PI / 2));

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(AI_pipeline);
        limelight.start();
    }

    @Override
    public void start() {

        follower.startTeleopDrive();
        follower.update();
    }

    // =====================================
    // START AUTONOMOUS PATH
    // =====================================

    private void startSwitchSidePath() {

        Pose startPose = follower.getPose();

        double currentY = startPose.getY();

        Route selectedRoute;

        // Select route based on alliance and Y position

        if (blueTeam) {

            if (currentY > 72) {
                selectedRoute = BLUE_UPPER;
            } else {
                selectedRoute = BLUE_LOWER;
            }

        } else {

            if (currentY > 72) {
                selectedRoute = RED_UPPER;
            } else {
                selectedRoute = RED_LOWER;
            }
        }

        telemetry.addData("Current Y", currentY);

        if (!selectedRoute.isConfigured()) {

            telemetry.addData(
                    "Path Error",
                    "Configure waypoint coordinates first"
            );

            aiming = false;
            return;
        }

        Pose waypoint1 = new Pose(
                selectedRoute.waypoint1X,
                selectedRoute.waypoint1Y,
                Math.toRadians(selectedRoute.targetHeading)
        );

        Pose waypoint2 = new Pose(
                selectedRoute.waypoint2X,
                selectedRoute.waypoint2Y,
                Math.toRadians(selectedRoute.targetHeading)
        );

        Pose targetPose = new Pose(
                selectedRoute.targetX,
                selectedRoute.targetY,
                Math.toRadians(selectedRoute.targetHeading)
        );

        // Build path through prerequisite waypoints

        switchSidePath = follower.pathBuilder()
                .addPath(new BezierLine(startPose, waypoint1))
                .setLinearHeadingInterpolation(
                        startPose.getHeading(),
                        Math.toRadians(selectedRoute.targetHeading)
                )
                .addPath(new BezierLine(waypoint1, waypoint2))
                .setLinearHeadingInterpolation(
                        Math.toRadians(selectedRoute.targetHeading),
                        Math.toRadians(selectedRoute.targetHeading)
                )
                .addPath(new BezierLine(waypoint2, targetPose))
                .setLinearHeadingInterpolation(
                        Math.toRadians(selectedRoute.targetHeading),
                        Math.toRadians(selectedRoute.targetHeading)
                )
                .build();

        // Limit autonomous motor power
        follower.setMaxPower(0.5);

        follower.followPath(switchSidePath, true);

        autoRunning = true;
        aiming = false;

        telemetry.addData("Autonomous", "Path Started");
        telemetry.addData("Target X", targetPose.getX());
        telemetry.addData("Target Y", targetPose.getY());
    }

    // =====================================
    // MAIN LOOP
    // =====================================

    @Override
    public void loop() {

        follower.update();

        Pose currentPose = follower.getPose();

        if (currentPose == null) {

            follower.setTeleOpDrive(0, 0, 0, true);

            telemetry.addData(
                    "Error",
                    "Pedro Pathing pose is null"
            );

            telemetry.update();
            return;
        }

        telemetry.addData("Robot X", currentPose.getX());
        telemetry.addData("Robot Y", currentPose.getY());
        telemetry.addData(
                "Robot Heading",
                Math.toDegrees(currentPose.getHeading())
        );

        // Detect new A button press

        if (gamepad1.a && !LastA && !autoRunning) {
            aiming = !aiming;
        }

        LastA = gamepad1.a;

        // =================================
        // AUTONOMOUS MODE
        // =================================

        if (autoRunning) {

            telemetry.addData("Mode", "Autonomous");
            telemetry.addData("Path Busy", follower.isBusy());

            if (!follower.isBusy()) {

                autoRunning = false;
                aiming = false;

                follower.setMaxPower(1.0);
                follower.startTeleopDrive();

                telemetry.addData(
                        "Autonomous",
                        "Finished"
                );
            }
        }

        // =================================
        // AI MODE
        // =================================

        else if (aiming) {

            telemetry.addData("AI Branch", "Entered");

            // Allow controller movement during AI detection
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true
            );

            telemetry.addData("Mode", "AI Detection");

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {

                List<LLResultTypes.ClassifierResult> detections =
                        result.getClassifierResults();

                telemetry.addData("Result Valid", true);

                telemetry.addData(
                        "Detection Count",
                        detections == null ? 0 : detections.size()
                );

                if (detections != null && !detections.isEmpty()) {

                    // Find highest confidence classification

                    LLResultTypes.ClassifierResult best =
                            detections.get(0);

                    for (LLResultTypes.ClassifierResult detection : detections) {

                        if (detection.getConfidence() >
                                best.getConfidence()) {

                            best = detection;
                        }
                    }

                    String className = best.getClassName();

                    // Save latest AI classification
                    lastAIClass = className;
                    lastAIConfidence = best.getConfidence();

                    telemetry.addData("AI Class", className);
                    telemetry.addData("Confidence", best.getConfidence());

                    if (className.equalsIgnoreCase("0 Tilt Towards Blue")
                            && !autoRunning) {

                        telemetry.addData(
                                "AI Decision",
                                "Switch sides"
                        );

                        startSwitchSidePath();

                    } else if (
                            className.equalsIgnoreCase("1 Tilt Away Blue")
                    ) {

                        telemetry.addData(
                                "AI Decision",
                                "Stay in position"
                        );

                        aiming = false;
                    }

                } else {

                    telemetry.addData(
                            "AI",
                            "No classifications"
                    );
                }

            } else {

                telemetry.addData("Result Valid", false);
                telemetry.addData("AI", "No valid result");
            }
        }

        // =================================
        // DRIVER CONTROL
        // =================================

        else {

            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true
            );

            telemetry.addData("Mode", "Driving Mode");
        }

        // =================================
        // LAST DETECTED AI INFORMATION
        // =================================

        telemetry.addData("Last AI Class", lastAIClass);
        telemetry.addData(
                "Last AI Confidence",
                String.format("%.2f%%", lastAIConfidence * 100)
        );

        telemetry.update();
    }
}

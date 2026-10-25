package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.List;

@TeleOp(name = "AI_identification_Test_yajat 2")
public class AI_identification_test_yajat_2 extends OpMode {

    Limelight3A limelight;
    IMU imu;

    boolean LastA = false;

    // 0 = Manual driving
    // 1 = Turn to initial heading
    // 2 = Limelight AI aiming
    int aimStage = 0;

    final int AI_pipeline = 0;

    final double AIM_KP = 0.02;
    final double AIM_TOLERANCE = 1.5;
    final double MAX_TURN = 0.5;

    final double INITIAL_HEADING = 0.0;
    final double HEADING_KP = 0.015;
    final double HEADING_TOLERANCE = 2.0;

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    @Override
    public void init() {

        // Initialize drive motors
        frontLeft = hardwareMap.get(DcMotor.class, "motorLF");
        frontRight = hardwareMap.get(DcMotor.class, "motorRF");
        backLeft = hardwareMap.get(DcMotor.class, "motorLB");
        backRight = hardwareMap.get(DcMotor.class, "motorRB");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");

        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        ));

        imu.resetYaw();

        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(AI_pipeline);
        limelight.start();
    }

    @Override
    public void loop() {

        // Press A to start the sequence or cancel it
        if (gamepad1.a && !LastA) {
            if (aimStage == 0) {
                aimStage = 1;
            } else {
                aimStage = 0;
                driveRobot(0, 0, 0);
            }
        }

        LastA = gamepad1.a;

        // STAGE 0: Manual driving
        if (aimStage == 0) {

            driveRobot(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x
            );

            telemetry.addData("Mode", "Manual driving");
        }

        // STAGE 1: Turn to the chosen heading
        else if (aimStage == 1) {

            double heading = imu.getRobotYawPitchRollAngles()
                    .getYaw(AngleUnit.DEGREES);

            double error = INITIAL_HEADING - heading;

            // Normalize error to -180 through +180 degrees
            error = Math.toDegrees(Math.atan2(
                    Math.sin(Math.toRadians(error)),
                    Math.cos(Math.toRadians(error))
            ));

            telemetry.addData("Mode", "Turning to initial heading");
            telemetry.addData("Heading", heading);
            telemetry.addData("Heading error", error);

            if (Math.abs(error) < HEADING_TOLERANCE) {

                driveRobot(0, 0, 0);
                aimStage = 2;

            } else {

                double turn = HEADING_KP * error;

                turn = Math.max(
                        -MAX_TURN,
                        Math.min(MAX_TURN, turn)
                );

                driveRobot(0, 0, turn);
            }
        }

        // STAGE 2: Limelight AI aiming
        else if (aimStage == 2) {

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {

                List<LLResultTypes.DetectorResult> detections =
                        result.getDetectorResults();

                if (detections != null && !detections.isEmpty()) {

                    // Choose detection with highest confidence
                    LLResultTypes.DetectorResult best = detections.get(0);

                    for (LLResultTypes.DetectorResult detection : detections) {
                        if (detection.getConfidence() > best.getConfidence()) {
                            best = detection;
                        }
                    }

                    double tx = best.getTargetXDegrees();

                    telemetry.addData("AI Target", best.getClassName());
                    telemetry.addData("Confidence", best.getConfidence());
                    telemetry.addData("Horizontal Offset", tx);

                    if (Math.abs(tx) < AIM_TOLERANCE) {

                        driveRobot(0, 0, 0);
                        aimStage = 0;

                    } else {

                        double turn = AIM_KP * tx;

                        turn = Math.max(
                                -MAX_TURN,
                                Math.min(MAX_TURN, turn)
                        );

                        driveRobot(0, 0, turn);
                    }

                } else {

                    driveRobot(0, 0, 0);
                    telemetry.addData("AI", "No detections");
                }

            } else {

                driveRobot(0, 0, 0);
                telemetry.addData("AI", "No valid result");
            }
        }

        telemetry.addData("Aim stage", aimStage);
        telemetry.update();
    }

    // Direct mecanum drive control
    private void driveRobot(double forward, double strafe, double turn) {

        double frontLeftPower = forward + strafe + turn;
        double frontRightPower = forward - strafe - turn;
        double backLeftPower = forward - strafe + turn;
        double backRightPower = forward + strafe - turn;

        double max = Math.max(
                1.0,
                Math.max(
                        Math.max(
                                Math.abs(frontLeftPower),
                                Math.abs(frontRightPower)
                        ),
                        Math.max(
                                Math.abs(backLeftPower),
                                Math.abs(backRightPower)
                        )
                )
        );

        frontLeft.setPower(frontLeftPower / max);
        frontRight.setPower(frontRightPower / max);
        backLeft.setPower(backLeftPower / max);
        backRight.setPower(backRightPower / max);
    }

    @Override
    public void stop() {
        limelight.stop();
        driveRobot(0, 0, 0);
    }
}
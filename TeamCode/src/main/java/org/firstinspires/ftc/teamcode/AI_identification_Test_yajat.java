


package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import java.util.List;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;



@TeleOp(name = "AI_identification_Test_yajat")
public class AI_identification_Test_yajat extends OpMode {

    Limelight3A limelight;
    IMU imu;
    int aimStage = 0;
// 0 = manual driving
// 1 = turn to initial heading
// 2 = Limelight AI aiming

    final double INITIAL_HEADING = 0.0; // degrees
    final double HEADING_KP = 0.015;
    final double HEADING_TOLERANCE = 2.0;

    boolean aiming = false;
    boolean LastA = false;
    final int AI_pipeline = 0;
    final double AIM_KP = 0.02;
    final double AIM_TOLERENCE=1.5;
    final double MAX_TURN = 0.5;

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    @Override
    public void init() {

        frontLeft=hardwareMap.get(DcMotor.class, "motorLF");
        frontRight=hardwareMap.get(DcMotor.class,"motorRF");
        backLeft=hardwareMap.get(DcMotor.class,"motorLB");
        backRight=hardwareMap.get(DcMotor.class,"motorRB");


        limelight=hardwareMap.get(Limelight3A.class,"limelight");
        limelight.pipelineSwitch(AI_pipeline);
        limelight.start();

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");

        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        ));

        imu.resetYaw();
        }


    @Override
    public void loop(){


        // Detect a new press of A
        if (gamepad1.a && !LastA) {
            aiming = !aiming;
        }
        LastA = gamepad1.a;

        if (aiming) {

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {

                List<LLResultTypes.DetectorResult> detections =
                        result.getDetectorResults();

                if (detections != null && !detections.isEmpty()) {

                    // Choose the detection with the highest confidence
                    LLResultTypes.DetectorResult best = detections.get(0);

                    for (LLResultTypes.DetectorResult detection : detections) {
                        if (detection.getConfidence() > best.getConfidence()) {
                            best = detection;
                        }
                    }

                    // Horizontal angle from the camera crosshair to target
                    double tx = best.getTargetXDegrees();

                    // Stop when the target is centered
                    if (Math.abs(tx) < AIM_TOLERENCE) {

                        driveRobot(0, 0, 0);
                        aiming = false;

                    } else {

                        double turn = AIM_KP * tx;

                        turn = Math.max(
                                -MAX_TURN,
                                Math.min(MAX_TURN, turn)
                        );

                        driveRobot(0, 0, turn);
                    }

                    telemetry.addData("AI Target", best.getClassName());
                    telemetry.addData("Confidence", best.getConfidence());
                    telemetry.addData("Horizontal Offset", tx);

                } else {
                    driveRobot(0, 0, 0);
                    telemetry.addData("AI", "No detections");
                }

            } else {
                driveRobot(0, 0, 0);
                telemetry.addData("AI", "No valid result");
            }

        } else {

            driveRobot(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x
            );
        }
        telemetry.update();

    }
    private void driveRobot(double forward, double strafe, double turn) {
        double frontLeftPower = forward + strafe + turn;
        double frontRightPower = forward - strafe - turn;
        double backLeftPower = forward - strafe + turn;
        double backRightPower = forward + strafe - turn;

        double max = Math.max(
                1.0,
                Math.max(
                        Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                        Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))
                )
        );

        frontLeft.setPower(frontLeftPower / max);
        frontRight.setPower(frontRightPower / max);
        backLeft.setPower(backLeftPower / max);
        backRight.setPower(backRightPower / max);
    }
}



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



@TeleOp(name = "AI_identification_Test_yajat")
public class AI_identification_Test_yajat extends OpMode {
    Follower follower;
    Limelight3A limelight;

    boolean aiming = false;
    boolean LastA = false;
    final double TARGET_X = 72;
    final double TARGET_Y = 72;
    final int AI_pipeline = 0;
    final double AIM_KP = 0.02;
    final double AIM_TOLERENCE=1.5;
    final double TURN_KP = 0.8;
    final double MAX_TURN = 0.5;

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    @Override
    public void init() {

        frontLeft=hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight=hardwareMap.get(DcMotor.class,"frontRight");
        backLeft=hardwareMap.get(DcMotor.class,"backLeft");
        backRight=hardwareMap.get(DcMotor.class,"backRight");

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 72, Math.PI / 2));

        limelight=hardwareMap.get(Limelight3A.class,"limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        }


    @Override
    public void loop(){
        follower.update();

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

                        follower.setTeleOpDrive(0, 0, 0, true);
                        aiming = false;

                    } else {

                        double turn = AIM_KP * tx;

                        turn = Math.max(
                                -MAX_TURN,
                                Math.min(MAX_TURN, turn)
                        );

                        follower.setTeleOpDrive(0, 0, turn, true);
                    }

                    telemetry.addData("AI Target", best.getClassName());
                    telemetry.addData("Confidence", best.getConfidence());
                    telemetry.addData("Horizontal Offset", tx);

                } else {
                    follower.setTeleOpDrive(0, 0, 0, true);
                    telemetry.addData("AI", "No detections");
                }

            } else {
                follower.setTeleOpDrive(0, 0, 0, true);
                telemetry.addData("AI", "No valid result");
            }

        } else {

            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x,
                    true
            );
        }
        telemetry.update();

    }
}



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

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 72, Math.PI / 2));

        limelight=hardwareMap.get(Limelight3A.class,"limelight");
        limelight.pipelineSwitch(AI_pipeline);
        limelight.start();

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        @Override
        public void start(){
            follower.startTeleopDrive();
            follower.update();
        }


    @Override
    public void loop(){
        follower.update();
        if (follower.getPose() == null) {
            follower.setTeleOpDrive(0, 0, 0, true);
            telemetry.addData("Error", "Pedro Pathing pose is null");
            telemetry.update();
            return;
        }

        // Detect a new press of A
        if (gamepad1.a && !LastA) {
            aiming = !aiming;
        }
        LastA = gamepad1.a;

        if (aiming) {
            telemetry.addData("Mode", "Aiming Mode");

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {

                List<LLResultTypes.ClassifierResult> detections =
                        result.getClassifierResults();

                telemetry.addData("Result Valid", true);
                telemetry.addData("Detection Count",
                        detections == null ? 0 : detections.size());

                if (detections != null && !detections.isEmpty()) {

                    // Find the classification with the highest confidence
                    LLResultTypes.ClassifierResult best = detections.get(0);

                    for (LLResultTypes.ClassifierResult detection : detections) {
                        if (detection.getConfidence() > best.getConfidence()) {
                            best = detection;
                        }
                    }

                    telemetry.addData("AI Class", best.getClassName());
                    telemetry.addData("Confidence", best.getConfidence());

                    // Classifier cannot provide target direction
                    follower.setTeleOpDrive(0, 0, 0, true);

                } else {
                    follower.setTeleOpDrive(0, 0, 0, true);
                    aiming = false;
                    telemetry.addData("AI", "No classifications");
                }

            } else {
                follower.setTeleOpDrive(0, 0, 0, true);
                aiming = false;
                telemetry.addData("Result Valid", false);
                telemetry.addData("AI", "No valid result");
            }

        } else {
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    gamepad1.right_stick_x,
                    true
            );

            telemetry.addData("Mode", "Driving Mode");
        }
        telemetry.update();

    }
}
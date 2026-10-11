


package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;



@TeleOp(name = "AI_identification_Test_yajat")
public class AI_identification_Test_yajat extends OpMode {
    Follower follower;

    boolean aiming = false;
    boolean LastA = false;

    final double TARGET_X = 72;
    final double TARGET_Y = 72;
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

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        }


    @Override
    public void loop(){
        follower.update();

        // Detect a new press of A
        if (gamepad1.a && !LastA) {
            aiming = true;
        }
        LastA = gamepad1.a;

        if (aiming) {

            Pose pose = follower.getPose();

            // Vector from robot to field coordinate (72,72)
            double dx = TARGET_X - pose.getX();
            double dy = TARGET_Y - pose.getY();

            // Heading needed to face the target
            double targetHeading = Math.atan2(dy, dx);

            // Calculate the shortest angular difference
            double error = Math.atan2(
                    Math.sin(targetHeading - pose.getHeading()),
                    Math.cos(targetHeading - pose.getHeading())
            );

            // Stop turning when within 2 degrees
            if (Math.abs(error) < Math.toRadians(2)) {

                follower.setTeleOpDrive(0, 0, 0, true);
                aiming = false;

            } else {

                double turn = TURN_KP * error;

                // Limit turning power
                turn = Math.max(-MAX_TURN, Math.min(MAX_TURN, turn));

                // Prevent turning power from becoming too small
                if (Math.abs(turn) < 0.12) {
                    turn = Math.copySign(0.12, turn);
                }

                follower.setTeleOpDrive(0, 0, turn, true);
            }

        } else {

            // Manual mecanum driving
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x,
                    true
            );
        }
    }
}
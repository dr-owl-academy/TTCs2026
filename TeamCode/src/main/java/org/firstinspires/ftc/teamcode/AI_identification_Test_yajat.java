


package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name = "AI_identification_Test_yajat")
public class AI_identification_Test_yajat extends OpMode {

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

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        }


    @Override
    public void loop(){
        double x = -gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        double denominator = Math.max(
                Math.abs(y) + Math.abs(x) + Math.abs(turn), 1.0
        );
        frontLeft.setPower((y + x + turn) / denominator);
        backLeft.setPower((y - x + turn) / denominator);
        frontRight.setPower((y - x - turn) / denominator);
        backRight.setPower((y + x - turn) / denominator);
    }
}
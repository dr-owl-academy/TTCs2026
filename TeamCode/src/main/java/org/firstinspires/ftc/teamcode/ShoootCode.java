package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "yajat_shooter_code")
public class ShoootCode extends OpMode {
    DcMotor shooterflywheel;
    double shooterpower = 0.5;

    @Override
    public void init() {
        shooterflywheel=hardwareMap.get
                (DcMotor.class,"flywheel");
        shooterflywheel.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void loop() {
       if (gamepad1.dpad_up){
           shooterpower +=0.05;
       }
       if (gamepad1.dpad_down){
           shooterpower -=0.05;
       }
       shooterpower = Math.max(
               0.0,
               Math.min(1.0,shooterpower)
       );
        shooterflywheel.setPower(shooterpower);
    }
}




package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "yajat_shooter_code")
public class ShoootCode extends OpMode {
    DcMotor shooterflywheel;
    Servo ShooterHood;
    double shooterpower = 0.5;
    double HoodPower = 0.5;

    @Override
    public void init() {
        shooterflywheel=hardwareMap.get
                (DcMotor.class,"flywheel");
        shooterflywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        ShooterHood = hardwareMap.get(Servo.class,"");
        ShooterHood.setPosition(HoodPower);

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
       if (gamepad1.dpad_right){
           HoodPower +=0.02;
       }
       if (gamepad1.dpad_left){
           HoodPower -=0.02;
       }
       HoodPower = Math.max(0.0,Math.min(1.0,HoodPower));
       ShooterHood.setPosition(HoodPower);


       shooterflywheel.setPower(shooterpower);
        telemetry.addData( "shooter power",shooterpower);
        telemetry.addData("Hood Pos",HoodPower);
        telemetry.update();
    }
}




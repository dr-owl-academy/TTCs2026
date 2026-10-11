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
    double HoodPos = 0.0;

    @Override
    public void init() {
        shooterflywheel=hardwareMap.get
                (DcMotor.class,"flywheel");
        shooterflywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        ShooterHood = hardwareMap.get(Servo.class,"Servo");
        ShooterHood.setPosition(HoodPos);

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
           HoodPos +=0.02;
       }
       if (gamepad1.dpad_left){
           HoodPos -=0.02;
       }
       HoodPos = Math.max(0.0,Math.min(1.0, HoodPos));
       ShooterHood.setPosition(HoodPos);


       shooterflywheel.setPower(shooterpower);
        telemetry.addData( "shooter power",shooterpower);
        telemetry.addData("Hood Pos", HoodPos);
        telemetry.update();
    }
}




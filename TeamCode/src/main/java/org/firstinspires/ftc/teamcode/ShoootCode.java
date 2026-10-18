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


    // Hood servo position constraints
    final double MIN_HOOD_POS = 0.0;
    final double MAX_HOOD_POS = 1.0;

    // Adjustment increments
    final double SHOOTER_STEP = 0.05;
    final double HOOD_STEP = 0.02;

    boolean lastDpadUp = false;
    boolean lastDpadDown = false;
    boolean lastDpadRight = false;
    boolean lastDpadLeft = false;

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
        if (gamepad1.dpad_up && !lastDpadUp) {
            shooterpower += SHOOTER_STEP;
        }

        if (gamepad1.dpad_down && !lastDpadDown) {
            shooterpower -= SHOOTER_STEP;
        }

// Hood position
        if (gamepad1.dpad_right && !lastDpadRight) {
            HoodPos += HOOD_STEP;
        }

        if (gamepad1.dpad_left && !lastDpadLeft) {
            HoodPos -= HOOD_STEP;
        }

// Save current button states for the next loop
        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;
        lastDpadRight = gamepad1.dpad_right;
        lastDpadLeft = gamepad1.dpad_left;


       shooterflywheel.setPower(shooterpower);
        telemetry.addData( "shooter power",shooterpower);
        telemetry.addData("Hood Pos", HoodPos);
        telemetry.addData("servo position",ShooterHood.getPosition());
        telemetry.update();
    }
}




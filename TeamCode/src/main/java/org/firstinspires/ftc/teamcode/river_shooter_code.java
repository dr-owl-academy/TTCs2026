package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "river_shooter_code")
public class river_shooter_code extends OpMode {
    DcMotor shooterflywheel;
    Servo ShooterHood;
    double shooterpower = 0.5;
    double HoodPos = 0.1;


    // Hood servo position constraints
    final double MIN_HOOD_POS = 0.10;
    final double MAX_HOOD_POS = 0.46;

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

        shooterpower = Math.max(0.0, Math.min(1.0, shooterpower));

// Hood position
        if (gamepad1.dpad_right && !lastDpadRight) {
            HoodPos += HOOD_STEP;
        }

        if (gamepad1.dpad_left && !lastDpadLeft) {
            HoodPos -= HOOD_STEP;
        }
        HoodPos = Math.max(
                MIN_HOOD_POS,
                Math.min(MAX_HOOD_POS, HoodPos));

        ShooterHood.setPosition(HoodPos);

// Save current button states for the next loop
        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;
        lastDpadRight = gamepad1.dpad_right;
        lastDpadLeft = gamepad1.dpad_left;


        shooterflywheel.setPower(shooterpower);
        telemetry.addData( "Shooter power", "%.2f", shooterpower);
        telemetry.addData("Hood Pos", "%.4f", HoodPos);
        telemetry.addData("Servo Command", "%.4f",
                ShooterHood.getPosition());
        telemetry.addData("D-pad Right", gamepad1.dpad_right);
        telemetry.addData("D-pad Left", gamepad1.dpad_left);
        telemetry.update();
    }
}
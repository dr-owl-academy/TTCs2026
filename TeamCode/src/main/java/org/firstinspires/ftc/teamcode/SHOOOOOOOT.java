package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Shooter Test")
public class SHOOOOOOOT extends OpMode {

    private DcMotorEx shooter;

    private double shooterPower = 0.0;
    private final double speedIncrement = 0.05;

    @Override
    public void init() {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
    }

    @Override
    public void loop() {

        if (gamepad1.dpad_up) {
            shooterPower += speedIncrement;
        }

        if (gamepad1.dpad_down) {
            shooterPower -= speedIncrement;
        }

        shooterPower = Math.max(0.0, Math.min(1.0, shooterPower));

        shooter.setPower(shooterPower);

        telemetry.addData("Shooter Power", shooterPower);
        telemetry.update();
    }
}
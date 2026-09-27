package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name= "River_intake_speed")
public class river_intake_speed extends OpMode {

    DcMotor intake;

    double intakePower = 0.0;

    boolean lastUp = false;
    boolean lastDown = false;

    @Override
    public void init() {

        intake = hardwareMap.get(DcMotor.class, "intake");

        telemetry.addData("Intake Power", "%.1f", intakePower);
        telemetry.update();
    }

    @Override
    public void loop() {

        if (gamepad1.dpad_up && !lastUp) {
            intakePower -= 0.1;
        }

        intakePower = Math.max(-1.0, Math.min(1.0, intakePower));

        intake.setPower(intakePower);

        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;

        telemetry.addData("Intake Power", "%1f", intakePower);
        telemetry.update();
    }
}

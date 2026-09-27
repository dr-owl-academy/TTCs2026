package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class river_intake_speed extends LinearOpMode {

    DcMotor intake;

    @Override
    public void runOpMode() {

        intake = hardwareMap.get(DcMotor.class, "intake");

        double intakePower = 0.0;

        telemetry.addData("Intake Power", intakePower);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Increase power
            if (gamepad1.dpad_up) {
                intakePower += 0.1;
                sleep(150);
            }

            intakePower = Math.max(-1.0, Math.min(1.0, intakePower));

            intake.setPower(intakePower);

            telemetry.addData("Intake Power", "%.If", intakePower);
            telemetry.update();
        }

    }
}

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "shooter_code")
public class ShoootCode extends OpMode {
    DcMotor shooterflywheel;

    @Override
    public void init() {
        shooterflywheel=hardwareMap.get
                (DcMotor.class,"flywheel");
    }

    @Override
    public void loop() {
        double power = gamepad1.right_trigger;
        shooterflywheel.setPower(power);
    }
}




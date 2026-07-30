package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.mechanisms.DanielMechanism;
@TeleOp
public class TouchSensorPractice extends OpMode {
    DanielMechanism bench = new DanielMechanism();

    private DcMotor Motor;

    @Override
    public void init() {
        bench.init(hardwareMap);
        Motor = hardwareMap.get(DcMotor.class, "motor");
        Motor.setPower(0);
        Motor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void loop() {
        if (!bench.getTouchSensorState());
        Motor.setPower(0.5);

        telemetry.addData("Touch Sensor State", bench.getTouchSensorState());
        telemetry.addData("Motor Power: ", Motor.getPower());
        telemetry.update();
    }
}

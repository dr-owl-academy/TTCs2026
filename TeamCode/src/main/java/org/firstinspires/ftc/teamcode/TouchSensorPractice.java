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
    private int ballcounter = 0;
    private boolean lastTouchState = false;

    @Override
    public void init() {
        bench.init(hardwareMap);
        Motor = hardwareMap.get(DcMotor.class, "motor");
        Motor.setPower(0);
        Motor.setDirection(DcMotorSimple.Direction.FORWARD);
        //Motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void loop() {
        if (!bench.getTouchSensorState()){
            Motor.setPower(0.5);
        } else{
            Motor.setPower(0);
        }




        telemetry.addData("Touch Sensor State", bench.getTouchSensorState());
        telemetry.addData("Motor Power: ", Motor.getPower());
        telemetry.addData("Times clicked: ", Motor.getCurrentPosition());
        telemetry.update();
    }
}

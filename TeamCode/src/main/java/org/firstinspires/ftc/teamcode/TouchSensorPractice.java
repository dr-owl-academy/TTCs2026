package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.mechanisms.touchsensorBench;
@TeleOp
public class TouchSensorPractice extends OpMode {

    private int BallCounter=0;
    private boolean lastTouchState=false;
    private DcMotor motor;
    touchsensorBench bench = new touchsensorBench();

    @Override
    public void init() {
        bench.init(hardwareMap);

        motor = hardwareMap.get(DcMotor.class, "motor");

        motor.setDirection(FORWARD);

    }


    @Override
    public void loop() {
        telemetry.addData("Touch Sensor State", bench.getTouchSensorState());

        if(!bench.getTouchSensorState()){
           motor.setPower(0.3);
        }
        else {
            motor.setPower(0);
        }

        boolean currentTouchState = !bench.getTouchSensorState();
        if(currentTouchState&&!lastTouchState){BallCounter++;}

        lastTouchState=currentTouchState;
        telemetry.addData("Ball Count",BallCounter);

        telemetry.update();

    }
}



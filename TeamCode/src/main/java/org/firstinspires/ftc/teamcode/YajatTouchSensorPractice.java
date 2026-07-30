package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.mechanisms.touchsensorBench;
@TeleOp
public class YajatTouchSensorPractice extends OpMode {





    private int BallCounter=0;
    private boolean lastTouchState=false;
    private int intakeDirection = 0;
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

        if(gamepad1.right_bumper){intakeDirection=1;}
        else if(gamepad1.left_bumper){intakeDirection=-1;}
        else{intakeDirection=0;}

        motor.setPower(0.3*intakeDirection);

        boolean currentTouchState = !bench.getTouchSensorState();
        if(currentTouchState && !lastTouchState){
            if(intakeDirection==1){BallCounter++;}
            else if(intakeDirection==-1 && BallCounter>0){BallCounter--;}
        }

        lastTouchState=currentTouchState;
        telemetry.addData("Ball Count",BallCounter);

        telemetry.update();

    }
}



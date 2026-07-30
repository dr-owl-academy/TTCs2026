package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedroPathing.JeremyMechanisms.SimpleMbappeMechanism;

@TeleOp
public class JeremySensorPractice extends OpMode {
    SimpleMbappeMechanism bench = new SimpleMbappeMechanism(); // make a new test bench instance
    private DcMotor motor = null; // define a motor

    // define class variables
    double motorVelocity = 0.3; // speed of motor
    int sensorHoldTime = 0; // if held for enough time will turn off motor (3 second = 150 revolutions)
    boolean isMotorSpinning = false;
    int sensorClicks = 0; // how many times the sensor was clicked

    @Override
    public void init(){
        bench.init(hardwareMap);

        motor = hardwareMap.get(DcMotor.class, "motor"); // initialize motor
        motor.setPower(0);
        motor.setDirection(DcMotor.Direction.FORWARD);
    }

    @Override
    public void loop(){


        if(!bench.touchSensorState()){
            isMotorSpinning = true; // make the motor spin
            motorVelocity = -motorVelocity; // flip the direction
            sensorHoldTime++; // add one to the time
            sensorClicks++;

            if(sensorHoldTime >= 150){
                isMotorSpinning = false;
            }
        } else{
            sensorHoldTime = 0; // reset time if released
            if(isMotorSpinning){ // spin the motor only when told to
                motor.setPower(motorVelocity);
            }
        }

        telemetry.addData("Touch Sensor State: ", bench.touchSensorState());
        telemetry.addData("Motor Power: ", motor.getPower());
        telemetry.addData("Sensor Clicks: ", sensorClicks);
        telemetry.update();
    }
}

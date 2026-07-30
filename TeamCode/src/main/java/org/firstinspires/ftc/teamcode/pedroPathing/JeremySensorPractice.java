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
    double power;
    int sensorClicks = 0; // how many times the sensor was clicked
    boolean sensorClicked = false;

    @Override
    public void init(){
        bench.init(hardwareMap);

        motor = hardwareMap.get(DcMotor.class, "motor"); // initialize motor
        motor.setPower(0);
        motor.setDirection(DcMotor.Direction.FORWARD);
    }

    @Override
    public void loop(){
        // gamepad movement
        if (gamepad1.right_bumper) {  // intake
            power = motorVelocity;
            motor.setPower(power);

        } else if (gamepad1.left_bumper) {
            power = -motorVelocity;
            motor.setPower(power); // outake

        } else {
            power = 0;
            motor.setPower(0); //power off
        }

        if(!bench.touchSensorState()){ // if button is pressed
            if(!sensorClicked){
                if(power > 0){ // if the motor is spinning forward
                    sensorClicks++;
                } else if(power < 0){ // if the motor is spinning backward
                    sensorClicks--;
                }
                sensorClicked = true;
            }
        } else { // reset after sensor is released
            sensorClicked = false;
        }


        telemetry.addData("Touch Sensor State: ", bench.touchSensorState());
        telemetry.addData("Motor Power: ", motor.getPower());
        telemetry.addData("Balls: ", sensorClicks);
        telemetry.update();
    }
}

package org.firstinspires.ftc.teamcode.mechanisms;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class DanielMechanism {
    TouchSensor touchSensor;


    public void init(HardwareMap hwMap) {


        touchSensor = hwMap.get(TouchSensor.class, "touchSensor");


    }

    public boolean getTouchSensorState() {
        return touchSensor.isPressed();
    }

    public int numbercounter() {
        int counter = 0;
        if (touchSensor.isPressed()) {
            counter = counter + 1;

        }
        return counter;
    }
}

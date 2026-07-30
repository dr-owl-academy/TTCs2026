package org.firstinspires.ftc.teamcode.mechanisms;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DanielMechanism {
    private DigitalChannel touchSensor;




    public void init(HardwareMap hwMap) {


        touchSensor = hwMap.get(DigitalChannel.class,"touch_sensor");
        touchSensor.setMode(DigitalChannel.Mode.INPUT);

    }

    public boolean getTouchSensorState() {
        return touchSensor.getState();
    }
}

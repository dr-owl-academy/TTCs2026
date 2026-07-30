package org.firstinspires.ftc.teamcode.pedroPathing.JeremyMechanisms;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SimpleMbappeMechanism {

    public DigitalChannel touchSensor; // create a touch sensor

    public void init(HardwareMap hardwareMap){
        touchSensor = hardwareMap.get(DigitalChannel.class, "touchSensor"); // define the sensor as the one in the driver hub
        touchSensor.setMode(DigitalChannel.Mode.INPUT);  // make the sensor input
    }

    public boolean touchSensorState() { // set the touch sensor state
        return touchSensor.getState();
    }
}

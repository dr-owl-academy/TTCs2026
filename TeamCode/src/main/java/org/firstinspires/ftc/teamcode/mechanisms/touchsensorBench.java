package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class touchsensorBench {

    private DigitalChannel touchSensor;

    public void init(HardwareMap hwMap){
        touchSensor=hwMap.get(DigitalChannel.class,"touchSensor");
        touchSensor.setMode(DigitalChannel.Mode.INPUT);
    }

    public boolean getTouchSensorState(){
        return touchSensor.getState();
    }
}

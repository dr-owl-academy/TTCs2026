package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms.touchsensorBench;

public class TouchSensorPractice extends OpMode {

    touchsensorBench bench = new touchsensorBench();

    @Override
    public void init() {
        bench.init(hardwareMap);
    }


    @Override
    public void loop() {
        telemetry.addData("Touch Sensor State", bench.getTouchSensorState());

    }
}



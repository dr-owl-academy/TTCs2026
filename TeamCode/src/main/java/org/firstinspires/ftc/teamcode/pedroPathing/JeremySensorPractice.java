package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedroPathing.JeremyMechanisms.SimpleMbappeMechanism;

public class JeremySensorPractice extends OpMode {
    SimpleMbappeMechanism bench = new SimpleMbappeMechanism(); // make a new test bench instance
    private DcMotor motor = null; // define a motor

    @Override
    public void init(){
        bench.init(hardwareMap);

        motor = hardwareMap.get(DcMotor.class, "motor"); // initialize motor
    }

    @Override
    public void loop(){
        telemetry.addData("Touch Sensor State: ", bench.touchSensorState());

        if(!bench.touchSensorState()){
            motor.setDirection(DcMotor.Direction.FORWARD);
        }
    }
}

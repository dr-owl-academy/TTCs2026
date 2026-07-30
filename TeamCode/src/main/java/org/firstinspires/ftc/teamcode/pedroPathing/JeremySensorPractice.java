package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedroPathing.JeremyMechanisms.SimpleMbappeMechanism;

@TeleOp
public class JeremySensorPractice extends OpMode {
    SimpleMbappeMechanism bench = new SimpleMbappeMechanism(); // make a new test bench instance
    private DcMotor motor = null; // define a motor

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
            motor.setPower(0.5);
        }

        telemetry.addData("Touch Sensor State: ", bench.touchSensorState());
        telemetry.addData("Motor Power: ", motor.getPower());
        telemetry.update();
    }
}

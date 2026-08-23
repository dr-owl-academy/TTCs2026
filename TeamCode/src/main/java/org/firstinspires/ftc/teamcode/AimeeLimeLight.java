package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Green Ball Motor Test")
public class AimeeLimeLight extends OpMode {

    private DcMotor testMotor;

    private Limelight3A limelight;

    //
    private static final int GREEN_BALL_PIPELINE = 9;

    private static final double MOTOR_POWER = 0.2;


    @Override
    public void init() {

        //Motor
        testMotor = hardwareMap.get(
                DcMotor.class,
                "testMotor"
        );

        testMotor.setDirection(
                DcMotor.Direction.FORWARD
        );

        testMotor.setPower(0);
    }
}

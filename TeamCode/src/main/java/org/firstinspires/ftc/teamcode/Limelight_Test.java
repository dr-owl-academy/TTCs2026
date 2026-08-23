package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous (name = "Green Ball Motor Test")
public class Limelight_Test extends OpMode {

    private DcMotor testMotor;
    private Limelight3A limelight;

    private static final int GREEN_BALL_PIPELINE = 9;
    private static final double MOTOR_POWER = 0.2;

    @Override //ignore everything before this line
    public void init() {
//setting up motor
        testMotor = hardwareMap.get(
                DcMotor.class,
                "test_Motor"
        );

        testMotor.setDirection(
                DcMotor.Direction.FORWARD
        );

        testMotor.setPower(0);

        //Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(GREEN_BALL_PIPELINE); //tells limelight which pipeline you chose

        telemetry.addData("Status", "Initialized");
        telemetry.addData(
                "Pipeline",
                GREEN_BALL_PIPELINE
        );
    }

    @Override
    public void start() {
//start receiving results from Limelight
        limelight.start();
    }

    @Override
    public void loop() {

        LLResult result = limelight.getLatestResult();

        boolean targetDetected = false;

        //check whether green result exists
        if (result != null && result.isValid() && !result.getColorResults().isEmpty()) {

            targetDetected = true;
        }

        //Motor Control

        if(targetDetected) {

            testMotor.setPower(MOTOR_POWER);

        }  else {

            testMotor.setPower(0);

        }

        //telemetry

        telemetry.addData(
                "Green Ball Detected",
targetDetected
        );

        telemetry.addData(
                "Motor Power",
                testMotor.getPower()
        );

        telemetry.update();
    }

    @Override
    public void stop() {

        testMotor.setPower(0);

        limelight.stop();
    }
}

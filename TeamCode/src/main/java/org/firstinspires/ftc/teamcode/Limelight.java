package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Green Ball Motor Test")
public class Limelight extends OpMode {

    private DcMotor testMotor;
    private Limelight3A limelight;

    //Change to pipeline number used to tune greenballs
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


        //Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(GREEN_BALL_PIPELINE);

        limelight.pipelineSwitch(GREEN_BALL_PIPELINE);

        telemetry.addData("Status", "Initialized");

        telemetry.addData("Status", "Initialized");
        telemetry.addData(
                "Pipeline",
                GREEN_BALL_PIPELINE
        );
    }


    @Override
    public void start() {

        //start recieving results from limelight
        limelight.start();
    }


    @Override
    public void loop() {

        LLResult result = limelight.getLatestResult();

        boolean targetDetected = false;


        //Does green target exist
        if (result != null && result.isValid() && !result.getColorResults().isEmpty()) {

            targetDetected = true;
        }

        //Some stuff here
        //Control
        //more stuff

        if (targetDetected) {

            testMotor.setPower(MOTOR_POWER);

        } else {

            testMotor.setPower(0);
        }


        telemetry.addData(
                "Green Ball Detected",
                targetDetected
        );

        telemetry.update();
    }


    @Override
    public void stop() {

        testMotor.setPower(0);

        limelight.stop();
    }
}

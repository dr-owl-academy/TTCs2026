package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name= "yajatAutonBot", group = "StarterBot")
public class yajatAutonBot extends OpMode{

    DcMotor frontLeft;

    DcMotor frontRight;

    DcMotor backLeft;

    DcMotor backRight;

    double leftFrontPower;
    double leftBackPower;
    double rightFrontPower;
    double rightBackPower;


    ElapsedTime timer =new ElapsedTime();

    double drivePower = 0.5;

    @Override
    public void init() {
        // Connect motors to the Robot Configuration.
        frontLeft = hardwareMap.get(DcMotor.class,"frontLeft");
        frontRight = hardwareMap.get(DcMotor.class,"frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        // Same motor directions as the TeleOp.
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // Stop quickly instead of coasting.
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Ready");
    }

    void mecanumDrive(double forward,double strafe, double rotate){double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate),1);

        leftFrontPower = (forward + strafe + rotate) / denominator;
        rightFrontPower = (forward -strafe - rotate) / denominator;
        leftBackPower = (forward - strafe + rotate) / denominator;
        rightBackPower = (forward + strafe - rotate) / denominator;

        telemetry.addData("forward","%.2f",forward);
        telemetry.addData("strafing","%.2f",strafe);
        telemetry.addData("rotation","%.2f",rotate);

        frontLeft.setPower(leftFrontPower);
        frontRight.setPower(rightFrontPower);
        backLeft.setPower(leftBackPower);
        backRight.setPower(rightBackPower);}

    @Override
    public void start() {

        // Start the timer when the driver presses Play.
        timer.reset();
    }

    @Override
    public void loop() {

        double time = timer.seconds();

        if (time < 0.9) {

            // Side 1: Drive forward.
            mecanumDrive(0.5,0,0);



        } else if (time < 1.1) {

            // Pause.
            mecanumDrive(0,0,0);

        } else if (time < 2.3) {

            // Side 2: Strafe right.
            mecanumDrive(0,0.5,0);

        } else if (time < 2.5) {

            // Pause.
            mecanumDrive(0,0,0);

        } else if (time < 3.4) {

            // Side 3: Drive backward.
            mecanumDrive(-0.5,0,0);

        } else if (time < 3.6) {

            // Pause.
            mecanumDrive(0,0,0);

        } else if (time < 4.8) {

            // Side 4: Strafe left.
            mecanumDrive(0,-0.5,0);

        } else {

            // Finished.
            mecanumDrive(0,0,0);
        }

        telemetry.addData("Time", "%.1f seconds", time);
        telemetry.update();
    }

    @Override
    public void stop() {

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    }


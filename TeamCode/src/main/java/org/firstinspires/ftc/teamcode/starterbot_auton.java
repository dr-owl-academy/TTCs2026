package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "starterbot_auton")
public class starterbot_auton  extends OpMode {

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    ElapsedTime timer = new ElapsedTime();

    double drivePower = 0.50;

    @Override
    public void init() {

        // Connect motors to the Robot Configuration.
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
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

    @Override
    public void start() {

        // Start the timer when the driver presses Play.
        timer.reset();
    }

    @Override
    public void loop() {

        double time = timer.seconds();

        if (time < 1.5) {

            // Side 1: Drive forward.
            frontLeft.setPower(drivePower);
            frontRight.setPower(drivePower);
            backLeft.setPower(drivePower);
            backRight.setPower(drivePower);

        } else if (time < 2.0) {

            // Pause.
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

        } else if (time < 3.8) {

            // Side 2: Strafe right.
            frontLeft.setPower(drivePower);
            frontRight.setPower(-drivePower);
            backLeft.setPower(-drivePower);
            backRight.setPower(drivePower);

        } else if (time < 4.3) {

            // Pause.
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

        } else if (time < 5.8) {

            // Side 3: Drive backward.
            frontLeft.setPower(-drivePower);
            frontRight.setPower(-drivePower);
            backLeft.setPower(-drivePower);
            backRight.setPower(-drivePower);

        } else if (time < 6.3) {

            // Pause.
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

        } else if (time < 8.1) {

            // Side 4: Strafe left.
            frontLeft.setPower(-drivePower);
            frontRight.setPower(drivePower);
            backLeft.setPower(drivePower);
            backRight.setPower(-drivePower);

        } else {

            // Finished.
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
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
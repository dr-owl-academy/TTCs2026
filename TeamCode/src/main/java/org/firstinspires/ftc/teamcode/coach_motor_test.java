package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "coach_mecanum_motor_test")
public class coach_motor_test extends OpMode {

    // Drivetrain motors
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;

    // Intake motor
    private DcMotorEx intakeMotor;

    // Low power is safer for individual motor testing
    private static final double MOTOR_TEST_POWER = 0.25;

    private static final double INTAKE_VELOCITY = 2000;

    private String selectedMotor = "None";

    @Override
    public void init() {

        leftFrontDrive =
                hardwareMap.get(DcMotor.class, "frontLeft");

        rightFrontDrive =
                hardwareMap.get(DcMotor.class, "frontRight");

        leftBackDrive =
                hardwareMap.get(DcMotor.class, "backLeft");

        rightBackDrive =
                hardwareMap.get(DcMotor.class, "backRight");

        intakeMotor =
                hardwareMap.get(DcMotorEx.class, "intakemotor");

        /*
         * Both left motors are reversed so positive power makes
         * every wheel drive the robot forward.
         */
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);

        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        intakeMotor.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        intakeMotor.setZeroPowerBehavior(BRAKE);

        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        stopDriveMotors();
        intakeMotor.setVelocity(0);

        telemetry.addLine("Motor test initialized");
        telemetry.addLine("D-pad up: right-front");
        telemetry.addLine("D-pad left: left-front");
        telemetry.addLine("D-pad down: left-back");
        telemetry.addLine("D-pad right: right-back");
        telemetry.update();
    }

    @Override
    public void loop() {

        /*
         * Stop every drivetrain motor at the beginning of each loop.
         * The selected motor is then turned on below.
         */
        stopDriveMotors();
        selectedMotor = "None";

        if (gamepad1.dpad_up) {

            rightFrontDrive.setPower(MOTOR_TEST_POWER);
            selectedMotor = "Right Front";

        } else if (gamepad1.dpad_left) {

            leftFrontDrive.setPower(MOTOR_TEST_POWER);
            selectedMotor = "Left Front";

        } else if (gamepad1.dpad_down) {

            leftBackDrive.setPower(MOTOR_TEST_POWER);
            selectedMotor = "Left Back";

        } else if (gamepad1.dpad_right) {

            rightBackDrive.setPower(MOTOR_TEST_POWER);
            selectedMotor = "Right Back";
        }

        /*
         * Intake controls:
         * Right bumper = intake
         * Left bumper = reverse intake
         */
        if (gamepad1.right_bumper) {

            intakeMotor.setVelocity(INTAKE_VELOCITY);

        } else if (gamepad1.left_bumper) {

            intakeMotor.setVelocity(-INTAKE_VELOCITY);

        } else {

            intakeMotor.setVelocity(0);
        }

        telemetry.addData("Selected motor", selectedMotor);
        telemetry.addData("Test power", MOTOR_TEST_POWER);
        telemetry.addData(
                "Right Front Power",
                rightFrontDrive.getPower()
        );
        telemetry.addData(
                "Left Front Power",
                leftFrontDrive.getPower()
        );
        telemetry.addData(
                "Left Back Power",
                leftBackDrive.getPower()
        );
        telemetry.addData(
                "Right Back Power",
                rightBackDrive.getPower()
        );
        telemetry.addData(
                "Intake Velocity",
                intakeMotor.getVelocity()
        );
        telemetry.update();
    }

    private void stopDriveMotors() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    @Override
    public void stop() {
        stopDriveMotors();
        intakeMotor.setVelocity(0);
    }
}
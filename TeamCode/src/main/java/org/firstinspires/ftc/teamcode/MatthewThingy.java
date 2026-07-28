package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Matthew_Thingy")
public class MatthewThingy extends OpMode {

    // Drive motors
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;

    // Intake motor must be DcMotorEx to use setVelocity()
    private DcMotorEx intakeMotor;

    // Pedro is used only to read the robot's pose
    private Follower follower;

    // Drive motor powers for telemetry
    private double leftFrontPower;
    private double rightFrontPower;
    private double leftBackPower;
    private double rightBackPower;

    /*
     * Intake speed in encoder ticks per second.
     * Adjust this number after testing.
     */
    private static final double INTAKE_VELOCITY = 2000;

    @Override
    public void init() {

        /*
         * Create Pedro using the existing drivetrain
         * and Pinpoint configuration.
         */
        follower = Constants.createFollower(hardwareMap);

        // Starting coordinate
        follower.setStartingPose(new Pose(0, 0, 0));

        // Connect the drivetrain motors
        leftFrontDrive =
                hardwareMap.get(DcMotor.class, "frontLeft");

        rightFrontDrive =
                hardwareMap.get(DcMotor.class, "frontRight");

        leftBackDrive =
                hardwareMap.get(DcMotor.class, "backLeft");

        rightBackDrive =
                hardwareMap.get(DcMotor.class, "backRight");

        /*
         * Connect the intake as DcMotorEx.
         *
         * The Hardware Map name must be exactly:
         * intakemotor
         */
        intakeMotor =
                hardwareMap.get(DcMotorEx.class, "intakemotor");

        /*
         * Symmetric drivetrain:
         * both left motors are reversed.
         */
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);

        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Change to REVERSE if the intake spins backward
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);

        // Brake when motor power is zero
        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        intakeMotor.setZeroPowerBehavior(BRAKE);

        /*
         * Run the intake using its built-in encoder.
         *
         * Resetting the encoder is not necessary because
         * this code controls velocity rather than position.
         */
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Begin with all motors stopped
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);

        intakeMotor.setVelocity(0);

        // Read the starting Pinpoint position
        follower.updatePose();

        telemetry.addLine("Initialized");
        telemetry.addLine("Intake uses encoder velocity control");
        telemetry.update();
    }

    @Override
    public void loop() {

        /*
         * Update only Pedro's localization.
         * mecanumDrive() controls the drivetrain directly.
         */
        follower.updatePose();

        // Drive the robot
        mecanumDrive(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x
        );

        /*
         * Intake controls:
         *
         * Right bumper = intake
         * Left bumper  = reverse intake
         * Neither      = stop
         */
        if (gamepad1.right_bumper) {

            intakeMotor.setVelocity(INTAKE_VELOCITY);

        } else if (gamepad1.left_bumper) {

            intakeMotor.setVelocity(-INTAKE_VELOCITY);

        } else {

            intakeMotor.setVelocity(0);
        }

        // Read the robot's current Pedro pose
        Pose robotPose = follower.getPose();

        telemetry.addData(
                "X",
                "%.2f inches",
                robotPose.getX()
        );

        telemetry.addData(
                "Y",
                "%.2f inches",
                robotPose.getY()
        );

        telemetry.addData(
                "Heading",
                "%.1f degrees",
                Math.toDegrees(robotPose.getHeading())
        );

        /*
         * Target velocity is what the code requests.
         * Actual velocity is what the encoder measures.
         */
        telemetry.addData(
                "Intake Target",
                "%.0f ticks/sec",
                intakeMotor.getTargetPosition()
        );

        telemetry.addData(
                "Intake Velocity",
                "%.0f ticks/sec",
                intakeMotor.getVelocity()
        );

        telemetry.update();
    }

    void mecanumDrive(
            double forward,
            double strafe,
            double rotate) {

        /*
         * Keep all motor powers between -1 and 1
         * while maintaining their relative ratios.
         */
        double denominator = Math.max(
                Math.abs(forward)
                        + Math.abs(strafe)
                        + Math.abs(rotate),
                1
        );

        leftFrontPower =
                (forward + strafe + rotate) / denominator;

        rightFrontPower =
                (forward - strafe - rotate) / denominator;

        leftBackPower =
                (forward - strafe + rotate) / denominator;

        rightBackPower =
                (forward + strafe - rotate) / denominator;

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    @Override
    public void stop() {

        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);

        intakeMotor.setVelocity(0);
    }
}
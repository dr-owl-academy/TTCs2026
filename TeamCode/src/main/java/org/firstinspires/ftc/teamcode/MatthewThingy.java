package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Matthew_Thingy")
public class MatthewThingy extends OpMode {

    // declare drive motors
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;
    private DcMotor intakemotor;
    private double leftFrontPower;
    private double rightFrontPower;
    private double leftBackPower;
    private double rightBackPower;

    /* Using Pedro to read robot coordinates
     */
    private Follower follower;


    // Code to run ONCE when the driver hits INIT
    @Override
    public void init() {

        /*
         * Create the Pedro Follower.
         * Constants.java must already be configured to use the Pinpoint localizer.
         */
        follower = Constants.createFollower(hardwareMap);

        //Change this to your desired starting pose: x, y in inches, pedro takes heading in radians
        follower.setStartingPose(new Pose(0, 0, Math.toRadians(0)));

        leftFrontDrive = hardwareMap.get(DcMotor.class, "frontLeft");

        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontRight");

        leftBackDrive = hardwareMap.get(DcMotor.class, "backLeft");

        rightBackDrive = hardwareMap.get(DcMotor.class, "backRight");

        // ADDED: Initialize intake motor
        intakemotor = hardwareMap.get(DcMotor.class, "intakemotor");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);

        // ADDED: Set intake brake mode (optional but recommended)
        intakemotor.setZeroPowerBehavior(BRAKE);

        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);

        // ADDED: Make sure intake starts off
        intakemotor.setPower(0);

        /*
         * Read the starting Pinpoint position.
         */
        follower.updatePose();

        telemetry.addLine("Initialized");

        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
    }

    @Override
    public void loop() {

        //Update only Pedro's position estimate.

        follower.updatePose();

        mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        // ADDED: Intake control with right trigger
        intakemotor.setPower(gamepad1.right_trigger);

        //Get the current Pedro pose.
        Pose robotPose = follower.getPose();

        double robotX = robotPose.getX();
        double robotY = robotPose.getY();
        double robotHeading = robotPose.getHeading();

        /*
         * Display robot coordinates.
         */
        telemetry.addData("X", "%.2f inches", robotX);
        telemetry.addData("Y", "%.2f inches", robotY);

        telemetry.addData("Heading", "%.1f degrees", Math.toDegrees(robotHeading));

        telemetry.update();
    }

    // Converts forward, strafe, and rotation commands into four mecanum-wheel motor powers.

    private void mecanumDrive(double forward, double strafe, double rotate) {

        // the denominator keeps all four motor powers between -1 and 1 while preserving their ratios.

        double denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate), 1);

        leftFrontPower = (forward + strafe + rotate) / denominator;
        rightFrontPower = (forward - strafe - rotate) / denominator;
        leftBackPower = (forward - strafe + rotate) / denominator;
        rightBackPower = (forward + strafe - rotate) / denominator;

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
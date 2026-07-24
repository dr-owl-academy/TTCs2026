package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Eason_mecanum", group = "StarterBot")
//@Disabled
public class Eason_mecanum extends OpMode {
    final double FEED_TIME_SECONDS = 0.50; //The feeder servos run this long when a shot is requested.
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;
    private static final double BLUE_GOAL_X = -57.0;
    private static final double BLUE_GOAL_Y = 57.0;

    private static final double RED_GOAL_X = 57.0;
    private static final double RED_GOAL_Y = 57.0;

    /*
     * When we control our launcher motor, we are using encoders. These allow the control system
     * to read the current speed of the motor and apply more or less power to keep it at a constant
     * velocity. Here we are setting the target, and minimum velocity that the launcher should run
     * at. The minimum velocity is a threshold for determining when to fire.
     */
    double kOffset = 0;
    double kTurn = 1.5;
    double driverTurn = 0;


    // Declare OpMode members.
    private DcMotor leftFrontDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightBackDrive = null;
    private PinpointLocalizer localizer = null;

    // Change this to your desired starting pose: x, y in inches, heading in radians
    private Pose2d initialRobotPose = new Pose2d(-24, -62, 0);
    private static final double PINPOINT_IN_PER_TICK = 0.0019684344326;

    ElapsedTime feederTimer = new ElapsedTime();

    /*
     * TECH TIP: State Machines
     * We use a "state machine" to control our launcher motor and feeder servos in this program.
     * The first step of a state machine is creating an enum that captures the different "states"
     * that our code can be in.
     * The core advantage of a state machine is that it allows us to continue to loop through all
     * of our code while only running specific code when it's necessary. We can continuously check
     * what "State" our machine is in, run the associated code, and when we are done with that step
     * move on to the next state.
     * This enum is called the "LaunchState". It reflects the current condition of the shooter
     * motor and we move through the enum when the user asks our code to fire a shot.
     * It starts at idle, when the user requests a launch, we enter SPIN_UP where we get the
     * motor up to speed, once it meets a minimum speed then it starts and then ends the launch process.
     * We can use higher level code to cycle through these states. But this allows us to write
     * functions and autonomous routines in a way that avoids loops within loops, and "waits".
     */
    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }

    private LaunchState launchState;

    // Setup a variable for each drive wheel to save power level for telemetry
    double leftFrontPower;
    double rightFrontPower;
    double leftBackPower;
    double rightBackPower;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        launchState = LaunchState.IDLE;

        /*
         * Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step.
         */
        leftFrontDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontRight");
        leftBackDrive = hardwareMap.get(DcMotor.class, "backLeft");
        rightBackDrive = hardwareMap.get(DcMotor.class, "backRight");
        /*
         * To drive forward, most robots need the motor on one side to be reversed,
         * because the axles point in opposite directions. Pushing the left stick forward
         * MUST make robot go forward. So adjust these two lines based on your first test drive.
         * Note: The settings here assume direct drive on left and right wheels. Gear
         * Reduction or 90 Deg drives may require direction flips
         */
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        /*
         * Here we set our launcher to the RUN_USING_ENCODER runmode.
         * If you notice that you have no control over the velocity of the motor, it just jumps
         * right to a number much higher than your set point, make sure that your encoders are plugged
         * into the port right beside the motor itself. And that the motors polarity is consistent
         * through any wiring.
         */
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain. As the robot stops much quicker.
         */
        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);

        // coach: Initialize PinpointLocalizer with starting pose
        localizer = new PinpointLocalizer(hardwareMap, PINPOINT_IN_PER_TICK, initialRobotPose);

        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Initial Pose", "(%.2f, %.2f, %.2f rad)", initialRobotPose.position.x, initialRobotPose.position.y, initialRobotPose.heading.toDouble());
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

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        /*
         * Here we call a function called arcadeDrive. The arcadeDrive function takes the input from
         * the joysticks, and applies power to the left and right drive motor to move the robot
         * as requested by the driver. "arcade" refers to the control style we're using here.
         * Much like a classic arcade game, when you move the left joystick forward both motors
         * work to drive the robot forward, and when you move the right joystick left and right
         * both motors work to rotate the robot. Combinations of these inputs can be used to create
         * more complex maneuvers.
         */
        PoseVelocity2d currentVelocity = localizer.update();
        Pose2d currentPose = localizer.getPose();

// hold right bumper to auto-aim
        if (gamepad1.right_bumper) {
            driverTurn = spintoRed(currentPose);
        } else {
            driverTurn = gamepad1.right_stick_x;
        }

        mecanumDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x, driverTurn);


        /*
        if (gamepad2.dpadUpWasPressed()) {
            LAUNCHER_TARGET_VELOCITY += 10;
        }

        if (gamepad2.dpadDownWasPressed()) {
            LAUNCHER_TARGET_VELOCITY -= 10;
        }
        */

        /*
         * Now we call our "Launch" function.
         */




// Distance to BLUE goal
        double distToBlue = Math.hypot(BLUE_GOAL_X - currentPose.position.x, BLUE_GOAL_Y - currentPose.position.y);

// Distance to RED goal
        double distToRed = Math.hypot(RED_GOAL_X - currentPose.position.x, RED_GOAL_Y - currentPose.position.y);


        double robotX = currentPose.position.x;
        double robotY = currentPose.position.y;
        double robotHeading = currentPose.heading.toDouble();

        double dx = RED_GOAL_X - robotX;
        double dy = RED_GOAL_Y - robotY;

        double targetAngle = -Math.atan2(dx, dy); // radians
        double angleError = targetAngle - robotHeading;

        // wrap to [-pi, pi]
        //angleError = Math.atan2(Math.sin(angleError), Math.cos(angleError));

        /*
         * Show the state and motor powers
         */
        telemetry.addData("State", launchState);
        telemetry.addData("Pose", "(%.1f, %.1f, %.1f)", currentPose.position.x, currentPose.position.y, Math.toDegrees(currentPose.heading.toDouble()));
        telemetry.addData("Velocity", "(%.1f, %.1f, %.1f)", currentVelocity.linearVel.x, currentVelocity.linearVel.y, Math.toDegrees(currentVelocity.angVel));
        telemetry.addData("Dist Blue", "%.1f in", distToBlue);
        telemetry.addData("Dist Red", "%.1f in", distToRed);
        telemetry.addData("targetAngle", Math.toDegrees(targetAngle));
        telemetry.addData("angleError", Math.toDegrees(angleError));
        telemetry.update();

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    void mecanumDrive(double forward, double strafe, double rotate){
        double dimension =
        leftFrontPower = (forward+strafe+rotate)/dimension;
        rightFrontPower = (forward-strafe-rotate)/dimension;
        leftBackPower = (forward-strafe+rotate)/dimension;
        rightBackPower = (forward+strafe-rotate)/dimension;

    }

    double velocityFromDistance(double x) {
        // Only clamp minimum (no upper clamp)
        x = Math.max(18, x);
/*
        return  0.0000487634 * x * x * x
                - 0.0120502 * x * x
                + 6.84276 * x
                + 1021.17195;
        */
        return -0.000439386 * x * x * x
                + 0.128207 * x * x
                - 5.0367 * x
                + 1298.79524;
    }

    double spintoRed (Pose2d pose2d) {
        double robotX = pose2d.position.x;
        double robotY = pose2d.position.y;
        double robotHeading = pose2d.heading.toDouble(); // radians

        double dx = RED_GOAL_X - robotX;
        double dy = RED_GOAL_Y - robotY;

        double targetAngle = -Math.atan2(dx, dy); // radians
        double angleError = targetAngle - robotHeading;

        // wrap to [-pi, pi], can also use mod but more complicated
        angleError = Math.atan2(Math.sin(angleError), Math.cos(angleError));
        return -kTurn * angleError;
    }
}
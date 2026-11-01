package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
public class PIDF_flywheel extends OpMode {

    // Hardware and tuning variables
    public DcMotorEx flywheelMotor;
    public double TargetVelocity = 2030;
    public double lowVelocity = 900;
    public double curTargetVelocity = TargetVelocity;

    // PIDF constants: adjusted using the gamepad
    public double p = 0.0;
    public double f = 15.58;

    // Step adjustment sensitivity
    public double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001};
    public int stepIndex = 1;

    // ADDED: Panels telemetry
    private TelemetryManager panelsTelemetry;

    // ADDED: previous button states for one adjustment per press
    private boolean previousY = false;
    private boolean previousB = false;
    private boolean previousLeft = false;
    private boolean previousRight = false;
    private boolean previousUp = false;
    private boolean previousDown = false;

    @Override
    public void init() {
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setDirection(DcMotor.Direction.REVERSE);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(p, 0, 0, f);
        flywheelMotor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        // ADDED: initialize Panels and send status to both displays
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        panelsTelemetry.debug("Init Complete");
        panelsTelemetry.debug("Y: High/Low velocity | B: Step size");
        panelsTelemetry.debug("D-pad Left/Right: F | Up/Down: P");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        // Read button states once per loop
        boolean y = gamepad1.y;
        boolean b = gamepad1.b;
        boolean left = gamepad1.dpad_left;
        boolean right = gamepad1.dpad_right;
        boolean up = gamepad1.dpad_up;
        boolean down = gamepad1.dpad_down;

        // CHANGED: toggle only once per Y press
        if (y && !previousY) {
            if (curTargetVelocity == TargetVelocity) {
                curTargetVelocity = lowVelocity;
            } else {
                curTargetVelocity = TargetVelocity;
            }
        }

        // CHANGED: cycle step size only once per B press
        if (b && !previousB) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        // CHANGED: adjust F only once per D-pad press
        if (left && !previousLeft) {
            f -= stepSizes[stepIndex];
        }
        if (right && !previousRight) {
            f += stepSizes[stepIndex];
        }

        // CHANGED: adjust P only once per D-pad press
        if (up && !previousUp) {
            p += stepSizes[stepIndex];
        }
        if (down && !previousDown) {
            p -= stepSizes[stepIndex];
        }

        previousY = y;
        previousB = b;
        previousLeft = left;
        previousRight = right;
        previousUp = up;
        previousDown = down;

        // Apply PIDF coefficients and command motor velocity
        PIDFCoefficients newPIDF = new PIDFCoefficients(p, 0, 0, f);
        flywheelMotor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER, newPIDF);
        flywheelMotor.setVelocity(curTargetVelocity);

        // Gather performance data; velocity units are encoder ticks/sec
        double curVelocity = flywheelMotor.getVelocity();
        double error = curTargetVelocity - curVelocity;

        // FIXED: Panels Graph reads numeric key/value telemetry.
        // Use addData(), not graph(), with this Panels API.
        panelsTelemetry.addData("Target Velocity", curTargetVelocity);
        panelsTelemetry.addData("Actual Velocity", curVelocity);
        panelsTelemetry.addData("Velocity Error", error);
        panelsTelemetry.addData("Tuning P", p);
        panelsTelemetry.addData("Tuning F", f);
        panelsTelemetry.addData("Current Step Size", stepSizes[stepIndex]);

        // ADDED: send to Panels and Driver Hub
        panelsTelemetry.update(telemetry);
    }

    // ADDED: stop the flywheel when the OpMode ends
    @Override
    public void stop() {
        flywheelMotor.setPower(0);
    }
}
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

@TeleOp(name = "river_distance_calc", group = "Test")
public class River_Distance_calc extends LinearOpMode {

    private GoBildaPinpointDriver pinpoint;

    // Optional: motors (not used for driving, just so the config matches)
    private DcMotor motorRF, motorRB, motorLF, motorLB, intake;

    private final double HIVE_X = 72.0;
    private final double HIVE_Y = 72.0;

    @Override
    public void runOpMode() {
        // Motors (match your config names)
        motorRF = hardwareMap.get(DcMotor.class, "motorRF");
        motorRB = hardwareMap.get(DcMotor.class, "motorRB");
        motorLF = hardwareMap.get(DcMotor.class, "motorLF");
        motorLB = hardwareMap.get(DcMotor.class, "motorLB");
        intake  = hardwareMap.get(DcMotor.class, "intake motor");

        // Pinpoint
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Set starting position: 72, 72, 90 degrees
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 72, 72, AngleUnit.DEGREES, 90));

        telemetry.addLine("Push the robot by hand");
        telemetry.addLine("Starting at 72, 72, 90");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            pinpoint.update();

            Pose2D pose = pinpoint.getPosition();

            double x = pose.getX(DistanceUnit.INCH);
            double y = pose.getY(DistanceUnit.INCH);
            double heading = pose.getHeading(AngleUnit.DEGREES);

            double distance = Math.hypot(HIVE_X - x, HIVE_Y - y);

            telemetry.addData("X", "%.1f", x);
            telemetry.addData("Y", "%.1f", y);
            telemetry.addData("Heading", "%.1f", heading);
            telemetry.addData("Distance to Hive", "%.1f", distance);
            telemetry.update();
        }
    }
}
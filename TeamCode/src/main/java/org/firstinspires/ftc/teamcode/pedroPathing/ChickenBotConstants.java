package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Disabled
public class ChickenBotConstants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(8.4)
            .forwardZeroPowerAcceleration(-34.34415518547419)
            .lateralZeroPowerAcceleration(-54.76122077)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.3, 0, 0.0001, 0.01))
            .headingPIDFCoefficients(new PIDFCoefficients(0.6, 0, 0.2, 0.01));


    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("frontRight")
            .leftFrontMotorName("frontLeft")
            .rightRearMotorName("backRight")
            .leftRearMotorName("backLeft")
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(59.146876858288226)
            .yVelocity(50.5002934814);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 0.85, 0.27);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(1.25)
            .strafePodX(-4.62)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pipackage org.firstinspires.ftc.teamcode.pedroPathing;\n" +
                    "\n" +
                    "import com.pedropathing.control.PIDFCoefficients;\n" +
                    "import com.pedropathing.follower.Follower;\n" +
                    "import com.pedropathing.follower.FollowerConstants;\n" +
                    "import com.pedropathing.ftc.FollowerBuilder;\n" +
                    "import com.pedropathing.ftc.drivetrains.MecanumConstants;\n" +
                    "import com.pedropathing.ftc.localization.constants.PinpointConstants;\n" +
                    "import com.pedropathing.paths.PathConstraints;\n" +
                    "import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;\n" +
                    "import com.qualcomm.robotcore.eventloop.opmode.Disabled;\n" +
                    "import com.qualcomm.robotcore.hardware.DcMotorSimple;\n" +
                    "import com.qualcomm.robotcore.hardware.HardwareMap;\n" +
                    "\n" +
                    "import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;\n" +
                    "\n" +
                    "\n" +
                    "@Disabled\n" +
                    "public class ChickenBotConstants {\n" +
                    "    public static FollowerConstants followerConstants = new FollowerConstants()\n" +
                    "            .mass(8.4)\n" +
                    "            .forwardZeroPowerAcceleration(-34.34415518547419)\n" +
                    "            .lateralZeroPowerAcceleration(-54.76122077)\n" +
                    "            .translationalPIDFCoefficients(new PIDFCoefficients(0.3, 0, 0.0001, 0.01))\n" +
                    "            .headingPIDFCoefficients(new PIDFCoefficients(0.6, 0, 0.2, 0.01));\n" +
                    "\n" +
                    "\n" +
                    "    public static MecanumConstants driveConstants = new MecanumConstants()\n" +
                    "            .maxPower(1)\n" +
                    "            .rightFrontMotorName(\"frontRight\")\n" +
                    "            .leftFrontMotorName(\"frontLeft\")\n" +
                    "            .rightRearMotorName(\"backRight\")\n" +
                    "            .leftRearMotorName(\"backLeft\")\n" +
                    "            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)\n" +
                    "            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)\n" +
                    "            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)\n" +
                    "            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)\n" +
                    "            .xVelocity(59.146876858288226)\n" +
                    "            .yVelocity(50.5002934814);\n" +
                    "\n" +
                    "    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 0.85, 0.27);\n" +
                    "\n" +
                    "    public static Follower createFollower(HardwareMap hardwareMap) {\n" +
                    "        return new FollowerBuilder(followerConstants, hardwareMap)\n" +
                    "                .pathConstraints(pathConstraints)\n" +
                    "                .mecanumDrivetrain(driveConstants)\n" +
                    "                .pinpointLocalizer(localizerConstants)\n" +
                    "                .build();\n" +
                    "    }\n" +
                    "\n" +
                    "    public static PinpointConstants localizerConstants = new PinpointConstants()\n" +
                    "            .forwardPodY(1.25)\n" +
                    "            .strafePodX(-4.62)\n" +
                    "            .distanceUnit(DistanceUnit.INCH)\n" +
                    "            .hardwareMapName(\"pinpoint\")\n" +
                    "            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)\n" +
                    "            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);\n" +
                    "\n" +
                    "\n" +
                    "\n" +
                    "}npoint")
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);



}
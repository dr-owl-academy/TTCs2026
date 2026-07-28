package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//(other values might also change but I either forgot OR didn't add it (glanced over the code))   -----**

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants().mass(7.05) //new mass of robot

            //.forwardZeroPowerAcceleration(-34.34415518547419) //change later   -----**
            //.lateralZeroPowerAcceleration(-54.76122077) //change later   -----**
            //.translationalPIDFCoefficients(new PIDFCoefficients(0.3,0,0.0001,0.01)) //change later   -----**
            //.headingPIDFCoefficients(new PIDFCoefficients(0.6,0,0.2,0.01)); //change later   -----**


    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")
            .leftRearMotorName("backLeft")
            .leftFrontMotorName("frontLeft")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(37.49650465597318)
            //.yVelocity(50.5002934814); //change later   -----**


    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(1.5) //note that these values may be wrong (as of writing this)
            .strafePodX(-6.5) //note that these values may be wrong (as of writing this)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);


    public static PathConstraints pathConstraints = new PathConstraints(0.99,
            100,
            1,
            1);  //change breakingStrength and breakingStart later   -----**

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}

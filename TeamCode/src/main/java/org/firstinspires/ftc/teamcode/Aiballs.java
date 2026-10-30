package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import java.util.List;
import edu.wpi.first.math.geometry.Translation2d;
import com.ctre.phoenix6.swerve.SwerveRequest;
public class Aiballs {
    @Override
    public void loop() {
        // Check if the 'A' button is currently pressed down
        if (gamepad1.a) {
            double currentx = follower.getpose().get();
        }
    }
}

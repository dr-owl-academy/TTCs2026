package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Hanming_Simple_Auton")
public class Hanming_Simple_Auton {
}

    private enum Autostate {
        Init_turn,
        Wait_turn,
        Init_drive,
        Wait_drive,
        Complete
}

private Bot follower;
private path pathChain;
private